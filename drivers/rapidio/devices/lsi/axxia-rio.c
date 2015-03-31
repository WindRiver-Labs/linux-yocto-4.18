/*
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.
 */

/* #define IODEBUG */
/* #define EXTRA1DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/dmapool.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <mach/rio.h>
#include "axxia-rio.h"
#include "axxia-rio-irq.h"

#define USE_DME_TIMEOUT
static DEFINE_SPINLOCK(rio_io_lock);

#ifdef IODEBUG
#define	IODP(...)	pr_info(__VA_ARGS__)
#else
#define	IODP(...)
#endif

#ifdef EXTRA1DEBUG
#define	EXT1P(...)	pr_info(__VA_ARGS__)
#else
#define	EXT1P(...)
#endif

#define RIO_IO_READ_HOME        0x00
#define RIO_MAINT_READ          0x01
#define RIO_MAINT_WRITE         0x10
#define RIO_NREAD               0x02
#define RIO_NWRITE              0x20
#define RIO_NWRITE_R            0x40
#define RIO_SWRITE              0x80
/**
 * NOTE:
 *
 * sRIO Bridge in AXXIA is what it is...
 *
 * - Paged access to configuration registers makes local config
 *   read a non-atomic operation.
 *
 * - Big and Little Endian mode registers
 *   Big Endian:
 *       0x0000-0xFFFC   - RapidIO Standard Registers
 *       0x10000-0x1FFFC - Endpoint Controller Specific Registers
 *   Little Endian
 *       0x20000-0x3FFFC - Peripheral Bus Bridge Specific Registers
 *
 * "SRIO_CONF" registers in AXXIA (e.g. page selection register)
 * are also Little Endian.  SRIO_CONF is organized as follows:
 *
 * - 0x000 .. 0x7ff    Fixed mapping to SRIO/RAB endpoint controller specific
 *                     registers equivalent to 0x20000 .. 0x207ff.  The
 *                     RAB_APB_CSR register within this block is used to
 *                     control the page selection of the 'paged mapping'
 *                     block.
 * - 0x800 .. 0xfff    Paged mapping to SRIO generic+endpoint controller
 *                     specific registers equivalent to 0x00000 .. 0x3ffff
 *
 * To avoid an extra spin-lock layer in __axxia_local_config_read
 * and __axxia_local_config_write, perform all internal driver accesses
 * to local config registers through the generic rio driver API.
 *
 * Accesses through the generic driver:__rio_local_write_config_32(),
 * __rio_local_read_config_32(), rio_mport_write_config_32() and
 * rio_mport_read_config_32() all use spin_lock_irqsave() /
 * spin_unlock_irqrestore(), to ensure local access restrictions.
 *
 */

/**
 * __axxia_local_config_read - Generate a AXXIA local config space read
 * @priv: Master port private data
 * @offset: Offset into configuration space
 * @data: Value to be read into
 *
 * Generates a AXXIA local configuration space read.
 * Returns %0 on success or %-EINVAL on failure.
 */
int axxia_local_config_read(struct rio_priv *priv,
			    u32 offset,
			    u32 *data)
{
	u32 page_sel;


	if ((offset >= RAB_REG_BASE) &&
	    (offset < (RAB_REG_BASE+SRIO_CONF_SPACE_SIZE_FIXED))) {
		/*
		 * Peripheral Bus Bridge Specific Registers
		 * (0x2_0000-0x2_0FFC)
		 */
		*data = ioread32(priv->regs_win_fixed + (offset & 0x7ff));
	} else {
		/* Set correct page to operate on */
		page_sel = (offset & 0x00fff800) << 5;
		iowrite32(page_sel, priv->regs_win_fixed + RAB_APB_CSR_BASE);

		if (offset < RAB_REG_BASE) {
			/*
			* Registers:
			*   RapidIO Standard (0x0000-0xFFFC)
			*   Endpoint Controller Specific (0x1_0000-0x1_FFFC)
			*/
			*data = ioread32be(priv->regs_win_paged +
						(offset & 0x7ff));
		} else if ((offset >= RAB_REG_BASE) &&
			   (offset < SRIO_SPACE_SIZE)) {
			/*
			* Peripheral Bus Bridge Specific Registers
			* (0x2_0000-0x3_FFFC)
			*/
			*data = ioread32(priv->regs_win_paged +
						(offset & 0x7ff));
		} else {
			dev_err(priv->dev,
				"RIO: Reading config register not specified for AXXIA (0x%8.8x)\n",
				offset);
		}
	}

	IODP("rio[%d]: ACR(%08x, <%08x)\n", priv->mport->id, offset, *data);

	return 0;
}

/**
 * axxia_local_config_write - Generate a AXXIA local config space write
 * @priv: Master port private data
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a AXXIA local configuration space write.
 * Returns %0 on success or %-EINVAL on failure.
 */
int axxia_local_config_write(struct rio_priv *priv,
				      u32 offset,
				      u32 data)
{
	u32 page_sel;

	if ((offset >= RAB_REG_BASE) &&
	    (offset < (RAB_REG_BASE+SRIO_CONF_SPACE_SIZE_FIXED))) {
		/*
		 * Peripheral Bus Bridge Specific Registers
		 * (0x2_0000-0x2_0FFC)
		 */
		iowrite32(data, priv->regs_win_fixed + (offset & 0x7ff));
	} else {
		/* Set correct page to operate on */
		page_sel = (offset & 0x00fff800) << 5;
		iowrite32(page_sel, priv->regs_win_fixed + RAB_APB_CSR_BASE);

		if (offset < RAB_REG_BASE) {
			/*
			* Registers:
			*   RapidIO Standard (0x0000-0xFFFC)
			*   Endpoint Controller Specific (0x1_0000-0x1_FFFC)
			*/
			iowrite32be(data, priv->regs_win_paged +
						(offset & 0x7ff));
		} else if ((offset >= RAB_REG_BASE) &&
			   (offset < SRIO_SPACE_SIZE)) {
			/*
			* Peripheral Bus Bridge Specific Registers
			* (0x2_0000-0x3_FFFC)
			*/
			iowrite32(data, priv->regs_win_paged +
						(offset & 0x7ff));
		} else {
			dev_err(priv->dev,
				"RIO: Trying to write to config register not specified for AXIA (0x%8.8x)\n",
				offset);
		}
	}

	IODP("rio[%d]: ACW(%08x, >%08x)\n", priv->mport->id, offset, data);

	return 0;
}

/**
 * axxia_rio_local_config_read - Generate a AXXIA local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a AXXIA local configuration space read.
 * Returns %0 on success or %-EINVAL on failure.
 */
static
int axxia_rio_local_config_read(struct rio_mport *mport,
				int index, u32 offset, int len, u32 *data)
{
	struct rio_priv *priv = mport->priv;
	int rc;

	if (len != sizeof(u32))
		return -EINVAL;
	rc = axxia_local_config_read(priv, offset, data);

	return rc;
}

/**
 * axxia_rio_local_config_write - Generate a AXXIA local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a AXXIA local configuration space write.
 * Returns %0 on success or %-EINVAL on failure.
 */
static
int axxia_rio_local_config_write(struct rio_mport *mport,
				 int index, u32 offset, int len, u32 data)
{
	struct rio_priv *priv = mport->priv;
	int rc;

	if (len != sizeof(u32))
		return -EINVAL;
	rc = axxia_local_config_write(priv, offset, data);

	return rc;
}

/**
 * axxia_rio_config_read - Generate a AXXIA read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a AXXIA read maintenance transaction.
 * Returns %0 on success or %-EINVAL on failure.
 */

static
int axxia_rio_config_read(struct rio_mport *mport, int index,
			  u16 destid, u8 hopcount, u32 offset,
			  int len, u32 *val)
{
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb = NULL;
	u8 *addr;
	u32 rval = 0;
	u32 rbar = 0, ctrl;
	int rc = 0;
	u32 error_code = 0;

	aoutb = &priv->outb_atmu[priv->maint_win_id];
	if (aoutb == NULL)
		return -EINVAL;

	/* 16MB maintenance windows possible */
	/* Allow only aligned access to maintenance registers */
	if (offset > (CONFIG_RIO_MAINT_WIN_SIZE - len) ||
		!IS_ALIGNED(offset, len))
		return -EINVAL;

	axxia_local_config_read(priv,
				       RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				       &ctrl);

	if (TTYPE_VAL(ctrl)) { /* Not maintenance */
		dev_err(priv->dev,
			"(%s): Window is not setup for Maintenance operations. 0x%8.8x\n",
			__func__, ctrl);
		return -EINVAL;
	}

	rbar &= ~HOP_COUNT(0xff);     /* Hop Count clear */
	rbar |= HOP_COUNT(hopcount);  /* Hop Count set */
	axxia_local_config_write(priv,
				 RAB_APIO_AMAP_RBAR(priv->maint_win_id),
				 rbar);

	ctrl &= ~TARGID(0xffff); /* Target id clear */
	ctrl |= TARGID(destid);  /* Target id set */
	axxia_local_config_write(priv,
				 RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				 ctrl);

	addr = (u8 *) aoutb->win +
		(offset & (CONFIG_RIO_MAINT_WIN_SIZE - 1));

	switch (len) {
	case 1:
		IN_SRIO8(addr, rval, rc);
		break;
	case 2:
		IN_SRIO16(addr, rval, rc);
		break;
	case 4:
		IN_SRIO32(addr, rval, rc);
		break;
	default:
		rc = -EINVAL;
	}

	axxia_local_config_read(priv, 0x608, &error_code);
	if (0 != error_code) {
		rc = -EINVAL;
		*val = 0xffffffffu;
		/* clear error code */
		axxia_local_config_write(priv,  0x608, 0);
	}

	if (rc) {
		dev_dbg(priv->dev,
			"axxia_rio_config_read: Error when reading\n");
		dev_dbg(priv->dev,
			"rio[%d]: RCR(did=%x, hc=%02x, %08x, <%08x)\n",
			mport->id, destid, hopcount, offset, rval);
	} else
		*val = rval;

	IODP("rio[%d]: RCR(did=%x, hc=%02x, %08x, <%08x)\n",
		mport->id, destid, hopcount, offset, rval);

	return rc;
}

/**
 * axxia_rio_config_write - Generate a AXXIA write maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an AXXIA write maintenance transaction.
 * Returns %0 on success or %-EINVAL on failure.
 */
static
int axxia_rio_config_write(struct rio_mport *mport, int index,
			   u16 destid, u8 hopcount, u32 offset,
			   int len, u32 val)
{
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb = NULL;
	u8 *data;
	u32 rbar = 0, ctrl, rval;
	int rc = 0;
	u32 error_code = 0;

	IODP("rio[%d]: RCW(did=%x, hc=%02x, %08x, >%08x)\n",
		mport->id, destid, hopcount, offset, val);

	/* Argument validation */

	aoutb = &priv->outb_atmu[priv->maint_win_id];
	if (aoutb == NULL)
		return -EINVAL;

	/* 16MB maintenance windows possible */
	/* Allow only aligned access to maintenance registers */
	if (offset > (CONFIG_RIO_MAINT_WIN_SIZE - len) ||
		!IS_ALIGNED(offset, len))
		return -EINVAL;

	axxia_local_config_read(priv,
				RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				&ctrl);

	if (TTYPE_VAL(ctrl)) { /* Not maintenance */
		dev_err(priv->dev,
			"(%s): Window is not setup for Maintenance operations.\n",
			__func__);
		rc = -EINVAL;
		goto err;
	}

	rbar &= ~HOP_COUNT(0xff);     /* Hop Count clear */
	rbar |= HOP_COUNT(hopcount);  /* Hop Count set */
	axxia_local_config_write(priv,
				 RAB_APIO_AMAP_RBAR(priv->maint_win_id),
				 rbar);

	ctrl &= ~TARGID(0xffff); /* Target id clear */
	ctrl |= TARGID(destid);  /* Target id set */
	axxia_local_config_write(priv,
				 RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				 ctrl);
	rval = val;
	data = (u8 *) aoutb->win +
		(offset & (CONFIG_RIO_MAINT_WIN_SIZE - 1));

	switch (len) {
	case 1:
		OUT_SRIO8(data, rval);
		break;
	case 2:
		OUT_SRIO16(data, rval);
		break;
	case 4:
		OUT_SRIO32(data, rval);
		break;
	default:
		rc = -EINVAL;
	}

	axxia_local_config_read(priv,  0x608, &error_code);
	if (0 != error_code) {

		dev_dbg(priv->dev,
			"axxia_rio_config_write: Error when writing\n");

		dev_dbg(priv->dev,
			"rio[%d]: RCW(did=%x, hc=%02x, %08x, >%08x)\n",
			mport->id, destid, hopcount, offset, val);

		rc = -EINVAL;
		/* clear error code */
		axxia_local_config_write(priv,  0x608, 0);
	}

err:
	return rc;
}

static inline int __flags2rio_tr_type(u32 mflags, u32 *trans_type)
{
	*trans_type = 0;
	/* Set type of window */
	if ((mflags == 0) || (mflags & RIO_NWRITE_R))
		*trans_type = TTYPE(NRD_NWR_R); /* nread and nwrite_r */
	else if (mflags & RIO_MAINT_WRITE)
		*trans_type = TTYPE(MRD_MWR); /* mread and mwrite */
	else if (mflags & RIO_NWRITE)
		*trans_type = TTYPE(NRD_NWR); /* nread and nwrite */
	else if (mflags & RIO_SWRITE)
		*trans_type = TTYPE(NRD_SWR); /* nread and swrite */
	else
		return -EINVAL;
	return 0;
}

#if 0
/**
 * axxia_rio_map_outb_mem -- Mapping outbound memory.
 * @mport:  RapidIO master port
 * @win:    Outbound ATMU window for this access
 *          - obtained by calling axxia_rio_req_outb_region.
 * @destid: Destination ID of transaction
 * @addr:   RapidIO space start address.
 * @res:    Mapping region phys and virt start address
 *
 * Return: 0 -- Success.
 *
 */
static int axxia_rio_map_outb_mem(struct rio_mport *mport, u32 win,
				u16 destid, u32 addr, u32 mflags,
				struct rio_map_addr *res)
{
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb;
	u32 rbar = 0, ctrl, trans_type;
	unsigned long flags;
	int rc;

	rc = __flags2rio_tr_type(mflags, &trans_type);
	if (rc < 0) {
		dev_err(priv->dev, "(%s) invalid transaction flags %x\n",
			__func__, mflags);
		return rc;
	}

	spin_lock_irqsave(&rio_io_lock, flags);

	aoutb = &priv->outb_atmu[win];
	if (unlikely(win >= RIO_OUTB_ATMU_WINDOWS ||
		     !(aoutb->in_use && aoutb->riores))) {
		spin_unlock_irqrestore(&rio_io_lock, flags);
		dev_err(priv->dev, "(%s) faulty ATMU window (%d, %d, %8.8x)\n",
			__func__, win, aoutb->in_use, (u32) aoutb->riores);
		return -EINVAL;
	}
	__rio_local_read_config_32(mport, RAB_APIO_AMAP_CTRL(win), &ctrl);

	if (TTYPE_VAL(ctrl) != trans_type) {
		ctrl &= ~TTYPE(0x3);
		ctrl |= trans_type;
	}
	if (TTYPE_VAL(ctrl)) { /* RIO address set - Not maintenance */
		rbar |= RIO_ADDR_BASE(addr);
		__rio_local_write_config_32(mport,
					    RAB_APIO_AMAP_RBAR(win),
					    rbar);
	}
	ctrl &= ~TARGID(0xffff); /* Target id clear */
	ctrl |= TARGID(destid); /* Target id set */
	ctrl |= ENABLE_AMBA; /* Enable window */
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_CTRL(win), ctrl);

	res->phys = aoutb->riores->start + RIO_ADDR_OFFSET(addr);
	res->va = aoutb->win + RIO_ADDR_OFFSET(addr);

	spin_unlock_irqrestore(&rio_io_lock, flags);

	return 0;
}
#endif
/**
 * axxia_rio_req_outb_region -- Request outbound region in the
 *                            RapidIO bus address space.
 * @mport:  RapidIO master port
 * @size:   The mapping region size.
 * @name:   Resource name
 * @flags:  Flags for mapping. 0 for using default flags.
 * @id:     Allocated outbound ATMU window id
 *
 * Return: 0 -- Success.
 *
 * This function will reserve a memory region that may
 * be used to create mappings from local iomem to rio space.
 */
static int axxia_rio_req_outb_region(struct rio_mport *mport,
				   resource_size_t size,
				   const char *name,
				   u32 mflags, u32 *id)
{
	u32 win, reg, win_size = 0, trans_type = 0, wabar = 0;
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb;
	int rc = 0;
	void __iomem *iowin;
	struct resource *riores;
	unsigned long flags;

	if (!(is_power_of_2(size))) {
		dev_err(priv->dev, "(%s) size is not power of 2 (%llu)\n",
			__func__, size);
		return -EFAULT;
	}
	rc = __flags2rio_tr_type(mflags, &trans_type);
	if (rc < 0) {
		dev_err(priv->dev, "(%s) invalid transaction flags %x\n",
			__func__, mflags);
		return rc;
	}

	spin_lock_irqsave(&rio_io_lock, flags);

	for (win = 0; win < RIO_OUTB_ATMU_WINDOWS; win++) {
		if (!(priv->outb_atmu[win].in_use))
			break;
	}

	if (win == RIO_OUTB_ATMU_WINDOWS) {
		spin_unlock_irqrestore(&rio_io_lock, flags);
		dev_err(priv->dev,
			"(%s) out of ATMU windows to use\n",
			__func__);
		return -ENOMEM;
	}
	aoutb = &priv->outb_atmu[win];
	aoutb->in_use = 1;
	aoutb->win = NULL;
	aoutb->riores = NULL;

	riores = kzalloc(sizeof(struct resource), GFP_ATOMIC);
	if (!riores) {
		aoutb->in_use = 0;
		spin_unlock_irqrestore(&rio_io_lock, flags);
		dev_err(priv->dev,
			"(%s) failed to allocate resources\n",
			__func__);
		return -ENOMEM;
	}

	spin_unlock_irqrestore(&rio_io_lock, flags);

	riores->name = name;
	riores->flags = IORESOURCE_MEM;
	if (allocate_resource(&mport->iores, riores,
			      size, mport->iores.start,
			      mport->iores.end, 0x400, NULL, NULL)) {
		/* Align on 1kB boundry */
		rc = -ENOMEM;
		goto out_err_resource;
	}

	iowin = ioremap(riores->start, size);
	if (!iowin) {
		rc = -ENOMEM;
		goto out_err_ioremap;
	}

	/* Set base address for window on PIO side */
	wabar = AXI_BASE_HIGH(riores->start);
	wabar |= AXI_BASE(riores->start);
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_ABAR(win), wabar);

	if (0 == WIN_SIZE((u32)size))
		size = 0x400u; /* make sure window size is at least 1KiB big*/


	/* Set size of window */
	win_size |= WIN_SIZE((u32)size);
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_SIZE(win), win_size);
	__rio_local_read_config_32(mport, RAB_APIO_AMAP_CTRL(win), &reg);
	reg &= ~TTYPE(0x3);
	reg |= trans_type;
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_CTRL(win), reg);

	spin_lock_irqsave(&rio_io_lock, flags);
	aoutb->win = iowin;
	aoutb->riores = riores;
	spin_unlock_irqrestore(&rio_io_lock, flags);

	*id = win;
	return 0;

out_err_ioremap:
	dev_err(priv->dev, "(%s) ioremap IO-mem failed\n",
		__func__);
	if (release_resource(riores))
		dev_err(priv->dev, "(%s) clean-up resource failed\n", __func__);
out_err_resource:
	dev_err(priv->dev, "(%s) alloc IO-mem for %s failed\n",
		__func__, name);
	kfree(riores);

	spin_lock_irqsave(&rio_io_lock, flags);
	aoutb->in_use = 0;
	spin_unlock_irqrestore(&rio_io_lock, flags);
	return rc;
}

/**
 * axxia_rio_release_outb_region -- Unreserve outbound memory region.
 * @mport: RapidIO master port
 * @win:   Allocated outbound ATMU window id
 *
 * Disables and frees the memory resource of an outbound memory region
 */
static void axxia_rio_release_outb_region(struct rio_mport *mport,
					u32 win)
{
	struct rio_priv *priv = mport->priv;
	u32 ctrl;
	unsigned long flags;

	if (unlikely(win >= RIO_OUTB_ATMU_WINDOWS))
		return;

	spin_lock_irqsave(&rio_io_lock, flags);

	__rio_local_read_config_32(mport, RAB_APIO_AMAP_CTRL(win), &ctrl);
	if (likely(priv->outb_atmu[win].in_use)) {
		struct atmu_outb *aoutb = &priv->outb_atmu[win];
		struct resource *riores = aoutb->riores;
		void __iomem *iowin = aoutb->win;

		__rio_local_write_config_32(mport,
					    RAB_APIO_AMAP_CTRL(win),
					    ctrl & ~ENABLE_AMBA);
		aoutb->riores = NULL;
		aoutb->win = NULL;

		spin_unlock_irqrestore(&rio_io_lock, flags);

		iounmap(iowin);
		if (release_resource(riores))
			dev_err(priv->dev, "(%s) clean-up resource failed\n",
				__func__);
		kfree(riores);

		spin_lock_irqsave(&rio_io_lock, flags);
		aoutb->in_use = 0;
	}

	spin_unlock_irqrestore(&rio_io_lock, flags);
}


/**
 * axxia_rio_set_mport_disc_mode - Set master port discovery/eumeration mode
 *
 * @mport: Master port
 *
 */
void axxia_rio_set_mport_disc_mode(struct rio_mport *mport)
{
	u32 result;

	if (mport->host_deviceid >= 0) {
		__rio_local_write_config_32(mport, RIO_GCCSR,
					    RIO_PORT_GEN_HOST |
					    RIO_PORT_GEN_MASTER |
					    RIO_PORT_GEN_DISCOVERED);
	} else {
		__rio_local_write_config_32(mport, RIO_GCCSR,
					    RIO_PORT_GEN_MASTER);
		__rio_local_write_config_32(mport, RIO_DID_CSR,
					    RIO_SET_DID(mport->sys_size,
					    RIO_ANY_DESTID(mport->sys_size)));
	}

#ifdef EXTRA1DEBUG
	__rio_local_read_config_32(mport, RIO_GCCSR, &result);
	EXT1P("rio[%d]: RIO_GEN_CTL_CSR set to 0x%X for main port\n",
		mport->id, result);
#endif

	__rio_local_write_config_32(mport, RIO_COMPONENT_TAG_CSR, 0xFFFF);

#ifdef	NOT_SUPPORTED
	/* Use the reset default setting of (0x00000000).  RAB does not
	 * support "Accept All=1".  We would need another ID value to use
	 * if we wanted to set the PTPN and PTE=1. */

	/* Set to receive any dist ID for serial RapidIO controller. */
	if (mport->phy_type == RIO_PHY_SERIAL)
		__rio_local_write_config_32(mport,
					    EPC_PNPTAACR(mport->port_ndx),
					    0x00000000);
#endif

#ifdef CONFIG_RAPIDIO_HOTPLUG
	if (CONFIG_RAPIDIO_SECOND_DEST_ID != DESTID_INVALID) {
		struct rio_priv *priv = mport->priv;

		result = EPC_PNADIDCSR_ADE;
		result |= EPC_PNADIDCSR_ADID_SMALL(
				CONFIG_RAPIDIO_SECOND_DEST_ID);
		__rio_local_write_config_32(mport,
					    EPC_PNADIDCSR(priv->port_ndx),
					    result);
		dev_dbg(priv->dev, "Port%dAltDevIdmCSR set to 0x%X\n",
			priv->port_ndx, CONFIG_RAPIDIO_SECOND_DEST_ID);
	}
#else
	/* Set the Alternate Destination ID to prevent "Machine Checks"
	** and aid the device enumeration / discovery process later on.
	*/
	{
		struct rio_priv *priv = mport->priv;

		result = EPC_PNADIDCSR_ADE;
		if (mport->sys_size)
			result |= EPC_PNADIDCSR_ADID_LARGE(~0);
		else
			result |= EPC_PNADIDCSR_ADID_SMALL(~0);
		__rio_local_write_config_32(mport,
					    EPC_PNADIDCSR(priv->port_ndx),
					    result);
		dev_dbg(priv->dev, "Port%dAltDevIdmCSR set to 0x%X\n",
			priv->port_ndx, result);
	}
#endif
}

/**
 * axxia_init_port_data - HW Setup of master port
 *
 * @mport: Master port
 *
 */
static void axxia_init_port_data(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ccsr, data;

#if defined(CONFIG_AXXIA_RIO_16B_ID)
	__rio_local_read_config_32(mport, RAB_SRDS_CTRL0, &data);
	__rio_local_write_config_32(mport, RAB_SRDS_CTRL0,
				    data | RAB_SRDS_CTRL0_16B_ID);
#endif
	/* Probe the master port phy type */
	__rio_local_read_config_32(mport, RIO_CCSR(priv->port_ndx), &ccsr);
	mport->phy_type = (ccsr & 1) ? RIO_PHY_SERIAL : RIO_PHY_PARALLEL;
	dev_dbg(priv->dev, "RapidIO PHY type: %s\n",
		 (mport->phy_type == RIO_PHY_PARALLEL) ? "parallel" :
		 ((mport->phy_type == RIO_PHY_SERIAL) ? "serial" :
		  "unknown"));

	__rio_local_read_config_32(mport, RIO_PEF_CAR, &data);
	mport->sys_size = (data & RIO_PEF_CTLS) >> 4;
	dev_dbg(priv->dev, "RapidIO Common Transport System size: %d\n",
		mport->sys_size ? 65536 : 256);

	__rio_local_read_config_32(mport, RIO_DEV_ID_CAR, &priv->devid);
	__rio_local_read_config_32(mport, RIO_DEV_INFO_CAR, &priv->devrev);
	{
		int i;
		static const u32 legacyids[] = {
			AXXIA_DEVID_ACP34XX,
			AXXIA_DEVID_ACP25XX,
		};
		__rio_local_read_config_32(mport, RAB_CTRL, &data);
		priv->intern_msg_desc = (data & 0x00001000) ? 1 : 0;
		for (i = 0; i < 2; i++) {
			if (priv->devid == legacyids[i])
				priv->intern_msg_desc = 1;
		}
		EXT1P("rio[%d]: RapidIO internal descriptors: %d (%x %x)\n",
			mport->id, priv->intern_msg_desc, priv->devid, data);
	}
}

/**
 * axxia_rio_info - Log Port HW setup
 *
 * @dev: RIO device
 * @ccsr: Port N Error and Command Status register
 *
 */
static void axxia_rio_info(struct device *dev, u32 ccsr)
{
	const char *str;

	if (ccsr & 1) {
		/* Serial phy */
		switch (ccsr >> 30) {
		case 0:
			str = "1";
			break;
		case 1:
			str = "4";
			break;
		default:
			str = "Unknown";
			break;
		}
		dev_dbg(dev, "Hardware port width: %s\n", str);

		switch ((ccsr >> 27) & 7) {
		case 0:
			str = "Single-lane 0";
			break;
		case 1:
			str = "Single-lane 2";
			break;
		case 2:
			str = "Four-lane";
			break;
		default:
			str = "Unknown";
			break;
		}
		dev_dbg(dev, "Training connection status: %s\n", str);
	} else {
		/* Parallel phy */
		if (!(ccsr & 0x80000000))
			dev_dbg(dev, "Output port operating in 8-bit mode\n");
		if (!(ccsr & 0x08000000))
			dev_dbg(dev, "Input port operating in 8-bit mode\n");
	}
}

/**
 * rio_start_port - Check the master port
 * @mport: Master port to be checked
 *
 * Check the type of the master port and if it is not ready try to
 * restart the connection.
 */
static int rio_start_port(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ccsr, escsr;

	/* Probe the master port phy type */
	__rio_local_read_config_32(mport, RIO_CCSR(priv->port_ndx), &ccsr);
	__rio_local_read_config_32(mport, RIO_ESCSR(priv->port_ndx), &escsr);

	if (escsr & RIO_ESCSR_PU) {

		dev_err(priv->dev,
			"Port is not ready/restart ordered. Try to restart connection...\n");

		/* Disable ports */
		ccsr |= RIO_CCSR_PD;
		__rio_local_write_config_32(mport, RIO_CCSR(priv->port_ndx),
						ccsr);
		switch (mport->phy_type) {
		case RIO_PHY_SERIAL:
			/* Set 1x lane */
			ccsr &= ~RIO_CCSR_PWO;
			ccsr |= RIO_CCSR_FORCE_LANE0;
			__rio_local_write_config_32(mport,
						RIO_CCSR(priv->port_ndx), ccsr);
			break;
		case RIO_PHY_PARALLEL:
			break;
		}

		/* Enable ports */
		ccsr &= ~RIO_CCSR_PD;
		__rio_local_write_config_32(mport, RIO_CCSR(priv->port_ndx),
					ccsr);
		msleep(100);
		__rio_local_read_config_32(mport, RIO_ESCSR(priv->port_ndx),
					&escsr);
		axxia_rio_info(priv->dev, ccsr);
		if (escsr & RIO_ESCSR_PU) {
			dev_dbg(priv->dev, "Port restart failed.\n");
			return -ENOLINK;
		} else {
			dev_dbg(priv->dev, "Port restart success!\n");
			return 0;
		}
	}

#ifdef EXTRA1DEBUG
	{
		u32 hdlcsr, rabver;

		__rio_local_read_config_32(mport, RIO_HOST_DID_LOCK_CSR,
					&hdlcsr);
		__rio_local_read_config_32(mport, RAB_VER, &rabver);

		pr_info("rio[%d]: AR[%d] DIDCAR[%x]=%08x RAB_VER[%x]=%08x\n",
			mport->id,
			__LINE__,
			RIO_DEV_ID_CAR, priv->devid,
			RAB_VER, rabver);
		pr_info("rio[%d]: AR[%d] [%x]=%08x [%x]=%08x [%x]=%08x\n",
			mport->id,
			__LINE__,
			RIO_CCSR(priv->port_ndx), ccsr,
			RIO_ESCSR(priv->port_ndx), escsr,
			RIO_HOST_DID_LOCK_CSR, hdlcsr);
	}
#endif /* defined(EXTRA1DEBUG) */

	dev_dbg(priv->dev, "Port is Ready\n");
	return 0;
}

/**
 * rio_rab_ctrl_setup - Bridge Control HW setup
 *
 * @mport: Master port
 *
 * Response Prio = request prio +1. 2) No AXI byte swap
 * Internal (RIO Mem) DME desc access
 * Priority based MSG arbitration
 * RIO & AMBA PIO Enable
 */
static void rio_rab_ctrl_setup(struct rio_mport *mport)
{
	u32 rab_ctrl;

	__rio_local_write_config_32(mport, AXI_TIMEOUT, 0x00001000);

#ifdef USE_DME_TIMEOUT
	__rio_local_write_config_32(mport, DME_TIMEOUT, 0xC0080000);
#else
	__rio_local_write_config_32(mport, DME_TIMEOUT, 0x00000000);
#endif

	rab_ctrl = 0;
	rab_ctrl |= (1 << 12);
	rab_ctrl |= (2 << 6);
	rab_ctrl |= 3;
	__rio_local_write_config_32(mport, RAB_CTRL, rab_ctrl);
}

/**
 * rio_rab_pio_enable - Setup Peripheral Bus bridge,
 *                      RapidIO <-> Peripheral bus, HW.
 *
 * @mport: Master port
 *
 * Enable AXI PIO + outbound nwrite/nread/maintenance
 * Enable RIO PIO (enable rx maint port-write packet)
 */
static void rio_rab_pio_enable(struct rio_mport *mport)
{
	__rio_local_write_config_32(mport, RAB_APIO_CTRL,
				    RAB_APIO_MAINT_MAP_EN |
				    RAB_APIO_MEM_MAP_EN |
				    RAB_APIO_PIO_EN);
	__rio_local_write_config_32(mport, RAB_RPIO_CTRL, RAB_RPIO_PIO_EN);
}

/**
 * rio_static_win_init -- Setup static ATMU window for maintenance
 *                        access and enable doorbells
 *
 * @mport: Master port
 *
 * Returns:
 * 0        - At success
 * -EFAULT  - Requested outbound region can not be claimed
 */
int axxia_rio_static_win_init(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ctrl;

	/* Enable inbound doorbell */
	__rio_local_write_config_32(mport, RAB_IB_DB_CSR, IB_DB_CSR_EN);

	/* Configure maintenance transaction window */
	if ((axxia_rio_req_outb_region(mport, CONFIG_RIO_MAINT_WIN_SIZE,
				     "rio_maint_win", RIO_MAINT_WRITE,
				     &priv->maint_win_id)) < 0)
		goto err;

	__rio_local_read_config_32(mport,
				   RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				   &ctrl);
	/* Enable window */
	ctrl |= ENABLE_AMBA;
	__rio_local_write_config_32(mport,
				    RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				    ctrl);

	return 0;
err:
	return -EFAULT;
}

/**
 * axxia_rio_static_win_release -- Release static ATMU maintenance window
 *                                 Disable doorbells
 *
 * @mport: Master port
 *
 */
void axxia_rio_static_win_release(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ibdb;

	/* Disable inbound doorbell */
	__rio_local_read_config_32(mport, RAB_IB_DB_CSR, &ibdb);
	ibdb &= ~IB_DB_CSR_EN;
	__rio_local_write_config_32(mport, RAB_IB_DB_CSR, ibdb);

	/* Release maintenance transaction window */
	axxia_rio_release_outb_region(mport, priv->maint_win_id);
}

/**
 * rio_parse_dtb - Parse RapidIO platform entry
 *
 * @dev: RIO platform device
 * @ndx: Which instance are we?
 * @law_start: Local Access Window start address from DTB
 * @law_size: Local Access Window size from DTB
 * @regs: RapidIO registers from DTB
 * @ob_num_dmes: Number of outbound DMEs available
 * @outb_dmes: RapidIO outbound DMEs array available;
 *                [0] for MSeg, [1] for SSeg
 * @ib_num_dmes: Number of inbound DMEs available
 * @inb_dmes: RapidIO inbound DMEs array available; 2 elements
 * @irq: RapidIO IRQ mapping from DTB
 *
 * Returns:
 * -EFAULT          At failure
 * 0                Success
 */
static int rio_parse_dtb(
	struct platform_device *dev,
	int *ndx,
	u64 *law_start,
	u64 *law_size,
	struct resource *regs,
	int *ob_num_dmes,
	int *outb_dmes,
	int *ib_num_dmes,
	int *inb_dmes,
	int *irq,
	struct event_regs *linkdown_reset)
{
	const u32 *dt_range, *cell;
	int rlen, rc;
	int paw, aw, sw;

	if (!dev->dev.of_node) {
		dev_err(&dev->dev, "Device OF-Node is NULL");
		return -EFAULT;
	}

	if (!of_device_is_available(dev->dev.of_node)) {
		EXT1P("rio[%d]: AR[%d] status = not available\n", 99, __LINE__);
		return -ENODEV;
	} else {
		EXT1P("rio[%d]: AR[%d] status = available\n", 99, __LINE__);
	}

	if (of_property_read_u32(dev->dev.of_node, "index", &rlen))
		return -ENODEV;
	*ndx = rlen;

	rc = of_address_to_resource(dev->dev.of_node, 0, regs);
	if (rc) {
		dev_err(&dev->dev, "Can't get %s property 'reg'\n",
			dev->dev.of_node->full_name);
		return -EFAULT;
	}
	dev_dbg(&dev->dev,
		"Of-device full name %s\n",
		 dev->dev.of_node->full_name);
	dev_dbg(&dev->dev, "Regs: %pR\n", regs);

	dt_range = of_get_property(dev->dev.of_node, "ranges", &rlen);

	if (!dt_range) {
		dev_err(&dev->dev, "Can't get %s property 'ranges'\n",
			dev->dev.of_node->full_name);
		return -EFAULT;
	}

	/* Get node address wide */
	cell = of_get_property(dev->dev.of_node, "#address-cells", NULL);
	if (cell)
		aw = *cell;
	else
		aw = of_n_addr_cells(dev->dev.of_node);
	if (aw > 3)			/* Anomaly in A15 build+parse */
		aw = 2;
	/* Get node size wide */
	cell = of_get_property(dev->dev.of_node, "#size-cells", NULL);
	if (cell)
		sw = *cell;
	else
		sw = of_n_size_cells(dev->dev.of_node);
	if (sw > 3)			/* Anomaly in A15 build+parse */
		sw = 2;
	/* Get parent address wide wide */
	paw = of_n_addr_cells(dev->dev.of_node);

	*law_start = of_read_number(dt_range + aw, paw);
	*law_size = of_read_number(dt_range + aw + paw, sw);

	dev_dbg(&dev->dev, "LAW: [mem 0x%016llx -- 0x%016llx]\n",
		*law_start, *law_start + *law_size - 1);

	outb_dmes[0] = outb_dmes[1] = 0;
	cell = of_get_property(dev->dev.of_node, "outb-dmes", &rlen);
	if (!cell) {
		ob_num_dmes[0] = 2;
		ob_num_dmes[1] = 1;
		outb_dmes[0] = 0x00000003;
		outb_dmes[1] = 0x00000001;
	} else {
		if (rlen < (4 * sizeof(int))) {
			dev_err(&dev->dev, "Invalid %s property 'outb-dmes'\n",
				dev->dev.of_node->full_name);
			return -EFAULT;
		}
		ob_num_dmes[0] = of_read_number(cell, 1);
		outb_dmes[0] = of_read_number(cell + 1, 1);
		ob_num_dmes[1] = of_read_number(cell + 2, 1);
		outb_dmes[1] = of_read_number(cell + 3, 1);
		if (((ob_num_dmes[0])+(ob_num_dmes[1])) > DME_MAX_OB_ENGINES) {
			dev_err(&dev->dev, "Invalid %s property 'outb-dmes'\n",
				dev->dev.of_node->full_name);
			return -EFAULT;
		}
	}
	dev_dbg(&dev->dev, "outb-dmes: MSeg[%d]=%08x SSeg[%d]=%08x\n",
		ob_num_dmes[0], outb_dmes[0], ob_num_dmes[1], outb_dmes[1]);

	inb_dmes[0] = inb_dmes[1] = 0;
	cell = of_get_property(dev->dev.of_node, "inb-dmes", &rlen);
	if (!cell) {
		ib_num_dmes[0] = DME_MAX_IB_ENGINES;
		ib_num_dmes[1] = 0;
		inb_dmes[0] = 0xffffffff;
		inb_dmes[1] = 0x00000000;
	} else {
		if (rlen < (4 * sizeof(int))) {
			dev_err(&dev->dev, "Invalid %s property 'inb-dmes'\n",
				dev->dev.of_node->full_name);
			return -EFAULT;
		}
		ib_num_dmes[0] = of_read_number(cell, 1);
		inb_dmes[0] = of_read_number(cell + 1, 1);
		ib_num_dmes[1] = of_read_number(cell + 2, 1);
		inb_dmes[1] = of_read_number(cell + 3, 1);
		if (((ib_num_dmes[0])+(ib_num_dmes[1])) > DME_MAX_IB_ENGINES) {
			dev_err(&dev->dev, "Invalid %s property 'inb-dmes'\n",
				dev->dev.of_node->full_name);
			return -EFAULT;
		}
	}
	dev_dbg(&dev->dev, "inb-dmes: MSeg[%d]=%08x SSeg[%d]=%08x\n",
		ib_num_dmes[0], inb_dmes[0], ib_num_dmes[1], inb_dmes[1]);

	*irq = irq_of_parse_and_map(dev->dev.of_node, 0);
	dev_dbg(&dev->dev, "irq: %d\n", *irq);

	memset(linkdown_reset, 0, sizeof(struct event_regs));
	dt_range = of_get_property(dev->dev.of_node, "linkdown-reset", &rlen);
	if (dt_range) {
		if (rlen < (6 * sizeof(int))) {
			dev_err(&dev->dev,
				"Invalid %s property 'linkdown-reset'\n",
				dev->dev.of_node->full_name);
			return -EFAULT;
		} else {
			linkdown_reset->phy_reset_start =
				of_read_number(dt_range + aw, paw);
			linkdown_reset->phy_reset_size =
				of_read_number(dt_range + aw + paw, sw);
			linkdown_reset->reg_addr =
				of_read_number(dt_range + 0, 1);
			linkdown_reset->reg_mask =
				of_read_number(dt_range + 1, 1);
			linkdown_reset->in_use = 1;
			EXT1P("rio: LDR st=%llx sz=%llx RA=%x MSK=%x iu=%d\n",
				linkdown_reset->phy_reset_start,
				linkdown_reset->phy_reset_size,
				linkdown_reset->reg_addr,
				linkdown_reset->reg_mask,
				linkdown_reset->in_use);
		}
	}

	return 0;
}

/**
 * rio_ops_setup - Alloc and initiate the RIO ops struct
 *
 * Returns:
 * ERR_PTR(-ENOMEM)      At failure
 * struct rio_ops *ptr   to initialized ops data at Success
 */
static struct rio_ops *rio_ops_setup(void)
{
	struct rio_ops *ops = kzalloc(sizeof(*ops), GFP_KERNEL);

	if (!ops)
		return ERR_PTR(-ENOMEM);

	ops->lcread = axxia_rio_local_config_read;
	ops->lcwrite = axxia_rio_local_config_write;
	ops->cread = axxia_rio_config_read;
	ops->cwrite = axxia_rio_config_write;
	ops->dsend = axxia_rio_doorbell_send;
	ops->pwenable = axxia_rio_pw_enable;
	ops->open_outb_mbox = axxia_open_outb_mbox;
	ops->open_inb_mbox = axxia_open_inb_mbox;
	ops->close_outb_mbox = axxia_close_outb_mbox;
	ops->close_inb_mbox = axxia_close_inb_mbox;
	ops->add_outb_message = axxia_ml_add_outb_message;
	ops->add_inb_buffer = axxia_add_inb_buffer;
	ops->get_inb_message = axxia_ml_get_inb_message;
#ifdef CONFIG_RAPIDIO_HOTPLUG
	ops->hotswap = axxia_rio_hotswap;
	ops->port_notify_cb = axxia_rio_port_notify_cb;
	ops->port_op_state = axxia_rio_port_op_state;
#endif
	return ops;
}

/**
 * rio_mport_dtb_setup - Alloc and initialize the master port data
 *                       structure with data retrieved from DTB
 *
 * @dev: RIO platform device
 * @law_start: Local Access Window start address from DTB
 * @law_size: Local Access Window size from DTB
 * @ops: RIO ops data structure
 *
 * Init mport data structure
 * Request RIO iomem resources
 * Register doorbell and mbox resources with generic RIO driver

 * Returns:
 * -ENOMEM                 At failure
 * struct rio_mport *ptr   to initialized mport data at Success
 */
static int rio_mport_dtb_setup(struct platform_device *dev,
			       int port_ndx,
			       u64 law_start,
			       u64 law_size,
			       struct rio_ops *ops,
			       struct rio_mport **ptr)
{
	int rc = 0;
	struct rio_mport *mport = kzalloc(sizeof(*mport), GFP_KERNEL);

	(*ptr) = NULL;

	if (!mport)
		return -ENOMEM;

	mport->index = port_ndx;

	INIT_LIST_HEAD(&mport->dbells);
	mport->iores.start = law_start;
	mport->iores.end = law_start + law_size - 1;
	mport->iores.flags = IORESOURCE_MEM;
	mport->iores.name = "rio_io_win";
	mport->iores.parent = NULL;
	mport->iores.child = NULL;
	mport->iores.sibling = NULL;

	if (request_resource(&iomem_resource, &mport->iores) < 0) {
		dev_err(&dev->dev,
			"RIO: Error requesting master port region 0x%016llx-0x%016llx\n",
			(u64)mport->iores.start, (u64)mport->iores.end);
		kfree(mport);
		return -ENOMEM;
	}
	rio_init_dbell_res(&mport->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0,
			RIO_MAX_RX_MBOX);
	rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0,
			RIO_MAX_TX_MBOX);
	sprintf(mport->name, "RIO%d mport", mport->id);

	mport->ops = ops;
	mport->phys_efptr = 0x100; /* define maybe */

	(*ptr) = mport;
	return rc;
}

/**
 * rio_priv_dtb_setup - Alloc and initialize the master port private data
 *                      structure with data retrieved from DTB
 *
 * @dev: RIO platform device
 * @regs: RapidIO registers from DTB
 * @mport: master port
 * @ndx: Instance Id of the controller description
 * @port_ndx: Port Id of the controller
 * @numObNumDmes: override num outbound DMEs available
 * @outb_dmes: RapidIO outbound DMEs array available; [0] for MSeg, [1] for SSeg
 * @numIbNumDmes: override num inbound DMEs available
 * @inb_dmes: RapidIO inbound DMEs array available; 2 elements
 * @irq: IRQ number
 *
 * Init master port private data structure
 *
 * Returns:
 * ERR_PTR(-ENOMEM)        At failure
 * struct rio_priv *ptr    to initialized priv data at Success
 */

static struct rio_priv *rio_priv_dtb_setup(
	struct platform_device *dev,
	struct resource *regs,
	struct rio_mport *mport,
	int ndx,
	int port_ndx,
	int *num_outb_dmes,
	int *outb_dmes,
	int *num_inb_dmes,
	int *inb_dmes,
	int irq,
	struct event_regs *linkdown_reset)
{
	struct rio_priv *priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	int i, rc;

	if (!priv)
		return ERR_PTR(-ENOMEM);

	/* master port driver handle (bidirectional reference supported) */
	mport->priv = priv;
	priv->cookie = LSI_AXXIA_RIO_COOKIE;
	priv->mport = mport;
	priv->ndx = ndx;
	priv->port_ndx = port_ndx;
	mutex_init(&priv->api_lock);
	/* Max descriptors */
	priv->desc_max_entries = RIO_MSG_MAX_ENTRIES;

	if (priv->intern_msg_desc) {
		/* Support for alloc_message_engine() */
		struct resource *dres = &priv->acpres[ACP_HW_DESC_RESOURCE];

		memset(dres, 0, sizeof(struct resource));
			/* 'virtual' mapping of descriptors */
		dres->start = 0;
		dres->end = priv->desc_max_entries - 1;
		dres->flags = ACP_RESOURCE_HW_DESC;
		dres->name = "rio_desc_win";
		dres->parent = NULL;
		dres->child = NULL;
		dres->sibling = NULL;

		if (request_resource(&iomem_resource, dres) < 0) {
			dev_err(&dev->dev,
				"RIO: Error requesting descriptor region 0x%016llx-0x%016llx\n",
				(u64)dres->start, (u64)dres->end);
			rc = -ENOMEM;
			goto err_fixed;
		}
	}

	/* Defined DMEs */
	if (outb_dmes) {
		priv->num_outb_dmes[0] = num_outb_dmes[0];
		priv->num_outb_dmes[1] = num_outb_dmes[1];
		priv->outb_dmes[0] = outb_dmes[0];
		priv->outb_dmes[1] = outb_dmes[1];
	}
	if (inb_dmes) {
		priv->num_inb_dmes[0] = num_inb_dmes[0];
		priv->num_inb_dmes[1] = num_inb_dmes[1];
		priv->inb_dmes[0] = inb_dmes[0];
		priv->inb_dmes[1] = inb_dmes[1];
	}

	/* Interrupt handling */
	priv->irq_line = irq;
	axxia_rio_port_irq_init(mport);

	/* Dev ptr for debug printouts */
	priv->dev = &dev->dev;

	/* Init ATMU data structures */
	for (i = 0; i < RIO_OUTB_ATMU_WINDOWS; i++) {
		priv->outb_atmu[i].in_use = 0;
		priv->outb_atmu[i].riores = NULL;
	}

	/* Setup local access */
	priv->regs_win_fixed = ioremap(regs->start, SRIO_CONF_SPACE_SIZE_FIXED);
	if (!priv->regs_win_fixed) {
		rc = -ENOMEM;
		goto err_fixed;
	}
	priv->regs_win_paged = ioremap(regs->start + SRIO_CONF_SPACE_SIZE_FIXED,
					SRIO_CONF_SPACE_SIZE_PAGED);
	if (!priv->regs_win_paged) {
		rc = -ENOMEM;
		goto err_paged;
	}
	if (linkdown_reset && linkdown_reset->in_use) {
		memcpy(&priv->linkdown_reset, linkdown_reset,
			sizeof(struct event_regs));
		priv->linkdown_reset.win =
			ioremap(linkdown_reset->phy_reset_start,
				linkdown_reset->phy_reset_size);
		if (!priv->linkdown_reset.win) {
			rc = -ENOMEM;
			goto err_linkdown;
		}
		EXT1P("rio[%d]: LDR win=%p\n", mport->id,
			priv->linkdown_reset.win);
	}

	return priv;

err_linkdown:
	if (priv->linkdown_reset.win)
		iounmap(priv->linkdown_reset.win);
	iounmap(priv->regs_win_paged);
err_paged:
	iounmap(priv->regs_win_fixed);
err_fixed:
	kfree(priv);
	return ERR_PTR(rc);
}

/**
 * axxia_rio_start_port - Start master port
 *
 * @mport: Master port
 *
 * Check the type of the master port and if it is not ready try to
 * restart the connection.
 * In hotplug mode we don't really care about connection state
 * elsewise we give up if the port is not up.
 *
 * Setup HW for basic memap access support:
 * enable AXI bridge, maintenance window, doorbells, etc..
 */
int axxia_rio_start_port(struct rio_mport *mport)
{
	int rc;
	struct rio_priv *priv = mport->priv;

	/*
	 * Set port line request ack timout 1.5 - 3 s
	 * Set port response timeout 1.5 - 3 s
	 */
	__rio_local_write_config_32(mport, RIO_PLTOCCSR, 0x7fffff);
	__rio_local_write_config_32(mport, RIO_PRTOCCSR, 0x7fffff);

	/* Check port training state:
	 */

	rc = rio_start_port(mport);
	if (rc < 0) {
#ifdef CONFIG_RAPIDIO_HOTPLUG
		dev_warn(priv->dev, "Link is down - will continue anyway\n");
#else
		dev_err(priv->dev, "Link is down - SRIO Init failed\n");
		return rc;
#endif
	}

	/* Enable memory mapped access
	 */
	rio_rab_ctrl_setup(mport);

	rio_rab_pio_enable(mport);

	/* Miscellaneous
	 */
	__rio_local_write_config_32(mport, RAB_OB_DME_TID_MASK,
				    OB_DME_TID_MASK);


	/* Setup maintenance window
	 * Enable doorbells
	 */
	rc = axxia_rio_static_win_init(mport);

	return rc;
}

/**
 * axxia_rio_setup - Setup AXXIA RapidIO interface
 * @dev: platform_device pointer
 *
 * Initializes AXXIA RapidIO hardware interface, configures
 * master port with system-specific info, and registers the
 * master port with the RapidIO subsystem.
 *
 * Init sequence is divided into two phases
 * 1:
 *    All one-time initialization: e.g. driver software structures,
 *    work queues, tasklets, sync resources etc. are allocated and
 *    and initialized. At this stage No HW access is possible, to avoid
 *    race conditions, all HW accesses to local configuration space must
 *    be handled through the generic RIO driver access functions and
 *    these may not be used prior to init of master port data structure.
 * 2:
 *    Setup and try to start RapidIO master port controller HW
 *    If the driver is built with hotplug support, the setup routine
 *    does not require that the link is up to complete successfully,
 *    the port may be restarted at any point later in time. Without
 *    hotplug the setup function will fail if link tranining sequence
 *    doesn't complete successfully.
 *
 * Returns:
 * <0           Failure
 * 0            Success
 */
static int axxia_rio_setup(struct platform_device *dev)
{
	int rc = -EFAULT;
	struct rio_ops *ops;
	struct rio_mport *mport;
	struct rio_priv *priv;
	struct resource regs;
	u64 law_start = 0, law_size = 0;
	int ndx = 0, irq = 0, port_ndx = 0;
	int numObDmes[2] = { 0, }, outb_dmes[2] = { 0, };
	int numIbDmes[2] = { 0, }, inb_dmes[2] = { 0, };
	struct event_regs linkdown_reset = { 0, };
#ifdef CONFIG_AXXIA_RIO_DS
	struct axxia_rio_ds_dtb_info ds_dtb_info; /* data_streaming */
#endif

	/* Get address boundaries, etc. from DTB */
	if (rio_parse_dtb(dev, &ndx, &law_start, &law_size, &regs,
			  &numObDmes[0], &outb_dmes[0],
			  &numIbDmes[0], &inb_dmes[0],
			  &irq, &linkdown_reset))
		return -EFAULT;

	rc = axxia_rapidio_board_init(dev, ndx, &port_ndx);
	if (rc != 0)
		return rc;
#ifdef CONFIG_AXXIA_RIO_DS
	rc = axxia_parse_dtb_ds(dev, &ds_dtb_info);
	if (rc != 0)
		return rc;
#endif
	/* Alloc and Initialize driver SW data structure */
	ops = rio_ops_setup();
	if (IS_ERR(ops)) {
		rc = PTR_ERR(ops);
		goto err_ops;
	}
	rc = rio_mport_dtb_setup(dev, port_ndx, law_start, law_size,
				 ops, &mport);
	if (rc != 0)
		goto err_port;
	priv = rio_priv_dtb_setup(dev, &regs, mport, ndx, port_ndx,
				  &numObDmes[0], &outb_dmes[0],
				  &numIbDmes[0], &inb_dmes[0],
				  irq, &linkdown_reset);
	if (IS_ERR(priv)) {
		rc = PTR_ERR(priv);
		goto err_priv;
	}

	/* !!! HW access to local config space starts here !!! */

	/* Get and set master port data
	 */
	axxia_init_port_data(mport);

	/* Start port and enable basic memmap access
	 */
	rc = axxia_rio_start_port(mport);
	if (rc < 0)
		goto err_maint;

	/* Hookup IRQ handlers
	 */
	if (axxia_rio_port_irq_enable(mport))
		goto err_irq;

	/* Hookup SYSFS support
	 */
	dev_set_drvdata(&dev->dev, mport);
#ifdef CONFIG_AXXIA_RIO_STAT
	axxia_rio_init_sysfs(dev);
#endif
#ifdef CONFIG_AXXIA_RIO_DS
	/* Data_streaming */
	if (ds_dtb_info.ds_enabled == 1) {
		rc = axxia_cfg_ds(mport, &ds_dtb_info);
		if (rc)
			goto err_mport;
		axxia_rio_ds_port_irq_init(mport);
	}
#endif
	/* Register port with core driver
	 */
	if (rio_register_mport(mport)) {
		dev_err(&dev->dev, "register mport failed\n");
		goto err_mport;
	}

	/* Correct the host device id if needed
	 */
	{
		u16 id = rio_local_get_device_id(mport);

		EXT1P("rio[%d]: AR[%d] devid=%d hdid=%d\n",
			mport->id, __LINE__,
			mport->host_deviceid, rio_local_get_device_id(mport));
		if (mport->host_deviceid < 0) {
			if ((id != 0xFF) && (mport->sys_size == 0))
				mport->host_deviceid = id;
			else if ((id != 0xFFFF) && (mport->sys_size != 0))
				mport->host_deviceid = id;
		}
	}

	/* Any changes needed based on device id / revision ?
	*/
	switch (priv->devid) {
	case AXXIA_DEVID_ACP25XX:
		priv->outb_dmes[1] = 0x00000000;
		break;
	case AXXIA_DEVID_AXM55XX:
		switch (priv->devrev) {
		case AXXIA_DEVREV_AXM55XX_V1_0:
			priv->outb_dmes[1] = 0x00000000;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	/* And set the discovery mode for this port before we go
	 */
	axxia_rio_set_mport_disc_mode(mport);

	EXT1P("rio[%p:%d]: priv=%p\n", mport, mport->id,
		priv);
	return 0;

err_mport:
	axxia_rio_port_irq_disable(mport);
#ifdef CONFIG_AXXIA_RIO_STAT
	axxia_rio_release_sysfs(dev);
#endif
err_irq:
	axxia_rio_static_win_release(mport);
err_maint:
	if (priv->linkdown_reset.win)
		iounmap(priv->linkdown_reset.win);
	iounmap(priv->regs_win_fixed);
	iounmap(priv->regs_win_paged);
	kfree(priv);
err_priv:
	kfree(mport);
err_port:
	kfree(ops);
err_ops:
	irq_dispose_mapping(irq);
	return rc;
}

/*
  The probe function for RapidIO peer-to-peer network.
*/
static int axxia_of_rio_rpn_probe(struct platform_device *dev)
{
	EXT1P(KERN_INFO "Setting up RapidIO peer-to-peer network %s\n",
	       dev->dev.of_node->full_name);

	return axxia_rio_setup(dev);
};

static const struct of_device_id axxia_of_rio_rpn_ids[] = {
	{ .compatible = "axxia, rapidio-delta", },
	{ .compatible = "acp, rapidio-delta", },
	{},
};

static struct platform_driver axxia_of_rio_rpn_driver = {
	.driver = {
		.name = "axxia-of-rio",
		.owner = THIS_MODULE,
		.of_match_table = axxia_of_rio_rpn_ids,
	},
	.probe = axxia_of_rio_rpn_probe,
};

static __init int axxia_of_rio_rpn_init(void)
{
	EXT1P(KERN_INFO "Register RapidIO platform driver\n");
	return platform_driver_register(&axxia_of_rio_rpn_driver);
}

subsys_initcall_sync(axxia_of_rio_rpn_init);
