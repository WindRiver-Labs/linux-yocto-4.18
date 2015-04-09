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

#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include "axxia-rio.h"
#define OB_DME_ENTRIES		(CONFIG_OB_DME_ENTRY_SIZE)
#define LINK_DOWN_TIMEOUT	(0x4BF0)

unsigned int axxia_dme_tmr_mode[2] = { AXXIA_IBDME_INTERRUPT_MODE,
					AXXIA_IBDME_TIMER_MODE };
static int axxia_timer_mode_setup(char *str)
{
	unsigned int tmr_mode[3];
	int i;
	(void)get_options(str, ARRAY_SIZE(tmr_mode), tmr_mode);
	for (i = 0; i < tmr_mode[0]; i++) {
		if (tmr_mode[i+1] > 1)
			pr_debug("Invalid parameter value for Timer Mode\n");
		else
			axxia_dme_tmr_mode[i] = AXXIA_IBDME_TIMER_MODE;
	}
	return 1;
}
__setup("axm_srio_tmr_mode=", axxia_timer_mode_setup);

static int axxia_int_mode_setup(char *str)
{
	unsigned int int_mode[3];
	int i;
	(void)get_options(str, ARRAY_SIZE(int_mode), int_mode);
	for (i = 0; i < int_mode[0]; i++) {
		if (int_mode[i+1] > 1)
			pr_debug("Invalid param value for Interrupt Mode\n");
		else
			axxia_dme_tmr_mode[i] = AXXIA_IBDME_INTERRUPT_MODE;
	}
	return 1;
}
__setup("axm_srio_int_mode=", axxia_int_mode_setup);

#define AXXIA_HRTIMER_DELAY	(200 * 1000UL)
unsigned int axxia_hrtimer_delay = AXXIA_HRTIMER_DELAY;
static int __init axxia_hrtimer_setup(char *str)
{
	get_option(&str, &axxia_hrtimer_delay);
	return 1;
}
__setup("axm_srio_tmr_period=", axxia_hrtimer_setup);

/**************************sRIO SERDES *****************************/
u32 srio_serdes_write32(struct rio_priv *priv, u32 addr, u32 val)
{
	void __iomem *regaddr;
	u32 regval = 0;

	regaddr = (priv->linkdown_reset.win) +
			APB2SER_SRIO_PHY0_CFG_OFFSET;
	iowrite32(val, (regaddr + SERDES_CMD0_OFFSET));
	regval = ((1<<SERDES_CMD1_VALID_SHIFT) |
			(0x1 << SERDES_CMD1_HWRITE_SHIFT) |
			(0x01<<SERDES_CMD1_TSHIFT_SHIFT) |
			(0x2<<SERDES_CMD1_HSZIE_SHIFT) |
			(0x2 << SERDES_CMD1_HTRANS_SHIFT) |
			(addr & SERDES_CMD1_HADDR_MASK));
	iowrite32(regval, (regaddr + SERDES_CMD1_OFFSET));

	regval = 0xffffffff;
	while (1) {
		regval = ioread32((regaddr + SERDES_CMD1_OFFSET));
		if (!(regval & (1 << SERDES_CMD1_VALID_SHIFT)))
			break;
	}

	regval = ioread32((regaddr + SERDES_READDATA1_OFFSET));
	if (regval & SERDES_READDATA1_HRESP_MASK) {
		dev_err(priv->dev, "SerDes write Failed... Returning 0\n");
		return 0;
	} else
		return 1;
}

u32 srio_serdes_read32(struct rio_priv *priv, u32 addr)
{
	void __iomem *regaddr;
	u32 regval = 0;

	regaddr = (priv->linkdown_reset.win) +
			APB2SER_SRIO_PHY0_CFG_OFFSET;
	regval = ((1<<SERDES_CMD1_VALID_SHIFT) |
			(0x01<<SERDES_CMD1_TSHIFT_SHIFT) |
			(0x2<<SERDES_CMD1_HSZIE_SHIFT) |
			(0x2 << SERDES_CMD1_HTRANS_SHIFT) |
			(addr & SERDES_CMD1_HADDR_MASK));

	iowrite32(regval, (regaddr + SERDES_CMD1_OFFSET));
	regval = 0xffffffff;
	while (1) {
		regval = ioread32((regaddr + SERDES_CMD1_OFFSET));
		if ((regval & (1 << SERDES_CMD1_VALID_SHIFT)) == 0x0)
			break;
	}
	regval = ioread32((regaddr + SERDES_READDATA1_OFFSET));
	if (regval & SERDES_READDATA1_HRESP_MASK) {
		dev_err(priv->dev, "SerDes Read Failed... Returning 0\n");
		return 0;
	}
	regval = ioread32((regaddr + SERDES_READDATA0_OFFSET));
	return regval;
}

/**************************sRIO SERDES Ends ***************************/
static void  ib_dme_irq_handler(struct rio_irq_handler *h/*, u32 state*/);
/****************************************************************************
**
** Implementation Note:
**
** The Message DME registers lie within the fixed page block in the RAB SRIO
** Configuration memory.  Thus, all or almost all of its register accesses
** do not require use of the RAB memory paging register.  On the other hand,
** the Message descriptor registers for the ACP34xx platform do lie outside
** of the fixed page block.  For safety, we will direct all of the accesses
** to the Message descriptor registers (on the ACP34xx platform and the like),
** through the RIO mport's lcread and lcwrite interfaces which use a software
** spin lock to control access.
**
*****************************************************************************/

static inline void __dme_dw_dbg(struct device *dev, struct rio_msg_dme *dme,
			u32 iout, u32 dw0, u32 dw1)
{
	int did, mb, let;
	char *io;
	char *id;

	if (dw0 & DME_DESC_DW0_ERROR_MASK) {
		did = DME_DESC_DW0_GET_DST_ID(dw0);
		let = DME_DESC_DW1_GET_LETTER(dw1);
		mb = DME_DESC_DW1_GET_MBOX(dw1);
		if (iout) {
			io = "OB";
			id = "DID";
		} else {
			io = "IB";
			id = "SID";
		}
#if defined(CONFIG_AXXIA_RIO_STAT)
		dme->desc_error_count++;
#endif
		if (dw0 & DME_DESC_DW0_RIO_ERR) {
			dev_err(dev,
			"%s RIO ERR: %s = %x,Type:11,mbox=%d,letter=%d\n",
			 io, id, did, mb, let);
#if defined(CONFIG_AXXIA_RIO_STAT)
			dme->desc_rio_err_count++;
#endif
		}
		if (dw0 & DME_DESC_DW0_AXI_ERR) {
			dev_err(dev,
			"%s AXI ERR: %s = %x,Type:11,mbox=%d,letter=%d\n",
			 io, id, did, mb, let);
#if defined(CONFIG_AXXIA_RIO_STAT)
			dme->desc_axi_err_count++;
#endif
		}
		if (dw0 & DME_DESC_DW0_TIMEOUT_ERR) {
			dev_err(dev,
			"%s TIMEOUT ERR: %s = %x,Type:11,mbox=%d,letter=%d\n",
			 io, id, did, mb, let);
#if defined(CONFIG_AXXIA_RIO_STAT)
			dme->desc_tmo_err_count++;
#endif
		}
	}
#if defined(CONFIG_AXXIA_RIO_STAT)
	dme->desc_done_count++;
#endif
}

#if defined(CONFIG_AXXIA_RIO_STAT)
static void reset_state_counters(struct rio_priv *priv)
{
	priv->rpio_compl_count = 0;
	priv->rpio_failed_count = 0;
	priv->apio_compl_count = 0;
	priv->apio_failed_count = 0;
	priv->rio_pw_count = 0;
	priv->rio_pw_msg_count = 0;
}
#endif /* defined(CONFIG_AXXIA_RIO_STAT) */

/**
 * thrd_irq_handler - Threaded interrupt handler
 * @irq: Linux interrupt number
 * @data: Pointer to interrupt-specific data
 *
 */
static irqreturn_t thrd_irq_handler(int irq, void *data)
{
	struct rio_irq_handler *h = data;
	struct rio_priv *priv = h->data;

	/**
	 * Invoke handler callback
	 */
	h->thrd_irq_fn(h);
	axxia_local_config_write(priv, h->irq_enab_reg_addr, h->irq_state_mask);
	return IRQ_HANDLED;
}

/**
 * hw_irq_handler - RIO HW interrupt handler
 * @irq: Linux interrupt number
 * @data: Pointer to interrupt-specific data
 *
 */
static irqreturn_t hw_irq_handler(int irq, void *data)
{
	struct rio_irq_handler *h = data;
	struct rio_priv *priv = h->data;
	u32 state;
	/**
	 * Get current interrupt state and clear latched state
	 * for interrupts handled by current thread.
	 */
	axxia_local_config_read(priv, h->irq_state_reg_addr, &state);
	state &= h->irq_state_mask;

	if (state) {
		axxia_local_config_write(priv, h->irq_enab_reg_addr, 0x0);
		return IRQ_WAKE_THREAD;
	}
	return IRQ_NONE;
}

static irqreturn_t hw_irq_dme_handler(int irq, void *data)
{
	struct rio_irq_handler *h;
	struct rio_priv *priv = data;

	h = &priv->ib_dme_irq;
	ib_dme_irq_handler(h);

	return IRQ_HANDLED;
}

/**
 * Caller must hold RAB lock
 */
int alloc_irq_handler(struct rio_irq_handler *h,
		     void *data,
		     const char *name)
{
	struct rio_priv *priv = data;/*mport->priv;*/
	u32 mask;
	int rc;

	if (test_and_set_bit(RIO_IRQ_ENABLED, &h->state))
		return -EBUSY;

	h->data = data;
	rc = request_threaded_irq(priv->irq_line,
				  hw_irq_handler,
				  thrd_irq_handler,
				  IRQF_TRIGGER_NONE | IRQF_SHARED,
				  name,
				  (void *)h);
	if (rc) {
		clear_bit(RIO_IRQ_ENABLED,  &h->state);
		h->data = NULL;
		return rc;
	}
	if (h->irq_enab_reg_addr) {
		axxia_local_config_read(priv, h->irq_enab_reg_addr, &mask);
		mask |= h->irq_state_mask;
		axxia_local_config_write(priv, h->irq_state_reg_addr, mask);
		axxia_local_config_write(priv, h->irq_enab_reg_addr, mask);
	}

	return rc;
}

/**
 * Caller must hold RAB lock
 */

void release_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	u32 mask;

	if (test_and_clear_bit(RIO_IRQ_ENABLED, &h->state)) {
		axxia_local_config_read(priv, h->irq_enab_reg_addr, &mask);
		mask &= ~h->irq_state_mask;
		axxia_local_config_write(priv, h->irq_enab_reg_addr, mask);
		free_irq(priv->irq_line, h);
		if (h->release_fn)
			h->release_fn(h);
	}
}

/**
 * MISC Indications
 */
#if defined(CONFIG_RAPIDIO_HOTPLUG)
static void rio_port_down_notify(struct rio_mport *mport)
{
	unsigned long flags;
	struct rio_priv *priv = mport->priv;

	spin_lock_irqsave(&priv->port_lock, flags);
	if (priv->port_notify_cb)
		priv->port_notify_cb(mport);

	spin_unlock_irqrestore(&priv->port_lock, flags);
}
#else
#define rio_port_down_notify(mport)
#endif

/**
 * __port_fatal_err - Check port error state and clear latched
 *                    error state to enable detection of new events.
 *
 * @mport: Master port
 *
 * Returns:
 * 1 -- port fatal error state is detected
 * 0 -- port ok
 */
static inline void __misc_fatal(struct rio_mport *mport,
				u32 misc_state)
{
	struct rio_priv *priv = mport->priv;
	u32 amast = 0;
	u32 aslv_state = 0;
	u32 aslv_addr = 0;
	u32 escsr, iecsr;

	dev_err(priv->dev, "*************Fatal Error************\n");
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &escsr);
	axxia_local_config_read(priv, EPC_IECSR(priv->port_ndx), &iecsr);

	/* clear latched state indications */
	/* Adding I2E to preserve idle sequence select bit which is R/w */
	axxia_local_config_write(priv, RIO_ESCSR(priv->port_ndx),
				(escsr & (RIO_ESCSR_I2E | RIO_EXCSR_WOLR)));
	dev_err(priv->dev, "port %d ESCSR(0x158) 0x%08x\n", priv->ndx, escsr);
	if (iecsr & EPC_IECSR_RETE) {
		dev_err(priv->dev, "Retry Error Threshold Exceeded\n");
		axxia_local_config_write(priv, EPC_IECSR(priv->port_ndx),
				(iecsr & EPC_IECSR_RETE));
	}
	if (misc_state & AMST_INT) {
		axxia_local_config_read(priv, RAB_AMAST_STAT, &amast);
		if (amast & RAB_AMAST_STAT_WRTO)
			dev_err(priv->dev, "AMST Write Response Timeout Error\n");
		if (amast & RAB_AMAST_STAT_RDTO)
			dev_err(priv->dev, "AMST Read Response Timeout Error\n");
		if (amast & RAB_AMAST_STAT_WRDE)
			dev_err(priv->dev, "AMST Write Decode Error\n");
		if (amast & RAB_AMAST_STAT_WRSE)
			dev_err(priv->dev, "AMST Write Slave Error\n");
		if (amast & RAB_AMAST_STAT_RDDE)
			dev_err(priv->dev, "AMST Read Decode Error\n");
		if (amast & RAB_AMAST_STAT_RDSE)
			dev_err(priv->dev, "AMST Read Slave Error\n");
		/* clear latched state */
		axxia_local_config_write(priv, RAB_AMAST_STAT, amast);
	}
	if (misc_state & ASLV_INT) {
		axxia_local_config_read(priv, RAB_ASLV_STAT_CMD,  &aslv_state);
		axxia_local_config_read(priv, RAB_ASLV_STAT_ADDR, &aslv_addr);
		if (aslv_state & RAB_ASLV_STAT_CMD_USUP) {
			dev_err(priv->dev, "AMBA Slave Unsupported Command\n");
			axxia_local_config_write(priv, RAB_ASLV_STAT_CMD,
					 (aslv_state & RAB_ASLV_STAT_CMD_USUP));
		}
	}
	if ((escsr & ESCSR_FATAL) ||
	    (iecsr & EPC_IECSR_RETE) ||
	    (misc_state & MISC_FATAL))
		rio_port_down_notify(mport);
}

/**
 * srio_sw_reset - Reset the SRIO (GRIO) module when it reaches a fatal
 *                 lockup state or if it received a reset control symbol
 */
static void srio_sw_reset(struct rio_priv *priv)
{
	u32 r32;
	u32 sval;

	r32 = srio_serdes_read32(priv, SRIO_PHY_CONTROL0_OFFSET);
	srio_serdes_write32(priv, SRIO_PHY_CONTROL0_OFFSET,
			(r32 | priv->linkdown_reset.reg_mask));
	while (1) {
		sval = srio_serdes_read32(priv, SRIO_PHY_CONTROL0_OFFSET);
		if ((sval & priv->linkdown_reset.reg_mask))
			break;
	}
	srio_serdes_write32(priv, SRIO_PHY_CONTROL0_OFFSET, (r32));
	sval = 0;
	while (1) {
		sval = srio_serdes_read32(priv, SRIO_PHY_CONTROL0_OFFSET);
		if (sval == r32)
			break;
	}
}

/**
 * PORT WRITE events
 */
/**
 * pw_irq_handler - AXXIA port write interrupt handler
 * @h: handler specific data
 * @state: PW Interrupt state
 *
 * Handles port write interrupts.
 */
static void pw_irq_handler(struct rio_irq_handler *h, u32 state)
{
	struct rio_priv *priv = h->data;
	struct rio_pw_irq *pw = priv->pw_data;
	u32 csr;
	int noofpw;
	u32 msg_word;

	if (pw == NULL) {
		dev_dbg(priv->dev, "Spurious port write message\n");
		return;
	}

	axxia_local_config_read(priv, RAB_IB_PW_CSR, &csr);
	noofpw = RAB_IB_PW_NUMWORDS(csr);
	dev_dbg(priv->dev, "%s: noofpw %d\n", __func__, noofpw);
	if (!(noofpw)) {
		dev_dbg(priv->dev, "PW Spurious Port Write\n");
		return;
	}
#if defined(CONFIG_AXXIA_RIO_STAT)
	priv->rio_pw_count++;
#endif
	while (noofpw) {

read_buff:
		axxia_local_config_read(priv, RAB_IB_PW_DATA, &msg_word);
		pw->msg_buffer[pw->msg_wc++] = BSWAP(msg_word);
		if (pw->msg_wc == 4) {
#if defined(CONFIG_AXXIA_RIO_STAT)
			priv->rio_pw_msg_count++;
#endif
			/*
			 * Pass the port-write message to RIO
			 * core for processing
			 */
			rio_inb_pwrite_handler(
					 (union rio_pw_msg *)pw->msg_buffer);
			pw->msg_wc = 0;
		}
		noofpw--;
		if (noofpw)
			goto read_buff;

		axxia_local_config_read(priv, RAB_IB_PW_CSR, &csr);
		noofpw = RAB_IB_PW_NUMWORDS(csr);
	}
}

static void axxia_rio_flush_pw(struct rio_mport *mport, int noofpw,
			     struct rio_pw_irq *pw_data)
{
	struct rio_priv *priv = mport->priv;
	u32 dummy;
	int x;

	dev_dbg(priv->dev, "(%s): flush %d words from pwbuff\n",
		__func__, noofpw);
	for (x = 0; x < noofpw; x++) {
		axxia_local_config_read(priv, RAB_IB_PW_DATA, &dummy);
		pw_data->discard_count++;
	}
	pw_data->msg_wc = 0;
}

/**
 * enable_pw - enable port-write interface unit
 * @h: Interrupt handler specific data
 *
 * Caller must hold RAB lock
 */
static int enable_pw(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	struct rio_pw_irq *pw_data;
	u32 rval;
	int rc = 0;

	if (priv->pw_data)
		return -EBUSY;

	pw_data = kzalloc(sizeof(struct rio_pw_irq), GFP_KERNEL);
	if (!pw_data)
		return -ENOMEM;

	axxia_local_config_read(priv, RAB_IB_PW_CSR, &rval);
	rval |= RAB_IB_PW_EN;
	axxia_rio_flush_pw(mport, RAB_IB_PW_NUMWORDS(rval), pw_data);
	axxia_local_config_write(priv, RAB_IB_PW_CSR, rval);
	priv->pw_data = pw_data;
	return rc;
}

/**
 * disable_pw - Disable port-write interface unit
 * @mport: pointer to struct rio_mport
 *
 * Caller must hold RAB lock
 */
static void disable_pw(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	struct rio_pw_irq *pw_data = priv->pw_data;
	u32 rval;

	if (pw_data == NULL)
		return;

	axxia_local_config_read(priv, RAB_IB_PW_CSR, &rval);
	rval &= ~RAB_IB_PW_EN;
	axxia_local_config_write(priv, RAB_IB_PW_CSR, rval);
	kfree(pw_data);
	priv->pw_data = NULL;
}


/**
 * misc_irq_handler - MISC interrupt handler
 * @h: handler specific data
 * @state: Interrupt state
 * Handles the Error, doorbell, Link reset request Interrupts
 */
static void misc_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	u32 misc_state;

	axxia_local_config_read(priv, RAB_INTR_STAT_MISC, &misc_state);
	/*
	 * Handle miscellaneous 'Link (IPG) Reset Request'
	 */
	if (misc_state & LINK_REQ_INT)
		srio_sw_reset(priv);

	if (misc_state & PORT_WRITE_INT)
		pw_irq_handler(h, misc_state & PORT_WRITE_INT);

	if (misc_state & (IB_DB_RCV_INT | OB_DB_DONE_INT))
		db_irq_handler(h,
			misc_state & (IB_DB_RCV_INT | OB_DB_DONE_INT));
	/**
	 * Notify platform if port is broken
	 */
	if (misc_state & MISC_FATAL)
		__misc_fatal(mport, misc_state);

	if (misc_state & GRIO_INT)
		dev_err(priv->dev, "GRIO Error Interrupt\n");
		/* TODO Need further Handling */
	if (misc_state & LL_TL_INT)
		dev_err(priv->dev, "Logical Layer Error Interrupt\n");
		/* TODO Need further Handling */
	if (misc_state & UNSP_RIO_REQ_INT)
		dev_dbg(priv->dev, "Unsupported RIO Request Received\n");
		/* TODO Need further Handling */
	if (misc_state & UNEXP_MSG_INT)
		dev_dbg_ratelimited(priv->dev,
			"Unexpected Inbound Data Message Received\n");
		/* TODO Need further Handling */

	axxia_local_config_write(priv, RAB_INTR_STAT_MISC, misc_state);
}

static void misc_release_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;

	disable_pw(mport);
}
/**
 * linkdown_irq_handler - Link Down interrupt Status interrupt handler
 * @h: handler specific data
 * @state: Interrupt state
 */
static void linkdown_irq_handler(struct rio_irq_handler *h/*, u32 state*/)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	u32 val;
	u32 val1;
	u32 rstate;

	__rio_local_read_config_32(mport, RAB_SRDS_STAT1, &rstate);
	__rio_local_read_config_32(mport, RAB_SRDS_CTRL2, &val);
	__rio_local_read_config_32(mport, RAB_SRDS_CTRL1, &val1);
	pr_info("Link Down: RAB_SRDS STAT1 = %x CTRL1 = %x CTRL2 = %x\n",
					rstate, val1, val);
	while (1) {
		axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &val);
		if (val & 0x2) {
			pr_info("Link up, Exiting Linkdown monitor Handler\n");
			__rio_local_read_config_32(mport, RAB_SRDS_CTRL1, &val);
			__rio_local_write_config_32(mport, RAB_SRDS_CTRL1,
								(val | 0x2));
			__rio_local_write_config_32(mport, RAB_SRDS_CTRL2,
							LINK_DOWN_TIMEOUT);
			break;
		}
	}
	__rio_local_read_config_32(mport, RAB_SRDS_STAT1, &rstate);
	__rio_local_read_config_32(mport, RAB_SRDS_CTRL2, &val);
	__rio_local_read_config_32(mport, RAB_SRDS_CTRL1, &val1);
}

/**
 * rpio_irq_handler - RPIO interrupt handler.
 * Service Peripheral Bus bridge, RapidIO -> Peripheral bus interrupt
 *
 * @h: handler specific data
 * @state: Interrupt state
 *
 */
static void rpio_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	u32 rstate;

	axxia_local_config_read(priv, RAB_INTR_STAT_RPIO, &rstate);
#if defined(CONFIG_AXXIA_RIO_STAT)
	if (rstate & RPIO_TRANS_COMPLETE)
		priv->rpio_compl_count++;
#endif
	if (rstate &  RPIO_TRANS_FAILED) {
		u32 rpio_stat;

		axxia_local_config_read(priv, RAB_RPIO_STAT, &rpio_stat);
		if (rpio_stat & RAB_RPIO_STAT_RSP_ERR)
			dev_dbg(priv->dev, "RPIO AXI Response Error\n");
		if (rpio_stat & RAB_RPIO_STAT_ADDR_MAP)
			dev_dbg(priv->dev, "RPIO Invalid Address Mapping Error\n");
		if (rpio_stat & RAB_RPIO_STAT_DISABLED)
			dev_dbg(priv->dev, "RPIO Engine Not Enabled\n");

		axxia_local_config_write(priv, RAB_RPIO_STAT, rpio_stat);
#if defined(CONFIG_AXXIA_RIO_STAT)
		priv->rpio_failed_count++;
#endif
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_RPIO, rstate);
}

/**
 * APIO
 */

/**
 * apio_irq_handler - APIO interrupt handler.
 * Service Peripheral Bus bridge, Peripheral bus -> RapidIO interrupt
 *
 * @h: handler specific data
 * @state: Interrupt state
 *
 */
static void apio_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	u32 astate;

	axxia_local_config_read(priv, RAB_INTR_STAT_APIO, &astate);
#if defined(CONFIG_AXXIA_RIO_STAT)
	if (astate & APIO_TRANS_COMPLETE)
		priv->apio_compl_count++;
#endif
	if (astate & APIO_TRANS_FAILED) {
		u32 apio_stat;

		axxia_local_config_read(priv, RAB_APIO_STAT, &apio_stat);
		if (apio_stat & RAB_APIO_STAT_RQ_ERR)
			dev_dbg(priv->dev, "APIO AXI Request Format Error\n");
		if (apio_stat & RAB_APIO_STAT_TO_ERR)
			dev_dbg(priv->dev, "APIO RIO Timeout Error\n");
		if (apio_stat & RAB_APIO_STAT_RSP_ERR)
			dev_dbg(priv->dev, "APIO RIO Response Error\n");
		if (apio_stat & RAB_APIO_STAT_MAP_ERR)
			dev_dbg(priv->dev, "APIO Invalid Address Mapping Error\n");
		if (apio_stat & RAB_APIO_STAT_MAINT_DIS)
			dev_dbg(priv->dev, "APIO Maintenance Mapping Not Enabled\n");
		if (apio_stat & RAB_APIO_STAT_MEM_DIS)
			dev_dbg(priv->dev, "APIO Memory Mapping Not Enabled\n");
		if (apio_stat & RAB_APIO_STAT_DISABLED)
			dev_dbg(priv->dev, "APIO Engine Not Enabled\n");
		axxia_local_config_write(priv, RAB_APIO_STAT, apio_stat);
#if defined(CONFIG_AXXIA_RIO_STAT)
		priv->apio_failed_count++;
#endif
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_APIO, astate);
}

/**
 * DOORBELL events
 */

/**
 * axxia_rio_rx_db_int_handler - AXXIA inbound doorbell interrupt handler
 * @mport: Master port with triggered interrupt
 * @mask: Interrupt register data
 *
 * Handles inbound doorbell interrupts.  Executes a callback on received
 * doorbell. Now called from the misc_irq thread, rio-misc-db.
 */
void rx_db_handler(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	struct rio_dbell *dbell;
	u32 csr, info;
	u8 num_msg;
	u16 src_id, db_info;
	int found;

	axxia_local_config_read(priv, RAB_IB_DB_CSR, &csr);
	num_msg = IB_DB_CSR_NUM_MSG(csr);

	for (; num_msg; num_msg--) {
		axxia_local_config_read(priv, RAB_IB_DB_INFO, &info);
		src_id = DBELL_SID(info);
		db_info = DBELL_INF(info);

		found = 0;
		dev_dbg(priv->dev,
			 "Processing doorbell, sid %4.4x info %4.4x\n",
			src_id, db_info);

		list_for_each_entry(dbell, &mport->dbells, node) {
			if (dbell->res->start <= db_info &&
			    (dbell->res->end >= db_info)) {
				found = 1;
				break;
			}
		}
		if (found) {
			/**
			 * NOTE: dst is set to 0 since we don't have
			 *       that value in the ACP
			 */
			if (dbell->dinb)
				dbell->dinb(mport, dbell->dev_id, src_id,
						0, db_info);
		} else {
			dev_dbg(priv->dev,
				"Spurious doorbell, sid %4.4x info %4.4x\n",
				src_id, db_info);
		}
	}
}

void db_irq_handler(struct rio_irq_handler *h, u32 state)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;

	/**
	 * Handle RX doorbell events
	 */
	if (state & IB_DB_RCV_INT)
		rx_db_handler(mport);

	/**
	 * Check for outbound doorbell Error conditions.
	 */
	if (state & OB_DB_DONE_INT) {
		int db;
		u32 csr;

		for (db = 0; db < MAX_OB_DB; db++) {
			axxia_local_config_read(priv, RAB_OB_DB_CSR(db), &csr);

			if (OB_DB_STATUS(csr) == OB_DB_STATUS_RETRY)
				dev_dbg(priv->dev,
				  "Rio Doorbell Retry received\n");
			else if (OB_DB_STATUS(csr) == OB_DB_STATUS_ERROR)
				dev_dbg(priv->dev,
				  "Rio Doorbell send Error\n");
			else if (OB_DB_STATUS(csr) == OB_DB_STATUS_TIMEOUT)
				dev_dbg(priv->dev,
				  "Rio Doorbell send Timeout\n");
		}
	}
}

/**
 * OBDME Events/Outbound Messages
 */

static void release_dme(struct kref *kref)
{
	struct rio_msg_dme *me = container_of(kref, struct rio_msg_dme, kref);
	struct rio_priv *priv = me->priv;
	struct rio_msg_desc *desc;
	int i;

	if (me->desc) {
		for (i = 0, desc = me->desc; i < me->entries; i++, desc++)
			kfree(desc->msg_virt);
		kfree(me->desc);
	}

	kfree(me->descriptors);

	if (priv->intern_msg_desc) {
		if (me->dres.parent)
			release_resource(&me->dres);
	}

	kfree(me);
}

static inline struct rio_msg_dme *dme_get(struct rio_msg_dme *me)
{
	if (me)
		kref_get(&me->kref);
	return me;
}

static inline void dme_put(struct rio_msg_dme *me)
{
	if (me)
		kref_put(&me->kref, release_dme);
}

static inline int check_dme(int dme_no,
			    int *num_dmes,
			    int *dmes_in_use,
			    int *dmes)
{
	int i;

	for (i = 0; i < 2; i++) {
		if (dme_no < num_dmes[i]) {
			if (dmes[i] & (1 << dme_no)) {
				if (dmes_in_use[i] & (1 << dme_no))
					return -EBUSY;	/* Already allocated */
				return 0;
			}
		} else {
			dme_no -= num_dmes[i];
		}
	}

	return -ENXIO;	/* Not available */
}

/*
 * Enforce a DME 'choice' previously made
 */
static inline int select_dme(int dme_no,
			     int *num_dmes,
			     int *dmes_in_use,
			     int *dmes,
			     int value)
{
	int i;

	for (i = 0; i < 2; i++) {
		if (dme_no < num_dmes[i]) {
			dmes_in_use[i] &= ~(1 << dme_no);
			dmes_in_use[i] |= (value << dme_no);
			return 0;
		} else {
			dme_no -= num_dmes[i];
		}
	}

	return -ENXIO;	/* Not available */
}

/* Selects the DME for an Mbox
 * based on its occupancy. Two Outbound DMEs
 * are shared among mailboxes
 */
static inline int choose_ob_dme_static(
	struct rio_priv	*priv,
	int mbox_dest,
	int buf_sz,
	struct rio_msg_dme **ob_dme)
{
	int  i, ndx, sz, min_entries = 0;
	int  dme_no = 0, ret_dme_no = -ENXIO;
	struct rio_msg_dme *ret_dme = NULL;
	struct rio_tx_dme *dme_s;

	/* Multi-segment vs single-segment DMEs */
	ndx = RIO_MBOX_TO_IDX(mbox_dest);
	switch (ndx) {
	case 0:
		if ((priv->num_outb_dmes[0] == 0) || (priv->outb_dmes[0] == 0))
			return -ENXIO;
		break;
	case 1:
		if ((priv->num_outb_dmes[1] == 0) || (priv->outb_dmes[1] == 0))
			return -ENXIO;
		dme_no += priv->num_outb_dmes[0];
		break;
	default:
		dev_err(priv->dev, "Attempt to select unknown OB DME type!\n");
		return -ENXIO;
	}

	/* Find one with fewest entries, or sufficient free entries */
	for (i = 0; i < priv->num_outb_dmes[ndx]; i++, dme_no++) {
		sz = RIO_OUTB_DME_TO_BUF_SIZE(priv, dme_no);

		if (sz > buf_sz)
			continue;

		dme_s = &priv->ob_dme_shared[dme_no];

		if (dme_s->ring_size_free > min_entries) {
			min_entries = dme_s->ring_size_free;
			ret_dme = dme_s->me;
			ret_dme_no = dme_no;
		}
	}

	(*ob_dme) = ret_dme;
	return ret_dme_no;
}

static void release_mbox(struct kref *kref)
{
	struct rio_rx_mbox *mb = container_of(kref, struct rio_rx_mbox, kref);
	struct rio_priv *priv = mb->mport->priv;
	int letter;
	u32 dme_no;

	/* Quickly disable the engines */
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		if (mb->me[letter])
			axxia_local_config_write(priv,
				   RAB_IB_DME_CTRL(mb->me[letter]->dme_no), 0);
	}

	/* And then release the remaining resources */
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		if (mb->me[letter]) {
			dme_no = mb->me[letter]->dme_no;
			dme_put(mb->me[letter]);
			select_dme(dme_no,
					&priv->num_inb_dmes[0],
					&priv->inb_dmes_in_use[0],
					&priv->inb_dmes[0], 0);
			priv->ib_dme[dme_no] = NULL;
		}
	}


	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++)
		kfree(mb->virt_buffer[letter]);

	kfree(mb);
}

static inline struct rio_rx_mbox *mbox_get(struct rio_rx_mbox *mb)
{
	if (mb)
		kref_get(&mb->kref);
	return mb;
}

static inline void mbox_put(struct rio_rx_mbox *mb)
{
	if (mb)
		kref_put(&mb->kref, release_mbox);
}

static int alloc_msg_descriptors(struct rio_mport *mport,
				  struct resource *dres,
				  int buf_sz,
				  int entries,
				  int need_to_init,
				  struct rio_msg_desc **desc,
				  struct rio_desc **descriptors)
{
	struct rio_priv *priv = mport->priv;
	struct rio_msg_desc *rdesc = NULL, *idesc;
	struct rio_desc *rdescriptors = NULL;
	int i;

	if (priv->intern_msg_desc) {
		dres->name = "DME_DESC";
		dres->flags = ACP_RESOURCE_HW_DESC;
		if (allocate_resource(&priv->acpres[ACP_HW_DESC_RESOURCE],
				dres, entries,
				priv->acpres[ACP_HW_DESC_RESOURCE].start,
				priv->acpres[ACP_HW_DESC_RESOURCE].end,
				0x1, NULL, NULL)) {
			memset(dres, 0, sizeof(*dres));
			goto err;
		}
	} else {
		dres->start = 0;
	}

	rdesc = kzalloc(sizeof(struct rio_msg_desc) * entries, GFP_ATOMIC);
	if (rdesc == NULL)
		goto err;
	rdescriptors = kzalloc(sizeof(struct rio_desc) * entries, GFP_ATOMIC);
	if (rdescriptors == NULL)
		goto err;

	for (i = 0, idesc = rdesc; i < need_to_init; i++, idesc++) {
		idesc->msg_virt = kzalloc(buf_sz, GFP_KERNEL);
		if (!idesc->msg_virt)
			goto err;
		idesc->msg_phys = virt_to_phys(idesc->msg_virt);
	}

	idesc--;
	idesc->last = DME_DESC_DW0_NXT_DESC_VALID;

	(*desc) = rdesc;
	(*descriptors) = rdescriptors;

	return 0;

err:
	kfree(rdesc);
	kfree(rdescriptors);
	return -ENOMEM;
}

static struct rio_msg_dme *alloc_message_engine(struct rio_mport *mport,
						int dme_no, void *dev_id,
						int buf_sz, int entries)
{
	struct rio_priv *priv = mport->priv;
	struct rio_msg_dme *me = kzalloc(sizeof(struct rio_msg_dme),
					 GFP_KERNEL);
	int rc = 0;

	if (!me)
		return ERR_PTR(-ENOMEM);

	memset(me, 0, sizeof(struct rio_msg_dme));

	kref_init(&me->kref);
	spin_lock_init(&me->lock);
	me->priv = priv;
	me->sz = 0;/*buf_sz;*/

	rc = alloc_msg_descriptors(mport, &me->dres, buf_sz, entries,
				entries, &me->desc, &me->descriptors);
	if (rc < 0)
		goto err;

	me->entries = entries;
	me->dev_id = dev_id;
	me->write_idx = 0;
	me->read_idx = 0;
	me->tx_dme_tmo = 0;
	me->dme_no = dme_no;

	return me;

err:
	dme_put(me);
	return ERR_PTR(rc);
}

/**
 * ob_dme_msg_handler - Outbound Data message handler
 * --- Called from OB DME irq handler thread ---
 * @h: Pointer to interrupt-specific data
 *
 * Handles outbound message interrupts. Executes a callback,
 * if available.
 *
 * @note:
 * HW descriptor fetch and update may be out of order.
 * Check state of all used descriptors and take care to not fall into
 * any of the traps that come with this design:
 *
 * Due to this (possibly) out of order execution in the HW, SW ack of
 * descriptors must be done atomically, re-enabling descriptors with
 * completed transactions while processing finished transactions may
 * break the ring and leave the DMA engine in a state where it doesn't
 * process new inserted requests.
 */
static void  ob_dme_msg_handler(struct rio_irq_handler *h, u32 dme_no)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	struct rio_msg_dme *dme = priv->ob_dme_shared[dme_no].me;
	u32 dw0;
	u32 dw1;
	int mbox;
	struct rio_tx_mbox *mb;
	unsigned int iteration = 0;

	/**
	 * Process all completed transactions
	 */
ob_dme_restart:
	while (dme->read_idx != dme->write_idx) {
		AXXIA_RIO_SYSMEM_BARRIER();
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(dme, dme->read_idx));
		if ((dw0 & DME_DESC_DW0_VALID) &&
			(dw0 & DME_DESC_DW0_READY_MASK)) {
			*((u32 *)DESC_TABLE_W0_MEM(dme, dme->read_idx))
					= dw0 & DME_DESC_DW0_NXT_DESC_VALID;
			dw1 = *((u32 *)DESC_TABLE_W1_MEM(dme,
						dme->read_idx));
			__dme_dw_dbg(priv->dev, dme, 1, dw0, dw1);
			dme->read_idx = (dme->read_idx + 1) &
						(dme->entries - 1);
			mbox = (dw1 >> 2) & 0x3;
			mb = priv->ob_mbox[mbox];
			if (mb) {
				if (mport->outb_msg[mbox].mcback) {
					mb->tx_slot = (mb->tx_slot + 1)
							%(mb->ring_size);
					mport->outb_msg[mbox].mcback(mport,
							mb->dev_id,
							mbox, mb->tx_slot);
				}
#ifdef CONFIG_AXXIA_RIO_STAT
				mb->compl_msg_count++;
#endif
			}
			iteration++;
		} else
			break;
	}
	if (iteration) {
		iteration = 0;
		goto ob_dme_restart;
	}
}

/**
 * ob_dme_irq_handler - Outbound message interrupt handler
 * --- Called in threaded irq handler ---
 * @h: Pointer to interrupt-specific data
 *
 * Handles outbound message interrupts. Calls the
 * msg handler if dscriptor xfer complete is set.
 * or reports the error
 */
enum hrtimer_restart ob_dme_tmr_handler(struct hrtimer *hr)
{
	struct rio_tx_dme *obd = container_of(hr, struct rio_tx_dme, tmr);
	struct rio_msg_dme *me = obd->me;
	struct rio_priv *priv = me->priv;
	struct rio_irq_handler *h = &priv->ob_dme_irq;
	u32 dme_stat, dme_no;

	dme_no = me->dme_no;
	axxia_local_config_read(priv, RAB_OB_DME_STAT(dme_no),
						&dme_stat);

	if (dme_stat & (OB_DME_STAT_DESC_FETCH_ERR |
				OB_DME_STAT_DESC_ERR |
				OB_DME_STAT_DESC_UPD_ERR))
		dev_err(priv->dev, "OB DME%d: Descriptor Error\n",
								dme_no);
	else {

		if (dme_stat & (OB_DME_STAT_DATA_TRANS_ERR |
				OB_DME_STAT_RESP_ERR |
				OB_DME_STAT_RESP_TO)) {
			if (dme_stat & OB_DME_STAT_DATA_TRANS_ERR)
				dev_err(priv->dev, "OB DME%d: Transaction Error\n",
								dme_no);
			if (dme_stat & OB_DME_STAT_RESP_ERR)
				dev_dbg_ratelimited(priv->dev,
						"OB DME%d: Response Error\n",
								dme_no);
			if (dme_stat & OB_DME_STAT_RESP_TO)
				dev_err(priv->dev, "OB DME%d: Response Timout Error\n",
								dme_no);
		}
		ob_dme_msg_handler(h, dme_no);
	}
	axxia_local_config_write(priv, RAB_OB_DME_STAT(dme_no),
							dme_stat);
	hrtimer_forward_now(&obd->tmr, ktime_set(0, axxia_hrtimer_delay));
	return HRTIMER_RESTART;
}

static int alloc_ob_dme_shared(struct rio_priv *priv,
			struct rio_tx_dme *dme_s, int dme_no)
{
	int rc = 0;
	int sz;
	struct rio_mport *mport = priv->mport;
	struct rio_msg_dme *me = NULL;
	struct rio_msg_desc *desc = NULL;
	u32 dw0, dw1, dw2, dw3;
	u64  desc_chn_start = 0;
	int entries = OB_DME_ENTRIES;
	int i;

	sz = RIO_OUTB_DME_TO_BUF_SIZE(priv, dme_no);
	entries = roundup_pow_of_two(entries);
	me = alloc_message_engine(mport,
				dme_no, NULL, sz, entries);
	if (IS_ERR(me)) {
		rc = PTR_ERR(me);
		goto err;
	}

	for (i = 0, desc = me->desc; i < entries; i++, desc++) {
		dw0 = 0;
		if (!priv->intern_msg_desc) {
			dw1 = 0;
			dw2 = (u32)(desc->msg_phys >>  8) & 0x3fffffff;
			*((u32 *)DESC_TABLE_W0_MEM(me, i)) = dw0;
			*((u32 *)DESC_TABLE_W1_MEM(me, i)) = dw1;
			*((u32 *)DESC_TABLE_W2_MEM(me, i)) = dw2;
			*((u32 *)DESC_TABLE_W3_MEM(me, i)) = 0;
		} else {
			dw1 = 0;
			dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
			__rio_local_write_config_32(mport,
				    DESC_TABLE_W0(me->dres.start+i), dw0);
			__rio_local_write_config_32(mport,
				    DESC_TABLE_W1(me->dres.start+i), dw1);
			__rio_local_write_config_32(mport,
				    DESC_TABLE_W2(me->dres.start+i), dw2);
			__rio_local_write_config_32(mport,
				    DESC_TABLE_W3(me->dres.start+i), 0);
		}
	}


	/**
	* Last descriptor - make ring.
	* Next desc table entry -> dw2.First desc address[37:36]
	*                       -> dw3.First desc address[35:4].
	* (desc_base + 0x10 * nr)
	*/
	desc--; i--;
	dw0 |= DME_DESC_DW0_NXT_DESC_VALID;
	if (!priv->intern_msg_desc) {
		desc_chn_start =
			(uintptr_t)virt_to_phys(me->descriptors);

		dw2  = *((u32 *)DESC_TABLE_W2_MEM(me, i));
		dw2 |= (desc_chn_start >> 4) & 0xc0000000;
		dw3  = desc_chn_start >> 4;
		*((u32 *)DESC_TABLE_W0_MEM(me, i)) = dw0;
		*((u32 *)DESC_TABLE_W2_MEM(me, i)) = dw2;
		*((u32 *)DESC_TABLE_W3_MEM(me, i)) = dw3;
	} else {
		desc_chn_start = DESC_TABLE_W0(me->dres.start);
		__rio_local_read_config_32(mport,
				DESC_TABLE_W2(me->dres.start+i), &dw2);
		dw2 |= ((desc_chn_start >> 8) & 0xc0000000);
		dw3  = 0;
		__rio_local_write_config_32(mport,
				DESC_TABLE_W0(me->dres.start+i), dw0);
		__rio_local_write_config_32(mport,
				DESC_TABLE_W2(me->dres.start+i), dw2);
		__rio_local_write_config_32(mport,
				DESC_TABLE_W3(me->dres.start+i), dw3);
	}
	test_and_set_bit(RIO_DME_OPEN, &me->state);
	dme_s->me = me;
	dme_s->ring_size = 0x0;
	dme_s->ring_size_free = entries;
err:
	return rc;
}
/**
 * open_outb_mbox_static - Initialize AXXIA outbound mailbox
 *			   using statically allocated DME descriptors.
 *
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox_id: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring for each letter
 * @prio: 0..3, higher number -> lower priority.
 *
 * Caller must hold RAB lock
 * If the specified mbox DME has already been opened/reserved, then we just
 * abort out of this operation with "busy", and without changing resource
 * allocation for the mbox DME.
 *
 * To increase efficiecny the Descriptors are allocated and initalized during
 * initialization time and then kept forever to be reused.
 *
 * Returns:
 * %0 if successful
 * %-EINVAL if an argument is invalid
 * %-ENOMEM if unable to allocate sufficient memory
 * %-ENODEV if unable to find a DME matching the input arguments
 */

static int open_outb_mbox_static(struct rio_mport *mport,
			void *dev_id, int mbox_id, int entries, int prio)
{
	int  rc = 0;
	int  dme_no, buf_sz = 0;
	struct rio_priv *priv = mport->priv;
	struct rio_tx_mbox *mb;/* = priv->ob_mbox[mbox_id];*/
	struct rio_msg_dme *me = NULL;
	unsigned long iflags0;
	u32 dme_ctrl, dme_stat, desc_addr, wait = 0;
	u64  desc_chn_start = 0;

	if ((mbox_id < 0) || (mbox_id > RIO_MAX_TX_MBOX) ||
	    (entries < 2) || (entries > priv->desc_max_entries))
		return -EINVAL;
	if (priv->ob_mbox[mbox_id])
		return -EINVAL;
	mb = kzalloc(sizeof(struct rio_tx_mbox), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;
	spin_lock_init(&mb->lock);
	mb->dme_no = 0xff;
#ifdef CONFIG_AXXIA_RIO_STAT
	mb->sent_msg_count = 0;
	mb->compl_msg_count = 0;
#endif
	spin_lock_irqsave(&mb->lock, iflags0);

	if (test_bit(RIO_MB_OPEN, &mb->state)) {
		spin_unlock_irqrestore(&mb->lock, iflags0);
		return -EINVAL;
	}

	/*
	** Pick the OB DME that we will use for this mailbox
	*/
		buf_sz = RIO_MBOX_TO_BUF_SIZE(mbox_id);

		dme_no = choose_ob_dme_static(priv, mbox_id, buf_sz, &me);
		if (dme_no < 0) {
			spin_unlock_irqrestore(&mb->lock, iflags0);
			rc = dme_no;
			goto err;
		}
		if (IS_ERR_OR_NULL(me)) {
			spin_unlock_irqrestore(&mb->lock, iflags0);
			rc = PTR_ERR(me);
			goto err;
		}

		if (!test_bit(RIO_DME_MAPPED, &me->state)) {
			do {
				axxia_local_config_read(priv,
					RAB_OB_DME_STAT(dme_no), &dme_stat);
				if (wait++ > 100) {
					rc = -EBUSY;
					goto err;
				}
			} while (dme_stat & OB_DME_STAT_TRANS_PEND);
			desc_chn_start =
				(uintptr_t)virt_to_phys(me->descriptors);

			dme_ctrl  = (prio & 0x3) << 4;
			dme_ctrl |= (u32)((desc_chn_start >> 6) & 0xc0000000);
			desc_addr  = (u32)desc_chn_start >> 4;
			axxia_local_config_write(priv,
				RAB_OB_DME_DESC_ADDR(dme_no), desc_addr);
			axxia_local_config_write(priv, RAB_OB_DME_CTRL(dme_no),
					dme_ctrl);
			me->dme_ctrl = dme_ctrl;
			me->dme_ctrl |= (DME_WAKEUP | DME_ENABLE);
			priv->ob_dme_irq.irq_state_mask |= (1 << dme_no);
			axxia_local_config_write(priv, RAB_INTR_STAT_ODME,
								1<<dme_no);
			axxia_local_config_write(priv, RAB_INTR_ENAB_ODME,
					priv->ob_dme_irq.irq_state_mask);
			test_and_set_bit(RIO_DME_MAPPED, &me->state);
		}


	mb->mport = mport;
	mb->mbox_no = mbox_id;
	mb->dme_no = dme_no;
	mb->me = me;
	mb->ring_size = entries;
	mb->tx_slot = 0;
	mb->dev_id = dev_id;
	me->sz++;
	mdelay(500); /* Delay added to ensure completion of any pending TX
			before Transmission on this Mailbox */

	if (me->sz == 1) {
		hrtimer_init(&priv->ob_dme_shared[dme_no].tmr,
				 CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		priv->ob_dme_shared[dme_no].tmr.function = ob_dme_tmr_handler;
		hrtimer_start(&priv->ob_dme_shared[dme_no].tmr,
				ktime_set(0, (axxia_hrtimer_delay)),
					HRTIMER_MODE_REL_PINNED);
	}

	test_and_set_bit(RIO_MB_MAPPED, &mb->state);

	priv->ob_dme_shared[dme_no].ring_size += entries;
	priv->ob_dme_shared[dme_no].ring_size_free -= entries;

	spin_unlock_irqrestore(&mb->lock, iflags0);

#ifdef CONFIG_AXXIA_RIO_STAT
	me->desc_done_count = 0;
	me->desc_error_count = 0;
	me->desc_rio_err_count = 0;
	me->desc_axi_err_count = 0;
	me->desc_tmo_err_count = 0;
#endif
	/**
	 * Finish updating the mailbox and DME state before we go
	 */
	test_and_set_bit(RIO_MB_OPEN, &mb->state);
	priv->ob_mbox[mbox_id] = mb;
	return 0;

err:
	spin_unlock_irqrestore(&mb->lock, iflags0);
	kfree(mb);
	return rc;
}


/**
 * release_outb_dme - Close AXXIA outbound DME engine structures
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Caller must hold RAB lock
 * Release all resources i.e. DMEs, descriptors, buffers, and so on.
 */

static void release_outb_dme(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	int i;
	struct rio_msg_dme *me;

	for (i = 0; i < DME_MAX_OB_ENGINES; i++) {
		me = priv->ob_dme_shared[i].me;
		if (me && test_bit(RIO_DME_OPEN, &me->state)) {
			if (test_bit(RIO_DME_MAPPED, &me->state)) {
				axxia_local_config_write(priv,
					RAB_OB_DME_CTRL(me->dme_no), 0);

				select_dme(me->dme_no,
					&priv->num_outb_dmes[0],
					&priv->outb_dmes_in_use[0],
					&priv->outb_dmes[0], 0);
			}

			dme_put(me);
		}
	}
	h->data = NULL;
}

/**
 * ib_dme_irq_handler - AXXIA inbound message interrupt handler
 * @mport: Master port with triggered interrupt
 * @mask: Interrupt register data
 *
 * Handles inbound message interrupts.  Executes a callback, if available,
 * on received message. Reports the Error.
 */
static void  ib_dme_irq_handler(struct rio_irq_handler *h/*, u32 state*/)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	int mbox_no;
	int letter;
	u32 dme_mask, mask;
ib_dme_restart:
	axxia_local_config_read(priv, RAB_INTR_STAT_IDME, &dme_mask);
	mask = dme_mask;
	if (!mask)
		return;
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, mask);
	/**
	 * Inbound mbox has 4 engines, 1 per letter.
	 * For each message engine that contributes to IRQ state,
	 * go through all descriptors in queue that have been
	 * written but not handled.
	 */
	while (dme_mask) {
		struct rio_msg_dme *me;
		u32 dme_stat;
		int dme_no = __fls(dme_mask);

		dme_mask ^= (1 << dme_no);
		me = priv->ib_dme[dme_no];
		/**
		 * Get and clear latched state
		 */
		axxia_local_config_read(priv,
					   RAB_IB_DME_STAT(dme_no), &dme_stat);
		axxia_local_config_write(priv,
					    RAB_IB_DME_STAT(dme_no), dme_stat);
		if (!me)
			continue;

		mbox_no = me->mbox;
		letter = me->letter;
		if (!(dme_stat & 0xff))
			continue;

		if (dme_stat & (IB_DME_STAT_DESC_XFER_CPLT |
				IB_DME_STAT_DESC_CHAIN_XFER_CPLT)) {
			if (mport->inb_msg[mbox_no].mcback)
				mport->inb_msg[mbox_no].mcback(mport,
					me->dev_id, mbox_no, letter);
		}

		if (dme_stat & IB_DME_STAT_ERROR_MASK) {
			if (dme_stat & (IB_DME_STAT_DESC_UPDATE_ERR |
					IB_DME_STAT_DESC_ERR |
					IB_DME_STAT_DESC_FETCH_ERR))
				dev_err(priv->dev,
				"IB Mbox%d Letter%d DME%d: Descriptor Error\n",
						mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_DATA_TRANS_ERR)
				dev_err(priv->dev,
				"IB Mbox%d Letter%d DME%d: Transaction Error\n",
						mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_MSG_ERR)
				dev_err(priv->dev,
				"IB MBOX%d Letter%d DME%d: Message Error\n",
						mbox_no, letter, dme_no);

			if (dme_stat & (IB_DME_STAT_MSG_TIMEOUT))
				dev_err(priv->dev,
				"IB MBOX%d Letter%d DME%d: SRIO Timeout\n",
						mbox_no, letter, dme_no);
		}

	}
	goto ib_dme_restart;
}

enum hrtimer_restart ib_dme_tmr_handler(struct hrtimer *hr)
{
	struct rio_rx_mbox *mb = container_of(hr, struct rio_rx_mbox, tmr);
	struct rio_mport *mport = mb->mport;
	struct rio_priv *priv = mport->priv;
	int mbox_no;
	int letter;
	u32 dme_mask, mask;
ib_dme_restart:
	axxia_local_config_read(priv, RAB_INTR_STAT_IDME, &dme_mask);
	dme_mask &= mb->irq_state_mask;
	mask = dme_mask;
	if (!mask) {
		hrtimer_forward_now(&mb->tmr,
				ktime_set(0, axxia_hrtimer_delay));
		return HRTIMER_RESTART;
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, mask);
	/**
	 * Inbound mbox has 4 engines, 1 per letter.
	 * For each message engine that contributes to IRQ state,
	 * go through all descriptors in queue that have been
	 * written but not handled.
	 */
	while (dme_mask) {
		struct rio_msg_dme *me;
		u32 dme_stat;
		int dme_no = __fls(dme_mask);

		dme_mask ^= (1 << dme_no);
		me = priv->ib_dme[dme_no];
		/**
		 * Get and clear latched state
		 */
		axxia_local_config_read(priv,
					   RAB_IB_DME_STAT(dme_no), &dme_stat);
		axxia_local_config_write(priv,
					    RAB_IB_DME_STAT(dme_no), dme_stat);
		if (!me)
			continue;

		mbox_no = me->mbox;
		letter = me->letter;
		if (!(dme_stat & 0xff))
			continue;

		if (dme_stat & (IB_DME_STAT_DESC_XFER_CPLT |
				IB_DME_STAT_DESC_CHAIN_XFER_CPLT)) {
			if (mport->inb_msg[mbox_no].mcback)
				mport->inb_msg[mbox_no].mcback(mport,
					me->dev_id, mbox_no, letter);
		}

		if (dme_stat & IB_DME_STAT_ERROR_MASK) {
			if (dme_stat & (IB_DME_STAT_DESC_UPDATE_ERR |
					IB_DME_STAT_DESC_ERR |
					IB_DME_STAT_DESC_FETCH_ERR))
				dev_err(priv->dev,
				"IB Mbox%d Letter%d DME%d: Descriptor Error\n",
						mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_DATA_TRANS_ERR)
				dev_err(priv->dev,
				"IB Mbox%d Letter%d DME%d: Transaction Error\n",
						mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_MSG_ERR)
				dev_err(priv->dev,
				"IB MBOX%d Letter%d DME%d: Message Error\n",
						mbox_no, letter, dme_no);

			if (dme_stat & (IB_DME_STAT_MSG_TIMEOUT))
				dev_err(priv->dev,
				"IB MBOX%d Letter%d DME%d: SRIO Timeout\n",
						mbox_no, letter, dme_no);
		}

	}
	goto ib_dme_restart;
}
/**
 * open_inb_mbox - Initialize AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open 0..(MID-1),
 *            0..3 multi segment,
 *            4..(MID-1) single segment
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring.  Sets up desciptor ring and memory
 * for messages for all 4 letters in the mailbox.  [This means
 * that the actual descriptor requirements are "4 * entries".]
 *
 * Returns %0 on success and %-EINVAL or %-ENOMEM on failure.
 */
static int open_inb_mbox(struct rio_mport *mport, void *dev_id,
			 int mbox, int entries)
{
	struct rio_priv *priv = mport->priv;
	struct rio_irq_handler *h = NULL;
	int i, letter;
	int rc, buf_sz;
	u32 irq_state_mask = 0;
	u32 dme_ctrl;
	struct rio_rx_mbox *mb;
	int j;

	if ((mbox < 0) || (mbox >= RIO_MAX_RX_MBOX))
		return -EINVAL;

	if ((entries < 2) || (entries > priv->desc_max_entries))
		return -EINVAL;
	h = &priv->ib_dme_irq;

	if (priv->ib_mbox[mbox] != NULL)
		return -EBUSY;

	buf_sz = RIO_MBOX_TO_BUF_SIZE(mbox);

	mb = kzalloc(sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;
	mb->mbox_no = mbox;

	kref_init(&mb->kref);
/* Adding 1 to entries to ensure the presence of invalid descriptor
 * in the circular buffer, to avoid the hardware getting into an
 * indefinite loop */
	entries = entries+1;
/* Rounding up to the power of two for efficient handling */
	entries = roundup_pow_of_two(entries);
	dev_dbg(priv->dev, "Opening inbound mbox %d with %d entries\n",
							mbox, entries);
	/**
	 *  Initialize rx buffer ring
	 */
	mb->mport = mport;
	mb->ring_size = entries;
	for (i = 0; i < RIO_MSG_MAX_LETTER; i++) {
		mb->virt_buffer[i] = kzalloc(mb->ring_size * sizeof(void *),
								GFP_KERNEL);
		if (!mb->virt_buffer[i]) {
			kfree(mb);
			return -ENOMEM;
		}
		mb->last_rx_slot[i] = 0;
		mb->next_rx_slot[i] = 0;
		for (j = 0; j < mb->ring_size; j++)
			mb->virt_buffer[i][j] = NULL;
	}

	/**
	 * Since we don't have the definition of letter in the generic
	 * RIO layer, we set up IB mailboxes for all letters for each
	 * mailbox.
	 */
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; ++letter) {
		int dme_no = 0;
		struct rio_msg_dme *me = NULL;
		struct rio_msg_desc *desc;
		u32 dw0, dw1, dw2, dw3;
		u64 desc_chn_start, desc_addr;
		u32 dme_stat, wait = 0;
		u32 buffer_size = (buf_sz > 256 ? 3 : 0);

		/* Search for a free DME, so we can more efficiently map
		 * them to the all of the mbox||letter combinations. */
		for (i = 0, rc = -1;
		     i < (priv->num_inb_dmes[0]+priv->num_inb_dmes[1]);
		     i++) {
			rc = check_dme(i, &priv->num_inb_dmes[0],
				&priv->inb_dmes_in_use[0], &priv->inb_dmes[0]);
			if (rc == 0) {
				dme_no = i;
				break;
			}
		}
		if (rc < 0)
			return rc;

		me = alloc_message_engine(mport,
					  dme_no,
					  dev_id,
					  buf_sz,
					  entries);
		if (IS_ERR(me)) {
			rc = PTR_ERR(me);
			goto err;
		}

		irq_state_mask |= (1 << dme_no);

		do {
			axxia_local_config_read(priv,
						   RAB_IB_DME_STAT(me->dme_no),
						   &dme_stat);
			if (wait++ > 100) {
				rc = -EBUSY;
				goto err;
			}
		} while (dme_stat & IB_DME_STAT_TRANS_PEND);

		mb->me[letter] = me;

		dw0 = ((buffer_size & 0x3) << 4) |
		      DME_DESC_DW0_EN_INT;
			/*Valid bit will be set in add_inb_buffer*/

		dw1 = DME_DESC_DW1_XMBOX(mbox) |
		      DME_DESC_DW1_MBOX(mbox)  |
		      DME_DESC_DW1_LETTER(letter);
		dw3 = 0;		/* 0 means, next contiguous addr
					 * Also next desc valid bit in dw0
					 * must be zero. */
		for (i = 0, desc = me->desc; i < entries; i++, desc++) {
			if (!priv->intern_msg_desc) {
				/* Reference AXX5500 Peripheral Subsystem
				 * Multicore Reference Manual, January 2013,
				 * Chapter 5, p. 584 */
				dw1 |= 0;
				dw2  = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
				*((u32 *)DESC_TABLE_W0_MEM(me,
						 i)) = dw0;
				*((u32 *)DESC_TABLE_W1_MEM(me,
						 i)) = dw1;
				*((u32 *)DESC_TABLE_W2_MEM(me,
						 i)) = dw2;
				*((u32 *)DESC_TABLE_W3_MEM(me,
						 i)) = dw3;
			} else {
				dw1 |= 0;
				dw2  = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
				__rio_local_write_config_32(mport,
					DESC_TABLE_W0(me->dres.start+i), dw0);
				__rio_local_write_config_32(mport,
					DESC_TABLE_W1(me->dres.start+i), dw1);
				__rio_local_write_config_32(mport,
					DESC_TABLE_W2(me->dres.start+i), dw2);
				__rio_local_write_config_32(mport,
					DESC_TABLE_W3(me->dres.start+i), dw3);
			}
		}

		/**
		 * Last descriptor - make ring.
		 * Next desc table entry -> dw2.First desc address[37:36].
		 *                       -> dw3.First desc address[35:4].
		 * (desc_base + 0x10 * nr)
		 */
		desc--; i--;
		dw0 |= DME_DESC_DW0_NXT_DESC_VALID;
		dw0 &= ~DME_DESC_DW0_VALID;
		if (!priv->intern_msg_desc) {
			desc_chn_start =
				(uintptr_t)virt_to_phys(me->descriptors);

			dw2  = *((u32 *)DESC_TABLE_W2_MEM(me, i));
			dw2 |= (desc_chn_start >> 4) & 0xc0000000;
			dw3  = desc_chn_start >> 4;
			*((u32 *)DESC_TABLE_W0_MEM(me, i)) = dw0;
			*((u32 *)DESC_TABLE_W2_MEM(me, i)) = dw2;
			*((u32 *)DESC_TABLE_W3_MEM(me, i)) = dw3;
		} else {
			desc_chn_start = DESC_TABLE_W0(me->dres.start);

			__rio_local_read_config_32(mport,
					    DESC_TABLE_W2(me->dres.start+i),
					    &dw2);
			dw3  = 0;
			dw2 |= ((desc_chn_start >> 8) & 0xc0000000);
			__rio_local_write_config_32(mport,
						DESC_TABLE_W0(me->dres.start+i),
						dw0);
			__rio_local_write_config_32(mport,
						DESC_TABLE_W2(me->dres.start+i),
						dw2);
			__rio_local_write_config_32(mport,
						DESC_TABLE_W3(me->dres.start+i),
						dw3);
		}

		/**
		 * Setup the DME including descriptor chain start address
		 */
		dme_ctrl = RAB_IB_DME_CTRL_XMBOX(mbox)    |
			   RAB_IB_DME_CTRL_MBOX(mbox)     |
			   RAB_IB_DME_CTRL_LETTER(letter) |
			   DME_WAKEUP                     |
			   DME_ENABLE;
		dme_ctrl |= (u32)((desc_chn_start >> 6) & 0xc0000000);
		desc_addr  = (u32)desc_chn_start >> 4;

		me->dme_ctrl = dme_ctrl;
		me->letter = letter;
		me->mbox = mbox;
		priv->ib_dme[dme_no] = me;

		axxia_local_config_write(priv,
					RAB_IB_DME_DESC_ADDR(dme_no),
					desc_addr);
		axxia_local_config_write(priv,
					RAB_IB_DME_CTRL(dme_no), dme_ctrl);

#ifdef CONFIG_AXXIA_RIO_STAT
		me->desc_done_count = 0;
		me->desc_error_count = 0;
		me->desc_rio_err_count = 0;
		me->desc_axi_err_count = 0;
		me->desc_tmo_err_count = 0;
#endif
		select_dme(dme_no, &priv->num_inb_dmes[0],
			&priv->inb_dmes_in_use[0], &priv->inb_dmes[0], 1);
	}

	/**
	* Create irq handler and enable MBOX irq
	*/

	mb->irq_state_mask = irq_state_mask;
	h->irq_state_mask |= irq_state_mask;
	priv->ib_mbox[mbox] = mb;
	AXXIA_RIO_SYSMEM_BARRIER();
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, irq_state_mask);

	if (priv->dme_mode == AXXIA_IBDME_TIMER_MODE) {
		hrtimer_init(&mb->tmr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		mb->tmr.function = ib_dme_tmr_handler;
		hrtimer_start(&mb->tmr, ktime_set(0, (axxia_hrtimer_delay)),
					HRTIMER_MODE_REL_PINNED);
	} else
		axxia_local_config_write(priv, RAB_INTR_ENAB_IDME,
						h->irq_state_mask);
	return 0;

err:
	mbox_put(mb);
	return rc;
}

/**
 * release_inb_mbox - Close AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Caller must hold RAB lock
 * Release all resources i.e. DMEs, descriptors, buffers, and so on.
 */

static void release_inb_mbox(struct rio_irq_handler *h)
{
	struct rio_rx_mbox *mb = h->data;
/*TODO*/
	h->data = NULL;
	mbox_put(mb);
}

void axxia_rio_port_get_state(struct rio_mport *mport, int cleanup)
{
	struct rio_priv *priv = mport->priv;
	u32 escsr, iecsr, state;

	if (cleanup) {
#if defined(CONFIG_AXXIA_RIO_STAT)
		reset_state_counters(priv);
#endif
		/**
		 * Clear latched state indications
		 */
		/* Miscellaneous Events */
		axxia_local_config_read(priv, RAB_INTR_STAT_MISC, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_MISC, state);
		/* Outbound Message Engine */
		axxia_local_config_read(priv, RAB_INTR_STAT_ODME, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_ODME, state);
		/* Inbound Message Engine */
		axxia_local_config_read(priv, RAB_INTR_STAT_IDME, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_IDME, state);
		/* Axxi Bus to RIO Events */
		axxia_local_config_read(priv, RAB_INTR_STAT_APIO, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_APIO, state);
		/* RIO to Axxia Bus Events */
		axxia_local_config_read(priv, RAB_INTR_STAT_RPIO, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_RPIO, state);
	}

	/* Master Port state */
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &escsr);
	axxia_local_config_read(priv, EPC_IECSR(priv->port_ndx), &iecsr);

	/* Adding I2E to preserve idle sequence select bit which is R/w */
	axxia_local_config_write(priv, RIO_ESCSR(priv->port_ndx),
				(escsr & (RIO_ESCSR_I2E | RIO_EXCSR_WOLR)));
}

/**
 * RIO MPORT Driver API
 */

/**
 * axxia_rio_port_irq_enable - Register RIO interrupt handler
 *
 * @mport: master port
 * @irq: IRQ mapping from DTB
 *
 * Caller must hold RAB lock
 *
 * Returns:
 * 0        Success
 * <0       Failure
 */
int axxia_rio_port_irq_enable(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	int rc;

	priv->dme_mode = axxia_dme_tmr_mode[priv->ndx];
	/**
	 * Clean up history
	 * from port reset/restart
	 */
	axxia_rio_port_get_state(mport, 1);
	rc = alloc_irq_handler(&priv->misc_irq, priv, "rio-misc-db");
	if (rc)
		goto out;

	__rio_local_write_config_32(mport, RAB_SRDS_CTRL1, 0x0);
	__rio_local_write_config_32(mport, RAB_SRDS_CTRL2,
					/*LINK_DOWN_TIMEOUT*/0x0);
	rc = alloc_irq_handler(&priv->linkdown_irq, priv, "rio-ld");
	if (rc)
		goto err1;
	rc = alloc_irq_handler(&priv->apio_irq, priv, "rio-apio");
	if (rc)
		goto err2;

	rc = alloc_irq_handler(&priv->rpio_irq, priv, "rio-rpio");
	if (rc)
		goto err3;
	priv->ib_dme_irq.data = priv;
	priv->ob_dme_irq.data = priv;

	if (priv->dme_mode == AXXIA_IBDME_INTERRUPT_MODE) {
		rc = request_threaded_irq(priv->irq_line,
				hw_irq_dme_handler, NULL,
				IRQF_TRIGGER_NONE | IRQF_SHARED,
				"rio-mb", (void *)priv);
		if (rc)
			goto err4;

		axxia_local_config_write(priv, RAB_INTR_ENAB_GNRL,
				    (RAB_INTR_ENAB_GNRL_SET | IB_DME_INT_EN));
	} else
		axxia_local_config_write(priv, RAB_INTR_ENAB_GNRL,
				    RAB_INTR_ENAB_GNRL_SET);
out:
	return rc;
err0:
	dev_warn(priv->dev, "RIO: unable to request irq.\n");
	goto out;
err4:
	release_irq_handler(&priv->rpio_irq);
err3:
	release_irq_handler(&priv->apio_irq);
err2:
	release_irq_handler(&priv->linkdown_irq);
err1:
	release_irq_handler(&priv->misc_irq);
	goto err0;
}

void axxia_rio_port_irq_disable(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	/**
	 * Mask top level IRQs
	 */
	axxia_local_config_write(priv, RAB_INTR_ENAB_GNRL, 0);
	/**
	 * free registered handlers
	 */
	release_irq_handler(&priv->misc_irq);
	release_irq_handler(&priv->ob_dme_irq);
	release_irq_handler(&priv->ib_dme_irq);
	release_irq_handler(&priv->apio_irq);
	release_irq_handler(&priv->rpio_irq);
}

int axxia_rio_pw_enable(struct rio_mport *mport, int enable)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;

	mutex_lock(&priv->api_lock);
	if (enable)
		rc = enable_pw(mport);
	else
		disable_pw(mport);
	mutex_unlock(&priv->api_lock);

	return rc;
}

/**
 * axxia_rio_doorbell_send - Send a doorbell message
 *
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of target device
 * @data: 16-bit info field of RapidIO doorbell message
 *
 * Sends a doorbell message.
 *
 * Returns %0 on success or %-EINVAL on failure.
 *
 * API protected by spin lock in generic rio driver.
 */
int axxia_rio_doorbell_send(struct rio_mport *mport,
			    int index, u16 destid, u16 data)
{
	struct rio_priv *priv = mport->priv;
	int db;
	u32 csr;

	for (db = 0; db < MAX_OB_DB; db++) {
		axxia_local_config_read(priv, RAB_OB_DB_CSR(db), &csr);
		if (OB_DB_STATUS(csr) == OB_DB_STATUS_DONE &&
		    OB_DB_STATUS(csr) != OB_DB_STATUS_RETRY) {

			csr = 0;
			csr |= OB_DB_DEST_ID(destid);
			csr |= OB_DB_PRIO(0x2); /* Good prio? */
			csr |= OB_DB_SEND;
			dev_dbg(priv->dev,
			   "Send doorbell 0x%04x to destid 0x%x\n",
				data, destid);
			axxia_local_config_write(priv, RAB_OB_DB_INFO(db),
						    OB_DB_INFO(data));
			axxia_local_config_write(priv, RAB_OB_DB_CSR(db),
						    csr);
			break;
		}
	}
	if (db == MAX_OB_DB)
		return -EBUSY;

	return 0;
}

/************/
/* OUTBOUND */
/************/
/**
 * axxia_open_outb_mbox - Initialize AXXIA outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox_dme: Mailbox to open
 * @entries: Number of entries in the outbound DME/mailbox ring for
 *           each letter
 *
 * Allocates and initializes descriptors.
 * We have N (e.g. 3) outbound mailboxes and M (e.g. 1024) message
 * descriptors.  The message descriptors are usable by inbound and
 * outbound message queues, at least until the point of binding.
 * Allocation/Distribution of message descriptors is flexible and
 * not restricted in any way other than that they must be uniquely
 * assigned/coherent to each mailbox/DME.
 *
 * Allocate memory for messages.
 * Each descriptor can hold a message of up to 4kB, though certain
 * DMEs or mailboxes may impose further limits on the size of the
 * messages.
 *
 * Returns %0 on success and %-EINVAL or %-ENOMEM on failure.
 */
int axxia_open_outb_mbox(
	struct rio_mport *mport,
	void *dev_id,
	int mbox_dme,
	int entries/*,
	int prio*/)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;

	mutex_lock(&priv->api_lock);
	rc = open_outb_mbox_static(mport, dev_id, mbox_dme,
					entries, 0x0/*prio*/);
	mutex_unlock(&priv->api_lock);

	return rc;
}

/**
 * axxia_close_outb_mbox - Shut down AXXIA outbound mailbox
 *
 * @mport: Master port implementing the outbound message unit
 * @mbox_id: Mailbox to close
 *
 * Disables the outbound message unit, frees all buffers, and
 * frees any other resources.
 */
void axxia_close_outb_mbox(struct rio_mport *mport, int mbox_id)
{
	struct rio_priv *priv = mport->priv;
	int dme_no;
	int wait_cnt = 0;
	struct rio_msg_dme *me;
	struct rio_tx_mbox *mb = NULL;


	if ((mbox_id < 0) ||
	    (mbox_id > RIO_MAX_TX_MBOX))
		return;
	mb = priv->ob_mbox[mbox_id];
	if ((!mb) ||
	    (!test_bit(RIO_MB_OPEN, &mb->state)))
		return;
	me = mb->me;

	mutex_lock(&priv->api_lock);
	clear_bit(RIO_MB_OPEN, &priv->ob_mbox[mbox_id]->state);
	while (me->write_idx != me->read_idx) {
		msleep(20);
		wait_cnt++;
		if (wait_cnt > 250)
			break;
	}
	if (wait_cnt > 250)
		pr_debug("Closed when outb mbox%d while transaction pending\n",
								mbox_id);
	priv->ob_mbox[mbox_id] = NULL;
	dme_no = mb->dme_no;
	mb->dme_no = 0xff;

	priv->ob_dme_shared[dme_no].ring_size -=
		mb->ring_size;

	priv->ob_dme_shared[dme_no].ring_size_free +=
		mb->ring_size;
	mb->dev_id = NULL;
	clear_bit(RIO_MB_MAPPED, &mb->state);
	kfree(mb);
	me->sz--;
	if (!(me->sz))
		hrtimer_cancel(&priv->ob_dme_shared[dme_no].tmr);
	mutex_unlock(&priv->api_lock);
}

static inline struct rio_msg_desc *get_ob_desc(struct rio_mport *mport,
						struct rio_msg_dme *mb)
{
	int desc_num = mb->write_idx;
	struct rio_priv *priv = mport->priv;
	struct rio_msg_desc *desc = &mb->desc[desc_num];
	int nxt_write_idx = (mb->write_idx + 1) & (mb->entries - 1);
	u32 dw0;

	if (nxt_write_idx != mb->read_idx) {
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(mb, desc_num));
		if (!(dw0 & DME_DESC_DW0_VALID))
			return desc;
		else
			dev_err(priv->dev, "Tx Desc error %d\n", mb->write_idx);
	}
	return NULL;
}

/**
 * axxia_add_outb_message - Add message to the AXXIA outbound message queue
 * --- Called in net core soft IRQ with local interrupts masked ---
 * --- And spin locked in master port net device handler        ---
 *
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox_dest: Destination mailbox
 * @letter: TID letter
 * @flags: 3 bit field,Critical Request Field[2] | Prio[1:0]
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the AXXIA outbound message queue.
 * Returns %0 on success
 *         %-EBUSY  on temporarily unavailable resource failure e.g. such
 *                     as waiting for an open entry in the outbound DME
 *                     descriptor chain
 *         %-EAGAIN on another kind of temporarily unavailable resource
 *                     failure
 *         %-EINVAL on invalid argument failure.
 *         %-ENODEV on unavailable resource failure e.g. no outbound DME
 *                     open that matches the kind of destination mailbox
 *         %-ENXIO  on incompatible argument failure e.g. trying to open
 *                     a single-segment mbox when none are available on
 *                     the platform
 */
int axxia_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
			     int mbox_dest, int letter, int flags,
			     void *buffer, size_t len/*, void *cookie*/)
{
	int rc = 0;
	u32 dw0, dw1;
	u16 destid = (rdev ? rdev->destid : mport->host_deviceid);
	struct rio_priv *priv = mport->priv;
	struct rio_tx_mbox *mb = priv->ob_mbox[mbox_dest];
	struct rio_msg_dme *me;
	struct rio_msg_desc *desc;
	u32 dw2_r, dw2;
	u32 idx;
	u32 seg;
	u32 lock = 0;
	u32 cp = 0;

	if (!mb)
		return -EINVAL;
	me = mb->me;
	if (me->sz > 1)
		lock = 1;

	/* Choose a free descriptor in a critical section */
	if (lock)
		spin_lock(&me->lock);
	desc = get_ob_desc(mport, me);
	if (!desc) {
		rc = -EBUSY;
		goto done;
	}


	/* Copy and clear rest of buffer */
	if ((u32)buffer > PAGE_OFFSET) {
		if ((u32)buffer & 0xFF) {
			if (unlikely(desc->msg_virt == NULL)) {
				rc = -ENXIO;
				goto done;
			}
			memcpy(desc->msg_virt, buffer, len);
			cp = 1;
		}
	} else {
		if (copy_from_user(desc->msg_virt, buffer, len)) {
			rc = -ENXIO;
			goto done;
		}
		cp = 1;
	}

	dw0 = DME_DESC_DW0_SRC_DST_ID(destid) |
	/*	DME_DESC_DW0_EN_INT|*/
		DME_DESC_DW0_VALID;

#if 0
	if (!(me->write_idx % 4))
		dw0 |=	DME_DESC_DW0_EN_INT;
#endif
	dw0 |= desc->last;/*DME_DESC_DW0_NXT_DESC_VALID;*/
	seg = len;
	if (seg < 256)
		seg = 256;
	seg = roundup_pow_of_two(seg) >> 7;
	dw1 = DME_DESC_DW1_PRIO(flags) |
		DME_DESC_DW1_CRF(flags) |
		(fls(seg)<<18) |
		DME_DESC_DW1_MSGLEN(len) |
		DME_DESC_DW1_XMBOX(mbox_dest) |
		DME_DESC_DW1_MBOX(mbox_dest) |
		DME_DESC_DW1_LETTER(letter);
	idx = me->write_idx;
	dw2_r  = *((u32 *)DESC_TABLE_W2_MEM(me, idx));
	if (cp)
		dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
	else
		dw2 = (u32)(virt_to_phys(buffer) >> 8) & 0x3fffffff;
	dw2 = (dw2_r & 0xc0000000) | dw2;
	me->write_idx = (me->write_idx+1) & (me->entries - 1);
	*((u32 *)DESC_TABLE_W2_MEM(me, idx)) = dw2;
	*((u32 *)DESC_TABLE_W1_MEM(me, idx)) = dw1;
	AXXIA_RIO_SYSMEM_BARRIER();
	*((u32 *)DESC_TABLE_W0_MEM(me, idx)) = dw0;

	if (lock)
		spin_unlock(&me->lock);
	else
		AXXIA_RIO_SYSMEM_BARRIER();
	/* Start / Wake up - the stored state is used to avoid a Read */
	axxia_local_config_write(priv, RAB_OB_DME_CTRL(me->dme_no),
							me->dme_ctrl);

#ifdef CONFIG_AXXIA_RIO_STAT
	priv->ob_mbox[mbox_dest]->sent_msg_count++;
#endif
	return rc;
done:
	if (lock)
		spin_unlock(&me->lock);
	return rc;
}

int axxia_ml_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
			     int mbox_dest, void *buffer, size_t len)
{
	return axxia_add_outb_message(mport, rdev, mbox_dest, 0, 0, buffer,
						len/*, NULL*/);
}

/**
 * axxia_open_inb_mbox - Initialize AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring.  Set up descriptor ring and memory
 * for messages for all letters in the mailbox.
 * Returns %0 on success and %-EINVAL or %-ENOMEM on failure.
 */
int axxia_open_inb_mbox(struct rio_mport *mport, void *dev_id,
			int mbox, int entries)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;

	mutex_lock(&priv->api_lock);
	rc = open_inb_mbox(mport, dev_id, mbox, entries);
	mutex_unlock(&priv->api_lock);

	return rc;
}

/**
 * axxia_close_inb_mbox - Shut down AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the inbound message unit, free all buffers, and
 * frees resources.
 */
void axxia_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	u32 letter;
	u32 dme_stat;
	u32 dme_no;

	if ((mbox < 0) || (mbox >= RIO_MAX_RX_MBOX))
		return;
	mutex_lock(&priv->api_lock);
	mb = priv->ib_mbox[mbox];
	if (mb == NULL) {
		mutex_unlock(&priv->api_lock);
		return;
	}
	priv->ib_dme_irq.irq_state_mask &= ~(mb->irq_state_mask);
	axxia_local_config_write(priv, RAB_INTR_ENAB_IDME,
					priv->ib_dme_irq.irq_state_mask);
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, mb->irq_state_mask);
	msleep(500);
	priv->ib_mbox[mbox] = NULL;
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		int wait = 0;

		if (mb->me[letter]) {
			dme_no = mb->me[letter]->dme_no;
			do {
				axxia_local_config_read(priv,
					RAB_IB_DME_STAT(dme_no), &dme_stat);
				if (wait++ > 10000)
					break;
			} while (dme_stat & IB_DME_STAT_TRANS_PEND);
			if (wait > 10000)
				dev_err(priv->dev,
					"Closing while Transaction pending\n");
			axxia_local_config_write(priv,
					RAB_IB_DME_CTRL(dme_no), 0);
		}
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, mb->irq_state_mask);
	mb->irq_state_mask = 0;
	msleep(100);
	if (priv->dme_mode == AXXIA_IBDME_TIMER_MODE)
		hrtimer_cancel(&mb->tmr);
	mbox_put(mb);
	mutex_unlock(&priv->api_lock);
}

/**
 * axxia_add_inb_buffer - Add buffer to the AXXIA inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the AXXIA inbound message queue.
 *
 * Returns %0 on success
 *         %-EINVAL on invalid argument failure.
 *         %-EBUSY  on temporarily unavailable resource failure e.g. such
 *                     as waiting for a filled entry in the inbound DME
 *                     descriptor chain
 */
int axxia_add_inb_buffer(struct rio_mport *mport, int mbox, void *buf)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	int rc = 0;
	struct rio_msg_dme *me;
	struct rio_msg_desc *desc;
	u32 dw0, dw2, dw2_r;

	mb = (priv->ib_mbox[mbox]);
	if (!mb)
		return -EINVAL;
	me = mb->me[0];
	/* Lockless circular buffer scheme */
	if (((me->write_idx + 1) & (me->entries - 1)) == me->read_idx)
		goto busy;
	if (mb->virt_buffer[0][me->write_idx]) {
		/* TODO Need to handle this case when DME encounters error */
		goto busy;
	}

	dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx));
	if (dw0 & DME_DESC_DW0_VALID) {
		dev_dbg(priv->dev, "Filling an already valid buffer %d %x\n",
							 me->write_idx, dw0);
		goto busy;
	}
	mb->virt_buffer[0][me->write_idx] = buf;
	if (!((u32)buf & 0xFF)) {
		dw2_r = *((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx));
		dw2 = (u32)(virt_to_phys(buf) >> 8) & 0x3fffffff;
		dw2 = (dw2_r & 0xc0000000) | dw2;
		*((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx)) = dw2;
	} else {
		desc = &me->desc[me->write_idx];
		dw2_r = *((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx));
		dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
		dw2 = (dw2_r & 0xc0000000) | dw2;
		*((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx)) = dw2;
	}

	AXXIA_RIO_SYSMEM_BARRIER();
	dw0 |= DME_DESC_DW0_VALID;
	*((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx)) = dw0;
	AXXIA_RIO_SYSMEM_BARRIER();
	me->write_idx = (me->write_idx + 1) & (me->entries - 1);
/*	axxia_local_config_read(priv,
		RAB_IB_DME_CTRL(me->dme_no), &dme_ctrl);
	dme_ctrl |= (DME_WAKEUP | DME_ENABLE);*/
	axxia_local_config_write(priv,
		RAB_IB_DME_CTRL(me->dme_no), me->dme_ctrl);
done:
	return rc;
busy:
	rc = -EBUSY;
	goto done;
}

/**
 * axxia_get_inb_message - Fetch an inbound message from the AXXIA
 *                         message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @letter: Inbound mailbox letter
 * @sz: size of returned buffer
 *
 * Gets the next available inbound message from the inbound message queue.
 *
 * Returns pointer to the message on success
 *         NULL on nothing available
 *         IS_ERR(ptr) on failure with extra information
 */
void *axxia_get_inb_message(struct rio_mport *mport, int mbox, int letter,
			    int *sz/*, int *slot, u16 *destid*/)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	struct rio_msg_dme *me;
	int num_proc = 0;
	void *buf = NULL;
	u32 idx;

	mb = (priv->ib_mbox[mbox]);
	if (!mb)
		return NULL;
	me = (mb->me[letter]);
	while (1) {
		struct rio_msg_desc *desc = &me->desc[me->read_idx];
		u32 dw0, dw1;

		idx = me->read_idx;
		buf = NULL;
		*sz = 0;
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, idx));
		dw1 = *((u32 *)DESC_TABLE_W1_MEM(me, idx));
		__dme_dw_dbg(priv->dev, me, 0, dw0, dw1);
		if ((dw0 & DME_DESC_DW0_ERROR_MASK) &&
		    (dw0 & DME_DESC_DW0_VALID)) {
			*((u32 *)DESC_TABLE_W0_MEM(me, idx)) =
					(dw0 & 0xff) | DME_DESC_DW0_VALID;
/*TODO Need to check here: May need to keep it valid for nocopy case
 *Proper Error Handling and add_inb_buffer Required */
			pr_err("Desc error %d\n", dw0);
			me->read_idx = (me->read_idx + 1) & (me->entries - 1);
			num_proc++;
		} else if ((dw0 & DME_DESC_DW0_DONE) &&
			   (dw0 & DME_DESC_DW0_VALID)) {
			int seg, buf_sz;

			AXXIA_RIO_SYSMEM_BARRIER();
			seg = DME_DESC_DW1_MSGLEN_F(dw1);
			buf_sz = DME_DESC_DW1_MSGLEN_B(seg);
			buf = mb->virt_buffer[letter][me->read_idx];
			if (!buf) {
				dev_err(priv->dev, "Buffer Get Error\n");
				goto err;
			}

			if ((u32)buf & 0xFF) {
				AXXIA_RIO_SYSMEM_BARRIER();
				memcpy(buf, desc->msg_virt, buf_sz);
			}
			mb->virt_buffer[letter][me->read_idx] = NULL;
			*((u32 *)DESC_TABLE_W0_MEM(me, idx)) =
					(dw0 & 0xfe);/*DME_DESC_INVALIDATE*/
			*sz = buf_sz;

			me->read_idx = (me->read_idx + 1) & (me->entries - 1);
			num_proc++;
			goto done;
		} else {
			goto done;
		}
	}

done:
	return buf;
err:
	buf = NULL;
	goto done;
}
EXPORT_SYMBOL(axxia_get_inb_message);

void *axxia_ml_get_inb_message(struct rio_mport *mport, int mbox)
{
	int sz;

	return axxia_get_inb_message(mport, mbox, 0, &sz);
}

void axxia_rio_port_irq_init(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	int i;

	/**
	 * Port general error indications
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->misc_irq.state);
	priv->misc_irq.irq_enab_reg_addr = RAB_INTR_ENAB_MISC;
	priv->misc_irq.irq_state_reg_addr = RAB_INTR_STAT_MISC;
	priv->misc_irq.irq_state_mask = AMST_INT | ASLV_INT |
					LINK_REQ_INT;
	priv->misc_irq.irq_state_mask |= IB_DB_RCV_INT |
					OB_DB_DONE_INT;
	priv->misc_irq.irq_state_mask |= PORT_WRITE_INT;
	priv->misc_irq.irq_state_mask |=
		GRIO_INT | LL_TL_INT |
		UNSP_RIO_REQ_INT | UNEXP_MSG_INT;

	priv->misc_irq.thrd_irq_fn = misc_irq_handler;
	priv->misc_irq.data = NULL;
	priv->misc_irq.release_fn = misc_release_handler;


	/**
	 * Deadman Monitor status interrupt
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->linkdown_irq.state);
	priv->linkdown_irq.irq_enab_reg_addr = 0;
	priv->linkdown_irq.irq_state_reg_addr = RAB_SRDS_STAT1;
	priv->linkdown_irq.irq_state_mask = RAB_SRDS_STAT1_LINKDOWN_INT;
	priv->linkdown_irq.thrd_irq_fn = linkdown_irq_handler;
	priv->linkdown_irq.data = NULL;
	priv->linkdown_irq.release_fn = NULL;

	/**
	 * Outbound messages
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->ob_dme_irq.state);
	priv->ob_dme_irq.irq_enab_reg_addr = RAB_INTR_ENAB_ODME;
	priv->ob_dme_irq.irq_state_reg_addr = RAB_INTR_STAT_ODME;
	priv->ob_dme_irq.irq_state_mask = 0;
/*	priv->ob_dme_irq.thrd_irq_fn = ob_dme_irq_handler;*/
	priv->ob_dme_irq.data = NULL;
	priv->ob_dme_irq.release_fn = release_outb_dme;

	for (i = 0; i < RIO_MAX_TX_MBOX; i++)
		priv->ob_mbox[i] = NULL;

/* Pre-Allocating the Outbound DME Descriptors*/
	i = roundup_pow_of_two(OB_DME_ENTRIES);
	pr_info("RIO: Configuring each outbound DME with %d entries\n", i);
/* MultiSegment DME*/
	for (i = 0; i < priv->num_outb_dmes[0]; i++)
		alloc_ob_dme_shared(priv, &priv->ob_dme_shared[i], i);
/* SingleSegment DME*/
	for (i = priv->num_outb_dmes[0];
		i < priv->num_outb_dmes[0] + priv->num_outb_dmes[1]; i++) {
		alloc_ob_dme_shared(priv, &priv->ob_dme_shared[i], i);
	}

	/**
	 * Inbound messages
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->ib_dme_irq.state);
	priv->ib_dme_irq.irq_enab_reg_addr = RAB_INTR_ENAB_IDME;
	priv->ib_dme_irq.irq_state_reg_addr = RAB_INTR_STAT_IDME;
	priv->ib_dme_irq.irq_state_mask = 0x0;/*IB_DME_INT_EN;*/
	priv->ib_dme_irq.thrd_irq_fn = ib_dme_irq_handler;
	priv->ib_dme_irq.data = NULL;
	priv->ib_dme_irq.release_fn = release_inb_mbox;

	for (i = 0; i < DME_MAX_IB_ENGINES; i++)
		priv->ib_dme[i] = NULL;

	for (i = 0; i < RIO_MAX_RX_MBOX; i++)
		priv->ib_mbox[i] = NULL;
	/**
	 * PIO
	 * Only when debug config
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->apio_irq.state);
/*	priv->apio_irq.mport = mport;*/
	priv->apio_irq.irq_enab_reg_addr = RAB_INTR_ENAB_APIO;
	priv->apio_irq.irq_state_reg_addr = RAB_INTR_STAT_APIO;
	priv->apio_irq.irq_state_mask = APIO_TRANS_FAILED;
#ifdef CONFIG_AXXIA_RIO_STAT
	priv->apio_irq.irq_state_mask |= APIO_TRANS_COMPLETE;
#endif
	priv->apio_irq.thrd_irq_fn = apio_irq_handler;
	priv->apio_irq.data = NULL;
	priv->apio_irq.release_fn = NULL;

	clear_bit(RIO_IRQ_ENABLED, &priv->rpio_irq.state);
	priv->rpio_irq.irq_enab_reg_addr = RAB_INTR_ENAB_RPIO;
	priv->rpio_irq.irq_state_reg_addr = RAB_INTR_STAT_RPIO;
	priv->rpio_irq.irq_state_mask = RPIO_TRANS_FAILED;
#ifdef CONFIG_AXXIA_RIO_STAT
	priv->rpio_irq.irq_state_mask |= RPIO_TRANS_COMPLETE;
#endif
	priv->rpio_irq.irq_state_mask = 0;
	priv->rpio_irq.thrd_irq_fn = rpio_irq_handler;
	priv->rpio_irq.data = NULL;
	priv->rpio_irq.release_fn = NULL;

}

#if defined(CONFIG_RAPIDIO_HOTPLUG)
int axxia_rio_port_notify_cb(struct rio_mport *mport,
			       int enable,
			       void (*cb)(struct rio_mport *mport))
{
	struct rio_priv *priv = mport->priv;
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&priv->port_lock, flags);
	if (enable) {
		if (priv->port_notify_cb)
			rc = -EBUSY;
		else
			priv->port_notify_cb = cb;
	} else {
		if (priv->port_notify_cb != cb)
			rc = -EINVAL;
		else
			priv->port_notify_cb = NULL;
	}
	spin_unlock_irqrestore(&priv->port_lock, flags);

	return rc;
}

int axxia_rio_port_op_state(struct rio_mport *mport)
{
	u32 escsr;

	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &escsr);

	if (escsr & RIO_ESCSR_PO)
		return MPORT_STATE_OPERATIONAL;
	else
		return MPORT_STATE_DOWN;
}
#endif
