/*
 *  Copyright (C) 2009 LSI Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/lsi-ncr.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/sizes.h>

static int ncr_available;
static int nca_big_endian = 1;
static int is_5500;
static int is_5600;
static int is_6700;
static void __iomem *nca;
static void __iomem *apb2ser0;

#define WFC_TIMEOUT (400000)

/*
 * We provide both 'normal' and 'nolock' versions of the
 * ncr_read/write functions. For normal operation we use
 * locking to provide thread-safe operation.
 * There are two levels of locking.
 *
 * 1. ncr_spin_lock -
 *      This is a high-level lock that protects the NCA PIO
 *      registers from concurrent use. The NCA PIO mechanism
 *      only supports a single thread of execution.
 *
 * 2. nca_access_lock -
 *       This is a low-level lock that protects each individual
 *       register read/write to the NCA registers. This is a
 *       workaround for a bug in rev 1.0 silicon where the bus
 *       interface may hang if the NCA is subjected to simultaneous
 *       requests from multiple masters.
 *
 * The 'nolock' versions of ncr_read/write should only be used in
 * special cases where the caller can guarantee there will be no
 * other threads of execution.
 */

/* Lock #1 : Protect NCA PIO registers from concurrent use. */
static DEFINE_RAW_SPINLOCK(ncr_spin_lock);

/* Lock #2 : Protect each individual NCA register access. */
DEFINE_RAW_SPINLOCK(nca_access_lock);
EXPORT_SYMBOL(nca_access_lock);

static unsigned long ncr_spin_flags;

#ifdef CONFIG_ARCH_AXXIA_NCR_RESET_CHECK
/*
 * define behavior if NCA register read/write is called while
 * the axxia device is being reset. Any attempt to access NCA
 * AXI registers while the NCA is in reset will hang the system.
 *
 * Due to higher level locking (ncr_spin_lock) this should not
 * occur as part of normal config ring access (ncr_read/write),
 * so we handle this condition as a BUG(). If it turns out there
 * is some valid case where this may occur we can re-implement
 * this as a wait loop.
 */
int ncr_reset_active;
EXPORT_SYMBOL(ncr_reset_active);

#define AXXIA_NCR_RESET_ACTIVE_CHECK()			\
	do { if (ncr_reset_active) BUG(); } while (0)
#else
#define AXXIA_NCR_RESET_ACTIVE_CHECK()
#endif

#define LOCK_DOMAIN 0

union command_data_register_0 {
	unsigned int raw;
	struct {
#ifdef __BIG_ENDIAN
		unsigned int start_done:1;
		unsigned int unused:6;
		unsigned int local_bit:1;
		unsigned int status:2;
		unsigned int byte_swap_enable:1;
		unsigned int cfg_cmpl_int_enable:1;
		unsigned int cmd_type:4;
		unsigned int dbs:16;
#else
		unsigned int dbs:16;
		unsigned int cmd_type:4;
		unsigned int cfg_cmpl_int_enable:1;
		unsigned int byte_swap_enable:1;
		unsigned int status:2;
		unsigned int local_bit:1;
		unsigned int unused:6;
		unsigned int start_done:1;
#endif
	} __packed bits;
} __packed;

union command_data_register_1 {
	unsigned int raw;
	struct {
		unsigned int target_address:32;
	} __packed bits;
} __packed;

union command_data_register_2 {
	unsigned int raw;
	struct {
#ifdef __BIG_ENDIAN
		unsigned int unused:16;
		unsigned int target_node_id:8;
		unsigned int target_id_address_upper:8;
#else
		unsigned int target_id_address_upper:8;
		unsigned int target_node_id:8;
		unsigned int unused:16;
#endif
	} __packed bits;
} __packed;

static int trace;
module_param(trace, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(trace, "Trace NCR Accesses");

static int trace_value_read;
module_param(trace_value_read, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(trace_value_read, "Trace NCR Value Read");

/*
 * ncr_register_read/write
 *   low-level access functions to Axxia registers,
 *   with checking to ensure device is not currently
 *   held in reset.
 */
unsigned int
ncr_register_read(unsigned int *address)
{
	unsigned int value;

	AXXIA_NCR_RESET_ACTIVE_CHECK();
	value = __raw_readl(address);

	if (0 == nca_big_endian)
		return value;

	return be32_to_cpu(value);
}

void
ncr_register_write(const unsigned int value, unsigned int *address)
{
	AXXIA_NCR_RESET_ACTIVE_CHECK();

	if (0 == nca_big_endian)
		__raw_writel(value, address);

	__raw_writel(cpu_to_be32(value), address);
}

/*
 * ncr_register_read/write_lock
 *   access functions for Axxia NCA block.
 *   These functions protect the register access with a spinlock.
 *   This is needed to workaround an AXM55xx v1.0 h/w bug.
 *
 */
static unsigned int
ncr_register_read_lock(unsigned int *address)
{
	unsigned int value;
	unsigned long flags;

	raw_spin_lock_irqsave(&nca_access_lock, flags);
	value = ncr_register_read(address);
	raw_spin_unlock_irqrestore(&nca_access_lock, flags);

	return value;
}

static void
ncr_register_write_lock(const unsigned value, unsigned int *address)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&nca_access_lock, flags);
	ncr_register_write(value, address);
	raw_spin_unlock_irqrestore(&nca_access_lock, flags);
}

/*
 * define two sets of function pointers for low-level register
 * access - one with locking and one without.
 */
struct ncr_io_fns {
	unsigned int (*rd)(unsigned int *address);
	void (*wr)(const unsigned int value, unsigned int *address);
};

struct ncr_io_fns ncr_io_fn_lock = {
	ncr_register_read_lock,
	ncr_register_write_lock
};

struct ncr_io_fns ncr_io_fn_nolock = {
	ncr_register_read,
	ncr_register_write
};

struct ncr_io_fns *default_io_fn;


/*
  ------------------------------------------------------------------------------
  ncr_lock

  Used to serialize all access to NCA PIO interface.
*/

int
ncr_lock(int domain)
{
	raw_spin_lock_irqsave(&ncr_spin_lock, ncr_spin_flags);

	return 0;
}
EXPORT_SYMBOL(ncr_lock);

/*
  ------------------------------------------------------------------------------
  ncr_unlock

  Used to serialize all access to NCA PIO interface.
*/

void
ncr_unlock(int domain)
{
	raw_spin_unlock_irqrestore(&ncr_spin_lock, ncr_spin_flags);

	return;
}
EXPORT_SYMBOL(ncr_unlock);

/*
  ------------------------------------------------------------------------------
  ncr_pio_error_dump
*/

static void
ncr_pio_error_dump(struct ncr_io_fns *io_fn, char *str)
{
	unsigned int cdr0, cdr1, cdr2;
	unsigned int stat0, stat1;

	cdr0 = io_fn->rd((unsigned int *)(nca + 0xf0));
	cdr1 = io_fn->rd((unsigned int *)(nca + 0xf4));
	cdr2 = io_fn->rd((unsigned int *)(nca + 0xf8));

	stat0 = io_fn->rd((unsigned int *)(nca + 0xe4));
	stat1 = io_fn->rd((unsigned int *)(nca + 0xe8));

	pr_err("lsi-ncr: %8s failed, error status : 0x%08x 0x%08x\n",
	       str, stat0, stat1);
	pr_err("lsi-ncr:  CDR0-2: 0x%08x 0x%08x 0x%08x\n",
	       cdr0, cdr1, cdr2);
}

/*
  ------------------------------------------------------------------------------
  ncr_check_pio_status
*/

static int
ncr_check_pio_status(struct ncr_io_fns *io_fn, char *str)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	union command_data_register_0 cdr0;

	/*
	  Make sure any previous commands completed, and check for errors.
	*/

	do {
		cdr0.raw = io_fn->rd((unsigned int *)(nca + 0xf0));
	} while ((0x1 == cdr0.bits.start_done) &&
		 (time_before(jiffies, timeout)));

	if (0x1 == cdr0.bits.start_done) {
		/* timed out without completing */
		pr_err("lsi-ncr: PIO operation timeout cdr0=0x%08x!\n",
		       cdr0.raw);
		ncr_pio_error_dump(io_fn, str);
		BUG();

		return -1;
	}

	if (cdr0.raw && (0x3 != cdr0.bits.status)) {
		/* completed with non-success status */
		ncr_pio_error_dump(io_fn, str);
		/* clear CDR0 to allow subsequent commands to complete */
		io_fn->wr(0, (unsigned int *) (nca + 0xf0));

		/*
		 * we now treat any config ring error as a BUG().
		 * this should never occur during normal operation with
		 * 'good' system software.
		 *
		 * In the debug/lab environment the config ring errors
		 * can occur more often. If this BUG() becomes too onerous
		 * we may provide a way for the RTE to suppress this BUG()
		 */
		BUG();
		return -1;
	}

	return 0;
}

union ncp_apb2ser_indirect_command {
	unsigned     raw;

	struct {
#ifdef __BIG_ENDIAN
		unsigned      valid                                     :  1;
		unsigned      hwrite                                    :  1;
		unsigned      tshift                                    :  4;
		unsigned      hsize                                     :  3;
		unsigned      htrans                                    :  2;
		unsigned      reserved                                  :  5;
		unsigned      haddr                                     : 16;
#else    /* Little Endian */
		unsigned      haddr                                     : 16;
		unsigned      reserved                                  :  5;
		unsigned      htrans                                    :  2;
		unsigned      hsize                                     :  3;
		unsigned      tshift                                    :  4;
		unsigned      hwrite                                    :  1;
		unsigned      valid                                     :  1;
#endif
	} __packed bits;
} __packed;

/*
  ------------------------------------------------------------------------------
  apb2ser_indirect_setup
*/

static int
apb2ser_indirect_setup(unsigned int region,
		       unsigned int *indirect_offset,
		       unsigned int *transfer_width)
{
	unsigned int node = NCP_NODE_ID(region);
	unsigned int target = NCP_TARGET_ID(region);
	unsigned int base;

	if (is_5600)
		if ((node < 0x110) || (node > 0x11a))
			return -1;

	if (is_6700)
		if ((node < 0x110) || (node > 0x11f))
			return -1;

	if (node <= 0x114) {
		base = (node - 0x110) * 2;
	} else if (node >= 0x116) {
		base = (node - 0x111) * 2;
	} else {
		if (is_5600)
			base = 0x14;
		else
			base = 0x1e;
	}

	*indirect_offset = ((base + target) * 0x10000);
	*transfer_width = (target > 0) ? 2 : 4;
	udelay(10);

	return 0;
}

/*
 * ------------------------------------------------------------------------------
 * apb2ser_indirect_access
 */

static int
apb2ser_indirect_access(unsigned int offset,
			unsigned int indirect_offset,
			unsigned int transfer_width,
			int write,
			unsigned int *value)
{
	union ncp_apb2ser_indirect_command indcmd;
	unsigned wfc;

	memset(&indcmd, 0, sizeof(union ncp_apb2ser_indirect_command));
	indcmd.bits.valid = 1;
	indcmd.bits.hwrite = (0 == write) ? 0 : 1;
	indcmd.bits.tshift = 0xf;
	indcmd.bits.htrans = 2;
	indcmd.bits.hsize = 2;
	indcmd.bits.haddr = offset;

	if (0 != write)
		writel(*value, (apb2ser0 + indirect_offset));

	pr_debug("ncr: indcmd.raw=0x%x\n", indcmd.raw);
	writel(indcmd.raw, (apb2ser0 + indirect_offset + 4));
	wfc = WFC_TIMEOUT;

	do {
		--wfc;
		indcmd.raw = readl(apb2ser0 + indirect_offset + 4);
	} while (1 == indcmd.bits.valid && 0 < wfc);

	if (0 == wfc) {
		pr_err("APB2SER Timeout!\n");

		return -1;
	}

	if (0 == write)
		*value = readl(apb2ser0 + indirect_offset + 8);

	return 0;
}

/*
 * ------------------------------------------------------------------------------
 * ncr_apb2ser
 */

static int
ncr_apb2ser(unsigned int region,
	    unsigned int offset,
	    int write,
	    unsigned int *value)
{
	int rc;
	unsigned int indirect_offset;
	unsigned int transfer_width;

	rc = apb2ser_indirect_setup(region, &indirect_offset, &transfer_width);

	if (0 != rc) {
		pr_err("APB2SER Indirect Setup Failed!\n");

		return -1;
	}

	rc = apb2ser_indirect_access(offset, indirect_offset, transfer_width,
				     write, value);

	if (0 != rc) {
		pr_err("APB2SER Indirect Setup Failed!\n");

		return -1;
	}

	return 0;
}

union ncp_cobalt_serdes_ctrl98 {
	unsigned short raw;

	struct {
#ifdef __BIG_ENDIAN
	unsigned short reserved_b53 : 13;
	unsigned short cr_ack_clear :  1;
	unsigned short cr_rd        :  1;
	unsigned short cr_wr        :  1;
#else    /* Little Endian */
	unsigned short cr_wr        :  1;
	unsigned short cr_rd        :  1;
	unsigned short cr_ack_clear :  1;
	unsigned short reserved_b53 : 13;
#endif
	} __packed bits;
} __packed;


union ncp_cobalt_serdes_ctrl99 {
	unsigned short raw;

	struct {
#ifdef __BIG_ENDIAN
		unsigned short reserved : 15;
		unsigned short cr_ack   :  1;
#else    /* Little Endian */
		unsigned short cr_ack   :  1;
		unsigned short reserved : 15;
#endif
	} __packed bits;
} __packed;

/*
 * ------------------------------------------------------------------------------
 * ncr_apb2ser_e12
 */

static int
ncr_apb2ser_e12(unsigned int region,
		unsigned int offset,
		int write,
		unsigned int *value)
{
	unsigned int indirect_offset;
	unsigned int transfer_width;
	union ncp_cobalt_serdes_ctrl98 hss_cobalt_ctrl_98 = {0};
	union ncp_cobalt_serdes_ctrl99 hss_cobalt_ctrl_99 = {0};
	unsigned short e12_addr = 0;
	unsigned int ctrl_96_off;
	unsigned int ctrl_97_off;
	unsigned int ctrl_98_off;
	unsigned int ctrl_99_off;
	unsigned int ctrl_224_off;

	if (0 !=
	    apb2ser_indirect_setup(region, &indirect_offset, &transfer_width))
		return -1;

	if (is_5600) {
		ctrl_96_off = 0x00c0;
		ctrl_97_off = 0x00c2;
		ctrl_98_off = 0x00c4;
		ctrl_99_off = 0x00c6;
		ctrl_224_off = 0x01c0;
	} else {
		ctrl_96_off = 0x0180;
		ctrl_97_off = 0x0184;
		ctrl_98_off = 0x0188;
		ctrl_99_off = 0x018c;
		ctrl_224_off = 0x0380;
		offset >>= 1;
	}

	if ((offset >= 0x1000) && (offset <= 0x10d0))
		e12_addr = (offset - 0x1000) / 2;
	else if (offset >= 0x2000)
		e12_addr = offset / 2;

	apb2ser_indirect_access(ctrl_96_off, indirect_offset, 4,
				1, (unsigned int *)&e12_addr);

	if (write) {
		apb2ser_indirect_access(ctrl_97_off, indirect_offset, 4, 1,
					value);
		hss_cobalt_ctrl_98.bits.cr_rd = 0; /* bus read strobe */
		hss_cobalt_ctrl_98.bits.cr_wr = 1;
	} else  {
		hss_cobalt_ctrl_98.bits.cr_rd = 1; /* bus read strobe */
		hss_cobalt_ctrl_98.bits.cr_wr = 0;
	}

	hss_cobalt_ctrl_98.bits.cr_ack_clear = 0;
	apb2ser_indirect_access(ctrl_98_off, indirect_offset, 4, 1,
				(unsigned int *)&hss_cobalt_ctrl_98.raw);

	/* poll for cr_ack to get set */
	do {
		apb2ser_indirect_access(ctrl_99_off, indirect_offset, 4, 0,
					(unsigned int *)
					&hss_cobalt_ctrl_99.raw);
	} while (0 == hss_cobalt_ctrl_99.bits.cr_ack);

	hss_cobalt_ctrl_98.bits.cr_rd = 0;
	hss_cobalt_ctrl_98.bits.cr_wr = 0;
	hss_cobalt_ctrl_98.bits.cr_ack_clear = 1;
	apb2ser_indirect_access(ctrl_98_off, indirect_offset, 4, 1,
				(unsigned int *)&hss_cobalt_ctrl_98.raw);

	hss_cobalt_ctrl_98.bits.cr_ack_clear = 0;
	apb2ser_indirect_access(ctrl_98_off, indirect_offset, 4, 1,
				(unsigned int *)&hss_cobalt_ctrl_98.raw);

	if (!write)
		apb2ser_indirect_access(ctrl_224_off, indirect_offset, 4, 0,
					value);

	return 0;
}

/*
  ------------------------------------------------------------------------------
  ncr_0x115_5500
*/

static int
ncr_0x115_5500(unsigned int region, unsigned int offset, int write,
	       unsigned int *value)
{
	unsigned int control;
	void __iomem *base;
	unsigned wfc_timeout = 400000;

	if (0xffff < offset)
		return -1;

	switch (NCP_TARGET_ID(region)) {
	case 0:
		base = (apb2ser0 + 0x1e0);
		break;
	case 1:
		base = (apb2ser0 + 0x1f0);
		break;
	case 2:
		base = (apb2ser0 + 0x200);
		break;
	case 3:
		base = (apb2ser0 + 0x210);
		break;
	case 4:
		base = (apb2ser0 + 0x220);
		break;
	case 5:
		base = (apb2ser0 + 0x230);
		break;
	default:
		return -1;
	}

	if ((NCP_TARGET_ID(region) == 0x1) ||
	    (NCP_TARGET_ID(region) == 0x4))
		control = 0x84c00000;
	else
		control = 0x85400000;

	if (0 != write)
		control |= 0x40000000;

	writel((control + offset), (base + 4));

	do {
		--wfc_timeout;
		*((unsigned long *)value) = readl(base + 4);
	} while (0 != (*((unsigned long *)value) & 0x80000000) &&
		 0 < wfc_timeout);

	if (0 == wfc_timeout)
		return -1;

	if (0 == write) {
		if ((NCP_TARGET_ID(region) == 0x1) ||
		    (NCP_TARGET_ID(region) == 0x4)) {
			*((unsigned short *)value) = readl(base + 8);
		} else {
			*((unsigned long *)value) = readl(base + 8);
		}
	}

	return 0;
}

/*
  ------------------------------------------------------------------------------
  ncr_axi2ser
*/

static int
ncr_axi2ser(unsigned int region, unsigned int offset, int write,
	    unsigned int *value)
{
	unsigned int *address;

	address = apb2ser0;

	switch (NCP_NODE_ID(region)) {
	case 0x153:
		if (0 != is_5500) {
			address += (offset & (~0x3));

			/*
			 * Copy from buffer to the data words.
			 */

			if (0 != write)
				*((unsigned long *)address) =
					*((unsigned long *)value);
			else
				*((unsigned long *)value) =
					*((unsigned long *)address);
		}
		break;
	case 0x155:
		address += 0x800000;
		break;
	case 0x156:
		address += 0xc00000;
		break;
	case 0x165:
		address += 0x1400000;
		break;
	case 0x167:
		address += 0x1c00000;
		break;
	default:
		BUG();
		break;
	}

	if (0x156 == NCP_NODE_ID(region))
		address += NCP_TARGET_ID(region) * 0x4000;
	else
		address += NCP_TARGET_ID(region) * 0x10000;

	address += offset;

	if (0 == write)
		*value = readl(address);
	else
		writel(*value, address);

	return 0;
}

/*
  ======================================================================
  ======================================================================
  Public Interface
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  __ncr_read
*/

static int
__ncr_read(struct ncr_io_fns *io_fn,
	   unsigned int region, unsigned long address, int number,
	   void *buffer)
{
	union command_data_register_0 cdr0;
	union command_data_register_1 cdr1;
	union command_data_register_2 cdr2;
	unsigned char *input = buffer;

	if (0 == ncr_available)
		return -1;

	pr_debug("%s:%d - region=0x%x node=0x%x target=0x%x\n",
	      __FILE__, __LINE__,
	      region, NCP_NODE_ID(region), NCP_TARGET_ID(region));

	if (0x110 <= NCP_NODE_ID(region) &&
	    0x11f >= NCP_NODE_ID(region)) {
		int rc;

		if (is_5500) {
			rc = ncr_0x115_5500(region, address, 0, buffer);
		} else if ((NCP_TARGET_ID(region) != 0) &&
			   (address >= 0x1000)) {
			rc = ncr_apb2ser_e12(region, address, 0, buffer);
		} else {
			rc = ncr_apb2ser(region, address, 0, buffer);
		}

		if (0 != rc)
			return -1;
	} else if (0x153 == NCP_NODE_ID(region) ||
		   0x155 == NCP_NODE_ID(region) ||
		   0x156 == NCP_NODE_ID(region) ||
		   0x165 == NCP_NODE_ID(region) ||
		   0x167 == NCP_NODE_ID(region)) {
		if (0 != ncr_axi2ser(region, address, 0, buffer))
			return -1;
	} else if (0x100 > NCP_NODE_ID(region)) {
		/* make sure any previous command has completed */
		if (0 != ncr_check_pio_status(io_fn, "previous"))
			return -1;

		/*
		  Set up the read command.
		*/

		cdr2.raw = 0;
		cdr2.bits.target_node_id = NCP_NODE_ID(region);
		cdr2.bits.target_id_address_upper = NCP_TARGET_ID(region);
		io_fn->wr(cdr2.raw, (unsigned int *) (nca + 0xf8));

		cdr1.raw = 0;
		cdr1.bits.target_address = (address >> 2);
		io_fn->wr(cdr1.raw, (unsigned int *) (nca + 0xf4));

		cdr0.raw = 0;
		cdr0.bits.start_done = 1;

		if (0xff == cdr2.bits.target_id_address_upper)
			cdr0.bits.local_bit = 1;

		cdr0.bits.cmd_type = 4;
		/* TODO: Verify number... */
		cdr0.bits.dbs = (number - 1);
		io_fn->wr(cdr0.raw, (unsigned int *) (nca + 0xf0));
		mb();

		/*
		  Wait for completion.
		*/
		if (0 != ncr_check_pio_status(io_fn, "read"))
			return -1;

		/*
_		  Copy data words to the buffer.
		*/

		address = (unsigned long)(nca + 0x1000);
		while (4 <= number) {
			*((unsigned int *) buffer) =
				io_fn->rd((unsigned int *) address);
			address += 4;
			buffer += 4;
			number -= 4;
		}

		if (0 < number) {
			unsigned int temp =
				io_fn->rd((unsigned int *) address);
			memcpy((void *) buffer, &temp, number);
		}
	} else {
		printk(KERN_ERR "Unhandled Region (r): 0x%x 0x%x 0%x 0x%lx\n",
		       region, NCP_NODE_ID(region), NCP_TARGET_ID(region),
		       address);

		return -1;
	}

	if (0 != trace) {
		printk("ncpRead");

		switch (number) {
		case 1:
			printk("   -w8 0.");
			break;
		case 2:
			printk("  -w16 0.");
			break;
		case 4:
			printk("       0.");
			break;
		default:
			break;
		}

		printk("%u.%u.0x00%08lx 1",
		       NCP_NODE_ID(region),
		       NCP_TARGET_ID(region),
		       address);

		if (0 != trace_value_read) {
			switch (number) {
			case 1:
				printk(" [0x%02x]\n",
				       *((unsigned char *)input));
				break;
			case 2:
				printk(" [0x%04x]\n",
				       *((unsigned short *)input));
				break;
			case 4:
				printk(" [0x%08x]\n",
				       *((unsigned int *)input));
				break;
			default:
				break;
			}
		} else {
			printk("\n");
		}
	}

	return 0;
}

/*
  ------------------------------------------------------------------------------
  ncr_read_nolock
*/

int
ncr_read_nolock(unsigned int region, unsigned int address,
		int number, void *buffer)
{
	if (0 == ncr_available)
		return -1;

	return __ncr_read(&ncr_io_fn_nolock, region, address, number, buffer);
}
EXPORT_SYMBOL(ncr_read_nolock);

/*
  ------------------------------------------------------------------------------
  ncr_read
*/

int
ncr_read(unsigned int region, unsigned int address, int number, void *buffer)
{
	int rc;

	if (0 == ncr_available)
		return -1;

	ncr_lock(LOCK_DOMAIN);

	/* call __ncr_read with chip version dependent io_fn */
	rc = __ncr_read(default_io_fn, region, address, number, buffer);

	ncr_unlock(LOCK_DOMAIN);

	return rc;
}

EXPORT_SYMBOL(ncr_read);

/*
  ------------------------------------------------------------------------------
  ncr_read32
*/

int
ncr_read32(unsigned int region, unsigned int offset, unsigned int *value)
{
	unsigned int val;
	int rc;

	rc = ncr_read(region, offset, 4, &val);
	pr_debug("%s:%d - read 0x%x from 0x%x.0x%x.0x%x rc=%d\n",
		 __FILE__, __LINE__, val,
		 NCP_NODE_ID(region), NCP_TARGET_ID(region), offset, rc);
	*value = val;

	return rc;
}

EXPORT_SYMBOL(ncr_read32);

/*
  ----------------------------------------------------------------------
  ncr_write
*/

static int
__ncr_write(struct ncr_io_fns *io_fn,
	    unsigned int region, unsigned int address, int number,
	    void *buffer)
{
	union command_data_register_0 cdr0;
	union command_data_register_1 cdr1;
	union command_data_register_2 cdr2;
	unsigned long data_word_base;
	int dbs = (number - 1);

	if (0 == ncr_available)
		return -1;

	if (0 != trace) {
		printk("ncpWrite");

		switch (number) {
		case 1:
			printk("  -w8 0.");
			break;
		case 2:
			printk(" -w16 0.");
			break;
		case 4:
			printk("      0.");
			break;
		default:
			break;
		}

		printk("%u.%u.0x00%08x",
		       NCP_NODE_ID(region),
		       NCP_TARGET_ID(region),
		       address);

		switch (number) {
		case 1:
			printk(" 0x%02x\n", *((unsigned char *)buffer));
			break;
		case 2:
			printk(" 0x%04x\n", *((unsigned short *)buffer));
			break;
		case 4:
			printk(" 0x%08x\n", *((unsigned int *)buffer));
			break;
		default:
			break;
		}
	}

	if (0x110 <= NCP_NODE_ID(region) &&
	    0x11f >= NCP_NODE_ID(region)) {
		int rc;

		if (is_5500) {
			rc = ncr_0x115_5500(region, address, 1, buffer);
		} else if ((NCP_TARGET_ID(region) != 0) &&
			   (address >= 0x1000)) {
			rc = ncr_apb2ser_e12(region, address, 1, buffer);
		} else {
			rc = ncr_apb2ser(region, address, 1, buffer);
		}

		if (0 != rc)
			return -1;
	} else if (0x153 == NCP_NODE_ID(region) ||
		   0x155 == NCP_NODE_ID(region) ||
		   0x156 == NCP_NODE_ID(region) ||
		   0x165 == NCP_NODE_ID(region) ||
		   0x167 == NCP_NODE_ID(region)) {
		if (0 != ncr_axi2ser(region, address, 1, buffer))
			return -1;
	} else if (0x100 > NCP_NODE_ID(region)) {
		/* make sure any previous command has completed */
		if (0 != ncr_check_pio_status(io_fn, "previous"))
			return -1;

		/*
		  Set up the write.
		*/

		cdr2.raw = 0;
		cdr2.bits.target_node_id = NCP_NODE_ID(region);
		cdr2.bits.target_id_address_upper = NCP_TARGET_ID(region);
		io_fn->wr(cdr2.raw, (unsigned *) (nca + 0xf8));

		cdr1.raw = 0;
		cdr1.bits.target_address = (address >> 2);
		io_fn->wr(cdr1.raw, (unsigned *) (nca + 0xf4));

		/*
		  Copy from buffer to the data words.
		*/

		data_word_base = (unsigned long)(nca + 0x1000);

		while (4 <= number) {
			io_fn->wr(*((unsigned int *)buffer),
				  (unsigned int *)data_word_base);
			data_word_base += 4;
			buffer += 4;
			number -= 4;
		}

		if (0 < number) {
			unsigned int temp = 0;

			memcpy((void *) &temp, (void *) buffer, number);
			io_fn->wr(temp, (unsigned *) data_word_base);
			data_word_base += number;
			buffer += number;
			number = 0;
		}

		cdr0.raw = 0;
		cdr0.bits.start_done = 1;

		if (0xff == cdr2.bits.target_id_address_upper)
			cdr0.bits.local_bit = 1;

		cdr0.bits.cmd_type = 5;
		/* TODO: Verify number... */
		cdr0.bits.dbs = dbs;
		io_fn->wr(cdr0.raw, (unsigned *) (nca + 0xf0));
		mb();

		/*
		  Wait for completion.
		*/

		if (0 != ncr_check_pio_status(io_fn, "write"))
			return -1;
	} else {
		printk(KERN_ERR "Unhandled Region (w): 0x%x 0x%x 0x%x 0x%x\n",
		       region, NCP_NODE_ID(region), NCP_TARGET_ID(region),
		       address);

		return -1;
	}

	return 0;
}

int
ncr_write_nolock(unsigned int region, unsigned int address, int number,
		 void *buffer)
{
	if (0 == ncr_available)
		return -1;

	/* call the __ncr_write function with nolock io_fn */
	return __ncr_write(&ncr_io_fn_nolock,
			   region, address, number, buffer);
}
EXPORT_SYMBOL(ncr_write_nolock);

int
ncr_write(unsigned int region, unsigned int address, int number,
	  void *buffer)
{
	int rc = 0;

	if (0 == ncr_available)
		return -1;

	/* grab the ncr_lock */
	ncr_lock(LOCK_DOMAIN);

	/* call the __ncr_write function with chip-version dependent io_fn */
	rc = __ncr_write(default_io_fn, region, address, number, buffer);

	/* free the ncr_lock */
	ncr_unlock(LOCK_DOMAIN);

	return rc;
}
EXPORT_SYMBOL(ncr_write);

/*
  ------------------------------------------------------------------------------
  ncr_write32
*/

int
ncr_write32(unsigned int region, unsigned int offset, unsigned int value)
{
	int rc;

	rc = ncr_write(region, offset, 4, &value);
	pr_debug("%s:%d - wrote 0x%x to 0x%x.0x%x.0x%x rc=%d\n",
		 __FILE__, __LINE__, value,
		 NCP_NODE_ID(region), NCP_TARGET_ID(region), offset, rc);

	return rc;
}

EXPORT_SYMBOL(ncr_write32);

/*
  ------------------------------------------------------------------------------
  ncr_start_trace
*/

void
ncr_start_trace(void)
{
	trace = 1;
}
EXPORT_SYMBOL(ncr_start_trace);

/*
 * ------------------------------------------------------------------------------
 * ncr_stop_trace
 */

void
ncr_stop_trace(void)
{
	trace = 0;
}
EXPORT_SYMBOL(ncr_stop_trace);

/*
 * ------------------------------------------------------------------------------
 * ncr_init
 */

static int
ncr_init(void)
{
#ifdef CONFIG_ARCH_AXXIA
	default_io_fn = &ncr_io_fn_nolock;


	if (of_find_compatible_node(NULL, NULL, "lsi,axm5500-amarillo")) {
		u32 pfuse;
		u32 chip_type;
		u32 chip_ver;
		void __iomem *syscon;

		syscon = ioremap(0x002010030000ULL, SZ_64K);

		if (WARN_ON(!syscon))
			return -ENODEV;

		/*
		 * read chip type/revision to determine if low-level locking
		 * is required and select the appropriate io_fns.
		 */

		pfuse = readl(syscon + 0x34);
		chip_type = pfuse & 0x1f;
		chip_ver  = (pfuse >> 8) & 0x7;

		if ((chip_type == 0 || chip_type == 9) && (chip_ver == 0)) {
			/* AXM5516v1.0 needs low-level locking */
			default_io_fn = &ncr_io_fn_lock;
			pr_debug("Using NCA lock functions (AXM5500 v1.0)\n");
		}

		iounmap(syscon);
	}

	if (of_find_compatible_node(NULL, NULL, "lsi,axm5500") ||
	    of_find_compatible_node(NULL, NULL, "lsi,axm5516")) {
		pr_debug("Using AXM5500 Addresses\n");
		nca = ioremap(0x002020100000ULL, 0x20000);
		apb2ser0 = ioremap(0x002010000000ULL, 0x10000);
		is_5500 = 1;
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axm5616")) {
		pr_debug("Using AXM5600 Addresses\n");
		nca = ioremap(0x8031080000ULL, 0x20000);
		apb2ser0 = ioremap(0x8002000000ULL, 0x4000000);
		is_5600 = 1;
		pr_debug("0x%lx 0x%lx\n",
			 (unsigned long)nca,
			 (unsigned long)apb2ser0);
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		pr_debug("Using AXC6700 Addresses\n");
		nca = ioremap(0x8020000000ULL, 0x20000);
		apb2ser0 = ioremap(0x8002000000ULL, 0x400000);
		is_6700 = 1;
		nca_big_endian = 0; /* The 6700 NCA is LE */
	} else {
		pr_err("No Valid Compatible String Found for NCR!\n");
		return -1;
	}
#else
	if (of_find_compatible_node(NULL, NULL, "lsi,acp3500")) {
		pr_debug("Using ACP3500 Addresses\n");
		nca = ioremap(0x002000520000ULL, 0x20000);
		default_io_fn = &ncr_io_fn_nolock;
	} else {
		pr_debug("Using ACP34xx Addresses\n");
		nca = ioremap(0x002000520000ULL, 0x20000);
		default_io_fn = &ncr_io_fn_lock;
	}
#endif

	pr_info("ncr: available\n");
	ncr_available = 1;

	return 0;
}

arch_initcall(ncr_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Register Ring access for Axxia");
