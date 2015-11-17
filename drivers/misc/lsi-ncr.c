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

#ifdef CONFIG_ARCH_AXXIA
#define NCA_PHYS_ADDRESS 0x002020100000ULL
#define APB2SER_PHY_PHYS_ADDRESS 0x002010000000ULL
#define SYSCON_PHYS_ADDR 0x002010030000ULL
#else
#define NCA_PHYS_ADDRESS 0x002000520000ULL
#endif

static void __iomem *nca_address;
#ifdef APB2SER_PHY_PHYS_ADDRESS
static void __iomem *apb2ser0_address;
#endif /* APB2SER_PHY_PHYS_ADDRESS */

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

#ifdef AXXIA_NCR_RESET_CHECK
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
#define AXXIA_NCR_RESET_ACTIVE_CHECK()				\
	do { if (ncr_reset_active) BUG(); } while (0)
#else
#define AXXIA_NCR_RESET_ACTIVE_CHECK()
#endif

#define LOCK_DOMAIN 0

union command_data_register_0 {
	unsigned long raw;
	struct {
#ifdef __BIG_ENDIAN
		unsigned long start_done:1;
		unsigned long unused:6;
		unsigned long local_bit:1;
		unsigned long status:2;
		unsigned long byte_swap_enable:1;
		unsigned long cfg_cmpl_int_enable:1;
		unsigned long cmd_type:4;
		unsigned long dbs:16;
#else
		unsigned long dbs:16;
		unsigned long cmd_type:4;
		unsigned long cfg_cmpl_int_enable:1;
		unsigned long byte_swap_enable:1;
		unsigned long status:2;
		unsigned long local_bit:1;
		unsigned long unused:6;
		unsigned long start_done:1;
#endif
	} __packed bits;
} __packed;

union command_data_register_1 {
	unsigned long raw;
	struct {
		unsigned long target_address:32;
	} __packed bits;
} __packed;

union command_data_register_2 {
	unsigned long raw;
	struct {
#ifdef __BIG_ENDIAN
		unsigned long unused:16;
		unsigned long target_node_id:8;
		unsigned long target_id_address_upper:8;
#else
		unsigned long target_id_address_upper:8;
		unsigned long target_node_id:8;
		unsigned long unused:16;
#endif
	} __packed bits;
} __packed;


/*
 * ncr_register_read/write
 *   low-level access functions to Axxia registers,
 *   with checking to ensure device is not currently
 *   held in reset.
 */
unsigned long
ncr_register_read(unsigned *address)
{
	unsigned long value;

	AXXIA_NCR_RESET_ACTIVE_CHECK();
	value = __raw_readl(address);

	return be32_to_cpu(value);
}

void
ncr_register_write(const unsigned value, unsigned *address)
{
	AXXIA_NCR_RESET_ACTIVE_CHECK();
	__raw_writel(cpu_to_be32(value), address);
}

/*
 * ncr_register_read/write_lock
 *   access functions for Axxia NCA block.
 *   These functions protect the register access with a spinlock.
 *   This is needed to workaround an AXM55xx v1.0 h/w bug.
 *
 */
static unsigned long
ncr_register_read_lock(unsigned *address)
{
	unsigned long value, flags;

	raw_spin_lock_irqsave(&nca_access_lock, flags);
	value = ncr_register_read(address);
	raw_spin_unlock_irqrestore(&nca_access_lock, flags);

	return value;
}

static void
ncr_register_write_lock(const unsigned value, unsigned *address)
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
	unsigned long (*rd) (unsigned *address);
	void (*wr) (const unsigned value, unsigned *address);
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


/* These are only needed on platforms there AMP mode of operation is supported
 * (currently only on PowerPC based Axxia platforms). In AMP mode, multiple OS
 * instances may be accessing the NCA registers, thus requiring a hardware
 * based spinlock like this.
 */
#ifdef CONFIG_PPC32
static void
ncr_amp_lock(int domain)
{
	unsigned long offset = (0xff80 + (domain * 4));

	while (ncr_register_read_lock((unsigned *)(nca_address + offset)) != 0)
		cpu_relax();
}

static void
ncr_amp_unlock(int domain)
{
	unsigned long offset = (0xff80 + (domain * 4));

	ncr_register_write_lock(0, (unsigned *)(nca_address + offset));
}
#else
static void ncr_amp_lock(int domain) {}
static void ncr_amp_unlock(int domain) {}
#endif

/**
 * Used to serialize all access to NCA PIO interface.
 */
int
ncr_lock(int domain)
{
	raw_spin_lock_irqsave(&ncr_spin_lock, ncr_spin_flags);
	ncr_amp_lock(domain);
	return 0;
}
EXPORT_SYMBOL(ncr_lock);

/**
 * Used to serialize all access to NCA PIO interface.
 */
void
ncr_unlock(int domain)
{
	ncr_amp_unlock(domain);
	raw_spin_unlock_irqrestore(&ncr_spin_lock, ncr_spin_flags);
}
EXPORT_SYMBOL(ncr_unlock);


/*
  ------------------------------------------------------------------------------
  ncr_pio_error_dump
*/

static void
ncr_pio_error_dump(struct ncr_io_fns *io_fn, char *str)
{
	unsigned long cdr0, cdr1, cdr2;
	unsigned long stat0, stat1;

	cdr0 = io_fn->rd((unsigned *)(nca_address + 0xf0));
	cdr1 = io_fn->rd((unsigned *)(nca_address + 0xf4));
	cdr2 = io_fn->rd((unsigned *)(nca_address + 0xf8));

	stat0 = io_fn->rd((unsigned *)(nca_address + 0xe4));
	stat1 = io_fn->rd((unsigned *)(nca_address + 0xe8));

	pr_err("lsi-ncr: %8s failed, error status : 0x%08lx 0x%08lx\n",
	       str, stat0, stat1);
	pr_err("lsi-ncr:  CDR0-2: 0x%08lx 0x%08lx 0x%08lx\n",
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
		cdr0.raw =
			io_fn->rd((unsigned *)(nca_address + 0xf0));
	} while ((0x1 == cdr0.bits.start_done) &&
		 (time_before(jiffies, timeout)));

	if (0x1 == cdr0.bits.start_done) {
		/* timed out without completing */
		pr_err("lsi-ncr: PIO operation timeout cdr0=0x%08lx!\n",
		       cdr0.raw);
		ncr_pio_error_dump(io_fn, str);
		BUG();
		return -1;
	}

	if (cdr0.raw && (0x3 != cdr0.bits.status)) {
		/* completed with non-success status */
		ncr_pio_error_dump(io_fn, str);
		/* clear CDR0 to allow subsequent commands to complete */
		io_fn->wr(0, (unsigned *) (nca_address + 0xf0));

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

/*
  ======================================================================
  ======================================================================
  Public Interface
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  ncr_read
*/

static int
__ncr_read(struct ncr_io_fns *io_fn,
	unsigned long region, unsigned long address, int number,
	void *buffer)
{
	union command_data_register_0 cdr0;
	union command_data_register_1 cdr1;
	union command_data_register_2 cdr2;

	if ((NCP_NODE_ID(region) != 0x0153) && (NCP_NODE_ID(region) != 0x115)) {
		/* make sure any previous command has completed */
		if (0 != ncr_check_pio_status(io_fn, "previous"))
			return -1;

		/*
		  Set up the read command.
		*/

		cdr2.raw = 0;
		cdr2.bits.target_node_id = NCP_NODE_ID(region);
		cdr2.bits.target_id_address_upper = NCP_TARGET_ID(region);
		io_fn->wr(cdr2.raw, (unsigned *) (nca_address + 0xf8));

		cdr1.raw = 0;
		cdr1.bits.target_address = (address >> 2);
		io_fn->wr(cdr1.raw, (unsigned *) (nca_address + 0xf4));

		cdr0.raw = 0;
		cdr0.bits.start_done = 1;

		if (0xff == cdr2.bits.target_id_address_upper)
			cdr0.bits.local_bit = 1;

		cdr0.bits.cmd_type = 4;
		/* TODO: Verify number... */
		cdr0.bits.dbs = (number - 1);
		io_fn->wr(cdr0.raw, (unsigned *) (nca_address + 0xf0));
		mb();

		/*
		  Wait for completion.
		*/
		if (0 != ncr_check_pio_status(io_fn, "read"))
			return -1;

		/*
		  Copy data words to the buffer.
		*/

		address = (unsigned long)(nca_address + 0x1000);
		while (4 <= number) {
			*((unsigned long *) buffer) =
				io_fn->rd((unsigned *) address);
			address += 4;
			buffer += 4;
			number -= 4;
		}

		if (0 < number) {
			unsigned long temp =
				io_fn->rd((unsigned *) address);
			memcpy((void *) buffer, &temp, number);
		}
	} else {
#ifdef APB2SER_PHY_PHYS_ADDRESS
		int wfc_timeout = WFC_TIMEOUT;

		if (NCP_NODE_ID(region) != 0x115) {
			void __iomem *targ_address = apb2ser0_address +
				(address & (~0x3));
			/*
			 * Copy data words to the buffer.
			 */

			while (4 <= number) {
				*((unsigned long *) buffer) =
					*((unsigned long *) targ_address);
				targ_address += 4;
				number -= 4;
			}
		} else {
			void __iomem *base;
			if (0xffff < address)
				return -1;

			switch (NCP_TARGET_ID(region)) {
			case 0:
				base = (apb2ser0_address + 0x1e0);
				break;
			case 1:
				base = (apb2ser0_address + 0x1f0);
				break;
			case 2:
				base = (apb2ser0_address + 0x200);
				break;
			case 3:
				base = (apb2ser0_address + 0x210);
				break;
			case 4:
				base = (apb2ser0_address + 0x220);
				break;
			case 5:
				base = (apb2ser0_address + 0x230);
				break;
			default:
				return -1;
			}
			if ((NCP_TARGET_ID(region) == 0x1) ||
			    (NCP_TARGET_ID(region) == 0x4)) {
				writel((0x84c00000 + address), (base + 4));
			} else {
				writel((0x85400000 + address), (base + 4));
			}
			do {
				--wfc_timeout;
				*((unsigned long *) buffer) =
					readl(base + 4);
			} while (0 != (*((unsigned long *) buffer) & 0x80000000)
				 && 0 < wfc_timeout);

			if (0 == wfc_timeout)
				return -1;

			if ((NCP_TARGET_ID(region) == 0x1) ||
			    (NCP_TARGET_ID(region) == 0x4)) {
				*((unsigned short *) buffer) =
					readl(base + 8);
			} else {
				*((unsigned long *) buffer) =
					readl(base + 8);
			}

		}
#else
		return -1;
#endif /* APB2SER_PHY_PHYS_ADDRESS */
	}


	return 0;
}


int
ncr_read_nolock(unsigned long region, unsigned long address,
		 int number, void *buffer)
{
	return __ncr_read(&ncr_io_fn_nolock,
		region, address, number, buffer);
}
EXPORT_SYMBOL(ncr_read_nolock);


int
ncr_read(unsigned long region, unsigned long address, int number,
	 void *buffer)
{
	int	rc;

	if (NULL == nca_address)
		return -1;

#ifdef APB2SER_PHY_PHYS_ADDRESS
	if (NULL == apb2ser0_address)
		return -1;
#endif /* APB2SER_PHY_PHYS_ADDRESS */

	ncr_lock(LOCK_DOMAIN);

	/* call __ncr_read with chip version dependent io_fn */
	rc = __ncr_read(default_io_fn, region, address, number, buffer);

	ncr_unlock(LOCK_DOMAIN);

	return rc;
}
EXPORT_SYMBOL(ncr_read);

/*
  ----------------------------------------------------------------------
  ncr_write
*/

static int
__ncr_write(struct ncr_io_fns *io_fn,
	unsigned long region, unsigned long address, int number,
	void *buffer)
{
	union command_data_register_0 cdr0;
	union command_data_register_1 cdr1;
	union command_data_register_2 cdr2;
	unsigned long data_word_base;
	int dbs = (number - 1);

	if ((NCP_NODE_ID(region) != 0x0153) && (NCP_NODE_ID(region) != 0x115)) {
		/* make sure any previous command has completed */
		if (0 != ncr_check_pio_status(io_fn, "previous"))
			return -1;

		/*
		  Set up the write.
		*/

		cdr2.raw = 0;
		cdr2.bits.target_node_id = NCP_NODE_ID(region);
		cdr2.bits.target_id_address_upper = NCP_TARGET_ID(region);
		io_fn->wr(cdr2.raw, (unsigned *) (nca_address + 0xf8));

		cdr1.raw = 0;
		cdr1.bits.target_address = (address >> 2);
		io_fn->wr(cdr1.raw, (unsigned *) (nca_address + 0xf4));

		/*
		  Copy from buffer to the data words.
		*/

		data_word_base = (unsigned long)(nca_address + 0x1000);

		while (4 <= number) {
			io_fn->wr(*((unsigned long *) buffer),
					   (unsigned *) data_word_base);
			data_word_base += 4;
			buffer += 4;
			number -= 4;
		}

		if (0 < number) {
			unsigned long temp = 0;

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
		io_fn->wr(cdr0.raw, (unsigned *) (nca_address + 0xf0));
		mb();

		/*
		  Wait for completion.
		*/

		if (0 != ncr_check_pio_status(io_fn, "write"))
			return -1;

	} else {
#ifdef APB2SER_PHY_PHYS_ADDRESS
		int wfc_timeout = WFC_TIMEOUT;

		if (NCP_NODE_ID(region) != 0x115) {
			void __iomem *targ_address = apb2ser0_address +
				(address & (~0x3));
			/*
			  Copy from buffer to the data words.
			*/

			while (4 <= number) {
				*((unsigned long *) targ_address) =
					*((unsigned long *) buffer);
				targ_address += 4;
				number -= 4;
			}
		} else {
			void __iomem *base;
			if (0xffff < address)
				return -1;

			switch (NCP_TARGET_ID(region)) {
			case 0:
				base = (apb2ser0_address + 0x1e0);
				break;
			case 1:
				base = (apb2ser0_address + 0x1f0);
				break;
			case 2:
				base = (apb2ser0_address + 0x200);
				break;
			case 3:
				base = (apb2ser0_address + 0x210);
				break;
			case 4:
				base = (apb2ser0_address + 0x220);
				break;
			case 5:
				base = (apb2ser0_address + 0x230);
				break;
			default:
				return -1;
			}
			if ((NCP_TARGET_ID(region) == 0x1) ||
			    (NCP_TARGET_ID(region) == 0x4)) {
				writel(*((unsigned short *) buffer), base);
				writel((0xc4c00000 + address), (base + 4));
			} else {
				writel(*((unsigned long *) buffer), base);
				writel((0xc5400000 + address), (base + 4));
			}
			do {
				--wfc_timeout;
				*((unsigned long *) buffer) =
					readl(base + 4);
			} while (0 != (*((unsigned long *) buffer) & 0x80000000)
				 && 0 < wfc_timeout);

			if (0 == wfc_timeout)
				return -1;
		}
#else
		return -1;
#endif /* APB2SER_PHY_PHYS_ADDRESS */
	}

	return 0;
}


int
ncr_write_nolock(unsigned long region, unsigned long address, int number,
	  void *buffer)
{
	/* call the __ncr_write function with nolock io_fn */
	return __ncr_write(&ncr_io_fn_nolock,
			region, address, number, buffer);
}
EXPORT_SYMBOL(ncr_write_nolock);


int
ncr_write(unsigned long region, unsigned long address, int number,
	  void *buffer)
{
	int rc = 0;

	if (NULL == nca_address)
		return -1;

#ifdef APB2SER_PHY_PHYS_ADDRESS
	if (NULL == apb2ser0_address)
		return -1;
#endif /* APB2SER_PHY_PHYS_ADDRESS */

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
  ----------------------------------------------------------------------
  ncr_init
*/

static int
ncr_init(void)
{
#ifdef CONFIG_ARCH_AXXIA
	u32 pfuse;
	u32 chip_type;
	u32 chip_ver;
	void __iomem *syscon;

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);

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
	} else {
		/* no low-level locking needed */
		default_io_fn = &ncr_io_fn_nolock;
	}

	iounmap(syscon);
#else
	/* 3500 doesn't need the lock, 3400 does. */
	if (of_find_compatible_node(NULL, NULL, "lsi,acp3500"))
		default_io_fn = &ncr_io_fn_nolock;
	else
		default_io_fn = &ncr_io_fn_lock;
#endif

	nca_address = ioremap(NCA_PHYS_ADDRESS, 0x20000);

#ifdef APB2SER_PHY_PHYS_ADDRESS
	apb2ser0_address = ioremap(APB2SER_PHY_PHYS_ADDRESS, 0x10000);
#endif /* APB2SER_PHY_PHYS_ADDRESS */

	pr_info("ncr: available\n");

	return 0;
}
core_initcall(ncr_init);

/*
  ----------------------------------------------------------------------
  ncr_exit
*/

static void __exit
ncr_exit(void)
{
	/* Unmap the NCA. */
	if (NULL != nca_address)
		iounmap(nca_address);

#ifdef APB2SER_PHY_PHYS_ADDRESS
	/* Unmap the APB2SER0 PHY. */
	if (NULL != apb2ser0_address)
		iounmap(apb2ser0_address);
#endif /* APB2SER_PHY_PHYS_ADDRESS */
}
__exitcall(ncr_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Register Ring access for LSI's ACP board");
