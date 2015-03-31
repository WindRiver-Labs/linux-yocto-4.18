
#include <asm/io.h>

/*
 * private copies of the ioread/write macros
 * These are defined with a different barrier
 * to avoid the outer_sync() call that's part
 * of the normal barrier.
 */
#define pvt_ioread32be(p)   ({ unsigned int __v = be32_to_cpu((__force __be32)__raw_readl(p)); dsb(); __v; })
#define pvt_iowrite32be(v, p) ({ dsb(); __raw_writel((__force __u32)cpu_to_be32(v), p); })

#define pvt_ioread32(p)   ({ unsigned int __v = (__raw_readl(p)); dsb(); __v; })
#define pvt_iowrite32(v, p) ({ dsb(); __raw_writel((__force __u32)(v), p); })

/* #define DDR_SHUTDOWN_DEBUG  */
#ifdef DDR_SHUTDOWN_DEBUG
#define dbg_write(v, p) pvt_iowrite32be(v, p)
#else
#define dbg_write(v, p)
#endif

/*
 * Wait For Completion timeout
 * how many loops to wait for the config ring access to complete
 */
#define WFC_TIMEOUT (400000)

/*
 * DDR status timeout
 * how many times we read the DDR status waiting for self refresh complete
 */
#define DDR_TIMEOUT (1000)

void ncp_ddr_shutdown(void *nca, void *apb, unsigned long ctl_244)
{
	unsigned long value;
	int two_elms = 0;
	int wfc_loop = 0;
	int ddr_loop = 0;

	/* determine if we are in one or two ELM/SMEM mode */
	value = pvt_ioread32(apb + 0x60004);
	two_elms = (value & 0x00000200);

	/*
	 * Issue command to put SMEM0 into self-refresh mode
	 *
	 * ncpWrite 0x22.0.0x3d0
	 */
	dbg_write(0xaaaa0001, (unsigned *)(nca + 0x1200));

	/* write register value into CDAR[0] */
	pvt_iowrite32be(ctl_244, (unsigned *)(nca + 0x1000));
	/* CDR2 - Node.target */
	pvt_iowrite32be(0x00002200, (unsigned *)(nca + 0xf8));
	/* CDR1 - word offset 0xf4 (byte offset 0x3d0) */
	pvt_iowrite32be(0x000000f4, (unsigned *)(nca + 0xf4));
	/* CDR0 - write command */
	pvt_iowrite32be(0x80050003, (unsigned *)(nca + 0xf0));
	wfc_loop = 0;
	do {
		if (wfc_loop++ > WFC_TIMEOUT) {
			dbg_write(value, (unsigned *)(nca + 0x11fc));
			dbg_write(0xffff0001, (unsigned *)(nca + 0x1200));
			goto do_reset;
		}
		dbg_write(wfc_loop, (unsigned *)(nca + 0x11f8));
		value = pvt_ioread32be((unsigned *)
				       (nca + 0xf0));
	} while ((0x80000000UL & value));
	dbg_write(0xaaaa0002, (unsigned *)(nca + 0x1200));

	if (two_elms) {
		/*
		 * Issue command to put SMEM1 into self-refresh mode
		 *
		 * ncpWrite 0x0f.0.0x3d0
		 */
		/* CDR2 - Node.target */
		pvt_iowrite32be(0x00000f00, (unsigned *)(nca + 0xf8));
		/* CDR0 - write command */
		pvt_iowrite32be(0x80050003, (unsigned *)(nca + 0xf0));
		wfc_loop = 0;
		do {
			if (wfc_loop++ > WFC_TIMEOUT) {
				dbg_write(value, (unsigned *)(nca + 0x11fc));
				dbg_write(0xffff0002,
					  (unsigned *)(nca + 0x1200));
				goto do_reset;
			}
			value = pvt_ioread32be((unsigned *)
					       (nca + 0xf0));
		} while ((0x80000000UL & value));
	}

	dbg_write(0xaaaa0003, (unsigned *)(nca + 0x1200));

	/*
	 * Poll for SMEM0 refresh-mode command completion
	 */
	/* CDR1 - word offset 0x104 (byte offset 0x410) */
	pvt_iowrite32be(0x00000104, (unsigned *)(nca + 0xf4));
	/* CDR2 - Node.target */
	pvt_iowrite32be(0x00002200, (unsigned *)(nca + 0xf8));
	ddr_loop = 0;
	do {
		if (ddr_loop++ > DDR_TIMEOUT) {
			dbg_write(value, (unsigned *)(nca + 0x11fc));
			dbg_write(0xffff0003, (unsigned *)(nca + 0x1200));
			goto do_reset;
		}
		pvt_iowrite32be(wfc_loop, (unsigned *)
				(nca + 0x11f0));

		/* issue config ring read */
		pvt_iowrite32be(0x80040003, (unsigned *)
				(nca + 0xf0));
		wfc_loop = 0;
		do {
			if (wfc_loop++ > WFC_TIMEOUT) {
				dbg_write(value, (unsigned *)(nca + 0x11fc));
				dbg_write(0xffff0004,
					  (unsigned *)(nca + 0x1200));
				goto do_reset;
			}
			value = pvt_ioread32be((unsigned *)
					       (nca + 0xf0));
		} while ((0x80000000UL & value));

		value = pvt_ioread32be((unsigned *)
				       (nca + 0x1000));

	} while ((value & 0x0200) == 0);
	dbg_write(0xaaaa0004, (unsigned *)(nca + 0x1200));

	if (two_elms) {
		/*
		 * Poll for SMEM1 refresh-mode command completion
		 */
		/* CDR2 - Node.target */
		pvt_iowrite32be(0x00000f00, (unsigned *)(nca + 0xf8));
		ddr_loop = 0;
		do {
			if (ddr_loop++ > DDR_TIMEOUT) {
				dbg_write(value, (unsigned *)(nca + 0x11fc));
				dbg_write(0xffff0005,
					  (unsigned *)(nca + 0x1200));
				goto do_reset;
			}

			/* issue config ring read */
			pvt_iowrite32be(0x80040003, (unsigned *)(nca + 0xf0));
			wfc_loop = 0;
			do {
				if (wfc_loop++ > WFC_TIMEOUT) {
					dbg_write(value,
						  (unsigned *)(nca + 0x11fc));
					dbg_write(0xffff0006,
						  (unsigned *)(nca + 0x1200));
					goto do_reset;
				}
				value =
				    pvt_ioread32be((unsigned *)(nca + 0xf0));
			} while ((0x80000000UL & value));

			value = pvt_ioread32be((unsigned *)
					       (nca + 0x1000));
			wfc_loop++;
		} while ((value & 0x0200) == 0);
	}

	dbg_write(0xaaaa0005, (unsigned *)(nca + 0x1200));

	/*
	 * Tell U-Boot to do a DDR retention-reset
	 * (i.e. set bit 0 of persist_scratch register)
	 */
	pvt_iowrite32(0x00000001, apb + 0x300dc);

	dbg_write(0xaaaa0006, (unsigned *)(nca + 0x1200));
do_reset:
	/*
	 * Issue Chip reset
	 */
	/* Intrnl Boot, 0xffff0000 Target */
	pvt_iowrite32(0x00000040, apb + 0x31004);
	/* Set ResetReadDone */
	pvt_iowrite32(0x80000000, apb + 0x3180c);
	/* Chip Reset */
	pvt_iowrite32(0x00080802, apb + 0x31008);

	while (1)
		;
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
}

void ncp_ddr_shutdown_dummy(void)
{
	wfi();
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
	__asm__ __volatile__("nop\n\t");
}
