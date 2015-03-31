#ifndef __LSI_MTC_IOCTLH
#define __LSI_MTC_IOCTLH

#include <linux/ioctl.h>

#define LSI_MTC_IOC_MAGIC 0x49

struct lsi_mtc_cfg_t {
	unsigned int   opMode;
	unsigned int   recMode;
	unsigned int   clkMod;
	unsigned int   clkSpeed;
	unsigned int   buffMode;
};



struct lsi_mtc_tckclk_gate_t {
	unsigned int gate_tck_test_logic_reset;
	unsigned int gate_tck;
};

struct lsi_mtc_stats_regs_t {
	unsigned int statsReg1;
	unsigned int statsReg2;
};


struct lsi_mtc_debug_regs_t {
	unsigned int debugReg0;
	unsigned int debugReg1;
	unsigned int debugReg2;
	unsigned int debugReg3;
	unsigned int debugReg4;
	unsigned int debugReg5;
};

/* debug operation */
#define MTC_DEBUG_OP           _IOWR(LSI_MTC_IOC_MAGIC, 0, int)

/* MTC configuration */
#define MTC_CFG                _IOW(LSI_MTC_IOC_MAGIC, 1, struct lsi_mtc_cfg_t)

/* configure enable/disable single step */
#define MTC_SINGLESTEP_ENABLE  _IOW(LSI_MTC_IOC_MAGIC, 2, int)

/*enale/disbale loop mode */
#define MTC_LOOPMODE_ENABLE    _IOW(LSI_MTC_IOC_MAGIC, 3, int)

/* rest MTC */
#define MTC_RESET              _IO(LSI_MTC_IOC_MAGIC, 4)

/* config gate tck clokc */
#define MTC_TCKCLK_GATE _IOW(LSI_MTC_IOC_MAGIC, 5, struct lsi_mtc_tckclk_gate_t)


/* start/stop execution */
#define MTC_STARTSTOP_EXEC     _IOW(LSI_MTC_IOC_MAGIC, 6, int)

/* single step execution */
#define MTC_SINGLESTEP_EXEC    _IO(LSI_MTC_IOC_MAGIC, 7)

/* continue after pause execution */
#define MTC_CONTINUE_EXEC      _IO(LSI_MTC_IOC_MAGIC, 8)

/* read stats registers */
#define MTC_READ_STATS  _IOR(LSI_MTC_IOC_MAGIC, 9, struct lsi_mtc_stats_regs_t)

/* read debug registers */
#define MTC_READ_DEBUG  _IOR(LSI_MTC_IOC_MAGIC, 10, struct lsi_mtc_debug_regs_t)

#endif    /* __LSI_MTC_IOCTLH*/
