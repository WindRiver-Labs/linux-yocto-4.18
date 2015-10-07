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

struct lsi_mtc_axi_capt_window_param_t {
	/* TDO Capture Monitor window size in bits;
	 0-disable 1-minitor first bit captured */
	unsigned int	captWindowCnt;
	/* TDO capture bit position to monitor within the window size */
	unsigned int	captWindowMonBit0;
	/* TDO capture bit value to monitor within the window size inverted */
	unsigned int	captWindowMonInv0;
	/* TDO capture bit position to monitor within the window size */
	unsigned int	captWindowMonBit1;
	/* TDO capture bit value to monitor within the window size inverted */
	unsigned int	captWindowMonInv1;
	/* TDO capture bit position to monitor within the window size */
	unsigned int	captWindowMonBit2;
	/* TDO capture bit value to monitor within the window size inverted */
	unsigned int	captWindowMonInv2;
};


struct lsi_mtc_axi_extmem_wm_t {
	unsigned int	highWaterMark;
	unsigned int	lowWaterMark;
};

struct lsi_mtc_axi_master_addr_t {
	unsigned int	strtAddrLow;
	unsigned int	strtAddrHigh;
	unsigned int	stopAddrLow;
	unsigned int	stopAddrHigh;
};

struct lsi_mtc_axi_status_regs_t {
	unsigned int axiStatusReg0;
	unsigned int axiStatusReg1;
	unsigned int axiStatusReg2;
	unsigned int axiStatusReg3;
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
#define MTC_TCKCLK_GATE _IOW(LSI_MTC_IOC_MAGIC, 5,\
				struct lsi_mtc_tckclk_gate_t)


/* start/stop execution */
#define MTC_STARTSTOP_EXEC     _IOW(LSI_MTC_IOC_MAGIC, 6, int)

/* single step execution */
#define MTC_SINGLESTEP_EXEC    _IO(LSI_MTC_IOC_MAGIC, 7)

/* continue after pause execution */
#define MTC_CONTINUE_EXEC      _IO(LSI_MTC_IOC_MAGIC, 8)

/* read stats registers */
#define MTC_READ_STATS  _IOR(LSI_MTC_IOC_MAGIC, 9,\
					 struct lsi_mtc_stats_regs_t)

/* read debug registers */
#define MTC_READ_DEBUG  _IOR(LSI_MTC_IOC_MAGIC, 10,\
					 struct lsi_mtc_debug_regs_t)


/* enable/disable AXI master External Program Memory mode   */
#define MTC_AXI_EXT_PRGM_MEM_ENABLE	_IOW(LSI_MTC_IOC_MAGIC, 11, int)

/* setup external program memory capture window settings */
#define MTC_AXI_CAPT_WINDOW_PARAM_SET	_IOW(LSI_MTC_IOC_MAGIC, 12,\
				 struct lsi_mtc_axi_capt_window_param_t)

/* Get external program memory capture window settings */
#define MTC_AXI_CAPT_WINDOW_PARAM_GET	_IOR(LSI_MTC_IOC_MAGIC, 13,\
			struct lsi_mtc_axi_capt_window_param_t)

/* Setup AXI Master Program FIFO Watermarks */
#define MTC_AXI_WATER_MARK_SET	 _IOW(LSI_MTC_IOC_MAGIC, 14,\
			struct lsi_mtc_axi_extmem_wm_t)

/* Get AXI Master Program FIFO Watermarks */
#define MTC_AXI_WATER_MARK_GET	 _IOR(LSI_MTC_IOC_MAGIC, 15,\
			struct lsi_mtc_axi_extmem_wm_t)

/* Setup AXI Master Read ARPROT Value */
#define MTC_AXI_M_ARPROT_SET	_IOW(LSI_MTC_IOC_MAGIC, 16, int)

/* Get AXI Master Read ARPROT Value */
#define MTC_AXI_M_ARPROT_GET	_IOR(LSI_MTC_IOC_MAGIC, 17, int)

/* Setup AXI Master Start and Stop Addresses. */
#define MTC_AXI_MASTER_ADDR_SET _IOW(LSI_MTC_IOC_MAGIC, 18,\
				 struct lsi_mtc_axi_master_addr_t)

/* Get AXI Master Start and Stop Addresses. */
#define MTC_AXI_MASTER_ADDR_GET _IOR(LSI_MTC_IOC_MAGIC, 19,\
			 struct lsi_mtc_axi_master_addr_t)

/* Read AXI Status registers. */
#define MTC_AXI_READ_STATUS _IOR(LSI_MTC_IOC_MAGIC, 20, \
			struct lsi_mtc_axi_status_regs_t)


#endif    /* __LSI_MTC_IOCTLH*/
