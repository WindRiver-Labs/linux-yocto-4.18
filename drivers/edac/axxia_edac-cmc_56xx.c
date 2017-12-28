/*
 * drivers/edac/axxia_edac-mc.c
 *
 * EDAC Driver for Intel's Axxia 5600 Configuration Memory Controller
 *
 * Copyright (C) 2016 Intel Inc.
 *
 * This file may be distributed under the terms of the
 * GNU General Public License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/lsi-ncr.h>
#include <linux/edac.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include "edac_module.h"
#include "axxia_edac.h"

#define FMT "%s: syscon lookup failed hence using hardcoded register address\n"

#define MPR_FMT2 "\n%3d %#010x %#010x"
#define MPR_FMT4 " %#010x %#010x"

#define MPR_HDR2	"Lp.    dram0      dram1"
#define MPR_HDR4	"      dram2      dram3"

#if defined(CONFIG_EDAC_AXXIA_CMEM_5600)
#define INTEL_EDAC_MOD_STR     "axxia56xx_edac"
#endif

#if defined(CONFIG_EDAC_AXXIA_CMEM_6700)
#define INTEL_EDAC_MOD_STR     "axxia67xx_edac"
#endif

#define AXI2_SER3_PHY_ADDR	0x008002c00000ULL
#define AXI2_SER3_PHY_SIZE	PAGE_SIZE

#define CM_MPR_PAGE		0x1

#define CM_56XX_DENALI_CTL_00	0x0
#define CM_56XX_DENALI_CTL_34	0x88
#define CM_56XX_DENALI_CTL_74	0x128
#define CM_56XX_DENALI_CTL_80	0x140

/* INT STATUS */
#define CM_56XX_DENALI_CTL_84	0x150

/* INT ACK */
#define CM_56XX_DENALI_CTL_85 0x154

/* INT MASK */
#define CM_56XX_DENALI_CTL_86 0x158

/* MPR PAGE DUMP REGS*/
#define CM_56XX_DENALI_CTL_35 0x8c

#define SYSCON_PERSIST_SCRATCH	 0xdc
#define CMEM_PERSIST_SCRATCH_BIT (0x1 << 7)

#define IRQ_NAME_LEN 16
#define MEMORY_CONTROLLERS 2
#define MAX_DQ 4
#define MAX_CS 2
#define MPR_CIRCULAR_BUF_LEN 16
#define MPR_PAGE_BYTES 4
#define MPR_ERRORS 2 /* CRC, CA Parity error */

#define INT_BIT_1  (0x00000002)
#define INT_BIT_2  (0x00000004)
#define INT_BIT_3  (0x00000008)
#define INT_BIT_4  (0x00000010)
#define INT_BIT_5  (0x00000020)
#define INT_BIT_6  (0x00000040)
#define INT_BIT_7  (0x00000080)
#define INT_BIT_8  (0x00000100)
#define INT_BIT_11 (0x00000800)
#define INT_BIT_21 (0x00200000)
#define INT_BIT_25 (0x02000000)
#define INT_BIT_30 (0x40000000)
#define INT_BIT_31 (0x80000000)

#define CM_INT_MASK_BASE_PROBE (~(\
			INT_BIT_8 |\
			INT_BIT_31))

#define CM_INT_MASK_BASE (~(\
			INT_BIT_1 |\
			INT_BIT_2 |\
			INT_BIT_3 |\
			INT_BIT_4 |\
			INT_BIT_5 |\
			INT_BIT_6 |\
			INT_BIT_7 |\
			INT_BIT_11 |\
			INT_BIT_21 |\
			INT_BIT_31))

#define CM_INT_MASK_FULL (~(\
			INT_BIT_1 |\
			INT_BIT_2 |\
			INT_BIT_3 |\
			INT_BIT_4 |\
			INT_BIT_5 |\
			INT_BIT_6 |\
			INT_BIT_7 |\
			INT_BIT_11 |\
			INT_BIT_21 |\
			INT_BIT_25 |\
			INT_BIT_30 |\
			INT_BIT_31))

#define CM_INT_MASK_ALL (0x7fffffff)
#define ALIVE_NOTIFICATION_PERIOD (90*1000)

static int log = 1;
module_param(log, int, 0644);
MODULE_PARM_DESC(log, "Log each error to kernel log.");

static int force_restart = 1;
module_param(force_restart, int, 0644);
MODULE_PARM_DESC(force_restart, "Machine restart on fatal error.");

static atomic64_t mc_counter = ATOMIC_INIT(0);
/*
 * Bit [31] = Logical OR of all lower bits.
 * Bit [30] = A CRC error occurred on the write data bus.
 * Bit [29] = The user-initiated DLL resync has completed.
 * Bit [28] = A state change has been detected on the dfi_init_complete
 *        signal after initialization.
 * Bit [27] = The assertion of the INHIBIT_DRAM_CMD parameter has successfully
 *        inhibited the command queue.
 * Bit [26] = The register interface-initiated mode register write has completed
 *        and another mode register write may be issued.
 * Bit [25] = MPR read command, initiated with a software MPR_READ request, is
 *        complete.
 * Bit [24] = Error received from the PHY on the DFI bus.
 * Bit [23] = RESERVED
 * Bit [22] = RESERVED
 * Bit [21] = A parity error has been detected on the address/control bus on
 *        a registered DIMM.
 * Bit [20] = The leveling operation has completed.
 * Bit [19] = A read leveling gate training operation has been requested.
 * Bit [18] = A read leveling operation has been requested.
 * Bit [17] = A write leveling operation has been requested.
 * Bit [16] = A DFI update error has occurred.  Error information can be found
 *        in the UPDATE_ERROR_STATUS parameter.
 * Bit [15] = A write leveling error has occurred. Error information can be
 *        found in the WRLVL_ERROR_STATUS parameter.
 * Bit [14] = A read leveling gate training error has occurred. Error
 *        information can be found in the RDLVL_ERROR_STATUS parameter.
 * Bit [13] = A read leveling error has occurred. Error information can be
 *        found in the RDLVL_ERROR_STATUS parameter.
 * Bit [12] = The user has programmed an invalid setting associated with user
 *        words per burst.
 *        Examples:
 *          Setting param_reduc when burst length = 2.
 *          A 1:2 MC:PHY clock ratio with burst length = 2.
 * Bit [11] = A wrap cycle crossing a DRAM page has been detected. This is
 *        unsupported & may result in memory data corruption.
 * Bit [10] = The BIST operation has been completed.
 * Bit [9] = The low power operation has been completed.
 * Bit [8] = The MC initialization has been completed.
 * Bit [7] = An error occurred on the port command channel.
 * Bit [6] = Multiple uncorrectable ECC events have been detected.
 * Bit [5] = An uncorrectable ECC event has been detected.
 * Bit [4] = Multiple correctable ECC events have been detected.
 * Bit [3] = A correctable ECC event has been detected.
 * Bit [2] = Multiple accesses outside the defined PHYSICAL memory space
 *        have occurred.
 * Bit [1] = A memory access outside the defined PHYSICAL memory space
 *        has occurred.
 * Bit [0] = The memory reset is valid on the DFI bus.

 * Of these 1, 2, 3, 4, 5, 6, 7, 11, 21, 25, and 30 are of interest.
 */

/*
 *   MPR dump processing - overview.
 *
 * As ALERT_N does not have information about failing cs
 * one need to collect dumps for all available cs. Below given example
 * for two cs0/cs1.
 *
 *   CMEM MC           cmmon_isr_sw         cmmon_wq
 *     |                   |                   |
 *     |                   |                   |
 *     |ALERT_N - int_status bit [30]          |
 *     |------------------>|                   |
 *     |                   |schedule cmmon_wq  |
 *     |                   |------------------>|
 *     |                   |                   |if(dump_in_progress==0)
 *     |                   |                   |  dump_in_progress=1
 *     |                   |                   |
 *     |CTL_34 cs0 page1 (trigger dump)        |
 *     |<--------------------------------------|
 *     |                   |                   |wait
 *     |int_status bit [25]|                   |
 *     |------------------>|                   |
 *     |                   |wake up cmmon_wq   |
 *     |                   |------------------>|
 *     |read MPR CTL_35-42 |                   |
 *     |<--------------------------------------|collect cs0 MPR page1
 *     |                   |                   |
 *     |                   |                   |
 *     |CTL_34 cs1 page1 (trigger dump)        |
 *     |<--------------------------------------|
 *     |                   |                   |wait
 *     |int_status bit [25]|                   |
 *     |------------------>|                   |
 *     |                   |wake up cmmon_wq   |
 *     |                   |------------------>|
 *     |read MPR CTL_35-42 |                   |
 *     |<--------------------------------------|collect cs1 MPR page1
 *     |                   |                   |
 *     |                   |                   |process dumps
 *     |                   |                   |dump_in_progress=0
 *     |                   |                   |
 */

/* INT_STATUS */
struct __packed cm_56xx_denali_ctl_84
{
	unsigned int int_status;
};


/* ACK */
struct __packed cm_56xx_denali_ctl_85
{
	unsigned int int_ack;
};

/* MASK */

struct __packed cm_56xx_denali_ctl_86
{
	unsigned int int_mask;
};

/* DRAM CLASS */
struct __packed cm_56xx_denali_ctl_00
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 version				: 16;
	unsigned	 reserved0				: 4;
	unsigned	 dram_class				: 4;
	unsigned	 reserved1				: 7;
	unsigned	 start					: 1;
#else	/* Little Endian */
	unsigned	 start					: 1;
	unsigned	 reserved1				: 7;
	unsigned	 dram_class				: 4;
	unsigned	 reserved0				: 4;
	unsigned	 version				: 16;
#endif
};

/* Trigger MPR */
struct __packed cm_56xx_denali_ctl_34
{
#ifdef CPU_BIG_ENDIAN
	unsigned      obsolete3                                 :  8;
	unsigned      reserved2                                 :  4;
	unsigned      read_mpr_go                               :  1;
	unsigned      read_mpr                                  :  3;
	unsigned      reserved3                                 :  7;
	unsigned      reserved1                                 :  1;
	unsigned      mrw_status                                :  8;
#else    /* Little Endian */
	unsigned      mrw_status                                :  8;
	unsigned      reserved1                                 :  1;
	unsigned      reserved3                                 :  7;
	unsigned      read_mpr                                  :  3;
	unsigned      read_mpr_go                               :  1;
	unsigned      reserved2                                 :  4;
	unsigned      obsolete3                                 :  8;
#endif
};

#ifdef CONFIG_DEBUG_EDAC_AXXIA_CMEM

#define CM_56XX_DENALI_CTL_62 0xf8

struct __packed cm_56xx_denali_ctl_62
{
#ifdef CPU_BIG_ENDIAN
	unsigned	reserved0				: 2;
	unsigned	xor_check_bits				: 14;
	unsigned	reserved1				: 7;
	unsigned	fwc					: 1;
	unsigned	reserved2				: 7;
	unsigned	ecc_en					: 1;
#else	/* Little Endian */
	unsigned	ecc_en					: 1;
	unsigned	reserved2				: 7;
	unsigned	fwc					: 1;
	unsigned	reserved1				: 7;
	unsigned	xor_check_bits				: 14;
	unsigned	reserved0				: 2;
#endif
};

#endif

struct __packed cm_56xx_denali_ctl_74
{
#ifdef CPU_BIG_ENDIAN
	unsigned	reserved0				: 5;
	unsigned	row_diff				: 3;
	unsigned	reserved1				: 6;
	unsigned	bank_diff				: 2;
	unsigned	reserved2				: 7;
	unsigned	zqcs_rotate				: 1;
	unsigned	reserved3				: 7;
	unsigned	zq_in_progress				: 1;
#else	/* Little Endian */
	unsigned	zq_in_progress				: 1;
	unsigned	reserved3				: 7;
	unsigned	zqcs_rotate				: 1;
	unsigned	reserved2				: 7;
	unsigned	bank_diff				: 2;
	unsigned	reserved1				: 6;
	unsigned	row_diff				: 3;
	unsigned	reserved0				: 5;
#endif
};

struct __packed cm_56xx_denali_ctl_80
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 reserved0				: 5;
	unsigned	 memdata_ratio_0			: 3;
	unsigned	 reserved1				: 7;
	unsigned	 reduc					: 1;
	unsigned	 reserved2				: 4;
	unsigned	 burst_on_fly_bit			: 4;
	unsigned	 reserved3				: 4;
	unsigned	 cs_map					: 4;
#else	/* Little Endian */
	unsigned	 cs_map					: 4;
	unsigned	 reserved3				: 4;
	unsigned	 burst_on_fly_bit			: 4;
	unsigned	 reserved2				: 4;
	unsigned	 reduc					: 1;
	unsigned	 reserved1				: 7;
	unsigned	 memdata_ratio_0			: 3;
	unsigned	 reserved0				: 5;
#endif
};

struct __packed mpr_dump {
/*! @brief This specifies 8 bits of page information from 4 registers per
 * DRAM as per JEDEC
 */
	u8 dram_0_page[MPR_PAGE_BYTES];
	u8 dram_1_page[MPR_PAGE_BYTES];
	u8 dram_2_page[MPR_PAGE_BYTES];
	u8 dram_3_page[MPR_PAGE_BYTES];
/*! @brief specifies which MPR Page(n) to read per JEDEC. 2-bit width */
	u8	mpr_page_id;
	u8	cs;
};

enum init_return_codes {ERR_STAGE_8 = -8,
			ERR_STAGE_7 = -7,
			ERR_STAGE_6 = -6,
			ERR_STAGE_5 = -5,
			ERR_STAGE_4 = -4,
			ERR_STAGE_3 = -3,
			ERR_STAGE_2 = -2,
			ERR_STAGE_1 = -1
};
enum events {
	EV_ILLEGAL = 0,
	EV_MULT_ILLEGAL,
	EV_CORR_ECC,
	EV_MULT_CORR_ECC,
	EV_UNCORR_ECC,
	EV_MULT_UNCORR_ECC,
	EV_PORT_ERROR,
	EV_WRAP_ERROR,
	EV_PARITY_ERROR,
	NR_EVENTS
};


static char *block_name[] = {
	"illegal",
	"mult_illegal",
	"ecc",
	"mult_ecc",
	"uncorr_ecc",
	"mult_uncorr_ecc",
	"port_error",
	"wrap_error",
	"parity_error",
	"alert_n_cs0_dram0_ca_par_error",
	"alert_n_cs0_dram0_crc_error",
	"alert_n_cs0_dram1_ca_par_error",
	"alert_n_cs0_dram1_crc_error",
	"alert_n_cs0_dram2_ca_par_error",
	"alert_n_cs0_dram2_crc_error",
	"alert_n_cs0_dram3_ca_par_error",
	"alert_n_cs0_dram3_crc_error",
	"alert_n_cs1_dram0_ca_par_error",
	"alert_n_cs1_dram0_crc_error",
	"alert_n_cs1_dram1_ca_par_error",
	"alert_n_cs1_dram1_crc_error",
	"alert_n_cs1_dram2_ca_par_error",
	"alert_n_cs1_dram2_crc_error",
	"alert_n_cs1_dram3_ca_par_error",
	"alert_n_cs1_dram3_crc_error",
};

static const u32 event_mask[NR_EVENTS] = {
	[EV_ILLEGAL]			= INT_BIT_1,
	[EV_MULT_ILLEGAL]		= INT_BIT_2,
	[EV_CORR_ECC]			= INT_BIT_3,
	[EV_MULT_CORR_ECC]		= INT_BIT_4,
	[EV_UNCORR_ECC]			= INT_BIT_5,
	[EV_MULT_UNCORR_ECC]		= INT_BIT_6,
	[EV_PORT_ERROR]			= INT_BIT_7,
	[EV_WRAP_ERROR]			= INT_BIT_11,
	[EV_PARITY_ERROR]		= INT_BIT_21,
};

static const struct event_logging {
	int		 fatal;
	const char *level;
	const char *name;
} event_logging[NR_EVENTS] = {
	[EV_ILLEGAL]		= {0, KERN_ERR, "Illegal access"},
	[EV_MULT_ILLEGAL]	= {0, KERN_ERR, "Illegal access"},
	[EV_CORR_ECC]		= {0, KERN_NOTICE, "Correctable ECC error"},
	[EV_MULT_CORR_ECC]	= {0, KERN_NOTICE, "Correctable ECC error"},
	[EV_UNCORR_ECC]		= {1, KERN_CRIT, "Uncorrectable ECC error"},
	[EV_MULT_UNCORR_ECC]	= {1, KERN_CRIT, "Uncorrectable ECC error"},
	[EV_PORT_ERROR]		= {0, KERN_CRIT, "Port error"},
	[EV_WRAP_ERROR]		= {0, KERN_CRIT, "Wrap error"},
	[EV_PARITY_ERROR]	= {0, KERN_CRIT, "Parity error"},
};

/* Private structure for common edac device */
struct event_counter {
	atomic_t counter;
	int edac_block_idx;
};

struct mc_edac_data {
	struct event_counter events[NR_EVENTS];
	struct event_counter alerts[MAX_CS][MAX_DQ][MPR_ERRORS];
	u8 mpr_page1[MPR_CIRCULAR_BUF_LEN][MAX_DQ][MPR_PAGE_BYTES];
	struct mpr_dump mpr;
	char irq_name[IRQ_NAME_LEN];
	int cs_count;
	int dram_count;
	int irq;
	int latest_mpr_page1_idx;
	raw_spinlock_t mpr_data_lock;
	struct mutex edac_sysfs_data_lock;
	wait_queue_head_t dump_wq;
	wait_queue_head_t event_wq;
	atomic_t dump_ready;
	atomic_t event_ready;
	atomic_t dump_in_progress;
};

struct intel_edac_dev_info {
	struct platform_device *pdev;
	struct mc_edac_data *data;
	char *ctl_name;
	char *blk_name;
	char *proc_name;
#ifdef CONFIG_DEBUG_EDAC_AXXIA_CMEM
	struct proc_dir_entry *dir_entry;
#endif
	struct mutex state_machine_lock;
	struct work_struct offload_alerts;
	struct work_struct offload_events;
	int finish_alerts;
	int finish_events;
	struct workqueue_struct *wq_alerts;
	struct workqueue_struct *wq_events;
	int is_ddr4;
	int is_controller_configured;
	int edac_idx;
	u32 cm_region;
	struct regmap *syscon;
	void __iomem *axi2ser3_region;
	struct edac_device_ctl_info *edac_dev;
	void (*check)(struct edac_device_ctl_info *edac_dev);
};

#ifdef CONFIG_DEBUG_EDAC_AXXIA_CMEM
static int setup_fault_injection(struct intel_edac_dev_info *dev_info,
					int fault, int enable)
{
	struct cm_56xx_denali_ctl_62 denali_ctl_62;

	if (ncr_read(dev_info->cm_region,
		CM_56XX_DENALI_CTL_62,
		4, &denali_ctl_62))
		goto error_read;

	denali_ctl_62.xor_check_bits = fault;

	if (ncr_write(dev_info->cm_region,
		CM_56XX_DENALI_CTL_62,
		4, (u32 *) &denali_ctl_62))
		goto error_write;

	if (ncr_read(dev_info->cm_region,
		CM_56XX_DENALI_CTL_62,
		4, &denali_ctl_62))
		goto error_read;

	denali_ctl_62.fwc = (enable > 0 ? 0x1 : 0x0);

	if (ncr_write(dev_info->cm_region,
		CM_56XX_DENALI_CTL_62,
		4, (u32 *) &denali_ctl_62))
		goto error_write;
	return 0;

error_read:
	printk_ratelimited("%s: Error reading denali_ctl_62\n",
		       dev_name(&dev_info->pdev->dev));
	return 1;

error_write:
	printk_ratelimited("%s: Error writing denali_ctl_62\n",
		       dev_name(&dev_info->pdev->dev));
	return 1;
}

static ssize_t mpr1_dump_show(struct edac_device_ctl_info
				 *edac_dev, char *data)
{
	u8 (*mpr_page1)[MAX_DQ][MPR_PAGE_BYTES];
	unsigned long flags;
	char *buf = NULL;
	ssize_t	 count = 0;
	ssize_t	 total_count = 0;
	int	 i = 0, j = 0;
	int latest_mpr_page1_idx;
	struct intel_edac_dev_info *dev_info = edac_dev->pvt_info;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		goto no_mem_buffer;

	mpr_page1 = kmalloc_array(MPR_CIRCULAR_BUF_LEN, sizeof(*mpr_page1),
		      GFP_KERNEL);
	if (mpr_page1 == NULL)
		goto no_mem_dump;

	/*
	 * If this cause a performance issue use rcu list, where each node
	 * is 2-dimentional array instead.
	 */
	raw_spin_lock_irqsave(&dev_info->data->mpr_data_lock, flags);
	latest_mpr_page1_idx = dev_info->data->latest_mpr_page1_idx;
	memcpy(mpr_page1, dev_info->data->mpr_page1,
			MPR_CIRCULAR_BUF_LEN * MAX_DQ * MPR_PAGE_BYTES);
	raw_spin_unlock_irqrestore(&dev_info->data->mpr_data_lock, flags);

	/* Now process on copied data ... */
	count = scnprintf(buf+total_count, PAGE_SIZE-total_count,
			"%s", MPR_HDR2);

	total_count += (count > 0 ? count : 0);

	if (dev_info->data->dram_count == MAX_DQ) {
		count = scnprintf(buf+total_count, PAGE_SIZE-total_count,
				"%s", MPR_HDR4);
		total_count += (count > 0 ? count : 0);
	}


	for (i = 0; i < MPR_CIRCULAR_BUF_LEN; ++i) {
		j = (latest_mpr_page1_idx
			+ MPR_CIRCULAR_BUF_LEN - i) % MPR_CIRCULAR_BUF_LEN;

		/* x8 base */
		count = scnprintf(buf+total_count,
				PAGE_SIZE-total_count,
				MPR_FMT2, i+1,
				*((u32 *) &mpr_page1[j][0]),
				*((u32 *) &mpr_page1[j][1]));

		total_count += (count > 0 ? count : 0);

		/* x16 addition */
		if (dev_info->data->dram_count == MAX_DQ) {
			count = scnprintf(buf+total_count,
					PAGE_SIZE-total_count,
					MPR_FMT4,
					*((u32 *) &mpr_page1[j][2]),
					*((u32 *) &mpr_page1[j][3]));

			total_count += (count > 0 ? count : 0);
		}
	}

	total_count = scnprintf(data, PAGE_SIZE, "%s\n", buf);

	/* free resourses */
	kfree(mpr_page1);
	kfree(buf);
	return total_count;

no_mem_dump:
	pr_info("Could not allocate memory for cmem MPR dump.\n");
	kfree(buf);
	return 0;
no_mem_buffer:
	pr_err("Could not allocate memory for cmem MPR buffer.\n");
	return 0;
}



static struct edac_dev_sysfs_attribute device_block_attr[] = {
	{
		.attr = {
			.name = "mpr_page1",
			.mode = (0644)
		},
		.show = mpr1_dump_show,
		.store = NULL
	},
	/* End of list */
	{
		.attr = {.name = NULL}
	}
};

static void axxia_mc_sysfs_attributes(struct edac_device_ctl_info *edac_dev)
{
		edac_dev->sysfs_attributes = &device_block_attr[0];
}
#endif

static inline void __attribute__((always_inline))
handle_events(struct intel_edac_dev_info *edac_dev,
		struct cm_56xx_denali_ctl_84 *denali_ctl_84)
{
	unsigned long set_val;
	int i;

	for (i = 0; i < NR_EVENTS; ++i) {
		if ((denali_ctl_84->int_status & event_mask[i]) != 0) {
			if (force_restart && event_logging[i].fatal) {
				if (IS_ERR(edac_dev->syscon)) {
					set_val = readl(
						edac_dev->axi2ser3_region
						+ SYSCON_PERSIST_SCRATCH);
					/* set bit 3 in pscratch reg */
					set_val = set_val
						| CMEM_PERSIST_SCRATCH_BIT;
					writel(set_val,
						edac_dev->axi2ser3_region +
						SYSCON_PERSIST_SCRATCH);
				} else {
					regmap_update_bits(edac_dev->syscon,
						SYSCON_PERSIST_SCRATCH,
						CMEM_PERSIST_SCRATCH_BIT,
						CMEM_PERSIST_SCRATCH_BIT);
				}
				pr_emerg("%s uncorrectable error\n",
						edac_dev->ctl_name);
				machine_restart(NULL);
			}
			atomic_inc(&edac_dev->data->events[i].counter);
		}
	}
}

static inline void __attribute__((always_inline))
inc_alert_counter(struct event_counter (*alert)[MAX_DQ][MPR_ERRORS],
			int cs, int dram, u8 val)
{
	/* PARITY */
	if (val & 0x40)
		atomic_inc(&alert[cs][dram][0].counter);

	/* CRC */
	if (val & 0x80)
		atomic_inc(&alert[cs][dram][1].counter);
}

static inline void __attribute__((always_inline))
store_mpr_dump(struct intel_edac_dev_info *edac_dev, int cs)
{
	struct mpr_dump *mpr = &edac_dev->data->mpr;
	int idx;

	edac_dev->data->latest_mpr_page1_idx++;
	/* used smart bitwise op instead of %= MPR_CIRCULAR_BUF_LEN */
	/* MPR_CIRCULAR_BUF_LEN is power of 2 */
	idx = (edac_dev->data->latest_mpr_page1_idx &=
			(MPR_CIRCULAR_BUF_LEN - 1));

	memcpy((char *) &edac_dev->data->mpr_page1[idx],
		(char *) &mpr->dram_0_page[0],
		MAX_DQ * MPR_PAGE_BYTES);
}

static inline void __attribute__((always_inline))
update_alert_counters(struct intel_edac_dev_info *edac_dev, int cs)
{
	/* Casting magic
	 *
	 * mpr.dram_0_page[0] -> dram[0][0]
	 * mpr.dram_0_page[1] -> dram[0][1]
	 * mpr.dram_0_page[2] -> dram[0][2]
	 * mpr.dram_0_page[3] -> dram[0][3]
	 * ...
	 * mpr.dram_1_page[0] -> dram[1][0]
	 * ...
	 * mpr.dram_8_page[0] -> dram[8][0]
	 */
	u8 (*dram)[MPR_PAGE_BYTES] =
		(u8 (*)[MPR_PAGE_BYTES]) (&edac_dev->data->mpr.dram_0_page[0]);
	int i;

	for (i = 0; i < MAX_DQ; ++i)
		inc_alert_counter(edac_dev->data->alerts, cs, i, dram[i][3]);

}

static inline int __attribute__((always_inline))
collect_mpr_dump(struct intel_edac_dev_info *edac_dev, u8 page, int cs)
{
	struct mpr_dump *mpr = &edac_dev->data->mpr;
	unsigned long flags;
	u32 regval;
	int i;

	mpr->mpr_page_id = page;

	for (i = 0; i < MPR_PAGE_BYTES; ++i) {
		if (ncr_read(edac_dev->cm_region,
				(CM_56XX_DENALI_CTL_35 + (0x8 * i)),
				4, &regval))
			goto error_read;

		mpr->dram_0_page[i] = regval & 0xff;
		mpr->dram_1_page[i] = ((regval & 0xff00) >> 8);

		if (edac_dev->data->dram_count == MAX_DQ) {
			mpr->dram_2_page[i] = ((regval & 0xff0000) >> 16);
			mpr->dram_3_page[i] = ((regval & 0xff000000) >> 24);
		}
	}
	raw_spin_lock_irqsave(&edac_dev->data->mpr_data_lock, flags);
	store_mpr_dump(edac_dev, cs);
	raw_spin_unlock_irqrestore(&edac_dev->data->mpr_data_lock, flags);

	update_alert_counters(edac_dev, cs);
	return 0;

error_read:
	printk_ratelimited("%s: Memory error reading MC mpr page\n",
		dev_name(&edac_dev->pdev->dev));
	return 1;
}

static irqreturn_t
cmmon_isr_hw(int interrupt, void *device)
{
	return IRQ_WAKE_THREAD;
}

static int initialize(struct intel_edac_dev_info *dev_info);
static int enable_workers(struct intel_edac_dev_info *dev_info);
static void uninitialize(struct intel_edac_dev_info *dev_info,
			int ret, int only_disable);

static irqreturn_t
cmmon_isr_sw(int interrupt, void *device)
{
	struct intel_edac_dev_info *dev_info = device;
	struct cm_56xx_denali_ctl_84 denali_ctl_84;
	struct cm_56xx_denali_ctl_85 denali_ctl_85 = {0};
	struct cm_56xx_denali_ctl_86 denali_ctl_86;
	int ret = 0;

	/*
	 * NOTE:
	 * ISR function is only reading int_status, and write into int_act
	 * registers.
	 *
	 * - first handle critical events, which might require restart
	 *  (handle_events) and then to the job outside isr
	 * - second collect MPR dump if any exists and then trigger new if
	 *   needed - all outside isr,
	 * - third wake up job outside isr to trigger mpr dump procedure when
	 *   ALERT_N reported (bit [30] is on)
	 */

	if (ncr_read(dev_info->cm_region, CM_56XX_DENALI_CTL_84,
		4, (u32 *) &denali_ctl_84))
		goto error_read;

	if (denali_ctl_84.int_status & INT_BIT_8) {
		if (dev_info->is_controller_configured == 0) {
			ret = initialize(dev_info);
			if (ret)
				goto error_init;

			ret = enable_workers(dev_info);
			if (ret)
				goto error_init;

			dev_info->is_controller_configured = 1;
		}

		if (dev_info->is_ddr4)
			denali_ctl_86.int_mask = CM_INT_MASK_FULL;
		else
			denali_ctl_86.int_mask = CM_INT_MASK_BASE;

		if (ncr_write(dev_info->cm_region,
					CM_56XX_DENALI_CTL_86,
					4, (u32 *) &denali_ctl_86)) {
			goto error_write;
		}
		return IRQ_HANDLED;
	}

	/*
	 * SAFETY CHECK
	 * one cannot go further if driver is not fully functional!!!
	 */
	if (dev_info->is_controller_configured == 0)
		return IRQ_HANDLED;


	handle_events(dev_info, &denali_ctl_84);
	atomic_set(&dev_info->data->event_ready, 1);
	wake_up(&dev_info->data->event_wq);

	denali_ctl_85.int_ack =
		(denali_ctl_84.int_status &
		   (~(INT_BIT_25 | INT_BIT_31 | INT_BIT_8)));

	if (dev_info->is_ddr4) {
		if (denali_ctl_84.int_status & INT_BIT_25) {
			atomic_set(&dev_info->data->dump_ready, 1);
			wake_up(&dev_info->data->dump_wq);
			denali_ctl_85.int_ack |= INT_BIT_25;
		}

		if (denali_ctl_84.int_status & INT_BIT_30) {
			atomic_inc(&dev_info->data->dump_in_progress);
			wake_up(&dev_info->data->dump_wq);
			denali_ctl_85.int_ack |= INT_BIT_30;
		}
	}

	if (ncr_write(dev_info->cm_region, CM_56XX_DENALI_CTL_85,
		4, (u32 *) &denali_ctl_85))
		goto error_write;

	return IRQ_HANDLED;

error_write:
	printk_ratelimited("%s: Error writing interrupt status\n",
		       dev_name(&dev_info->pdev->dev));
	return IRQ_HANDLED;
error_read:
	printk_ratelimited("%s: Error reading interrupt status\n",
		       dev_name(&dev_info->pdev->dev));
	return IRQ_HANDLED;
error_init:
	printk_ratelimited("%s: Error during driver initialization\n",
		       dev_name(&dev_info->pdev->dev));
	uninitialize(dev_info, ret,
			dev_info->is_controller_configured == 0 ? 1 : 0);
	return IRQ_HANDLED;
}


static void intel_cm_alerts_error_check(struct edac_device_ctl_info *edac_dev)
{
	struct intel_edac_dev_info *dev_info =
			(struct intel_edac_dev_info *) edac_dev->pvt_info;
	struct event_counter (*alerts)[MAX_DQ][MPR_ERRORS] =
			dev_info->data->alerts;
	struct cm_56xx_denali_ctl_34 denali_ctl_34;
	int i, j, k, ret;
	u32 counter;

start:
	/* keep hung up monitor happy 90 sec's */
	if (wait_event_timeout(dev_info->data->dump_wq,
		atomic_read(&dev_info->data->dump_in_progress),
		msecs_to_jiffies(ALIVE_NOTIFICATION_PERIOD)) == 0)
		goto start;

	if (dev_info->finish_alerts)
		goto finish;

	for (i = 0; i < dev_info->data->cs_count; ++i) {

		/* trigger dump */
		if (ncr_read(dev_info->cm_region,
			CM_56XX_DENALI_CTL_34,
			4, (u32 *) &denali_ctl_34))
			goto error_read;

		/* bits [2] cs number */
		/* bits [1:0] page number */
		denali_ctl_34.read_mpr = 4*i + CM_MPR_PAGE;
		if (ncr_write(dev_info->cm_region,
			CM_56XX_DENALI_CTL_34,
			4, (u32 *) &denali_ctl_34))
			goto error_write;

		/* bit [4] trigger dump */
		denali_ctl_34.read_mpr_go = 1;
		if (ncr_write(dev_info->cm_region,
			CM_56XX_DENALI_CTL_34,
			4, (u32 *) &denali_ctl_34))
			goto error_write;

		/* wait */
		ret = wait_event_timeout(dev_info->data->dump_wq,
			   atomic_read(&dev_info->data->dump_ready),
			   msecs_to_jiffies(1000));
		if (dev_info->finish_alerts)
			goto finish;
		if (ret == 0)
			goto timeout_error;

		atomic_set(&dev_info->data->dump_ready, 0);
		/* collect data */
		collect_mpr_dump(dev_info, CM_MPR_PAGE, i);
	}

	/* process all dumps - update counters */

	mutex_lock(&dev_info->data->edac_sysfs_data_lock);
	for (i = 0; i < dev_info->data->cs_count; ++i) {
		for (j = 0;  j < dev_info->data->dram_count; ++j) {
			for (k = 0;  k < MPR_ERRORS; ++k) {
				/*
				 * TODO - How can one determine event type?
				 *	recoverable/unrecoverable
				 */
				counter = atomic_xchg(&alerts[i][j][k].counter,
							0);
				if (counter)
					edac_device_handle_multi_ce(edac_dev, 0,
						alerts[i][j][k].edac_block_idx,
						counter, edac_dev->ctl_name);
			}
		}
	}
	mutex_unlock(&dev_info->data->edac_sysfs_data_lock);
	atomic_set(&dev_info->data->dump_in_progress, 0);
	goto start;

timeout_error:
	printk_ratelimited("Timeout occurred during MPR dump.\n");
	atomic_set(&dev_info->data->dump_ready, 0);
	atomic_set(&dev_info->data->dump_in_progress, 0);
	goto start;

error_read:
error_write:
	printk_ratelimited("Could not collect MPR dump.\n");
	atomic_set(&dev_info->data->dump_in_progress, 0);
	goto start;

finish:
	atomic_set(&dev_info->data->dump_ready, 0);
	atomic_set(&dev_info->data->dump_in_progress, 0);
}

static void intel_cm_events_error_check(struct edac_device_ctl_info *edac_dev)
{
	struct intel_edac_dev_info *dev_info =
			(struct intel_edac_dev_info *) edac_dev->pvt_info;
	struct event_counter *events = dev_info->data->events;
	int i;
	u32 counter;

	while (1) {
		if (wait_event_timeout(dev_info->data->event_wq,
			atomic_read(&dev_info->data->event_ready),
			msecs_to_jiffies(ALIVE_NOTIFICATION_PERIOD)) == 0)
			continue;

		atomic_set(&dev_info->data->event_ready, 0);

		if (dev_info->finish_events)
			break;

		mutex_lock(&dev_info->data->edac_sysfs_data_lock);
		for (i = 0; i < NR_EVENTS; ++i) {
			counter = atomic_xchg(&events[i].counter, 0);
			if (counter)
				switch (i) {
				/*
				 * TODO - How can one determine event type?
				 *	recoverable/unrecoverable
				 */
				case EV_ILLEGAL:
				case EV_MULT_ILLEGAL:
				case EV_UNCORR_ECC:
				case EV_MULT_UNCORR_ECC:
					edac_device_handle_multi_ue(edac_dev,
						0, i, counter,
						edac_dev->ctl_name);
					break;
				case EV_CORR_ECC:
				case EV_MULT_CORR_ECC:
				case EV_PORT_ERROR:
				case EV_WRAP_ERROR:
				case EV_PARITY_ERROR:
					edac_device_handle_multi_ce(edac_dev,
						0, i, counter,
						edac_dev->ctl_name);
					break;
				default:
					printk_ratelimited(
						"ERROR EVENT MISSING.\n");
				}
		}
		mutex_unlock(&dev_info->data->edac_sysfs_data_lock);
	}
}

static void axxia_alerts_work(struct work_struct *work)
{
	struct intel_edac_dev_info *dev_info =
		container_of(work, struct intel_edac_dev_info, offload_alerts);

	intel_cm_alerts_error_check(dev_info->edac_dev);
}

static void axxia_events_work(struct work_struct *work)
{
	struct intel_edac_dev_info *dev_info =
		container_of(work, struct intel_edac_dev_info, offload_events);

	intel_cm_events_error_check(dev_info->edac_dev);
}

static int get_active_cs(struct intel_edac_dev_info *dev_info)
{
	struct cm_56xx_denali_ctl_80 denali_ctl_80 = {0};
	int i;
	int cs = 0;

	if (ncr_read(dev_info->cm_region, CM_56XX_DENALI_CTL_80,
		4, (u32 *) &denali_ctl_80)) {
		pr_err("Could not read active CS number.\n");
		return 0;
	}

	for (i = 0; i < MAX_CS; ++i)
		if (denali_ctl_80.cs_map & (0x1 << i))
			++cs;

	return cs;
}

static int get_active_dram(struct intel_edac_dev_info *dev_info)
{
	struct cm_56xx_denali_ctl_74 denali_ctl_74 = {0};
	int dram = 0;

	if (ncr_read(dev_info->cm_region, CM_56XX_DENALI_CTL_74,
		4, (u32 *) &denali_ctl_74)) {
		pr_err("Could not read number of lanes.\n");
		return dram;
	}

	if (denali_ctl_74.bank_diff == 0)
		dram = MAX_DQ/2;

	if (denali_ctl_74.bank_diff == 1)
		dram = MAX_DQ;

	return dram;
}

static int get_ddr4(struct intel_edac_dev_info *dev_info)
{
	struct cm_56xx_denali_ctl_00 denali_ctl_00;

	if (ncr_read(dev_info->cm_region, CM_56XX_DENALI_CTL_00,
		4, (u32 *) &denali_ctl_00)) {
		pr_err("Could not read ddr version.\n");
		return -1;
	}

	if (denali_ctl_00.dram_class == 0xa)
		return 1;

	return 0;
}

static void uninitialize(struct intel_edac_dev_info *dev_info,
			int ret, int only_disable)
{
	struct cm_56xx_denali_ctl_86 denali_ctl_86;

	switch (ret) {
	case ERR_STAGE_8:
		if (dev_info->data->irq) {
			disable_irq(dev_info->data->irq);
			devm_free_irq(&dev_info->pdev->dev,
					dev_info->data->irq, dev_info);
			dev_info->data->irq = 0;
		}
		/* fall-through */
	case ERR_STAGE_7:
		denali_ctl_86.int_mask = CM_INT_MASK_ALL;
		if (ncr_write(dev_info->cm_region,
					CM_56XX_DENALI_CTL_86,
					4, (u32 *) &denali_ctl_86)) {
			pr_err("Could not mask interrupts (%s - ctl_86).\n",
				dev_info->ctl_name);
		}
		if (only_disable)
			break;
		/* fall-through */
	case ERR_STAGE_6:
		if (dev_info->is_ddr4) {
			dev_info->finish_alerts = 1;
			atomic_inc(&dev_info->data->dump_in_progress);
			atomic_set(&dev_info->data->dump_ready, 1);
			wake_up(&dev_info->data->dump_wq);
			cancel_work_sync(&dev_info->offload_alerts);
		}
		dev_info->finish_events = 1;
		atomic_set(&dev_info->data->event_ready, 1);
		wake_up(&dev_info->data->event_wq);
		cancel_work_sync(&dev_info->offload_events);
		/* fall-through */
	case ERR_STAGE_5:
		if (dev_info->is_ddr4)
			if (dev_info->wq_alerts) {
				destroy_workqueue(dev_info->wq_alerts);
				dev_info->wq_alerts = NULL;
			}
		/* fall-through */
	case ERR_STAGE_4:
		if (dev_info->wq_events) {
			destroy_workqueue(dev_info->wq_events);
			dev_info->wq_events = NULL;

		}
		/* fall-through */
	case ERR_STAGE_3:
		edac_device_del_device(&dev_info->pdev->dev);
		/* fall-through */
	case ERR_STAGE_2:
		if (dev_info->edac_dev) {
			edac_device_free_ctl_info(dev_info->edac_dev);
			dev_info->edac_dev = NULL;
		}
		/* fall-through */
	case ERR_STAGE_1:
		/* fall-through */
	default:
		break;
	}
}

static int initialize(struct intel_edac_dev_info *dev_info)
{
	struct edac_device_instance *instance;
	struct edac_device_block *block;
	int i, j, k, l;

	int cs_count = MAX_CS;
	int dram_count = MAX_DQ;

	cs_count = get_active_cs(dev_info);
	if (cs_count == 0) {
		pr_err("Could not get cs number. Is config loaded?\n");
		return ERR_STAGE_1;
	}

	dram_count = get_active_dram(dev_info);
	if (dram_count == 0) {
		pr_err("Could not get dram number. Is config loaded?\n");
		return ERR_STAGE_1;
	}

	dev_info->is_ddr4 = get_ddr4(dev_info);

	if (dev_info->is_ddr4 == -1) {
		pr_err("Could not get dram version. Is config loaded?\n");
		return ERR_STAGE_1;
	}
	/*dev_info->is_ddr4 = 1;*/

	dev_info->finish_alerts = 0;
	dev_info->finish_events = 0;

	dev_info->data->cs_count = cs_count;
	dev_info->data->dram_count = dram_count;

	dev_info->edac_dev =
		edac_device_alloc_ctl_info(0, dev_info->ctl_name,
					 1, dev_info->blk_name,
					 NR_EVENTS + (dev_info->is_ddr4 ?
					 cs_count * dram_count * MPR_ERRORS
					 :
					 0),
					 0, NULL, 0, dev_info->edac_idx);

	if (!dev_info->edac_dev) {
		pr_info("No memory for edac device\n");
		return ERR_STAGE_1;
	}

	dev_info->edac_dev->log_ce = 0;

	instance = &dev_info->edac_dev->instances[0];

	/* It just gives more descriptive name. */
	for (i = 0; i < NR_EVENTS; ++i) {
		block = &instance->blocks[i];
		snprintf(block->name,
			sizeof(block->name),
			 "%s", block_name[i]);
		dev_info->data->events[i].edac_block_idx = i;
	}
	/*
	 * NOTE, please notice that 'i' index is
	 * further used in following loops. This is done
	 * intentionally. Edac is using index for all instances,
	 * each instance shall be however named based on correct
	 * cs, dram, ca/crc. Those might differ between HW versions.
	 * CS 1-2
	 * DRAM 1-2,
	 * CRC/CA Parity - always 2 events.
	 */
	if (dev_info->is_ddr4) {
		for (j = 0; j < cs_count; ++j) {
			for (l = 0; l < dram_count; ++l) {
				for (k = 0; k < MPR_ERRORS; ++k, ++i) {
					int idx =  NR_EVENTS +
						MAX_DQ * MPR_ERRORS * j +
						MPR_ERRORS * l + k;

					dev_info->data->alerts[j][l][k].
						edac_block_idx = i;
					block = &instance->blocks[i];
					snprintf(block->name,
						sizeof(block->name),
						 "%s", block_name[idx]);
				}
			}
		}
	}

	dev_info->edac_dev->pvt_info = dev_info;
	dev_info->edac_dev->dev = &dev_info->pdev->dev;
	dev_info->edac_dev->ctl_name = dev_info->ctl_name;
	dev_info->edac_dev->mod_name = INTEL_EDAC_MOD_STR;
	dev_info->edac_dev->dev_name = dev_name(&dev_info->pdev->dev);
	dev_info->edac_dev->edac_check = NULL;

#ifdef CONFIG_DEBUG_EDAC_AXXIA_CMEM
	if (dev_info->is_ddr4)
		axxia_mc_sysfs_attributes(dev_info->edac_dev);
#endif

	if (edac_device_add_device(dev_info->edac_dev) != 0) {
		pr_info("Unable to add edac device for %s\n",
			dev_info->ctl_name);
		return ERR_STAGE_2;
	}

	return 0;
}

static int enable_workers(struct intel_edac_dev_info *dev_info)
{
	atomic_set(&dev_info->data->dump_ready, 0);
	atomic_set(&dev_info->data->event_ready, 0);
	atomic_set(&dev_info->data->dump_in_progress, 0);

	dev_info->wq_events = alloc_workqueue("%s-events", WQ_MEM_RECLAIM, 1,
						(dev_info->ctl_name));
	if (!dev_info->wq_events)
		return ERR_STAGE_3;

	if (dev_info->is_ddr4) {
		dev_info->wq_alerts =
			alloc_workqueue("%s-alerts", WQ_MEM_RECLAIM, 1,
					(dev_info->ctl_name));
		if (!dev_info->wq_alerts)
			return ERR_STAGE_4;
	}
	if (dev_info->is_ddr4)
		INIT_WORK(&dev_info->offload_alerts, axxia_alerts_work);

	INIT_WORK(&dev_info->offload_events, axxia_events_work);

	if (dev_info->is_ddr4)
		queue_work(dev_info->wq_alerts, &dev_info->offload_alerts);
	queue_work(dev_info->wq_events, &dev_info->offload_events);

	return 0;
}

static int enable_driver_irq(struct intel_edac_dev_info *dev_info)
{
	int irq = -1, rc = 0;
	struct cm_56xx_denali_ctl_86 denali_ctl_86;

	snprintf(&dev_info->data->irq_name[0], IRQ_NAME_LEN,
			"%s-mon", dev_info->ctl_name);

	irq = platform_get_irq(dev_info->pdev, 0);
	if (irq < 0) {
		pr_err("Could not get irq number.\n");
		return ERR_STAGE_5;
	}

	/*
	 * Enable memory controller interrupts.
	 */
	if (dev_info->is_controller_configured) {
		if (dev_info->is_ddr4)
			denali_ctl_86.int_mask = CM_INT_MASK_FULL;
		else
			denali_ctl_86.int_mask = CM_INT_MASK_BASE;
	} else
		denali_ctl_86.int_mask = CM_INT_MASK_BASE_PROBE;

	if (ncr_write(dev_info->cm_region, CM_56XX_DENALI_CTL_86,
		4, (u32 *) &denali_ctl_86)) {
		pr_err("Could not write interrupt mask reg (%s - ctl_86).\n",
			dev_info->ctl_name);
		return ERR_STAGE_6;
	}

	dev_info->data->irq = irq;
	rc = devm_request_threaded_irq(&dev_info->pdev->dev, irq,
			cmmon_isr_hw, cmmon_isr_sw, IRQF_ONESHOT,
			&dev_info->data->irq_name[0], dev_info);

	if (rc) {
		pr_err("Could not register interrupt handler (%s).\n",
			dev_info->ctl_name);

		dev_info->data->irq = 0;

		/* Mask all interrupts in controller */
		denali_ctl_86.int_mask = CM_INT_MASK_ALL;
		if (ncr_write(dev_info->cm_region, CM_56XX_DENALI_CTL_86,
					4, (u32 *) &denali_ctl_86)) {
			pr_err("Could not mask interrupts (%s - ctl_86).\n",
					dev_info->ctl_name);
			return ERR_STAGE_7;
		}

		return ERR_STAGE_6;
	}
	return 0;
}


#ifdef CONFIG_DEBUG_EDAC_AXXIA_CMEM
static ssize_t
axxia_cmem_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
	char *buf = NULL;
	struct intel_edac_dev_info *dev_info =
		(struct intel_edac_dev_info *) filp->private_data;
	ssize_t len;

	if (*offset > 0)
		return 0;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		goto no_mem_buffer;

	mutex_lock(&dev_info->state_machine_lock);

	/*
	 * Do not modify this. Content is used by rte driver.
	 * Once changed modify rte code.
	 */
	len = snprintf(buf, PAGE_SIZE-1, "Node: 0x%x\n"
		"Command available:\n"
		"          dump - triggers mpr_page1 dump.\n"
		"          cerror - enable correctable error injection.\n"
		"          uerror - enable uncorrectable error injection.\n"
		"          disable - disable errors injection\n"
		" When error is enabled run a sequence:\n"
		"  ncpWrite -w 32 0x%x.0x0.0x4 0x11223344\n"
		"  ncpRead 0x%x.0x0.0x4\n",
		(int) dev_info->cm_region >> 16,
		(int) dev_info->cm_region >> 16,
		(int) dev_info->cm_region >> 16);

	mutex_unlock(&dev_info->state_machine_lock);

	buf[len] = '\0';
	if (copy_to_user(buffer, buf, len))
		len = -EFAULT;

	kfree(buf);
	*offset += len;
	return len;

no_mem_buffer:
	pr_err("Could not allocate memory for cmem edac control buffer.\n");
	return -ENOSPC;
}

static ssize_t
axxia_cmem_write(struct file *file, const char __user *buffer,
		 size_t count, loff_t *ppos)
{
	char *buf = NULL;
	struct intel_edac_dev_info *dev_info =
		(struct intel_edac_dev_info *) file->private_data;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (buf == NULL)
		goto no_mem_buffer;

	memset(buf, 0, count + 1);

	if (copy_from_user(buf, buffer, count)) {
		pr_err("Could not copy data from user.\n");
		goto cfu_failed;
	}

	if (!strncmp(buf, "dump", 4)) {
		atomic_inc(&dev_info->data->dump_in_progress);
		wake_up(&dev_info->data->dump_wq);
	}
	if (!strncmp(buf, "cerror", 6)) {
		/* 0x75 0x75 */
		setup_fault_injection(dev_info, 0x3af5, 1);
	}
	if (!strncmp(buf, "uerror", 6)) {
		/* 0x3 0x3 */
		setup_fault_injection(dev_info, 0x183, 1);
	}
	if (!strncmp(buf, "disable", 7)) {
		/* disable injection */
		setup_fault_injection(dev_info, 0x0, 0);
	}

	kfree(buf);
	return count;

cfu_failed:
	kfree(buf);
	return -EFAULT;

no_mem_buffer:
	pr_err("Could not allocate memory for cmem edac control buffer.\n");
	return -ENOSPC;
}

int axxia_cmem_open(struct inode *inode, struct file *filp)
{
	try_module_get(THIS_MODULE);
	filp->private_data = PDE_DATA(inode);
	return 0;
}

int axxia_cmem_close(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	filp->private_data = 0;
	return 0;
}

static const struct file_operations axxia_edac_cmem_proc_ops = {
	.owner      = THIS_MODULE,
	.open       = axxia_cmem_open,
	.read       = axxia_cmem_read,
	.write      = axxia_cmem_write,
	.release    = axxia_cmem_close,
	.llseek     = noop_llseek
};

static void remove_procfs_entry(struct intel_edac_dev_info *dev_info)
{
	if (dev_info && dev_info->dir_entry) {
		proc_remove(dev_info->dir_entry);
		dev_info->dir_entry = NULL;
	}
}
#endif

static int intel_edac_mc_probe(struct platform_device *pdev)
{
	int i, j, k, l;
	int count;
	struct intel_edac_dev_info *dev_info = NULL;
	struct resource *io;
	struct device_node *np = pdev->dev.of_node;
	struct cm_56xx_denali_ctl_00 denali_ctl_00;
	int ret = -1;

	count = atomic64_inc_return(&mc_counter);
	if ((count - 1) == MEMORY_CONTROLLERS)
		goto err_nodev;

	dev_info = devm_kzalloc(&pdev->dev, sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		goto err_nomem;

	dev_info->ctl_name =
		devm_kzalloc(&pdev->dev, 32*sizeof(char), GFP_KERNEL);
	if (!dev_info->ctl_name)
		goto err_nomem;

	dev_info->blk_name =
		devm_kzalloc(&pdev->dev, 32*sizeof(char), GFP_KERNEL);
	if (!dev_info->blk_name)
		goto err_nomem;

	dev_info->data =
		devm_kzalloc(&pdev->dev, sizeof(*dev_info->data), GFP_KERNEL);
	if (!dev_info->data)
		goto err_nomem;

	init_waitqueue_head(&dev_info->data->dump_wq);
	init_waitqueue_head(&dev_info->data->event_wq);

	raw_spin_lock_init(&dev_info->data->mpr_data_lock);
	mutex_init(&dev_info->data->edac_sysfs_data_lock);
	mutex_init(&dev_info->state_machine_lock);

	strncpy(dev_info->ctl_name, np->name, 32);
	dev_info->ctl_name[31] = '\0';

	strncpy(dev_info->blk_name, "ECC", 32);
	dev_info->blk_name[31] = '\0';

	edac_op_state = EDAC_OPSTATE_POLL;

	dev_info->pdev = pdev;
	dev_info->edac_idx = edac_device_alloc_index();
	dev_info->data->irq = 0;

	/* setup all counters */
	for (i = 0; i < NR_EVENTS; ++i)
		atomic_set(&dev_info->data->events[i].counter, 0);

	for (j = 0; j < MAX_CS; ++j) {
		for (l = 0; l < MAX_DQ; ++l) {
			for (k = 0; k < MPR_ERRORS; ++k, ++i) {
				atomic_set(&dev_info->data->
						alerts[j][l][k].counter, 0);
			}
		}
	}

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		dev_err(&pdev->dev, "Unable to get mem resource\n");
		goto err_init;
	}
	dev_info->cm_region = io->start;
	dev_info->syscon =
		syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(dev_info->syscon)) {
		pr_info(FMT, np->name);
		dev_info->axi2ser3_region = ioremap(AXI2_SER3_PHY_ADDR,
			AXI2_SER3_PHY_SIZE);
		if (!dev_info->axi2ser3_region) {
			pr_err("ioremap of axi2ser3 region failed\n");
			goto err_init;
		}
	}

	if (ncr_read(dev_info->cm_region, CM_56XX_DENALI_CTL_00,
		4, (u32 *) &denali_ctl_00)) {
		pr_err("Could not read ddr version.\n");
		goto err_init;
	}

	if (denali_ctl_00.start == 1) {
		/* uboot has configured CMEM */
		if (denali_ctl_00.dram_class == 0xa) {
			pr_info("%s supports mpr dump (DDR4).\n",
					dev_info->ctl_name);
			dev_info->is_ddr4 = 1;
		}
		if (denali_ctl_00.dram_class == 0x6) {
			pr_info("%s doesn't support mpr dump (DDR3).\n",
				dev_info->ctl_name);
		}
		dev_info->is_controller_configured = 1;

		ret = initialize(dev_info);
		if (ret)
			goto err_uninit;

		ret = enable_workers(dev_info);
		if (ret)
			goto err_uninit;

		ret = enable_driver_irq(dev_info);
		if (ret)
			goto err_uninit;

	} else {
		/* CMEM is not configured */
		dev_info->is_controller_configured = 0;

		ret = enable_driver_irq(dev_info);
		if (ret)
			goto err_uninit;

		pr_info("CMEM base init: controller: %s DEV %s (INTERRUPT).\n",
			dev_info->ctl_name,
			dev_name(&dev_info->pdev->dev));
	}

#ifdef CONFIG_DEBUG_EDAC_AXXIA_CMEM
	/* in this case create procfs file to be used by rte */
	dev_info->proc_name =
		devm_kzalloc(&pdev->dev, 32*sizeof(char),
				GFP_KERNEL);
	if (!dev_info->proc_name)
		goto err_uninit;

	snprintf(dev_info->proc_name, 31*sizeof(char),
		"driver/axxia_edac_%s_control",
		dev_info->ctl_name);

	/* each instance shall know each private data */
	dev_info->dir_entry =
		proc_create_data(dev_info->proc_name, 0200,
				NULL, &axxia_edac_cmem_proc_ops,
				dev_info);

	if (dev_info->dir_entry == NULL) {
		pr_err("Could not create proc entry for %s.\n",
				dev_info->ctl_name);
		goto err_uninit;
	}
#endif
	return 0;


err_uninit:
	uninitialize(dev_info, ret,
			dev_info->is_controller_configured == 0 ? 1 : 0);
err_init:
	mutex_destroy(&dev_info->data->edac_sysfs_data_lock);
	mutex_destroy(&dev_info->state_machine_lock);
	atomic64_dec(&mc_counter);
	return 1;
err_nomem:
	atomic64_dec(&mc_counter);
	return -ENOMEM;
err_nodev:
	atomic64_dec(&mc_counter);
	return -ENODEV;
}



static int intel_edac_mc_remove(struct platform_device *pdev)
{
	struct intel_edac_dev_info *dev_info =
		(struct intel_edac_dev_info *) &pdev->dev;

	if (dev_info) {
#ifdef CONFIG_DEBUG_EDAC_AXXIA_CMEM
		remove_procfs_entry(dev_info);
#endif

		uninitialize(dev_info, ERR_STAGE_8,
			dev_info->is_controller_configured == 0 ? 1 : 0);

		if (dev_info->edac_dev != NULL) {
			edac_device_del_device(&dev_info->pdev->dev);
			edac_device_free_ctl_info(dev_info->edac_dev);
		}

		mutex_destroy(&dev_info->data->edac_sysfs_data_lock);
		mutex_destroy(&dev_info->state_machine_lock);

		atomic64_dec(&mc_counter);
	}
	platform_device_unregister(pdev);
	return 0;
}

static const struct of_device_id intel_edac_cmmon_match[] = {
	{ .compatible = "intel,cmmon" },
	{ }
};
MODULE_DEVICE_TABLE(platform, intel_edac_cmmon_match);

static struct platform_driver intel_edac_mc_driver = {
	.probe = intel_edac_mc_probe,
	.remove = intel_edac_mc_remove,
	.driver = {
		.name = "intel_edac_cmmon",
		.of_match_table = intel_edac_cmmon_match,
	}
};
module_platform_driver(intel_edac_mc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Majtyka <marekx.majtyka@intel.com>");
MODULE_AUTHOR("Arun Joshi <arun.joshi@intel.com>");
