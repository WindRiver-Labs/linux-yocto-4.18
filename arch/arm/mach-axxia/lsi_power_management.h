/*
 * lsi_power_management.h
 *
 *  Created on: Jun 23, 2014
 *      Author: z8cpaul
 */

#ifndef LSI_POWER_MANAGEMENT_H_
#define LSI_POWER_MANAGEMENT_H_


#define     NCP_SYSCON_MCG_CSW_CPU                              (0x00000000)
#define     NCP_SYSCON_MCG_CSW_SYS                              (0x00000004)
#define     NCP_SYSCON_MCG_DIV_CPU                              (0x00000008)
#define     NCP_SYSCON_MCG_DIV_SYS                              (0x0000000c)
#define     NCP_SYSCON_CLKDEBUG                                 (0x00000010)
#define     NCP_SYSCON_EVENT_ENB                                (0x00000014)
#define     NCP_SYSCON_CPU_FAST_INT                             (0x00000018)
#define     NCP_SYSCON_GIC_DISABLE                              (0x0000001c)
#define     NCP_SYSCON_CP15SDISABLE                             (0x00000020)
#define     NCP_SYSCON_LRSTDISABLE                              (0x00000024)
#define     NCP_SYSCON_LDO_CTL                                  (0x00000028)
#define     NCP_SYSCON_SHWK_QOS                                 (0x0000002c)
#define     NCP_SYSCON_FUSE_RTO                                 (0x00000030)
#define     NCP_SYSCON_PFUSE                                    (0x00000034)
#define     NCP_SYSCON_FUSE_STAT                                (0x00000038)
#define     NCP_SYSCON_SCRATCH                                  (0x0000003c)
#define     NCP_SYSCON_MASK_IPI0                                (0x00000040)
#define     NCP_SYSCON_MASK_IPI1                                (0x00000044)
#define     NCP_SYSCON_MASK_IPI2                                (0x00000048)
#define     NCP_SYSCON_MASK_IPI3                                (0x0000004c)
#define     NCP_SYSCON_MASK_IPI4                                (0x00000050)
#define     NCP_SYSCON_MASK_IPI5                                (0x00000054)
#define     NCP_SYSCON_MASK_IPI6                                (0x00000058)
#define     NCP_SYSCON_MASK_IPI7                                (0x0000005c)
#define     NCP_SYSCON_MASK_IPI8                                (0x00000060)
#define     NCP_SYSCON_MASK_IPI9                                (0x00000064)
#define     NCP_SYSCON_MASK_IPI10                               (0x00000068)
#define     NCP_SYSCON_MASK_IPI11                               (0x0000006c)
#define     NCP_SYSCON_MASK_IPI12                               (0x00000070)
#define     NCP_SYSCON_MASK_IPI13                               (0x00000074)
#define     NCP_SYSCON_MASK_IPI14                               (0x00000078)
#define     NCP_SYSCON_MASK_IPI15                               (0x0000007c)
#define     NCP_SYSCON_MASK_IPI16                               (0x00000080)
#define     NCP_SYSCON_MASK_IPI17                               (0x00000084)
#define     NCP_SYSCON_MASK_IPI18                               (0x00000088)
#define     NCP_SYSCON_SPARE0                                   (0x0000008c)
#define     NCP_SYSCON_STOP_CLK_CPU                             (0x00000090)


#define     NCP_SYSCON_RESET_STATUS                             (0x00000100)
#define     NCP_SYSCON_RESET_CORE_STATUS                        (0x00000108)

#define     NCP_SYSCON_KEY                                      (0x00001000)
#define     NCP_SYSCON_RESET_CTL                                (0x00001008)
#define     NCP_SYSCON_RESET_CPU                                (0x0000100c)
#define     NCP_SYSCON_HOLD_CPU                                 (0x00001010)
#define     NCP_SYSCON_HOLD_PTM                                 (0x00001014)
#define     NCP_SYSCON_HOLD_L2                                  (0x00001018)
#define     NCP_SYSCON_HOLD_DBG                                 (0x0000101c)

#define     NCP_SYSCON_PWRUP_CPU_RST                            (0x00001030)

#define     NCP_SYSCON_RESET_AXIS                               (0x00001040)
#define     NCP_SYSCON_RESET_AXIS_ACCESS_SIZE                   (0x00000008)

#define     NCP_SYSCON_PWR_CLKEN                                (0x00001400)
#define     NCP_SYSCON_ENABLE_CLKEN_SET                         (0x00001404)
#define     NCP_SYSCON_PWR_ACINACTM                             (0x00001408)
#define     NCP_SYSCON_PWR_CHIPSELECTEN                         (0x0000140c)
#define     NCP_SYSCON_PWR_CSYSREQ_TS                           (0x00001410)
#define     NCP_SYSCON_PWR_CSYSREQ_CNT                          (0x00001414)
#define     NCP_SYSCON_PWR_CSYSREQ_ATB                          (0x00001418)
#define     NCP_SYSCON_PWR_CSYSREQ_APB                          (0x0000141c)
#define     NCP_SYSCON_PWR_PWRUPL2LGCSTG1                       (0x00001420)
#define     NCP_SYSCON_PWR_PWRUPL2LGCSTG2                       (0x00001424)
#define     NCP_SYSCON_PWR_PWRUPL2HSRAM                         (0x00001428)
#define     NCP_SYSCON_PWR_ACEPWRDNRQ                           (0x0000142c)
#define     NCP_SYSCON_PWR_ISOLATEL2MISC                        (0x00001430)
#define     NCP_SYSCON_PWR_NPWRUPL2LGCSTG1_ACK                  (0x00001438)
#define     NCP_SYSCON_PWR_NPWRUPL2HSRAM_ACK                    (0x0000143c)
#define     NCP_SYSCON_PWR_STANDBYWFIL2                         (0x00001440)
#define     NCP_SYSCON_PWR_CSYSACK_TS                           (0x00001444)
#define     NCP_SYSCON_PWR_CACTIVE_TS                           (0x00001448)
#define     NCP_SYSCON_PWR_CSYSACK_CNT                          (0x0000144c)
#define     NCP_SYSCON_PWR_CACTIVE_CNT                          (0x00001450)
#define     NCP_SYSCON_PWR_CSYSACK_ATB                          (0x00001454)
#define     NCP_SYSCON_PWR_CACTIVE_ATB                          (0x00001458)
#define     NCP_SYSCON_PWR_CSYSACK_APB                          (0x0000145c)
#define     NCP_SYSCON_PWR_CACTIVE_APB                          (0x00001460)
#define     NCP_SYSCON_PWR_NACEPWRDNACK                         (0x00001464)
#define     NCP_SYSCON_PWR_CACTIVEM_EAGM                        (0x00001468)
#define     NCP_SYSCON_PWR_CACTIVEM_EAGS                        (0x0000146c)
#define     NCP_SYSCON_PWR_CACTIVES_EAGM                        (0x00001470)
#define     NCP_SYSCON_PWR_CACTIVES_EAGS                        (0x00001474)
#define     NCP_SYSCON_PWR_PWRUPCPUSTG1                         (0x00001480)
#define     NCP_SYSCON_PWR_PWRUPCPUSTG2                         (0x00001484)
#define     NCP_SYSCON_PWR_PWRUPCPURAM                          (0x00001488)
#define     NCP_SYSCON_PWR_ISOLATECPU                           (0x0000148c)
#define     NCP_SYSCON_PWR_NPWRUPCPUSTG1_ACK                    (0x00001490)
#define     NCP_SYSCON_PWR_NPWRUPCPURAM_ACK                     (0x00001494)
#define     NCP_SYSCON_PWR_QACTIVE                              (0x00001498)
#define     NCP_SYSCON_PWR_STANDBYWFI                           (0x0000149c)
#define     NCP_SYSCON_PWR_STANDBYWFE                           (0x000014a0)
#define     NCP_SYSCON_PWR_DBGNOPWRDWN                          (0x000014a4)
#define     NCP_SYSCON_PWR_DBGPWRUPREQ                          (0x000014a8)
#define     NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2              (0x00001580)
#define     NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1              (0x00001584)
#define     NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0              (0x00001588)
#define     NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2              (0x0000158c)
#define     NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1              (0x00001590)
#define     NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0              (0x00001594)
#define     NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2              (0x00001598)
#define     NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1              (0x0000159c)
#define     NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0              (0x000015a0)
#define     NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2              (0x000015a4)
#define     NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1              (0x000015a8)
#define     NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0              (0x000015ac)

#define		RAM_BANK0_MASK			(0x0FFF0000)
#define		RAM_BANK1_LS_MASK		(0xF0000000)
#define		RAM_BANK1_MS_MASK		(0x000000FF)
#define		RAM_BANK2_MASK			(0x000FFF00)
#define		RAM_BANK3_MASK			(0xFFF00000)
#define		RAM_ALL_MASK			(0xFFFFFFFF)

/* DICKENS REGISTERS (Miscelaneous Node) */
#define		DKN_MN_NODE_ID				(0x0)
#define		DKN_DVM_DOMAIN_OFFSET		(0x0)
#define		DKN_MN_DVM_DOMAIN_CTL		(0x200)
#define		DKN_MN_DVM_DOMAIN_CTL_SET	(0x210)
#define		DKN_MN_DVM_DOMAIN_CTL_CLR	(0x220)

/* DICKENS HN-F (Fully-coherent Home Node) */
#define		DKN_HNF_NODE_ID					(0x20)
#define		DKN_HNF_TOTAL_NODES				(0x8)
#define		DKN_HNF_SNOOP_DOMAIN_CTL		(0x200)
#define		DKN_HNF_SNOOP_DOMAIN_CTL_SET	(0x210)
#define		DKN_HNF_SNOOP_DOMAIN_CTL_CLR	(0x220)

/* DICKENS clustid to Node */
#define		DKN_CLUSTER0_NODE		(1)
#define		DKN_CLUSTER1_NODE		(9)
#define		DKN_CLUSTER2_NODE		(11)
#define		DKN_CLUSTER3_NODE		(19)

/* PO RESET cluster id to bit */
#define		PORESET_CLUSTER0		(0x10000)
#define		PORESET_CLUSTER1		(0x20000)
#define		PORESET_CLUSTER2		(0x40000)
#define		PORESET_CLUSTER3		(0x80000)

/* SYSCON KEY Value */
#define VALID_KEY_VALUE			(0xAB)

#define MAX_NUM_CLUSTERS    (4)
#define CORES_PER_CLUSTER   (4)
#define MAX_IPI				(19)

typedef struct {
	u32 cpu;
	u32 cluster;
} pm_data;


void pm_cpu_shutdown(u32 cpu);
int pm_cpu_powerup(u32 cpu);
void pm_debug_read_pwr_registers(void);
void pm_dump_L2_registers(void);
int pm_cpu_logical_die(pm_data *pm_request);
int pm_cpul2_logical_die(pm_data *pm_request);
unsigned long pm_get_powered_down_cpu(void);
bool pm_cpu_last_of_cluster(u32 cpu);
void pm_dump_dickens(void);
void pm_init_cpu(u32 cpu);
void pm_cpu_logical_powerup(void);
bool pm_cpu_active(u32 cpu);
void pm_init_syscon(void);

extern bool pm_in_progress[];
extern bool cluster_power_up[];
extern u32 pm_cpu_powered_down;


#endif /* LSI_POWER_MANAGEMENT_H_ */
