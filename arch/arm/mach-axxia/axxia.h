#ifndef _AXXIA_H

void axxia_init_clocks(int is_sim);
void axxia_ddr_retention_init(void);
void axxia_platform_cpu_die(unsigned int cpu);
int axxia_platform_cpu_kill(unsigned int cpu);
void ncp_ddr_shutdown(void *, void *, unsigned long);
void flush_l3(void);

extern void axxia_secondary_startup(void);

extern struct smp_operations axxia_smp_ops;

/*
 * when defined, the RTE driver module will set/clear
 * the ncr_reset_active flag to indicate when Axxia device
 * reset is in progress. This flag will be checked by the
 * kernel lsi-ncr driver and ddr_retention code.
 */
#define AXXIA_NCR_RESET_CHECK
extern int ncr_reset_active;

extern void __iomem *syscon;
extern void __iomem *dickens;

#endif
