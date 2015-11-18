#ifndef _AXXIA_H

void axxia_init_clocks(int is_sim);
void axxia_ddr_retention_init(void);
void axxia_platform_cpu_die(unsigned int cpu);
int axxia_platform_cpu_kill(unsigned int cpu);
void ncp_ddr_shutdown(void *, void *, unsigned long);
void flush_l3(void);

extern void axxia_secondary_startup(void);

extern struct smp_operations axxia_smp_ops;

extern void __iomem *syscon;
extern void __iomem *dickens;

#endif
