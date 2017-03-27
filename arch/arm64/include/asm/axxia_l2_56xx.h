#ifndef __AXXIA_L2_MODULE__
#define __AXXIA_L2_MODULE__

#include <linux/types.h>

static inline u64 read_cpumerrsr(void)
{
	u64 val;

	asm volatile("mrs\t%x0, S3_1_c15_c2_2" : "=r" (val));
	return val;
}

static inline u64 read_l2merrsr(void)
{
	u64 val;

	asm volatile("mrs\t%x0, S3_1_c15_c2_3" : "=r" (val));
	return val;
}

inline void write_cpumerrsr(u64 val)
{

	asm volatile("msr\tS3_1_c15_c2_2, %x0"  : : "r" (val));
}

inline void write_l2merrsr(u64 val)
{

	asm volatile("msr\tS3_1_c15_c2_3, %x0" : :  "r" (val));
}


#endif /*__AXXIA_L2_MODULE__*/
