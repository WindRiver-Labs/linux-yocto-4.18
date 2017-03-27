#ifndef __AXXIA_L2_MODULE__
#define __AXXIA_L2_MODULE__

#include <linux/types.h>

static inline u64 read_cpumerrsr(void)
{
	u64 val;

	asm volatile("mrrc\tp15, 0, %Q0, %R0, c15" : "=r" (val));
	return val;
}

static inline u64 read_l2merrsr(void)
{
	u64 val;

	asm volatile("mrrc\tp15, 1, %Q0, %R0, c15" : "=r"(val));
	return val;
}

inline void write_cpumerrsr(u64 val)
{
	asm volatile("mcrr\tp15, 0, %Q0, %R0, c15" : : "r" (val));
}

inline void write_l2merrsr(u64 val)
{
	asm volatile("mcrr\tp15, 1, %Q0, %R0, c15" : : "r"(val));
}

#endif /*__AXXIA_L2_MODULE__*/
