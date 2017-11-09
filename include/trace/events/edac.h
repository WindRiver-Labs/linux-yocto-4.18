#undef TRACE_SYSTEM
#define TRACE_SYSTEM edac

#if !defined(_TRACE_EDAC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_EDAC_H

#include <linux/types.h>
#include <linux/tracepoint.h>


TRACE_EVENT(edac_l2cache_syndrome,
	TP_PROTO(u64 syndrome),

	TP_ARGS(syndrome),

	TP_STRUCT__entry(
		__field(u64, syndrome)
	),

	TP_fast_assign(
		__entry->syndrome = syndrome;
	),

	TP_printk("L2MERRSR_EL1=0x%016llx", (u64) __entry->syndrome)
);

TRACE_EVENT(edac_l2cache_counter,
	TP_PROTO(int counter),

	TP_ARGS(counter),

	TP_STRUCT__entry(
		__field(int, counter)
	),

	TP_fast_assign(
		__entry->counter = counter;
	),

	TP_printk("l2 counter =%d",  __entry->counter)
);

TRACE_EVENT(edac_l1cache_syndrome,
	TP_PROTO(u64 syndrome),

	TP_ARGS(syndrome),

	TP_STRUCT__entry(
		__field(u64, syndrome)
	),

	TP_fast_assign(
		__entry->syndrome = syndrome;
	),

	TP_printk("CPUMERRSR_EL1=0x%016llx", (u64) __entry->syndrome)
);

TRACE_EVENT(edac_l1cache_counter,
	TP_PROTO(int counter),

	TP_ARGS(counter),

	TP_STRUCT__entry(
		__field(int, counter)
	),

	TP_fast_assign(
		__entry->counter = counter;
	),

	TP_printk("l1 counter =%d",  __entry->counter)
);
#endif /* _TRACE_EDAC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
