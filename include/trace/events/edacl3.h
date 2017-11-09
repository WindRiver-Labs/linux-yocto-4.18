#undef TRACE_SYSTEM
#define TRACE_SYSTEM edacl3

#if !defined(_TRACE_EDACL3_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_EDACL3_H

#include <linux/types.h>
#include <linux/tracepoint.h>


TRACE_EVENT(edacl3_error_types,
	TP_PROTO(u64 error_31_0, u64 error_63_32,
		u64 error_95_64, u64 error_159_128),

	TP_ARGS(error_31_0, error_63_32, error_95_64, error_159_128),

	TP_STRUCT__entry(
		__field(u64, error_31_0)
		__field(u64, error_63_32)
		__field(u64, error_95_64)
		__field(u64, error_159_128)
	),

	TP_fast_assign(
		__entry->error_31_0 = error_31_0;
		__entry->error_63_32 = error_63_32;
		__entry->error_95_64 = error_95_64;
		__entry->error_159_128 = error_159_128;
	),

	TP_printk("Err types(0-3):(0x%016llx, 0x%016llx, 0x%016llx, 0x%016llx)",
		(u64) __entry->error_31_0, (u64) __entry->error_63_32,
		(u64) __entry->error_95_64, (u64) __entry->error_159_128
	)
);

TRACE_EVENT(edacl3_sig_vals,
	TP_PROTO(u64 sig_val_63_0, u64 sig_val_127_64, u64 sig_val_191_128),

	TP_ARGS(sig_val_63_0, sig_val_127_64, sig_val_191_128),

	TP_STRUCT__entry(
		__field(u64, sig_val_63_0)
		__field(u64, sig_val_127_64)
		__field(u64, sig_val_191_128)
	),

	TP_fast_assign(
		__entry->sig_val_63_0 = sig_val_63_0;
		__entry->sig_val_127_64 = sig_val_127_64;
		__entry->sig_val_191_128 = sig_val_191_128;
	),

	TP_printk("Sig_val_regs(0-3):(0x%016llx, 0x%016llx, 0x%016llx)",
		(u64) __entry->sig_val_63_0, (u64) __entry->sig_val_127_64,
		(u64) __entry->sig_val_191_128
	)
);

TRACE_EVENT(edacl3_syndromes,
	TP_PROTO(u64 syndrome0, u64 syndrome1),

	TP_ARGS(syndrome0, syndrome1),

	TP_STRUCT__entry(
		__field(u64, syndrome0)
		__field(u64, syndrome1)
	),

	TP_fast_assign(
		__entry->syndrome0 = syndrome0;
		__entry->syndrome1 = syndrome1;
	),

	TP_printk("Syndromes(0-1):(0x%016llx, 0x%016llx)",
		(u64) __entry->syndrome0, (u64) __entry->syndrome1
	)
);


TRACE_EVENT(edacl3_smc_results,
	TP_PROTO(struct arm_smccc_res *ptr),

	TP_ARGS(ptr),

	TP_STRUCT__entry(
		__field(u64, a0)
		__field(u64, a1)
		__field(u64, a2)
		__field(u64, a3)
	),

	TP_fast_assign(
		__entry->a0 = ptr->a0;
		__entry->a1 = ptr->a1;
		__entry->a2 = ptr->a2;
		__entry->a3 = ptr->a3;
	),

	TP_printk("Smc(a0-a3):(0x%llx, 0x%llx, 0x%llx, 0x%llx)",
		(u64) __entry->a0, (u64) __entry->a1,
		(u64) __entry->a2, (u64) __entry->a3
	)
);

#endif /* _TRACE_EDACL3_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
