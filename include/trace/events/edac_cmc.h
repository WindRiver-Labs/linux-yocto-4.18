#undef TRACE_SYSTEM
#define TRACE_SYSTEM edac_cmc

#if !defined(_TRACE_EDAC_CMC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_EDAC_CMC_H

#include <linux/types.h>
#include <linux/tracepoint.h>


TRACE_EVENT(edac_cmc_int_status,
	TP_PROTO(u32 node, u32 int_status),

	TP_ARGS(node, int_status),

	TP_STRUCT__entry(
		__field(u32, node)
		__field(u32, int_status)
	),

	TP_fast_assign(
		__entry->node = node;
		__entry->int_status = int_status;
	),

	TP_printk("CMEM(node=0x%x) int_status=0x%08x",
		(u32) __entry->node, (u32) __entry->int_status)
);

TRACE_EVENT(edac_cmc_dump_processed,
	TP_PROTO(u32 node, u32 cs, u32 dram, u32 val),

	TP_ARGS(node, cs, dram, val),

	TP_STRUCT__entry(
		__field(u32, node)
		__field(u32, cs)
		__field(u32, dram)
		__field(u32, val)
	),

	TP_fast_assign(
		__entry->node = node;
		__entry->cs = cs;
		__entry->dram = dram;
		__entry->val = val;
	),

	TP_printk("CMEM(node=0x%x) cs=%d dram=%d value=0x%x",
		(u32) __entry->node, (u32) __entry->cs,
		(u32) __entry->dram, (u32) __entry->val)
);

TRACE_EVENT(edac_cmc_dump_collected,
	TP_PROTO(u32 node, u32 cs, u32 byte, u32 dram, u32 val),

	TP_ARGS(node, cs, byte, dram, val),

	TP_STRUCT__entry(
		__field(u32, node)
		__field(u32, cs)
		__field(u32, byte)
		__field(u32, dram)
		__field(u32, val)
	),

	TP_fast_assign(
		__entry->node = node;
		__entry->cs = cs;
		__entry->byte = byte;
		__entry->dram = dram;
		__entry->val = val;
	),

	TP_printk("CMEM(node=0x%x) cs=%d dram_%d_page[byte=%d]=0x%x",
		(u32) __entry->node, (u32) __entry->cs, (u32) __entry->dram,
		(u32) __entry->byte, (u32) __entry->val)
);

TRACE_EVENT(edac_cmc_dump_triggered,
	TP_PROTO(u32 node, u32 cs),

	TP_ARGS(node, cs),

	TP_STRUCT__entry(
		__field(u32, node)
		__field(u32, cs)
	),

	TP_fast_assign(
		__entry->node = node;
		__entry->cs = cs;
	),

	TP_printk("CMEM(node=0x%x) cs=%d",
			(u32) __entry->node, (u32) __entry->cs)
);
#endif /* _TRACE_EDAC_CMC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
