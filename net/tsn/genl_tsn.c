/*
 * Copyright 2017-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *       names of any contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/if_vlan.h>
#include <net/genetlink.h>
#include <net/netlink.h>
#include <linux/version.h>
#include <net/tsn.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 12, 0)
#define NLA_PARSE_NESTED(a, b, c, d) nla_parse_nested(a, b, c, d)
#else
#define NLA_PARSE_NESTED(a, b, c, d) nla_parse_nested(a, b, c, d, NULL)
#endif
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 6, 7)
#define NLA_PUT_U64(a, b, c) nla_put_u64(a, b, c)
#else
#define NLA_PUT_U64(a, b, c) nla_put_u64_64bit(a, b, c, NLA_U64)
#endif
/* the netlink family */
static struct genl_family tsn_family;

enum TSN_REPLY_VALUE {
	TSN_SUCCESS = 0,
	TSN_NODEVOPS,
	TSN_ATTRERR,
	TSN_DEVRETERR,
};

static const struct nla_policy tsn_cmd_policy[TSN_CMD_ATTR_MAX + 1] = {
	[TSN_CMD_ATTR_MESG]		= { .type = NLA_STRING },
	[TSN_CMD_ATTR_DATA]		= { .type = NLA_S32 },
	[TSN_ATTR_IFNAME]		= { .type = NLA_STRING },
	[TSN_ATTR_PORT_NUMBER]	= { .type = NLA_U8 },
	[TSN_ATTR_QBV]			= { .type = NLA_NESTED },
	[TSN_ATTR_STREAM_IDENTIFY] = { .type = NLA_NESTED },
	[TSN_ATTR_QCI_SP]		= { .type = NLA_NESTED },
	[TSN_ATTR_QCI_SFI]		= { .type = NLA_NESTED },
	[TSN_ATTR_QCI_SGI]		= { .type = NLA_NESTED },
	[TSN_ATTR_QCI_FMI]		= { .type = NLA_NESTED },
	[TSN_ATTR_CBS]			= { .type = NLA_NESTED },
	[TSN_ATTR_TSD]			= { .type = NLA_NESTED },
	[TSN_ATTR_QBU]			= { .type = NLA_NESTED },
	[TSN_ATTR_CT]			= { .type = NLA_NESTED },
	[TSN_ATTR_CBGEN]                = { .type = NLA_NESTED },
	[TSN_ATTR_CBREC]                = { .type = NLA_NESTED },
	[TSN_ATTR_PCPMAP]               = { .type = NLA_NESTED },
};

static const struct nla_policy ct_policy[TSN_CT_ATTR_MAX + 1] = {
	[TSN_CT_ATTR_QUEUE_STATE]	= { .type = NLA_U8 }
};

static const struct nla_policy cbgen_policy[TSN_CBGEN_ATTR_MAX + 1] = {
	[TSN_CBGEN_ATTR_INDEX]	        = { .type = NLA_U32 },
	[TSN_CBGEN_ATTR_PORT_MASK]	= { .type = NLA_U8 },
	[TSN_CBGEN_ATTR_SPLIT_MASK]	= { .type = NLA_U8 },
	[TSN_CBGEN_ATTR_SEQ_LEN]	= { .type = NLA_U8 },
	[TSN_CBGEN_ATTR_SEQ_NUM]        = { .type = NLA_U32 },
};

static const struct nla_policy cbrec_policy[TSN_CBREC_ATTR_MAX + 1] = {
	[TSN_CBREC_ATTR_INDEX]          = { .type = NLA_U32 },
	[TSN_CBREC_ATTR_SEQ_LEN]	= { .type = NLA_U8 },
	[TSN_CBREC_ATTR_HIS_LEN]        = { .type = NLA_U8 },
	[TSN_CBREC_ATTR_TAG_POP_EN]     = { .type = NLA_FLAG },
};

static const struct nla_policy pcpmap_policy[TSN_PCPMAP_ATTR_MAX + 1] = {
	[TSN_PCPMAP_ATTR_ENABLE]	= { .type = NLA_FLAG},
};

static const struct nla_policy qbu_policy[TSN_QBU_ATTR_MAX + 1] = {
	[TSN_QBU_ATTR_ADMIN_STATE] 		= { .type = NLA_U8 },
	[TSN_QBU_ATTR_HOLD_ADVANCE] 	= { .type = NLA_U32},
	[TSN_QBU_ATTR_RELEASE_ADVANCE] 	= { .type = NLA_U32},
	[TSN_QBU_ATTR_ACTIVE]			= { .type = NLA_FLAG},
	[TSN_QBU_ATTR_HOLD_REQUEST] 	= { .type = NLA_U8},
};

static const struct nla_policy cbs_policy[TSN_CBS_ATTR_MAX + 1] = {
	[TSN_CBS_ATTR_TC_INDEX]		= { .type = NLA_U8},
	[TSN_CBS_ATTR_BW]			= { .type = NLA_U8},
};

static const struct nla_policy tsd_policy[TSN_TSD_ATTR_MAX + 1] = {
	[TSN_TSD_ATTR_ENABLE]			= { .type = NLA_FLAG},
	[TSN_TSD_ATTR_DISABLE]			= { .type = NLA_FLAG},
	[TSN_TSD_ATTR_PERIOD]			= { .type = NLA_U32},
	[TSN_TSD_ATTR_MAX_FRM_NUM]		= { .type = NLA_U32},
	[TSN_TSD_ATTR_CYCLE_NUM]		= { .type = NLA_U32},
	[TSN_TSD_ATTR_LOSS_STEPS]		= { .type = NLA_U32},
	[TSN_TSD_ATTR_SYN_IMME]			= { .type = NLA_FLAG},
};

static const struct nla_policy qbv_policy[TSN_QBV_ATTR_MAX + 1] = {
	[TSN_QBV_ATTR_ADMINENTRY]	= {	.type = NLA_NESTED},
	[TSN_QBV_ATTR_OPERENTRY] = { .type = NLA_NESTED},
	[TSN_QBV_ATTR_ENABLE] 	= { .type = NLA_FLAG},
	[TSN_QBV_ATTR_DISABLE]	= { .type = NLA_FLAG},
	[TSN_QBV_ATTR_CONFIGCHANGE] = { .type = NLA_FLAG},
	[TSN_QBV_ATTR_CONFIGCHANGETIME] = { .type = NLA_U64},
	[TSN_QBV_ATTR_MAXSDU]		= { .type = NLA_U32},
	[TSN_QBV_ATTR_GRANULARITY] 	= { .type = NLA_U32},
	[TSN_QBV_ATTR_CURRENTTIME] 	= { .type = NLA_U64},
	[TSN_QBV_ATTR_CONFIGPENDING] = {.type = NLA_FLAG},
	[TSN_QBV_ATTR_CONFIGCHANGEERROR] = { .type = NLA_U64},
	[TSN_QBV_ATTR_LISTMAX] 	= { .type = NLA_U32},
};

static const struct nla_policy qbv_ctrl_policy[TSN_QBV_ATTR_CTRL_MAX + 1] = {
	[TSN_QBV_ATTR_CTRL_LISTCOUNT]		= { .type = NLA_U32},
	[TSN_QBV_ATTR_CTRL_GATESTATE]		= { .type = NLA_U8},
	[TSN_QBV_ATTR_CTRL_CYCLETIME]		= { .type = NLA_U32},
	[TSN_QBV_ATTR_CTRL_CYCLETIMEEXT]	= { .type = NLA_U32},
	[TSN_QBV_ATTR_CTRL_BASETIME]		= { .type = NLA_U64},
	[TSN_QBV_ATTR_CTRL_LISTENTRY]		= { .type = NLA_NESTED},
};

static const struct nla_policy qbv_entry_policy[TSN_QBV_ATTR_ENTRY_MAX + 1] = {
	[TSN_QBV_ATTR_ENTRY_ID]	= { .type = NLA_U32},
	[TSN_QBV_ATTR_ENTRY_GC]	= { .type = NLA_U8},
	[TSN_QBV_ATTR_ENTRY_TM]	= { .type = NLA_U32},
};

static const struct nla_policy cb_streamid_policy[TSN_STREAMID_ATTR_MAX + 1] = {
	[TSN_STREAMID_ATTR_INDEX] 	= { .type = NLA_U32},
	[TSN_STREAMID_ATTR_ENABLE] 	= { .type = NLA_FLAG},
	[TSN_STREAMID_ATTR_DISABLE]	= { .type = NLA_FLAG},
	[TSN_STREAMID_ATTR_STREAM_HANDLE]	= { .type = NLA_S32},
	[TSN_STREAMID_ATTR_IFOP]	= { .type = NLA_U32},
	[TSN_STREAMID_ATTR_OFOP]	= { .type = NLA_U32},
	[TSN_STREAMID_ATTR_IFIP]	= { .type = NLA_U32},
	[TSN_STREAMID_ATTR_OFIP]	= { .type = NLA_U32},
	[TSN_STREAMID_ATTR_TYPE]	= { .type = NLA_U8},
	[TSN_STREAMID_ATTR_NDMAC]	= { .type = NLA_U64},
	[TSN_STREAMID_ATTR_NTAGGED]	= { .type = NLA_U8},
	[TSN_STREAMID_ATTR_NVID]		= { .type = NLA_U16},
	[TSN_STREAMID_ATTR_SMAC]	= { .type = NLA_U64},
	[TSN_STREAMID_ATTR_STAGGED]	= { .type = NLA_U8},
	[TSN_STREAMID_ATTR_SVID]		= { .type = NLA_U16},
	[TSN_STREAMID_ATTR_COUNTERS_PSI] = { .type = NLA_U64},
	[TSN_STREAMID_ATTR_COUNTERS_PSO] = { .type = NLA_U64},
	[TSN_STREAMID_ATTR_COUNTERS_PSPPI] = { .type = NLA_U64},
	[TSN_STREAMID_ATTR_COUNTERS_PSPPO] = { .type = NLA_U64},
};

static const struct nla_policy qci_sfi_policy[TSN_QCI_SFI_ATTR_MAX + 1] = {
	[TSN_QCI_SFI_ATTR_INDEX]		= { .type = NLA_U32},
	[TSN_QCI_SFI_ATTR_ENABLE]		= { .type = NLA_FLAG},
	[TSN_QCI_SFI_ATTR_DISABLE]		= { .type = NLA_FLAG},
	[TSN_QCI_SFI_ATTR_STREAM_HANDLE] = { .type = NLA_S32},
	[TSN_QCI_SFI_ATTR_PRIO_SPEC] 	= { .type = NLA_S8},
	[TSN_QCI_SFI_ATTR_GATE_ID]		= { .type = NLA_U32},
	[TSN_QCI_SFI_ATTR_FILTER_TYPE]	= { .type = NLA_U8},
	[TSN_QCI_SFI_ATTR_FLOW_ID]		= { .type = NLA_S32},
	[TSN_QCI_SFI_ATTR_MAXSDU]		= { .type = NLA_U16},
	[TSN_QCI_SFI_ATTR_COUNTERS]		= { .len = sizeof(struct tsn_qci_psfp_sfi_counters)},
	[TSN_QCI_SFI_ATTR_OVERSIZE_ENABLE]	= { .type = NLA_FLAG},
	[TSN_QCI_SFI_ATTR_OVERSIZE]		= { .type = NLA_FLAG},
};

#if 0
static const struct nla_policy qci_sfi_counters_policy[TSN_QCI_SFI_ATTR_COUNT_MAX + 1] = {
	[TSN_QCI_SFI_ATTR_MATCH]		= { .type = NLA_U64},
	[TSN_QCI_SFI_ATTR_PASS]			= { .type = NLA_U64},
	[TSN_QCI_SFI_ATTR_DROP]			= { .type = NLA_U64},
	[TSN_QCI_SFI_ATTR_SDU_DROP]		= { .type = NLA_U64},
	[TSN_QCI_SFI_ATTR_SDU_PASS]		= { .type = NLA_U64},
	[TSN_QCI_SFI_ATTR_RED]			= { .type = NLA_U64},
};
#endif

static const struct nla_policy qci_sgi_policy[] = {
	[TSN_QCI_SGI_ATTR_INDEX]		= { .type = NLA_U32},
	[TSN_QCI_SGI_ATTR_ENABLE]		= { .type = NLA_FLAG},
	[TSN_QCI_SGI_ATTR_DISABLE]		= { .type = NLA_FLAG},
	[TSN_QCI_SGI_ATTR_CONFCHANGE]	= { .type = NLA_FLAG},
	[TSN_QCI_SGI_ATTR_IRXEN]		= { .type = NLA_FLAG},		/* Invalid rx enable*/
	[TSN_QCI_SGI_ATTR_IRX]			= { .type = NLA_FLAG},
	[TSN_QCI_SGI_ATTR_OEXEN]		= { .type = NLA_FLAG},		/* Octet exceed enable */
	[TSN_QCI_SGI_ATTR_OEX]			= { .type = NLA_FLAG},
	[TSN_QCI_SGI_ATTR_ADMINENTRY]	= { .type = NLA_NESTED},
	[TSN_QCI_SGI_ATTR_OPERENTRY]	= { .type = NLA_NESTED},
	[TSN_QCI_SGI_ATTR_CCTIME]		= { .type = NLA_U64},	/* config change time */
	[TSN_QCI_SGI_ATTR_TICKG]		= { .type = NLA_U32},
	[TSN_QCI_SGI_ATTR_CUTIME]		= { .type = NLA_U64},
	[TSN_QCI_SGI_ATTR_CPENDING]		= { .type = NLA_FLAG},
	[TSN_QCI_SGI_ATTR_CCERROR]		= { .type = NLA_U64},
};

static const struct nla_policy qci_sgi_ctrl_policy[] = {
	[TSN_SGI_ATTR_CTRL_INITSTATE]	= { .type = NLA_FLAG},
	[TSN_SGI_ATTR_CTRL_LEN]			= { .type = NLA_U8},
	[TSN_SGI_ATTR_CTRL_CYTIME]		= { .type = NLA_U32},
	[TSN_SGI_ATTR_CTRL_CYTIMEEX]	= { .type = NLA_U32},
	[TSN_SGI_ATTR_CTRL_BTIME]		= { .type = NLA_U64},
	[TSN_SGI_ATTR_CTRL_INITIPV]		= { .type = NLA_S8},
	[TSN_SGI_ATTR_CTRL_GCLENTRY]	= { .type = NLA_NESTED},
};

static const struct nla_policy qci_sgi_gcl_policy[] = {
	[TSN_SGI_ATTR_GCL_GATESTATE]	= { .type = NLA_FLAG},
	[TSN_SGI_ATTR_GCL_IPV]			= { .type = NLA_S8},
	[TSN_SGI_ATTR_GCL_INTERVAL]		= { .type = NLA_U32},
	[TSN_SGI_ATTR_GCL_OCTMAX]		= { .type = NLA_U32},
};

static const struct nla_policy qci_fmi_policy[] = {
	[TSN_QCI_FMI_ATTR_INDEX]	= { .type = NLA_U32},
	[TSN_QCI_FMI_ATTR_ENABLE]	= { .type = NLA_FLAG},
	[TSN_QCI_FMI_ATTR_DISABLE]	= { .type = NLA_FLAG},
	[TSN_QCI_FMI_ATTR_CIR]		= { .type = NLA_U32},
	[TSN_QCI_FMI_ATTR_CBS]		= { .type = NLA_U32},
	[TSN_QCI_FMI_ATTR_EIR]		= { .type = NLA_U32},
	[TSN_QCI_FMI_ATTR_EBS]		= { .type = NLA_U32},
	[TSN_QCI_FMI_ATTR_CF]		= { .type = NLA_FLAG},
	[TSN_QCI_FMI_ATTR_CM]		= { .type = NLA_FLAG},
	[TSN_QCI_FMI_ATTR_DROPYL]	= { .type = NLA_FLAG},
	[TSN_QCI_FMI_ATTR_MAREDEN]	= { .type = NLA_FLAG},
	[TSN_QCI_FMI_ATTR_MARED]	= { .type = NLA_FLAG},
	[TSN_QCI_FMI_ATTR_COUNTERS]	= { .len = sizeof(struct tsn_qci_psfp_fmi_counters)},
};

static int tsn_prepare_reply(struct genl_info *info, u8 cmd, struct sk_buff **skbp, size_t size)
{
	struct sk_buff *skb;
	void *reply;

	/* If new attributes are added, please revisit this allocation
	 */
	skb = genlmsg_new(size, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	if (!info) {
		nlmsg_free(skb);
		return -EINVAL;
	}

	reply = genlmsg_put_reply(skb, info, &tsn_family, 0, cmd);
	if (!reply) {
		nlmsg_free(skb);
		return -EINVAL;
	}

	*skbp = skb;
	return 0;
}

static int tsn_mk_reply(struct sk_buff *skb, int aggr, void *data, int len)
{
    /* add a netlink attribute to a socket buffer */
	return nla_put(skb, aggr, len, data);
}

static int tsn_send_reply(struct sk_buff *skb, struct genl_info *info)
{
	struct genlmsghdr *genlhdr = nlmsg_data(nlmsg_hdr(skb));
	void *reply = genlmsg_data(genlhdr);

	genlmsg_end(skb, reply);

	return genlmsg_reply(skb, info);
}

static int cmd_attr_echo_message(struct genl_info *info)
{
	struct nlattr *na;
	char *msg;
	struct sk_buff *rep_skb;
	size_t size;
	int ret;

	na = info->attrs[TSN_CMD_ATTR_MESG];
	if (!na)
		return -EINVAL;

	msg = (char *)nla_data(na);
	pr_info("tsn generic netlink receive echo mesg %s\n", msg);

	size = nla_total_size(strlen(msg) + 1);

	ret = tsn_prepare_reply(info, TSN_CMD_REPLY, &rep_skb, size + NLMSG_ALIGN(MAX_USER_SIZE));
	if (ret < 0)
		return ret;

	ret = tsn_mk_reply(rep_skb, TSN_CMD_ATTR_MESG, msg, size);
	if (ret < 0)
		goto err;

	return tsn_send_reply(rep_skb, info);

err:
	nlmsg_free(rep_skb);
	return ret;
}

static int cmd_attr_echo_data(struct genl_info *info)
{
	struct nlattr *na;
	s32	data;
	struct sk_buff *rep_skb;
	size_t size;
	int ret;

	/*read data */
	na = info->attrs[TSN_CMD_ATTR_DATA];
	if (!na)
		return -EINVAL;

	data = nla_get_s32(info->attrs[TSN_CMD_ATTR_DATA]);
	pr_info("tsn generic netlink receive echo data %d\n", data);

	/* send back */
	size = nla_total_size(sizeof(s32));

	ret = tsn_prepare_reply(info, TSN_CMD_REPLY, &rep_skb, size + NLMSG_ALIGN(MAX_USER_SIZE));
	if (ret < 0)
		return ret;

	/* netlink lib func */
	ret = nla_put_s32(rep_skb, TSN_CMD_ATTR_DATA, data);
	if (ret < 0)
		goto err;

	return tsn_send_reply(rep_skb, info);

err:
	nlmsg_free(rep_skb);
	return ret;
}

static int tsn_echo_cmd(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_CMD_ATTR_MESG]) {
		pr_info("tsn get attr TSN_CMD_ATTR_MESG in tsn_echo_cmd\n");
		return cmd_attr_echo_message(info);
	} else if (info->attrs[TSN_CMD_ATTR_DATA]) {
		pr_info("tsn get attr TSN_CMD_ATTR_DATA in tsn_echo_cmd\n");
		return cmd_attr_echo_data(info);
	}

	pr_info("tsn get no attr in tsn_echo_cmd\n");
	return -EINVAL;
}

static int tsn_simple_reply(struct genl_info *info, u32 cmd, char *portname, s32 retvalue)
{
	struct sk_buff *rep_skb;
	size_t size;
	int ret;

	/* send back */
	size = nla_total_size(strlen(portname) + 1);
	size += nla_total_size(sizeof(s32));

	ret = tsn_prepare_reply(info, cmd, &rep_skb, size + NLMSG_ALIGN(MAX_USER_SIZE));
	if (ret < 0)
		return ret;

	/* netlink lib func */
	ret = nla_put_string(rep_skb, TSN_ATTR_IFNAME, portname);
	if (ret < 0)
		return ret;

	ret = nla_put_s32(rep_skb, TSN_CMD_ATTR_DATA, retvalue);
	if (ret < 0)
		return ret;

	return tsn_send_reply(rep_skb, info);
}

int tsn_init_check(struct genl_info *info, struct net_device **ndev)
{
	struct nlattr *na;
	char *portname;
	struct net_device *netdev;

	na = info->attrs[TSN_ATTR_IFNAME];
	if (!na) {
		tsn_simple_reply(info, TSN_CMD_REPLY, "no portname", -TSN_ATTRERR);
		return -EINVAL;
	}

	portname = (char *)nla_data(na);
	netdev = __dev_get_by_name(genl_info_net(info), portname);
	if (!netdev) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EOPNOTSUPP;
	}

	pr_info("tsn: cmd_cb_streamid_set : netdev index is %d name is %s\n",
			netdev->ifindex, netdev->name);

	if (!netdev->tsn_ops) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EOPNOTSUPP;
	}

	*ndev = netdev;

	return 0;
}

static int cmd_cb_streamid_set(struct genl_info *info)
{
	struct nlattr *na, *sid[TSN_STREAMID_ATTR_MAX + 1];
	u32 sid_index;
	u8 iden_type = 1;
	bool enable;
	int ret;
	struct net_device *netdev;
	struct tsn_cb_streamid sidconf;
	const struct tsn_ops *tsnops;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	tsnops = netdev->tsn_ops;

	memset(&sidconf, 0, sizeof(struct tsn_cb_streamid));

	if (!info->attrs[TSN_ATTR_STREAM_IDENTIFY])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_STREAM_IDENTIFY];

	ret = NLA_PARSE_NESTED(sid, TSN_STREAMID_ATTR_MAX, na, cb_streamid_policy);
	if (ret) {
		return -EINVAL;
	}

	if (!sid[TSN_STREAMID_ATTR_INDEX])
		return -EINVAL;

	sid_index = nla_get_u32(sid[TSN_STREAMID_ATTR_INDEX]);

	if (sid[TSN_STREAMID_ATTR_ENABLE])
		enable = true;
	else if (sid[TSN_STREAMID_ATTR_DISABLE])
		enable = false;
	else
		return -EINVAL;

	if (!enable)
		goto loaddev;

	if (sid[TSN_STREAMID_ATTR_TYPE])
		iden_type = nla_get_u8(sid[TSN_STREAMID_ATTR_TYPE]);
	else
		return -EINVAL;

	sidconf.type = iden_type;
	switch (iden_type) {
	case STREAMID_NULL:
		if (!sid[TSN_STREAMID_ATTR_NDMAC] ||
		    !sid[TSN_STREAMID_ATTR_NTAGGED] ||
		    !sid[TSN_STREAMID_ATTR_NVID]) {
			return -EINVAL;
		}

		sidconf.para.nid.dmac = nla_get_u64(sid[TSN_STREAMID_ATTR_NDMAC]);
		sidconf.para.nid.tagged = nla_get_u8(sid[TSN_STREAMID_ATTR_NTAGGED]);
		sidconf.para.nid.vid = nla_get_u16(sid[TSN_STREAMID_ATTR_NVID]);
		break;
	case STREAMID_SMAC_VLAN:
		/* TODO: not supportted yet */
		if (!sid[TSN_STREAMID_ATTR_SMAC] ||
		    !sid[TSN_STREAMID_ATTR_STAGGED] ||
		    !sid[TSN_STREAMID_ATTR_SVID]) {
			return -EINVAL;
		}

		sidconf.para.sid.smac = nla_get_u64(sid[TSN_STREAMID_ATTR_SMAC]);
		sidconf.para.sid.tagged = nla_get_u8(sid[TSN_STREAMID_ATTR_STAGGED]);
		sidconf.para.sid.vid = nla_get_u16(sid[TSN_STREAMID_ATTR_SVID]);
		break;
	case STREAMID_DMAC_VLAN:

	case STREAMID_IP:

	default:
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	if (sid[TSN_STREAMID_ATTR_STREAM_HANDLE])
		sidconf.handle = nla_get_s32(sid[TSN_STREAMID_ATTR_STREAM_HANDLE]);

	if (sid[TSN_STREAMID_ATTR_IFOP])
		sidconf.ifac_oport = nla_get_u32(sid[TSN_STREAMID_ATTR_IFOP]);
	if (sid[TSN_STREAMID_ATTR_OFOP])
		sidconf.ofac_oport = nla_get_u32(sid[TSN_STREAMID_ATTR_OFOP]);
	if (sid[TSN_STREAMID_ATTR_IFIP])
		sidconf.ifac_iport = nla_get_u32(sid[TSN_STREAMID_ATTR_IFIP]);
	if (sid[TSN_STREAMID_ATTR_OFIP])
		sidconf.ofac_iport = nla_get_u32(sid[TSN_STREAMID_ATTR_OFIP]);

loaddev:
	if (!tsnops->cb_streamid_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EOPNOTSUPP;
	}

	tsnops->cb_streamid_set(netdev, sid_index, enable, &sidconf);

	/* simple reply here. To be continue */
	if (tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, 0))
		return -1;

	return 0;
}

static int tsn_cb_streamid_set(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_cb_streamid_set(info);
		return 0;
	}

	return -1;
}

static int cmd_cb_streamid_get(struct genl_info *info)
{
	struct nlattr *na, *sidattr, *sid[TSN_STREAMID_ATTR_MAX + 1];
	u32 sid_index;
	struct genlmsghdr *genlhdr;
	struct sk_buff *rep_skb;
	int ret, i;
	int valid;
	struct net_device *netdev;
	struct tsn_cb_streamid sidconf;
	struct tsn_cb_streamid_counters sidcounts;
	const struct tsn_ops *tsnops;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return -EINVAL;

	tsnops = netdev->tsn_ops;

	memset(&sidconf, 0, sizeof(struct tsn_cb_streamid));
	memset(&sidcounts, 0, sizeof(struct tsn_cb_streamid_counters));

	if (!info->attrs[TSN_ATTR_STREAM_IDENTIFY])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_STREAM_IDENTIFY];

	ret = NLA_PARSE_NESTED(sid, TSN_STREAMID_ATTR_MAX, na, cb_streamid_policy);
	if (ret)
		return -EINVAL;

	if (!sid[TSN_STREAMID_ATTR_INDEX])
		return -EINVAL;

	sid_index = nla_get_u32(sid[TSN_STREAMID_ATTR_INDEX]);

	if (!tsnops->cb_streamid_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EINVAL;
	} else {
		valid = tsnops->cb_streamid_get(netdev, sid_index, &sidconf);
		if (valid < 0) {
			tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
			return -1;
		}
	}

	/* send back */
	genlhdr = info->genlhdr;
	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	/* input netlink the parameters */
	sidattr = nla_nest_start(rep_skb, TSN_ATTR_STREAM_IDENTIFY);
	if (!sidattr) {
		ret = -EINVAL;
		goto err;
	}

	nla_put_u32(rep_skb, TSN_STREAMID_ATTR_INDEX, sid_index);

	if (valid == 1)
		nla_put_flag(rep_skb, TSN_STREAMID_ATTR_ENABLE);
	else if (valid == 0)
		nla_put_flag(rep_skb, TSN_STREAMID_ATTR_DISABLE);
	else {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		goto err;
	}

	nla_put_s32(rep_skb, TSN_STREAMID_ATTR_STREAM_HANDLE, sidconf.handle);

	nla_put_u32(rep_skb, TSN_STREAMID_ATTR_IFOP, sidconf.ifac_oport);
	nla_put_u32(rep_skb, TSN_STREAMID_ATTR_OFOP, sidconf.ofac_oport);
	nla_put_u32(rep_skb, TSN_STREAMID_ATTR_IFIP, sidconf.ifac_iport);
	nla_put_u32(rep_skb, TSN_STREAMID_ATTR_OFIP, sidconf.ofac_iport);

	nla_put_u8(rep_skb, TSN_STREAMID_ATTR_TYPE, sidconf.type);

	switch (sidconf.type) {
	case STREAMID_NULL:
		NLA_PUT_U64(rep_skb, TSN_STREAMID_ATTR_NDMAC, sidconf.para.nid.dmac);
		nla_put_u16(rep_skb, TSN_STREAMID_ATTR_NVID, sidconf.para.nid.vid);
		nla_put_u8(rep_skb, TSN_STREAMID_ATTR_NTAGGED, sidconf.para.nid.tagged);
		break;
	case STREAMID_SMAC_VLAN:
		NLA_PUT_U64(rep_skb, TSN_STREAMID_ATTR_SMAC, sidconf.para.sid.smac);
		nla_put_u16(rep_skb, TSN_STREAMID_ATTR_SVID, sidconf.para.sid.vid);
		nla_put_u8(rep_skb, TSN_STREAMID_ATTR_STAGGED, sidconf.para.sid.tagged);
		break;
	case STREAMID_DMAC_VLAN:
	case STREAMID_IP:
	default:
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		goto err;
	}

	if (tsnops->cb_streamid_counters_get == NULL) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		goto err;
	} else {
		ret = tsnops->cb_streamid_counters_get(netdev, sid_index, &sidcounts);
		if (ret < 0) {
			tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
			goto err;
		}
	}

	NLA_PUT_U64(rep_skb, TSN_STREAMID_ATTR_COUNTERS_PSI, sidcounts.per_stream.input);
	NLA_PUT_U64(rep_skb, TSN_STREAMID_ATTR_COUNTERS_PSO, sidcounts.per_stream.output);
	for (i = 0; i < 32; i++) {
		NLA_PUT_U64(rep_skb, TSN_STREAMID_ATTR_COUNTERS_PSPPI, sidcounts.per_streamport[i].input);
		NLA_PUT_U64(rep_skb, TSN_STREAMID_ATTR_COUNTERS_PSPPO, sidcounts.per_streamport[i].output);
	}

	nla_nest_end(rep_skb, sidattr);
	/* end netlink input the parameters */

	/* netlink lib func */
	ret = nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name);
	if (ret < 0)
		goto err;

	ret = nla_put_s32(rep_skb, TSN_CMD_ATTR_DATA, 0);
	if (ret < 0)
		goto err;

	return tsn_send_reply(rep_skb, info);

err:
	nlmsg_free(rep_skb);

	return ret;
}

static int tsn_cb_streamid_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_cb_streamid_get(info);
		return 0;
	}

	return -1;
}

static int cmb_cb_streamid_counters_get(struct genl_info *info)
{
	return 0;
}

static int tsn_cb_streamid_counters_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmb_cb_streamid_counters_get(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_sfi_set(struct genl_info *info)
{
	struct nlattr *na, *sfi[TSN_QCI_SFI_ATTR_MAX + 1];
	u32 sfi_handle;
	bool enable;
	int ret;
	struct net_device *netdev;
	struct tsn_qci_psfp_sfi_conf sficonf;
	const struct tsn_ops *tsnops;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	tsnops = netdev->tsn_ops;

	memset(&sficonf, 0, sizeof(struct tsn_qci_psfp_sfi_conf));

	if (!info->attrs[TSN_ATTR_QCI_SFI])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_QCI_SFI];

	ret = NLA_PARSE_NESTED(sfi, TSN_QCI_SFI_ATTR_MAX, na, qci_sfi_policy);
	if (ret) {
		pr_info("tsn: parse value TSN_QCI_SFI_ATTR_MAX  error.");
		return -EINVAL;
	}

	if (!sfi[TSN_QCI_SFI_ATTR_INDEX])
		return -EINVAL;

	sfi_handle = nla_get_u32(sfi[TSN_QCI_SFI_ATTR_INDEX]);

	if (sfi[TSN_QCI_SFI_ATTR_ENABLE])
		enable = true;
	else if (sfi[TSN_QCI_SFI_ATTR_DISABLE]) {
		enable = false;
		goto loaddrive;
	} else {
		pr_err("tsn: must provde ENABLE or DISABLE attribute.\n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	if (!sfi[TSN_QCI_SFI_ATTR_GATE_ID]) {
		pr_err("tsn: must provide stream gate index\n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	if (!sfi[TSN_QCI_SFI_ATTR_STREAM_HANDLE])
		sficonf.stream_handle_spec = -1;
	else
		sficonf.stream_handle_spec = nla_get_s32(sfi[TSN_QCI_SFI_ATTR_STREAM_HANDLE]);

	if (!sfi[TSN_QCI_SFI_ATTR_PRIO_SPEC])
		sficonf.priority_spec = -1;
	else
		sficonf.priority_spec = nla_get_s8(sfi[TSN_QCI_SFI_ATTR_PRIO_SPEC]);

	sficonf.stream_gate_instance_id = nla_get_u32(sfi[TSN_QCI_SFI_ATTR_GATE_ID]);

	if (sfi[TSN_QCI_SFI_ATTR_MAXSDU])
		sficonf.stream_filter.maximum_sdu_size = nla_get_u16(sfi[TSN_QCI_SFI_ATTR_MAXSDU]);
	else
		sficonf.stream_filter.maximum_sdu_size = 0;

	if (sfi[TSN_QCI_SFI_ATTR_FLOW_ID])
		sficonf.stream_filter.flow_meter_instance_id = nla_get_s32(sfi[TSN_QCI_SFI_ATTR_FLOW_ID]);
	else
		sficonf.stream_filter.flow_meter_instance_id = -1;

	if (sfi[TSN_QCI_SFI_ATTR_OVERSIZE_ENABLE])
		sficonf.block_oversize_enable = true;

	if (sfi[TSN_QCI_SFI_ATTR_OVERSIZE])
		sficonf.block_oversize = true;

loaddrive:
	if (!tsnops->qci_sfi_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EINVAL;
	}

	tsnops->qci_sfi_set(netdev, sfi_handle, enable, &sficonf);

	if (tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS))
		return -1;

	return 0;
}

static int tsn_qci_sfi_set(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_sfi_set(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_sfi_get(struct genl_info *info)
{
	struct nlattr *na, *sfiattr;
	struct nlattr *sfi[TSN_QCI_SFI_ATTR_MAX + 1];
	u32 sfi_handle;
	struct sk_buff *rep_skb;
	int ret, valid = 0;
	struct net_device *netdev;
	struct genlmsghdr *genlhdr;
	struct tsn_qci_psfp_sfi_conf sficonf;
	struct tsn_qci_psfp_sfi_counters sficount;
	const struct tsn_ops *tsnops;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	genlhdr = info->genlhdr;

	if (!info->attrs[TSN_ATTR_QCI_SFI])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_QCI_SFI];

	ret = NLA_PARSE_NESTED(sfi, TSN_QCI_SFI_ATTR_MAX, na, qci_sfi_policy);
	if (ret) {
		return -EINVAL;
	}

	if (!sfi[TSN_QCI_SFI_ATTR_INDEX])
		return -EINVAL;

	sfi_handle = nla_get_u32(sfi[TSN_QCI_SFI_ATTR_INDEX]);

	tsnops = netdev->tsn_ops;

	memset(&sficonf, 0, sizeof(struct tsn_qci_psfp_sfi_conf));
	memset(&sficount, 0, sizeof(struct tsn_qci_psfp_sfi_counters));

	if (!tsnops->qci_sfi_get || !tsnops->qci_sfi_counters_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EINVAL;
	} else {
		valid = tsnops->qci_sfi_get(netdev, sfi_handle, &sficonf);

		if (valid < 0) {
			tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
			return -1;
		}

		tsnops->qci_sfi_counters_get(netdev, sfi_handle, &sficount);
	}

	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		goto err;

	sfiattr = nla_nest_start(rep_skb, TSN_ATTR_QCI_SFI);
	if (!sfiattr) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		ret = -EINVAL;
		goto err;
	}

	nla_put_u32(rep_skb, TSN_QCI_SFI_ATTR_INDEX, sfi_handle);

	if (valid)
		nla_put_flag(rep_skb, TSN_QCI_SFI_ATTR_ENABLE);
	else
		nla_put_flag(rep_skb, TSN_QCI_SFI_ATTR_DISABLE);

	nla_put_s32(rep_skb, TSN_QCI_SFI_ATTR_STREAM_HANDLE, sficonf.stream_handle_spec);
	nla_put_s8(rep_skb, TSN_QCI_SFI_ATTR_PRIO_SPEC, sficonf.priority_spec);
	nla_put_u32(rep_skb, TSN_QCI_SFI_ATTR_GATE_ID, sficonf.stream_gate_instance_id);

	if (sficonf.stream_filter.maximum_sdu_size)
		nla_put_u16(rep_skb, TSN_QCI_SFI_ATTR_MAXSDU, sficonf.stream_filter.maximum_sdu_size);
	if (sficonf.stream_filter.flow_meter_instance_id >= 0)
		nla_put_s32(rep_skb, TSN_QCI_SFI_ATTR_FLOW_ID, sficonf.stream_filter.flow_meter_instance_id);

	if (sficonf.block_oversize_enable)
		nla_put_flag(rep_skb, TSN_QCI_SFI_ATTR_OVERSIZE_ENABLE);
	if (sficonf.block_oversize)
		nla_put_flag(rep_skb, TSN_QCI_SFI_ATTR_OVERSIZE);

	nla_put(rep_skb, TSN_QCI_SFI_ATTR_COUNTERS, sizeof(struct tsn_qci_psfp_sfi_counters), &sficount);

	nla_nest_end(rep_skb, sfiattr);

	return tsn_send_reply(rep_skb, info);
err:
	nlmsg_free(rep_skb);
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
	return ret;
}

static int tsn_qci_sfi_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_sfi_get(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_sfi_counters_get(struct genl_info *info)
{
	struct nlattr *na, *sfiattr;
	struct nlattr *sfi[TSN_QCI_SFI_ATTR_MAX + 1];
	u32 sfi_handle;
	struct sk_buff *rep_skb;
	int ret;
	struct net_device *netdev;
	struct genlmsghdr *genlhdr;
	struct tsn_qci_psfp_sfi_counters sficount;
	const struct tsn_ops *tsnops;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	genlhdr = info->genlhdr;

	if (!info->attrs[TSN_ATTR_QCI_SFI])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_QCI_SFI];

	ret = NLA_PARSE_NESTED(sfi, TSN_QCI_SFI_ATTR_MAX, na, qci_sfi_policy);
	if (ret) {
		return -EINVAL;
	}

	if (!sfi[TSN_QCI_SFI_ATTR_INDEX])
		return -EINVAL;

	sfi_handle = nla_get_u32(sfi[TSN_QCI_SFI_ATTR_INDEX]);

	tsnops = netdev->tsn_ops;

	memset(&sficount, 0, sizeof(struct tsn_qci_psfp_sfi_counters));
	if (!tsnops->qci_sfi_counters_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = tsnops->qci_sfi_counters_get(netdev, sfi_handle, &sficount);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}

	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		goto err;

	sfiattr = nla_nest_start(rep_skb, TSN_ATTR_QCI_SFI);
	if (!sfiattr) {
		ret = -EINVAL;
		goto err;
	}

	nla_put_u32(rep_skb, TSN_QCI_SFI_ATTR_INDEX, sfi_handle);

	tsnops->qci_sfi_counters_get(netdev, sfi_handle, &sficount);

	nla_put(rep_skb, TSN_QCI_SFI_ATTR_COUNTERS, sizeof(struct tsn_qci_psfp_sfi_counters), &sficount);

	nla_nest_end(rep_skb, sfiattr);

	return tsn_send_reply(rep_skb, info);
err:
	nlmsg_free(rep_skb);
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
	return ret;
}

static int tsn_qci_sfi_counters_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_sfi_counters_get(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_sgi_set(struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *sgia[TSN_QCI_SGI_ATTR_MAX + 1];
	struct nlattr *admin[TSN_SGI_ATTR_CTRL_MAX + 1];
	int ret = 0;
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	struct tsn_qci_psfp_sgi_conf sgi;
	struct tsn_qci_psfp_gcl *gcl = NULL;
	u16 sgi_handle = 0;
	u16 listcount = 0;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	tsnops = netdev->tsn_ops;

	memset(&sgi, 0, sizeof(struct tsn_qci_psfp_sgi_conf));

	if (!info->attrs[TSN_ATTR_QCI_SGI]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_QCI_SGI];

	ret = NLA_PARSE_NESTED(sgia, TSN_QCI_SGI_ATTR_MAX, na, qci_sgi_policy);
	if (ret) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	if (sgia[TSN_QCI_SGI_ATTR_ENABLE] && sgia[TSN_QCI_SGI_ATTR_DISABLE]) {
		pr_err("tsn: enable or disable?\n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -1;
	}

	if (sgia[TSN_QCI_SGI_ATTR_INDEX])
		sgi_handle = nla_get_u32(sgia[TSN_QCI_SGI_ATTR_INDEX]);

	if (sgia[TSN_QCI_SGI_ATTR_DISABLE]) {
		sgi.gate_enabled = 0;
		goto loaddev;
	} else {
		/* set default to be enable*/
		sgi.gate_enabled = 1;
	}

	if (sgia[TSN_QCI_SGI_ATTR_CONFCHANGE])
		sgi.config_change = 1;

	if (sgia[TSN_QCI_SGI_ATTR_IRXEN])
		sgi.block_invalid_rx_enable = 1;

	if (sgia[TSN_QCI_SGI_ATTR_IRX])
		sgi.block_invalid_rx = 1;

	if (sgia[TSN_QCI_SGI_ATTR_OEXEN])
		sgi.block_octets_exceeded_enable = 1;

	if (sgia[TSN_QCI_SGI_ATTR_OEX])
		sgi.block_octets_exceeded = 1;

	if (sgia[TSN_QCI_SGI_ATTR_ADMINENTRY]) {
		struct nlattr *entry;
		int rem;
		int count = 0;

		na = sgia[TSN_QCI_SGI_ATTR_ADMINENTRY];
		ret = NLA_PARSE_NESTED(admin, TSN_SGI_ATTR_CTRL_MAX, na, qci_sgi_ctrl_policy);

		/* Other parameters in admin control */
		if (admin[TSN_SGI_ATTR_CTRL_INITSTATE])
			sgi.admin.gate_states = 1;

		if (admin[TSN_SGI_ATTR_CTRL_CYTIME])
			sgi.admin.cycle_time = nla_get_u32(admin[TSN_SGI_ATTR_CTRL_CYTIME]);

		if (admin[TSN_SGI_ATTR_CTRL_CYTIMEEX])
			sgi.admin.cycle_time_extension = nla_get_u32(admin[TSN_SGI_ATTR_CTRL_CYTIMEEX]);

		if (admin[TSN_SGI_ATTR_CTRL_BTIME])
			sgi.admin.base_time = nla_get_u64(admin[TSN_SGI_ATTR_CTRL_BTIME]);

		if (admin[TSN_SGI_ATTR_CTRL_INITIPV])
			sgi.admin.init_ipv = nla_get_s8(admin[TSN_SGI_ATTR_CTRL_INITIPV]);
		else
			sgi.admin.init_ipv = -1;

		if (admin[TSN_SGI_ATTR_CTRL_LEN]) {
			sgi.admin.control_list_length = nla_get_u8(admin[TSN_SGI_ATTR_CTRL_LEN]);
			listcount = sgi.admin.control_list_length;
		}

		if (!listcount) {
			pr_info("tsn: no TSN_SGI_ATTR_CTRL_LEN attribute, length is 0\n");
			goto loaddev;
		}

		gcl = (struct tsn_qci_psfp_gcl *)kmalloc(listcount *
					 sizeof(struct tsn_qci_psfp_gcl), GFP_KERNEL);

		memset(gcl, 0, listcount * sizeof(struct tsn_qci_psfp_gcl));

		/* Check the whole admin attrs, checkout the TSN_SGI_ATTR_CTRL_GCLENTRY attributes */
		nla_for_each_nested(entry, na, rem) {
			struct nlattr *gcl_entry[TSN_SGI_ATTR_GCL_MAX + 1];

			if (nla_type(entry) != TSN_SGI_ATTR_CTRL_GCLENTRY)
				continue;

			/* parse each TSN_SGI_ATTR_CTRL_GCLENTRY */
			ret = NLA_PARSE_NESTED(gcl_entry, TSN_SGI_ATTR_GCL_MAX, entry, qci_sgi_gcl_policy);
			/* Parse gate control list */
			if (gcl_entry[TSN_SGI_ATTR_GCL_GATESTATE])
				(gcl + count)->gate_state = 1;

			if (gcl_entry[TSN_SGI_ATTR_GCL_IPV])
				(gcl + count)->ipv = nla_get_s8(gcl_entry[TSN_SGI_ATTR_GCL_IPV]);

			if (gcl_entry[TSN_SGI_ATTR_GCL_INTERVAL])
				(gcl + count)->time_interval = nla_get_u32(gcl_entry[TSN_SGI_ATTR_GCL_INTERVAL]);

			if (gcl_entry[TSN_SGI_ATTR_GCL_OCTMAX])
				(gcl + count)->octet_max = nla_get_u32(gcl_entry[TSN_SGI_ATTR_GCL_OCTMAX]);

			count++;

			if (count >= listcount)
				break;
		}

		if (count < listcount) {
			tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
			pr_err("tsn: gate list count less than TSN_SGI_ATTR_CTRL_LEN\n");
			if (gcl != NULL)
				kfree(gcl);
			return -EINVAL;
		}

	} else
		pr_info("tsn: no admin list parameters setting\n");

loaddev:
	if (!tsnops->qci_sgi_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		if (gcl != NULL)
			kfree(gcl);
		return -EINVAL;
	}

	sgi.admin.gcl = gcl;

	ret = tsnops->qci_sgi_set(netdev, sgi_handle, &sgi);
	if (gcl != NULL)
		kfree(gcl);
	if (!ret)
		return tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);

	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
	return -1;
}

static int tsn_qci_sgi_set(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_sgi_set(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_sgi_get(struct genl_info *info)
{
	struct nlattr *na, *sgiattr, *adminattr, *sglattr;
	struct nlattr *sgi[TSN_QCI_SGI_ATTR_MAX + 1];
	struct sk_buff *rep_skb;
	int ret;
	struct net_device *netdev;
	struct genlmsghdr *genlhdr;
	struct tsn_qci_psfp_sgi_conf sgiadmin;
	struct tsn_qci_psfp_gcl *gcl = NULL;
	const struct tsn_ops *tsnops;
	u16 sgi_handle;
	u8 listcount, i;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_QCI_SGI]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		pr_err("tsn: no sgi handle input \n");
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_QCI_SGI];

	ret = NLA_PARSE_NESTED(sgi, TSN_QCI_SGI_ATTR_MAX, na, qci_sgi_policy);
	if (ret) {
		return -EINVAL;
	}

	if (!sgi[TSN_QCI_SGI_ATTR_INDEX]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		pr_err("tsn: no sgi handle input \n");
		return -EINVAL;
	}

	sgi_handle = nla_get_u32(sgi[TSN_QCI_SGI_ATTR_INDEX]);

	/* Get config data from device */
	genlhdr = info->genlhdr;

	tsnops = netdev->tsn_ops;

	memset(&sgiadmin, 0, sizeof(struct tsn_qci_psfp_sgi_conf));

	if (!tsnops->qci_sgi_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = tsnops->qci_sgi_get(netdev, sgi_handle, &sgiadmin);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -1;
	}

	/* Form netlink reply data */
	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	/* Down 1 */
	sgiattr = nla_nest_start(rep_skb, TSN_ATTR_QCI_SGI);
	if (!sgiattr)
		return -EMSGSIZE;

	nla_put_u32(rep_skb, TSN_QCI_SGI_ATTR_INDEX, sgi_handle);

	/* Gate enable? sgiadmin.gate_enabled */
	if (sgiadmin.gate_enabled)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_ENABLE);
	else
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_DISABLE);

	if (sgiadmin.config_change)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_CONFCHANGE);

	if (sgiadmin.block_invalid_rx_enable)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_IRXEN);

	if (sgiadmin.block_invalid_rx)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_IRX);

	if (sgiadmin.block_octets_exceeded_enable)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_OEXEN);

	if (sgiadmin.block_octets_exceeded)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_OEX);

	/* Administration Down 2 */
	adminattr = nla_nest_start(rep_skb, TSN_QCI_SGI_ATTR_ADMINENTRY);
	if (!adminattr)
		return -EMSGSIZE;

	if (sgiadmin.admin.gate_states)
		nla_put_flag(rep_skb, TSN_SGI_ATTR_CTRL_INITSTATE);

	nla_put_u32(rep_skb, TSN_SGI_ATTR_CTRL_CYTIME, sgiadmin.admin.cycle_time);

	nla_put_u32(rep_skb, TSN_SGI_ATTR_CTRL_CYTIMEEX, sgiadmin.admin.cycle_time_extension);
	NLA_PUT_U64(rep_skb, TSN_SGI_ATTR_CTRL_BTIME, sgiadmin.admin.base_time);
	nla_put_u8(rep_skb, TSN_SGI_ATTR_CTRL_INITIPV, sgiadmin.admin.init_ipv);

	listcount = sgiadmin.admin.control_list_length;
	if (!listcount)
		goto out1;

	if (!sgiadmin.admin.gcl) {
		pr_err("error: list lenghth is not zero, but no gate control list\n");
		ret = -TSN_DEVRETERR;
		goto err;
	}

	gcl = sgiadmin.admin.gcl;

	/* loop list */
	for (i = 0; i < listcount; i++) {
		s8 ipv;
		u32 ti, omax;

		if ((gcl + i) == NULL) {
			pr_err("error: list count larger than gate list buffer can get\n");
			ret = -TSN_DEVRETERR;
			kfree(sgiadmin.admin.gcl);
			goto err;
		}

		/* Adminastration entry down 3 */
		sglattr = nla_nest_start(rep_skb, TSN_SGI_ATTR_CTRL_GCLENTRY);
		if (!sglattr)
			return -EMSGSIZE;
		ipv = (gcl + i)->ipv;
		ti = (gcl + i)->time_interval;
		omax = (gcl + i)->octet_max;

		if ((gcl + i)->gate_state)
			nla_put_flag(rep_skb, TSN_SGI_ATTR_GCL_GATESTATE);

		nla_put_s8(rep_skb, TSN_SGI_ATTR_GCL_IPV, ipv);
		nla_put_u32(rep_skb, TSN_SGI_ATTR_GCL_INTERVAL, ti);
		nla_put_u32(rep_skb, TSN_SGI_ATTR_GCL_OCTMAX, omax);

		/* End administration entry down 3 */
		nla_nest_end(rep_skb, sglattr);
	}

	kfree(sgiadmin.admin.gcl);
	nla_put_u8(rep_skb, TSN_SGI_ATTR_CTRL_LEN, listcount);

out1:
	/* End adminastration down 2 */
	nla_nest_end(rep_skb, adminattr);

	/* End down 1 */
	nla_nest_end(rep_skb, sgiattr);

	return tsn_send_reply(rep_skb, info);
err:
	nlmsg_free(rep_skb);
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, ret);
	return -1;
}

static int tsn_qci_sgi_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_sgi_get(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_sgi_status_get(struct genl_info *info)
{
	struct nlattr *na, *sgiattr, *operattr, *sglattr;
	struct nlattr *sgi[TSN_QCI_SGI_ATTR_MAX + 1];
	struct sk_buff *rep_skb;
	int ret;
	struct net_device *netdev;
	struct genlmsghdr *genlhdr;
	struct tsn_psfp_sgi_status sgistat;
	struct tsn_qci_psfp_gcl *gcl = NULL;
	const struct tsn_ops *tsnops;
	u16 sgi_handle;
	u8 listcount;
	int valid, i;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_QCI_SGI]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		pr_err("tsn: no sgi handle input \n");
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_QCI_SGI];

	ret = NLA_PARSE_NESTED(sgi, TSN_QCI_SGI_ATTR_MAX, na, qci_sgi_policy);
	if (ret) {
		return -EINVAL;
	}

	if (!sgi[TSN_QCI_SGI_ATTR_INDEX]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		pr_err("tsn: no sgi handle input \n");
		return -EINVAL;
	}

	sgi_handle = nla_get_u32(sgi[TSN_QCI_SGI_ATTR_INDEX]);

	/* Get status data from device */
	genlhdr = info->genlhdr;

	tsnops = netdev->tsn_ops;

	memset(&sgistat, 0, sizeof(struct tsn_psfp_sgi_status));

	if (!tsnops->qci_sgi_status_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	valid = tsnops->qci_sgi_status_get(netdev, sgi_handle, &sgistat);
	if (valid < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -1;
	}

	/* Form netlink reply data */
	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	/* Down 1 */
	sgiattr = nla_nest_start(rep_skb, TSN_ATTR_QCI_SGI);
	if (!sgiattr)
		return -EMSGSIZE;

	nla_put_u32(rep_skb, TSN_QCI_SGI_ATTR_INDEX, sgi_handle);

	/* Gate enable? */
	if (valid == 1)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_ENABLE);
	else
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_DISABLE);

	nla_put_u32(rep_skb, TSN_QCI_SGI_ATTR_TICKG, sgistat.tick_granularity);
	NLA_PUT_U64(rep_skb, TSN_QCI_SGI_ATTR_CCTIME, sgistat.config_change_time);
	NLA_PUT_U64(rep_skb, TSN_QCI_SGI_ATTR_CUTIME, sgistat.current_time);
	NLA_PUT_U64(rep_skb, TSN_QCI_SGI_ATTR_CCERROR, sgistat.config_change_error);

	if (sgistat.config_pending)
		nla_put_flag(rep_skb, TSN_QCI_SGI_ATTR_CPENDING);

	/* operation Down 2 */
	operattr = nla_nest_start(rep_skb, TSN_QCI_SGI_ATTR_OPERENTRY);
	if (!operattr)
		return -EMSGSIZE;

	if (sgistat.oper.gate_states)
		nla_put_flag(rep_skb, TSN_SGI_ATTR_CTRL_INITSTATE);

	nla_put_u32(rep_skb, TSN_SGI_ATTR_CTRL_CYTIME, sgistat.oper.cycle_time);

	nla_put_u32(rep_skb, TSN_SGI_ATTR_CTRL_CYTIMEEX, sgistat.oper.cycle_time_extension);
	NLA_PUT_U64(rep_skb, TSN_SGI_ATTR_CTRL_BTIME, sgistat.oper.base_time);
	nla_put_u8(rep_skb, TSN_SGI_ATTR_CTRL_INITIPV, sgistat.oper.init_ipv);

	/* Loop list */
	listcount = sgistat.oper.control_list_length;
	if (!listcount)
		goto out1;

	if (sgistat.oper.gcl == NULL) {
		pr_err("error: list lenghth is not zero, but no gate control list\n");
		ret = -TSN_DEVRETERR;
		goto err;
	}

	gcl = sgistat.oper.gcl;

	/* loop list */
	for (i = 0; i < listcount; i++) {
		s8 ipv;
		u32 ti, omax;

		if ((gcl + i) == NULL) {
			pr_err("error: list count larger than gate list buffer can get\n");
			ret = -TSN_DEVRETERR;
			kfree(sgistat.oper.gcl);
			goto err;
		}

		/* Operation entry down 3 */
		sglattr = nla_nest_start(rep_skb, TSN_SGI_ATTR_CTRL_GCLENTRY);
		if (!sglattr)
			return -EMSGSIZE;
		ipv = (gcl + i)->ipv;
		ti = (gcl + i)->time_interval;
		omax = (gcl + i)->octet_max;

		if ((gcl + i)->gate_state)
			nla_put_flag(rep_skb, TSN_SGI_ATTR_GCL_GATESTATE);

		nla_put_s8(rep_skb, TSN_SGI_ATTR_GCL_IPV, ipv);
		nla_put_u32(rep_skb, TSN_SGI_ATTR_GCL_INTERVAL, ti);
		nla_put_u32(rep_skb, TSN_SGI_ATTR_GCL_OCTMAX, omax);

		pr_info("tsn: gate: %d  ipv: %d  time: %d octet: %d\n", (gcl + i)->gate_state, ipv, ti, omax);

		/* End operation entry down 3 */
		nla_nest_end(rep_skb, sglattr);
	}

	kfree(sgistat.oper.gcl);
	nla_put_u8(rep_skb, TSN_SGI_ATTR_CTRL_LEN, listcount);
out1:
	/* End operation down 2 */
	nla_nest_end(rep_skb, operattr);

	/* End down 1 */
	nla_nest_end(rep_skb, sgiattr);

	return tsn_send_reply(rep_skb, info);
err:
	nlmsg_free(rep_skb);
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
	return -1;
}

static int tsn_qci_sgi_status_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_sgi_status_get(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_fmi_set(struct genl_info *info)
{
	struct nlattr *na, *fmi[TSN_QCI_FMI_ATTR_MAX + 1];
	u32 index;
	int ret;
	struct net_device *netdev;
	struct tsn_qci_psfp_fmi fmiconf;
	const struct tsn_ops *tsnops;
	bool enable = 0;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	tsnops = netdev->tsn_ops;

	memset(&fmiconf, 0, sizeof(struct tsn_qci_psfp_fmi));

	if (!info->attrs[TSN_ATTR_QCI_FMI])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_QCI_FMI];

	ret = NLA_PARSE_NESTED(fmi, TSN_QCI_FMI_ATTR_MAX, na, qci_fmi_policy);
	if (ret) {
		pr_info("tsn: parse value TSN_QCI_FMI_ATTR_MAX  error.");
		return -EINVAL;
	}

	if (!fmi[TSN_QCI_FMI_ATTR_INDEX])
		return -EINVAL;

	index = nla_get_u32(fmi[TSN_QCI_FMI_ATTR_INDEX]);

	if (fmi[TSN_QCI_FMI_ATTR_DISABLE])
		goto loaddev;

	enable = 1;

	if (fmi[TSN_QCI_FMI_ATTR_CIR])
		fmiconf.cir = nla_get_u32(fmi[TSN_QCI_FMI_ATTR_CIR]);

	if (fmi[TSN_QCI_FMI_ATTR_CBS])
		fmiconf.cbs = nla_get_u32(fmi[TSN_QCI_FMI_ATTR_CBS]);

	if (fmi[TSN_QCI_FMI_ATTR_EIR])
		fmiconf.eir = nla_get_u32(fmi[TSN_QCI_FMI_ATTR_EIR]);

	if (fmi[TSN_QCI_FMI_ATTR_EBS])
		fmiconf.ebs = nla_get_u32(fmi[TSN_QCI_FMI_ATTR_EBS]);

	if (fmi[TSN_QCI_FMI_ATTR_CF])
		fmiconf.cf = 1;

	if (fmi[TSN_QCI_FMI_ATTR_CM])
		fmiconf.cm = 1;

	if (fmi[TSN_QCI_FMI_ATTR_DROPYL])
		fmiconf.drop_on_yellow = 1;

	if (fmi[TSN_QCI_FMI_ATTR_MAREDEN])
		fmiconf.mark_red_enable = 1;

	if (fmi[TSN_QCI_FMI_ATTR_MARED])
		fmiconf.mark_red = 1;

loaddev:

	if (!tsnops->qci_fmi_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EINVAL;
	}

	tsnops->qci_fmi_set(netdev, index, enable, &fmiconf);

	if (tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS))
		return -1;

	return 0;
}

static int tsn_qci_fmi_set(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_fmi_set(info);
		return 0;
	}

	return -1;
}

static int cmd_qci_fmi_get(struct genl_info *info)
{
	struct nlattr *na, *fmi[TSN_QCI_FMI_ATTR_MAX + 1], *fmiattr;
	u32 index;
	struct sk_buff *rep_skb;
	int ret;
	struct net_device *netdev;
	struct tsn_qci_psfp_fmi fmiconf;
	struct tsn_qci_psfp_fmi_counters counters;
	const struct tsn_ops *tsnops;
	struct genlmsghdr *genlhdr;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_QCI_FMI])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_QCI_FMI];

	ret = NLA_PARSE_NESTED(fmi, TSN_QCI_FMI_ATTR_MAX, na, qci_fmi_policy);
	if (ret) {
		pr_info("tsn: parse value TSN_QCI_FMI_ATTR_MAX  error.");
		return -EINVAL;
	}

	if (!fmi[TSN_QCI_FMI_ATTR_INDEX])
		return -EINVAL;

	index = nla_get_u32(fmi[TSN_QCI_FMI_ATTR_INDEX]);

	/* Get data from device */
	tsnops = netdev->tsn_ops;

	memset(&fmiconf, 0, sizeof(struct tsn_qci_psfp_fmi));
	memset(&counters, 0, sizeof(struct tsn_qci_psfp_fmi_counters));

	if (!tsnops->qci_fmi_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EINVAL;
	}

	tsnops->qci_fmi_get(netdev, index, &fmiconf, &counters);

	genlhdr = info->genlhdr;

	/* Form netlink reply data */
	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	fmiattr = nla_nest_start(rep_skb, TSN_ATTR_QCI_FMI);
	if (!fmiattr)
		return -EMSGSIZE;

	nla_put_u32(rep_skb, TSN_QCI_FMI_ATTR_INDEX, index);

	nla_put_u32(rep_skb, TSN_QCI_FMI_ATTR_CIR, fmiconf.cir);
	nla_put_u32(rep_skb, TSN_QCI_FMI_ATTR_CBS, fmiconf.cbs);
	nla_put_u32(rep_skb, TSN_QCI_FMI_ATTR_EIR, fmiconf.eir);
	nla_put_u32(rep_skb, TSN_QCI_FMI_ATTR_EBS, fmiconf.ebs);

	if (fmiconf.cf)
		nla_put_flag(rep_skb, TSN_QCI_FMI_ATTR_CF);

	if (fmiconf.cm)
		nla_put_flag(rep_skb, TSN_QCI_FMI_ATTR_CM);

	if (fmiconf.drop_on_yellow)
		nla_put_flag(rep_skb, TSN_QCI_FMI_ATTR_DROPYL);

	if (fmiconf.mark_red_enable)
		nla_put_flag(rep_skb, TSN_QCI_FMI_ATTR_MAREDEN);

	if (fmiconf.mark_red)
		nla_put_flag(rep_skb, TSN_QCI_FMI_ATTR_MAREDEN);

	nla_put(rep_skb, TSN_QCI_FMI_ATTR_COUNTERS,
		sizeof(struct tsn_qci_psfp_fmi_counters), &counters);

	nla_nest_end(rep_skb, fmiattr);

	return tsn_send_reply(rep_skb, info);

	nlmsg_free(rep_skb);
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
	return -1;
}

static int tsn_qci_fmi_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qci_fmi_get(info);
		return 0;
	}

	return -1;
}

static int cmd_qbv_set(struct genl_info *info)
{
	struct nlattr *na, *na1;
	struct nlattr *qbv_table, *qbv[TSN_QBV_ATTR_MAX + 1], *qbvctrl[TSN_QBV_ATTR_CTRL_MAX + 1];
	int rem;
	int ret = 0;
	struct net_device *netdev;
	struct tsn_qbv_conf qbvconfig;
	const struct tsn_ops *tsnops;

	struct tsn_qbv_entry *gatelist = NULL;
	int count = 0;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	tsnops = netdev->tsn_ops;

	memset(&qbvconfig, 0, sizeof(struct tsn_qbv_conf));

	if (!info->attrs[TSN_ATTR_QBV])
		return -EINVAL;

	na = info->attrs[TSN_ATTR_QBV];

	ret = NLA_PARSE_NESTED(qbv, TSN_QBV_ATTR_MAX, na, qbv_policy);
	if (ret) {
		return -EINVAL;
	}

	if (qbv[TSN_QBV_ATTR_ENABLE]) {
		qbvconfig.gate_enabled = 1;
	} else {
		goto setdrive;
	}

	if (qbv[TSN_QBV_ATTR_CONFIGCHANGE]) {
		qbvconfig.config_change = 1;
	}

	if (!qbv[TSN_QBV_ATTR_ADMINENTRY]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -1;
	}

	na1 = qbv[TSN_QBV_ATTR_ADMINENTRY];
	NLA_PARSE_NESTED(qbvctrl, TSN_QBV_ATTR_CTRL_MAX, na1, qbv_ctrl_policy);

	if (qbvctrl[TSN_QBV_ATTR_CTRL_CYCLETIME]) {
		qbvconfig.admin.cycle_time = nla_get_u32(qbvctrl[TSN_QBV_ATTR_CTRL_CYCLETIME]);
	}

	if (qbvctrl[TSN_QBV_ATTR_CTRL_CYCLETIMEEXT]) {
		qbvconfig.admin.cycle_time_extension = nla_get_u32(qbvctrl[TSN_QBV_ATTR_CTRL_CYCLETIMEEXT]);
	}

	if (qbvctrl[TSN_QBV_ATTR_CTRL_BASETIME]) {
		qbvconfig.admin.base_time = nla_get_u64(qbvctrl[TSN_QBV_ATTR_CTRL_BASETIME]);
	}

	if (qbvctrl[TSN_QBV_ATTR_CTRL_GATESTATE]) {
		qbvconfig.admin.gate_states = nla_get_u8(qbvctrl[TSN_QBV_ATTR_CTRL_GATESTATE]);
	}

	if (qbvctrl[TSN_QBV_ATTR_CTRL_LISTCOUNT]) {
		int listcount;

		listcount = nla_get_u32(qbvctrl[TSN_QBV_ATTR_CTRL_LISTCOUNT]);

		qbvconfig.admin.control_list_length = listcount;

		gatelist = (struct tsn_qbv_entry *)kmalloc(listcount * sizeof(struct tsn_qbv_entry), GFP_KERNEL);

		nla_for_each_nested(qbv_table, na1, rem) {
			struct nlattr *qbv_entry[TSN_QBV_ATTR_ENTRY_MAX + 1];

			if (nla_type(qbv_table) != TSN_QBV_ATTR_CTRL_LISTENTRY)
				continue;

			ret = NLA_PARSE_NESTED(qbv_entry, TSN_QBV_ATTR_ENTRY_MAX, qbv_table, qbv_entry_policy);
			if (ret) {
				return -EINVAL;
			}

			(gatelist + count)->gate_state = nla_get_u8(qbv_entry[TSN_QBV_ATTR_ENTRY_GC]);
			(gatelist + count)->time_interval = nla_get_u32(qbv_entry[TSN_QBV_ATTR_ENTRY_TM]);
			count++;
			if (count > listcount)
				break;
		}
	}

	if (gatelist != NULL)
		qbvconfig.admin.control_list = gatelist;

setdrive:
	if (!tsnops->qbv_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		goto err;
	}

	ret = tsnops->qbv_set(netdev, &qbvconfig);

	/* send back */
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);

err:
	if (gatelist != NULL)
		kfree(gatelist);

	return ret;
}

static int tsn_qbv_set(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qbv_set(info);
		return 0;
	}

	return -1;
}

static int cmd_qbv_get(struct genl_info *info)
{
	struct nlattr *qbv, *qbvadminattr;
	struct sk_buff *rep_skb;
	int ret;
	int len = 0, i = 0;
	struct net_device *netdev;
	struct genlmsghdr *genlhdr;
	struct tsn_qbv_conf qbvconf;
	const struct tsn_ops *tsnops;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	genlhdr = info->genlhdr;

	tsnops = netdev->tsn_ops;

	memset(&qbvconf, 0, sizeof(struct tsn_qbv_conf));

	if (!tsnops->qbv_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = tsnops->qbv_get(netdev, &qbvconf);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return ret;
	}

	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	qbv = nla_nest_start(rep_skb, TSN_ATTR_QBV);
	if (!qbv)
		return -EMSGSIZE;

	qbvadminattr = nla_nest_start(rep_skb, TSN_QBV_ATTR_ADMINENTRY);

	if (qbvconf.admin.control_list != NULL) {
		/* TODO: should add a nested attribute wrap the admin/oper para list */
		len = qbvconf.admin.control_list_length;
		nla_put_u32(rep_skb, TSN_QBV_ATTR_CTRL_LISTCOUNT, len);

		for (i = 0; i < len; i++) {
			struct nlattr *qbv_table;
			u8 gs = (qbvconf.admin.control_list + i)->gate_state;
			u32 tp = (qbvconf.admin.control_list + i)->time_interval;

			qbv_table = nla_nest_start(rep_skb, TSN_QBV_ATTR_CTRL_LISTENTRY);

			nla_put_u32(rep_skb, TSN_QBV_ATTR_ENTRY_ID, i);
			nla_put_u8(rep_skb, TSN_QBV_ATTR_ENTRY_GC, gs);
			nla_put_u32(rep_skb, TSN_QBV_ATTR_ENTRY_TM, tp);
			nla_nest_end(rep_skb, qbv_table);
		}

		if (qbvconf.admin.gate_states)
			nla_put_u8(rep_skb, TSN_QBV_ATTR_CTRL_GATESTATE, qbvconf.admin.gate_states);

		if (qbvconf.admin.cycle_time)
			nla_put_u32(rep_skb, TSN_QBV_ATTR_CTRL_CYCLETIME, qbvconf.admin.cycle_time);

		if (qbvconf.admin.cycle_time_extension)
			nla_put_u32(rep_skb, TSN_QBV_ATTR_CTRL_CYCLETIMEEXT, qbvconf.admin.cycle_time_extension);

		if (qbvconf.admin.base_time)
			NLA_PUT_U64(rep_skb, TSN_QBV_ATTR_CTRL_BASETIME, qbvconf.admin.base_time);

		kfree(qbvconf.admin.control_list);

	} else
		pr_info("tsn: error get administrator data.");

	nla_nest_end(rep_skb, qbvadminattr);

	if (qbvconf.gate_enabled)
		nla_put_flag(rep_skb, TSN_QBV_ATTR_ENABLE);
	else
		nla_put_flag(rep_skb, TSN_QBV_ATTR_DISABLE);

	if (qbvconf.maxsdu)
		nla_put_u32(rep_skb, TSN_QBV_ATTR_MAXSDU, qbvconf.maxsdu);

	if (qbvconf.config_change)
		nla_put_flag(rep_skb, TSN_QBV_ATTR_CONFIGCHANGE);

	nla_nest_end(rep_skb, qbv);

	tsn_send_reply(rep_skb, info);

	return ret;
}

static int cmd_qbv_status_get(struct genl_info *info)
{
	struct nlattr *qbv, *qbvoperattr;
	struct sk_buff *rep_skb;
	int ret;
	int len = 0, i = 0;
	struct net_device *netdev;
	struct genlmsghdr *genlhdr;
	struct tsn_qbv_status qbvstatus;
	const struct tsn_ops *tsnops;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	genlhdr = info->genlhdr;

	tsnops = netdev->tsn_ops;

	memset(&qbvstatus, 0, sizeof(struct tsn_qbv_status));

	if (!tsnops->qbv_get_status) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = tsnops->qbv_get_status(netdev, &qbvstatus);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return ret;
	}

	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	qbv = nla_nest_start(rep_skb, TSN_ATTR_QBV);
	if (!qbv)
		return -EMSGSIZE;

	qbvoperattr = nla_nest_start(rep_skb, TSN_QBV_ATTR_OPERENTRY);

	if (qbvstatus.oper.control_list != NULL) {
		len = qbvstatus.oper.control_list_length;
		nla_put_u32(rep_skb, TSN_QBV_ATTR_CTRL_LISTCOUNT, len);
		for (i = 0; i < len; i++) {
			struct nlattr *qbv_table;
			u8 gs = (qbvstatus.oper.control_list + i)->gate_state;
			u32 tp = (qbvstatus.oper.control_list + i)->time_interval;

			qbv_table = nla_nest_start(rep_skb, TSN_QBV_ATTR_CTRL_LISTENTRY);

			nla_put_u32(rep_skb, TSN_QBV_ATTR_ENTRY_ID, i);
			nla_put_u8(rep_skb, TSN_QBV_ATTR_ENTRY_GC, gs);
			nla_put_u32(rep_skb, TSN_QBV_ATTR_ENTRY_TM, tp);
			nla_nest_end(rep_skb, qbv_table);
		}

		if (qbvstatus.oper.gate_states)
			nla_put_u8(rep_skb, TSN_QBV_ATTR_CTRL_GATESTATE, qbvstatus.oper.gate_states);

		if (qbvstatus.oper.cycle_time)
			nla_put_u32(rep_skb, TSN_QBV_ATTR_CTRL_CYCLETIME, qbvstatus.oper.cycle_time);

		if (qbvstatus.oper.cycle_time_extension)
			nla_put_u32(rep_skb, TSN_QBV_ATTR_CTRL_CYCLETIMEEXT, qbvstatus.oper.cycle_time_extension);

		if (qbvstatus.oper.base_time)
			NLA_PUT_U64(rep_skb, TSN_QBV_ATTR_CTRL_BASETIME, qbvstatus.oper.base_time);

		kfree(qbvstatus.oper.control_list);
	} else {
		pr_info("tsn: error get operation list data.");
	}

	nla_nest_end(rep_skb, qbvoperattr);

	if (qbvstatus.config_change_time)
		NLA_PUT_U64(rep_skb, TSN_QBV_ATTR_CONFIGCHANGETIME, qbvstatus.config_change_time);

	if (qbvstatus.tick_granularity)
		nla_put_u32(rep_skb, TSN_QBV_ATTR_GRANULARITY, qbvstatus.tick_granularity);

	if (qbvstatus.current_time)
		NLA_PUT_U64(rep_skb, TSN_QBV_ATTR_CURRENTTIME, qbvstatus.current_time);

	if (qbvstatus.config_pending)
		nla_put_flag(rep_skb, TSN_QBV_ATTR_CONFIGPENDING);

	if (qbvstatus.config_change_error)
		NLA_PUT_U64(rep_skb, TSN_QBV_ATTR_CONFIGCHANGEERROR, qbvstatus.config_change_error);

	if (qbvstatus.supported_list_max)
		nla_put_u32(rep_skb, TSN_QBV_ATTR_LISTMAX, qbvstatus.supported_list_max);

	nla_nest_end(rep_skb, qbv);

	tsn_send_reply(rep_skb, info);

	return ret;
}

static int tsn_qbv_status_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qbv_status_get(info);
	}
	return 0;
}

static int tsn_qbv_get(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[TSN_ATTR_IFNAME]) {
		cmd_qbv_get(info);
	}
	return 0;
}

static int tsn_cbs_set(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *cbsa[TSN_CBS_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	int ret;
	u8 tc, bw;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_CBS]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_CBS];


	tsnops = netdev->tsn_ops;

	if (!tsnops->cbs_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = NLA_PARSE_NESTED(cbsa, TSN_CBS_ATTR_MAX, na, cbs_policy);
	if (ret) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	if (!cbsa[TSN_CBS_ATTR_TC_INDEX]) {
		pr_err("tsn: no TSN_CBS_ATTR_TC_INDEX input \n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}
	tc = nla_get_u8(cbsa[TSN_CBS_ATTR_TC_INDEX]);


	if (!cbsa[TSN_CBS_ATTR_BW]) {
		pr_err("tsn: no TSN_CBS_ATTR_BW input \n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	bw = nla_get_u8(cbsa[TSN_CBS_ATTR_BW]);
	if (bw > 100) {
		pr_err("tsn: TSN_CBS_ATTR_BW isn't in the range of 0~100 \n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	ret = tsnops->cbs_set(netdev, tc, bw);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);
	return 0;
}

static int tsn_cbs_get(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na, *cbsattr;
	struct nlattr *cbsa[TSN_CBS_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	struct sk_buff *rep_skb;
	int ret;
	struct genlmsghdr *genlhdr;
	u8 tc;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_CBS]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	tsnops = netdev->tsn_ops;

	if (!tsnops->cbs_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	na = info->attrs[TSN_ATTR_CBS];
	ret = NLA_PARSE_NESTED(cbsa, TSN_CBS_ATTR_MAX, na, cbs_policy);
	if (ret) {
		pr_err("tsn: parse value TSN_CBS_ATTR_MAX error.");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	/* Get status data from device */
	genlhdr = info->genlhdr;

	/* Form netlink reply data */
	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb,
			NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	cbsattr = nla_nest_start(rep_skb, TSN_ATTR_CBS);
	if (!cbsattr)
		return -EMSGSIZE;

	if (!cbsa[TSN_CBS_ATTR_TC_INDEX]) {
		pr_err("tsn: must to specify the TSN_CBS_ATTR_TC_INDEX attribute\n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}
	tc = nla_get_u8(cbsa[TSN_CBS_ATTR_TC_INDEX]);

	ret = tsnops->cbs_get(netdev, tc);
	if (ret < 0) {
		pr_err("tsn: cbs_get return error\n");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	nla_put_u8(rep_skb, TSN_CBS_ATTR_BW, ret & 0XF);

	nla_nest_end(rep_skb, cbsattr);
	return tsn_send_reply(rep_skb, info);
}

static int cmd_qbu_set(struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *qbua[TSN_QBU_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	int ret;
	u8 preemptable = 0;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_QBU]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_QBU];

	ret = NLA_PARSE_NESTED(qbua, TSN_QBU_ATTR_MAX, na, qbu_policy);
	if (ret) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	if (qbua[TSN_QBU_ATTR_ADMIN_STATE])
		preemptable = nla_get_u8(qbua[TSN_QBU_ATTR_ADMIN_STATE]);
	else
		pr_info("Disable Qbu since no preemptable TSN_QBU_ATTR_ADMIN_STATE config!\n");

	tsnops = netdev->tsn_ops;

	if (!tsnops->qbu_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EINVAL;
	}

	ret = tsnops->qbu_set(netdev, preemptable);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}

	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);
	return 0;
}

static int tsn_qbu_set(struct sk_buff *skb, struct genl_info *info)
{
	pr_info("tsn_qbu_set receive message\n");
	if (info->attrs[TSN_ATTR_IFNAME]) {
		return cmd_qbu_set(info);
	}

	return -1;
}

static int cmd_qbu_get_status(struct genl_info *info)
{
	struct nlattr *qbuattr;
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	struct sk_buff *rep_skb;
	int ret;
	struct genlmsghdr *genlhdr;
	struct tsn_preempt_status pps;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	/* Get status data from device */
	genlhdr = info->genlhdr;

	tsnops = netdev->tsn_ops;

	memset(&pps, 0, sizeof(struct tsn_preempt_status));

	if (!tsnops->qbu_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = tsnops->qbu_get(netdev, &pps);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -1;
	}

	/* Form netlink reply data */
	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb, NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	qbuattr = nla_nest_start(rep_skb, TSN_ATTR_QBU);
	if (!qbuattr)
		return -EMSGSIZE;

	nla_put_u8(rep_skb, TSN_QBU_ATTR_ADMIN_STATE, pps.admin_state);
	nla_put_u32(rep_skb, TSN_QBU_ATTR_HOLD_ADVANCE, pps.hold_advance);
	nla_put_u32(rep_skb, TSN_QBU_ATTR_RELEASE_ADVANCE, pps.release_advance);
	if (pps.preemption_active)
		nla_put_flag(rep_skb, TSN_QBU_ATTR_ACTIVE);

	nla_put_u8(rep_skb, TSN_QBU_ATTR_HOLD_REQUEST, pps.hold_request);
	nla_nest_end(rep_skb, qbuattr);

	return tsn_send_reply(rep_skb, info);
}

static int tsn_qbu_get_status(struct sk_buff *skb, struct genl_info *info)
{
	pr_info("tsn_qbu_get_status receive message\n");
	if (info->attrs[TSN_ATTR_IFNAME]) {
		return cmd_qbu_get_status(info);
	}

	return -1;
}

static int tsn_tsd_set(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *ntsd[TSN_TSD_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	struct tsn_tsd tsd;
	int ret;

	memset(&tsd, 0, sizeof(struct tsn_tsd));

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_TSD]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_TSD];

	ret = NLA_PARSE_NESTED(ntsd, TSN_TSD_ATTR_MAX, na, tsd_policy);
	if (ret) {
		printk("tsn: parse value TSN_TSD_ATTR_MAX error.");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	if (!netdev->tsn_ops)
		return -EINVAL;

	tsnops = netdev->tsn_ops;

	if (!tsnops->tsd_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -EINVAL;
	}

	if (nla_get_flag(ntsd[TSN_TSD_ATTR_DISABLE])) {
		tsd.enable = false;
	} else {
		if (ntsd[TSN_TSD_ATTR_PERIOD]) {
			tsd.period = nla_get_u32(ntsd[TSN_TSD_ATTR_PERIOD]);
		}

		if (!tsd.period) {
			printk("tsn: parse value TSN_TSD_ATTR_PERIOD error.");
				tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
				return -EINVAL;
		}

		if (ntsd[TSN_TSD_ATTR_MAX_FRM_NUM])
			tsd.maxFrameNum = nla_get_u32(ntsd[TSN_TSD_ATTR_MAX_FRM_NUM]);

		if (ntsd[TSN_TSD_ATTR_SYN_IMME])
			tsd.syn_flag = 2; // Cycle timer begins immediately.
		else
			tsd.syn_flag = 1; // TSD enable and Cycle timer will begin at the first frame coming.

		tsd.enable = true;
	}

	ret = tsnops->tsd_set(netdev, &tsd);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}

	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);
	return 0;
}

static int tsn_tsd_get(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na, *tsdattr;
	struct nlattr *tsda[TSN_TSD_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	struct sk_buff *rep_skb;
	int ret;
	struct genlmsghdr *genlhdr;
	struct tsn_tsd_status tts;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_TSD]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	tsnops = netdev->tsn_ops;

	if (!tsnops->tsd_get) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = tsnops->tsd_get(netdev, &tts);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -1;
	}

	na = info->attrs[TSN_ATTR_TSD];

	ret = NLA_PARSE_NESTED(tsda, TSN_TSD_ATTR_MAX, na, tsd_policy);
	if (ret) {
		pr_err("tsn: parse value TSN_TSD_ATTR_MAX error.");
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	/* Get status data from device */
	genlhdr = info->genlhdr;

	/* Form netlink reply data */
	ret = tsn_prepare_reply(info, genlhdr->cmd, &rep_skb,
			NLMSG_ALIGN(MAX_ATTR_SIZE));
	if (ret < 0)
		return ret;

	if (nla_put_string(rep_skb, TSN_ATTR_IFNAME, netdev->name))
		return -EMSGSIZE;

	tsdattr = nla_nest_start(rep_skb, TSN_ATTR_TSD);
	if (!tsdattr)
		return -EMSGSIZE;

	nla_put_u32(rep_skb, TSN_TSD_ATTR_PERIOD, tts.period);
	nla_put_u32(rep_skb, TSN_TSD_ATTR_MAX_FRM_NUM, tts.maxFrameNum);
	nla_put_u32(rep_skb, TSN_TSD_ATTR_CYCLE_NUM, tts.cycleNum);
	nla_put_u32(rep_skb, TSN_TSD_ATTR_LOSS_STEPS, tts.loss_steps);
	nla_put_u32(rep_skb, TSN_TSD_ATTR_MAX_FRM_NUM, tts.maxFrameNum);

	if (!tts.enable)
		nla_put_flag(rep_skb, TSN_TSD_ATTR_DISABLE);
	else
		nla_put_flag(rep_skb, TSN_TSD_ATTR_ENABLE);

	if (tts.flag == 2)
		nla_put_flag(rep_skb, TSN_TSD_ATTR_SYN_IMME);

	nla_nest_end(rep_skb, tsdattr);
	return tsn_send_reply(rep_skb, info);
}

static int tsn_ct_set(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *cta[TSN_CT_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	int ret;
	u8 queue_stat;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_CT]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_CT];


	tsnops = netdev->tsn_ops;

	if (!tsnops->ct_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = NLA_PARSE_NESTED(cta, TSN_CT_ATTR_MAX, na, ct_policy);
	if (ret) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	queue_stat = nla_get_u8(cta[TSN_CT_ATTR_QUEUE_STATE]);

	ret = tsnops->ct_set(netdev, queue_stat);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);
	return 0;
}

static int tsn_cbgen_set(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *cbgena[TSN_CBGEN_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	int ret;
	u32 index;
	struct tsn_seq_gen_conf sg_conf;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_CBGEN]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_CBGEN];


	tsnops = netdev->tsn_ops;

	if (!tsnops->cbgen_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = NLA_PARSE_NESTED(cbgena, TSN_CBGEN_ATTR_MAX, na, cbgen_policy);
	if (ret) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	index = nla_get_u32(cbgena[TSN_CBGEN_ATTR_INDEX]);

	memset(&sg_conf, 0, sizeof(struct tsn_seq_gen_conf));
	sg_conf.iport_mask = nla_get_u8(cbgena[TSN_CBGEN_ATTR_PORT_MASK]);
	sg_conf.split_mask = nla_get_u8(cbgena[TSN_CBGEN_ATTR_SPLIT_MASK]);
	sg_conf.seq_len = nla_get_u8(cbgena[TSN_CBGEN_ATTR_SEQ_LEN]);
	sg_conf.seq_num = nla_get_u32(cbgena[TSN_CBGEN_ATTR_SEQ_NUM]);

	ret = tsnops->cbgen_set(netdev, index, &sg_conf);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);
	return 0;
}


static int tsn_cbrec_set(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *cbreca[TSN_CBREC_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	int ret;
	u32 index;
	struct tsn_seq_rec_conf sr_conf;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_CBREC]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_CBREC];


	tsnops = netdev->tsn_ops;

	if (!tsnops->cbrec_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = NLA_PARSE_NESTED(cbreca, TSN_CBREC_ATTR_MAX, na, cbrec_policy);
	if (ret) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	index = nla_get_u32(cbreca[TSN_CBREC_ATTR_INDEX]);

	memset(&sr_conf, 0, sizeof(struct tsn_seq_rec_conf));
	sr_conf.seq_len = nla_get_u8(cbreca[TSN_CBREC_ATTR_SEQ_LEN]);
	sr_conf.his_len = nla_get_u8(cbreca[TSN_CBREC_ATTR_HIS_LEN]);
	sr_conf.rtag_pop_en = nla_get_flag(cbreca[TSN_CBREC_ATTR_TAG_POP_EN]);

	ret = tsnops->cbrec_set(netdev, index, &sr_conf);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);
	return 0;
}

static int tsn_pcpmap_set(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na;
	struct nlattr *pcpmapa[TSN_PCPMAP_ATTR_MAX + 1];
	struct net_device *netdev;
	const struct tsn_ops *tsnops;
	int ret;
	bool enable = 0;

	ret = tsn_init_check(info, &netdev);
	if (ret)
		return ret;

	if (!info->attrs[TSN_ATTR_PCPMAP]) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	na = info->attrs[TSN_ATTR_PCPMAP];


	tsnops = netdev->tsn_ops;

	if (!tsnops->pcpmap_set) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_NODEVOPS);
		return -1;
	}

	ret = NLA_PARSE_NESTED(pcpmapa, TSN_PCPMAP_ATTR_MAX,
			       na, pcpmap_policy);
	if (ret) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_ATTRERR);
		return -EINVAL;
	}

	enable = nla_get_flag(pcpmapa[TSN_PCPMAP_ATTR_ENABLE]);
	ret = tsnops->pcpmap_set(netdev, enable);
	if (ret < 0) {
		tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, -TSN_DEVRETERR);
		return -EINVAL;
	}
	tsn_simple_reply(info, TSN_CMD_REPLY, netdev->name, TSN_SUCCESS);

	return 0;
}

static const struct genl_ops tsnnl_ops[] = {
	{
		.cmd		= TSN_CMD_ECHO,
		.doit		= tsn_echo_cmd,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QBV_SET,
		.doit		= tsn_qbv_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QBV_GET,
		.doit		= tsn_qbv_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QBV_GET_STATUS,
		.doit		= tsn_qbv_status_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CB_STREAMID_SET,
		.doit		= tsn_cb_streamid_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CB_STREAMID_GET,
		.doit		= tsn_cb_streamid_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CB_STREAMID_GET_COUNTS,
		.doit		= tsn_cb_streamid_counters_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_SFI_SET,
		.doit		= tsn_qci_sfi_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_SFI_GET,
		.doit		= tsn_qci_sfi_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_SFI_GET_COUNTS,
		.doit		= tsn_qci_sfi_counters_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_SGI_SET,
		.doit		= tsn_qci_sgi_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_SGI_GET,
		.doit		= tsn_qci_sgi_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
#if 0
	{
		.cmd		= TSN_CMD_QCI_SGI_SET_LIST,
		.doit		= tsn_qci_sgi_list_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_SGI_GET_LIST,
		.doit		= tsn_qci_sgi_list_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
#endif
	{
		.cmd		= TSN_CMD_QCI_SGI_GET_STATUS,
		.doit		= tsn_qci_sgi_status_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_FMI_SET,
		.doit		= tsn_qci_fmi_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QCI_FMI_GET,
		.doit		= tsn_qci_fmi_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CBS_SET,
		.doit		= tsn_cbs_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CBS_GET,
		.doit		= tsn_cbs_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QBU_SET,
		.doit		= tsn_qbu_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_QBU_GET_STATUS,
		.doit		= tsn_qbu_get_status,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_TSD_SET,
		.doit		= tsn_tsd_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_TSD_GET,
		.doit		= tsn_tsd_get,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CT_SET,
		.doit		= tsn_ct_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CBGEN_SET,
		.doit		= tsn_cbgen_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_CBREC_SET,
		.doit		= tsn_cbrec_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
	{
		.cmd		= TSN_CMD_PCPMAP_SET,
		.doit		= tsn_pcpmap_set,
		.policy		= tsn_cmd_policy,
		.flags		= GENL_ADMIN_PERM,
	},
};

static struct genl_family tsn_family = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0)
	.id			= GENL_ID_GENERATE,
#endif
/*	.hdrsize	= NLMSG_ALIGN(MAX_USER_SIZE),*/
	.name		= TSN_GENL_NAME,
	.version	= TSN_GENL_VERSION,
	.maxattr	= TSN_CMD_ATTR_MAX,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	.module		= THIS_MODULE,
	.ops		= tsnnl_ops,
	.n_ops		= ARRAY_SIZE(tsnnl_ops),
#endif
};

static int __init tsn_genetlink_init(void)
{
	int ret;

	pr_info("tsn generic netlink module v%d init...\n", TSN_GENL_VERSION);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 10, 0)
	ret = genl_register_family_with_ops(&tsn_family, tsnnl_ops);
#else
	ret = genl_register_family(&tsn_family);
#endif
	if (ret != 0) {
		pr_info("failed to init tsn generic netlink example module\n");
		return ret;
	}

	return 0;
}

static void __exit tsn_genetlink_exit(void)
{
	int ret;

	ret = genl_unregister_family(&tsn_family);
	if (ret != 0) {
		pr_info("failed to unregister family:%i\n", ret);
	}
}

module_init(tsn_genetlink_init);
module_exit(tsn_genetlink_exit);
MODULE_LICENSE("GPL");

