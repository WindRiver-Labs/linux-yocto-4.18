/* SPDX-License-Identifier: GPL-2.0
 * Marvell OcteonTx2 RVU Admin Function driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MBOX_H
#define MBOX_H

#include <linux/etherdevice.h>
#include <linux/sizes.h>

#include "rvu_struct.h"

#define MBOX_SIZE		SZ_64K

/* AF/PF: PF initiated, PF/VF VF initiated */
#define MBOX_DOWN_RX_START	0
#define MBOX_DOWN_RX_SIZE	(46 * SZ_1K)
#define MBOX_DOWN_TX_START	(MBOX_DOWN_RX_START + MBOX_DOWN_RX_SIZE)
#define MBOX_DOWN_TX_SIZE	(16 * SZ_1K)
/* AF/PF: AF initiated, PF/VF PF initiated */
#define MBOX_UP_RX_START	(MBOX_DOWN_TX_START + MBOX_DOWN_TX_SIZE)
#define MBOX_UP_RX_SIZE		SZ_1K
#define MBOX_UP_TX_START	(MBOX_UP_RX_START + MBOX_UP_RX_SIZE)
#define MBOX_UP_TX_SIZE		SZ_1K

#if MBOX_UP_TX_SIZE + MBOX_UP_TX_START != MBOX_SIZE
# error "incorrect mailbox area sizes"
#endif

#define INTR_MASK(pfvfs) ((pfvfs < 64) ? (BIT_ULL(pfvfs) - 1) : (~0ull))

#define MBOX_RSP_TIMEOUT	1000 /* in ms, Time to wait for mbox response */

#define MBOX_MSG_ALIGN		16  /* Align mbox msg start to 16bytes */

/* Mailbox directions */
#define MBOX_DIR_AFPF		0  /* AF replies to PF */
#define MBOX_DIR_PFAF		1  /* PF sends messages to AF */
#define MBOX_DIR_PFVF		2  /* PF replies to VF */
#define MBOX_DIR_VFPF		3  /* VF sends messages to PF */
#define MBOX_DIR_AFPF_UP	4  /* AF sends messages to PF */
#define MBOX_DIR_PFAF_UP	5  /* PF replies to AF */
#define MBOX_DIR_PFVF_UP	6  /* PF sends messages to VF */
#define MBOX_DIR_VFPF_UP	7  /* VF replies to PF */

struct otx2_mbox_dev {
	void	    *mbase;   /* This dev's mbox region */
	spinlock_t  mbox_lock;
	u16         msg_size; /* Total msg size to be sent */
	u16         rsp_size; /* Total rsp size to be sure the reply is ok */
	u16         num_msgs; /* No of msgs sent or waiting for response */
	u16         msgs_acked; /* No of msgs for which response is received */
};

struct otx2_mbox {
	struct pci_dev *pdev;
	void   *hwbase;  /* Mbox region advertised by HW */
	void   *reg_base;/* CSR base for this dev */
	u64    trigger;  /* Trigger mbox notification */
	u16    tr_shift; /* Mbox trigger shift */
	u64    rx_start; /* Offset of Rx region in mbox memory */
	u64    tx_start; /* Offset of Tx region in mbox memory */
	u16    rx_size;  /* Size of Rx region */
	u16    tx_size;  /* Size of Tx region */
	u16    ndevs;    /* The number of peers */
	struct otx2_mbox_dev *dev;
};

/* Header which preceeds all mbox messages */
struct mbox_hdr {
	u16  num_msgs;   /* No of msgs embedded */
};

/* Header which preceeds every msg and is also part of it */
struct mbox_msghdr {
	u16 pcifunc;     /* Who's sending this msg */
	u16 id;          /* Mbox message ID */
#define OTX2_MBOX_REQ_SIG (0xdead)
#define OTX2_MBOX_RSP_SIG (0xbeef)
	u16 sig;         /* Signature, for validating corrupted msgs */
#define OTX2_MBOX_VERSION (0x0001)
	u16 ver;         /* Version of msg's structure for this ID */
	u16 next_msgoff; /* Offset of next msg within mailbox region */
	int rc;          /* Msg process'ed response code */
};

void otx2_mbox_reset(struct otx2_mbox *mbox, int devid);
void otx2_mbox_destroy(struct otx2_mbox *mbox);
int otx2_mbox_init(struct otx2_mbox *mbox, void __force *hwbase,
		   struct pci_dev *pdev, void __force *reg_base,
		   int direction, int ndevs);
void otx2_mbox_msg_send(struct otx2_mbox *mbox, int devid);
int otx2_mbox_wait_for_rsp(struct otx2_mbox *mbox, int devid);
int otx2_mbox_busy_poll_for_rsp(struct otx2_mbox *mbox, int devid);
struct mbox_msghdr *otx2_mbox_alloc_msg_rsp(struct otx2_mbox *mbox, int devid,
					    int size, int size_rsp);
struct mbox_msghdr *otx2_mbox_get_rsp(struct otx2_mbox *mbox, int devid,
				      struct mbox_msghdr *msg);
int otx2_reply_invalid_msg(struct otx2_mbox *mbox, int devid,
			   u16 pcifunc, u16 id);
bool otx2_mbox_nonempty(struct otx2_mbox *mbox, int devid);
const char *otx2_mbox_id2name(u16 id);
static inline struct mbox_msghdr *otx2_mbox_alloc_msg(struct otx2_mbox *mbox,
						      int devid, int size)
{
	return otx2_mbox_alloc_msg_rsp(mbox, devid, size, 0);
}

/* Mailbox message types */
#define MBOX_MSG_MASK				0xFFFF
#define MBOX_MSG_INVALID			0xFFFE
#define MBOX_MSG_MAX				0xFFFF

#define MBOX_MESSAGES							\
/* Generic mbox IDs (range 0x000 - 0x1FF) */				\
M(READY,		0x001, msg_req, ready_msg_rsp)			\
M(ATTACH_RESOURCES,	0x002, rsrc_attach, msg_rsp)			\
M(DETACH_RESOURCES,	0x003, rsrc_detach, msg_rsp)			\
M(MSIX_OFFSET,		0x004, msg_req, msix_offset_rsp)		\
/* CGX mbox IDs (range 0x200 - 0x3FF) */				\
/* NPA mbox IDs (range 0x400 - 0x5FF) */				\
/* SSO/SSOW mbox IDs (range 0x600 - 0x7FF) */				\
/* TIM mbox IDs (range 0x800 - 0x9FF) */				\
/* CPT mbox IDs (range 0xA00 - 0xBFF) */				\
/* NPC mbox IDs (range 0x6000 - 0x7FFF) */				\
/* NIX mbox IDs (range 0x8000 - 0xFFFF) */				\

enum {
#define M(_name, _id, _1, _2) MBOX_MSG_ ## _name = _id,
MBOX_MESSAGES
#undef M
};

/* Mailbox message formats */

/* Generic request msg used for those mbox messages which
 * don't send any data in the request.
 */
struct msg_req {
	struct mbox_msghdr hdr;
};

/* Generic rsponse msg used a ack or response for those mbox
 * messages which doesn't have a specific rsp msg format.
 */
struct msg_rsp {
	struct mbox_msghdr hdr;
};

struct ready_msg_rsp {
	struct mbox_msghdr hdr;
	u16    sclk_feq;	/* SCLK frequency */
};

/* Structure for requesting resource provisioning.
 * 'modify' flag to be used when either requesting more
 * or to detach partial of a cetain resource type.
 * Rest of the fields specify how many of what type to
 * be attached.
 */
struct rsrc_attach {
	struct mbox_msghdr hdr;
	u8   modify:1;
	u8   npalf:1;
	u8   nixlf:1;
	u16  sso;
	u16  ssow;
	u16  timlfs;
	u16  cptlfs;
};

/* Structure for relinquishing resources.
 * 'partial' flag to be used when relinquishing all resources
 * but only of a certain type. If not set, all resources of all
 * types provisioned to the RVU function will be detached.
 */
struct rsrc_detach {
	struct mbox_msghdr hdr;
	u8 partial:1;
	u8 npalf:1;
	u8 nixlf:1;
	u8 sso:1;
	u8 ssow:1;
	u8 timlfs:1;
	u8 cptlfs:1;
};

#define MSIX_VECTOR_INVALID	0xFFFF
#define MAX_RVU_BLKLF_CNT	256

struct msix_offset_rsp {
	struct mbox_msghdr hdr;
	u16  npa_msixoff;
	u16  nix_msixoff;
	u8   sso;
	u8   ssow;
	u8   timlfs;
	u8   cptlfs;
	u16  sso_msixoff[MAX_RVU_BLKLF_CNT];
	u16  ssow_msixoff[MAX_RVU_BLKLF_CNT];
	u16  timlf_msixoff[MAX_RVU_BLKLF_CNT];
	u16  cptlf_msixoff[MAX_RVU_BLKLF_CNT];
};

#endif /* MBOX_H */
