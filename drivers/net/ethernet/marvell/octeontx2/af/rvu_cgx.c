// SPDX-License-Identifier: GPL-2.0
/* Marvell OcteonTx2 RVU Admin Function driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "rvu.h"
#include "cgx.h"

struct cgx_evq_entry {
	struct list_head evq_node;
	struct cgx_link_event link_event;
};

static inline u8 cgxlmac_id_to_bmap(u8 cgx_id, u8 lmac_id)
{
	return ((cgx_id & 0xF) << 4) | (lmac_id & 0xF);
}

static void *rvu_cgx_pdata(u8 cgx_id, struct rvu *rvu)
{
	if (cgx_id >= rvu->cgx_cnt)
		return NULL;

	return rvu->cgx_idmap[cgx_id];
}

static int rvu_map_cgx_lmac_pf(struct rvu *rvu)
{
	int cgx_cnt = rvu->cgx_cnt;
	int cgx, lmac_cnt, lmac;
	int pf = PF_CGXMAP_BASE;
	int size;

	if (!cgx_cnt)
		return 0;

	if (cgx_cnt > 0xF || MAX_LMAC_PER_CGX > 0xF)
		return -EINVAL;

	/* Alloc map table
	 * An additional entry is required since PF id starts from 1 and
	 * hence entry at offset 0 is invalid.
	 */
	size = (cgx_cnt * MAX_LMAC_PER_CGX + 1) * sizeof(u8);
	rvu->pf2cgxlmac_map = devm_kzalloc(rvu->dev, size, GFP_KERNEL);
	if (!rvu->pf2cgxlmac_map)
		return -ENOMEM;

	/* Initialize offset 0 with an invalid cgx and lmac id */
	rvu->pf2cgxlmac_map[0] = 0xFF;

	/* Reverse map table */
	rvu->cgxlmac2pf_map = devm_kzalloc(rvu->dev,
				  cgx_cnt * MAX_LMAC_PER_CGX * sizeof(u16),
				  GFP_KERNEL);
	if (!rvu->cgxlmac2pf_map)
		return -ENOMEM;

	rvu->cgx_mapped_pfs = 0;
	for (cgx = 0; cgx < cgx_cnt; cgx++) {
		lmac_cnt = cgx_get_lmac_cnt(rvu_cgx_pdata(cgx, rvu));
		for (lmac = 0; lmac < lmac_cnt; lmac++, pf++) {
			rvu->pf2cgxlmac_map[pf] = cgxlmac_id_to_bmap(cgx, lmac);
			rvu->cgxlmac2pf_map[CGX_OFFSET(cgx) + lmac] = 1 << pf;
			rvu->cgx_mapped_pfs++;
		}
	}
	return 0;
}

/* This is called from interrupt context and is expected to be atomic */
static int cgx_lmac_postevent(struct cgx_link_event *event, void *data)
{
	struct cgx_evq_entry *qentry;
	struct rvu *rvu = data;

	/* post event to the event queue */
	qentry = kmalloc(sizeof(*qentry), GFP_ATOMIC);
	if (!qentry)
		return -ENOMEM;
	qentry->link_event = *event;
	spin_lock(&rvu->cgx_evq_lock);
	list_add_tail(&qentry->evq_node, &rvu->cgx_evq_head);
	spin_unlock(&rvu->cgx_evq_lock);

	/* start worker to process the events */
	queue_work(rvu->cgx_evh_wq, &rvu->cgx_evh_work);

	return 0;
}

static void cgx_evhandler_task(struct work_struct *work)
{
	struct rvu *rvu = container_of(work, struct rvu, cgx_evh_work);
	struct cgx_evq_entry *qentry;
	struct cgx_link_event *event;
	unsigned long flags;

	do {
		/* Dequeue an event */
		spin_lock_irqsave(&rvu->cgx_evq_lock, flags);
		qentry = list_first_entry_or_null(&rvu->cgx_evq_head,
						  struct cgx_evq_entry,
						  evq_node);
		if (qentry)
			list_del(&qentry->evq_node);
		spin_unlock_irqrestore(&rvu->cgx_evq_lock, flags);
		if (!qentry)
			break; /* nothing more to process */

		event = &qentry->link_event;

		/* Do nothing for now */
		kfree(qentry);
	} while (1);
}

static void cgx_lmac_event_handler_init(struct rvu *rvu)
{
	struct cgx_event_cb cb;
	int cgx, lmac, err;
	void *cgxd;

	spin_lock_init(&rvu->cgx_evq_lock);
	INIT_LIST_HEAD(&rvu->cgx_evq_head);
	INIT_WORK(&rvu->cgx_evh_work, cgx_evhandler_task);
	rvu->cgx_evh_wq = alloc_workqueue("rvu_evh_wq", 0, 0);
	if (!rvu->cgx_evh_wq) {
		dev_err(rvu->dev, "alloc workqueue failed");
		return;
	}

	cb.notify_link_chg = cgx_lmac_postevent; /* link change call back */
	cb.data = rvu;

	for (cgx = 0; cgx < rvu->cgx_cnt; cgx++) {
		cgxd = rvu_cgx_pdata(cgx, rvu);
		for (lmac = 0; lmac < cgx_get_lmac_cnt(cgxd); lmac++) {
			err = cgx_lmac_evh_register(&cb, cgxd, lmac);
			if (err)
				dev_err(rvu->dev,
					"%d:%d handler register failed\n",
					cgx, lmac);
		}
	}
}

void rvu_cgx_wq_destroy(struct rvu *rvu)
{
	if (rvu->cgx_evh_wq) {
		flush_workqueue(rvu->cgx_evh_wq);
		destroy_workqueue(rvu->cgx_evh_wq);
		rvu->cgx_evh_wq = NULL;
	}
}

int rvu_cgx_probe(struct rvu *rvu)
{
	int i, err;

	/* find available cgx ports */
	rvu->cgx_cnt = cgx_get_cgx_cnt();
	if (!rvu->cgx_cnt) {
		dev_info(rvu->dev, "No CGX devices found!\n");
		return -ENODEV;
	}

	rvu->cgx_idmap = devm_kzalloc(rvu->dev, rvu->cgx_cnt * sizeof(void *),
				      GFP_KERNEL);
	if (!rvu->cgx_idmap)
		return -ENOMEM;

	/* Initialize the cgxdata table */
	for (i = 0; i < rvu->cgx_cnt; i++)
		rvu->cgx_idmap[i] = cgx_get_pdata(i);

	/* Map CGX LMAC interfaces to RVU PFs */
	err = rvu_map_cgx_lmac_pf(rvu);
	if (err)
		return err;

	/* Register for CGX events */
	cgx_lmac_event_handler_init(rvu);
	return 0;
}
