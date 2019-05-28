/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include "pko.h"
#include "fpa.h"

#define DRV_NAME "octeontx-pko"
#define DRV_VERSION "0.1"

static atomic_t pko_count = ATOMIC_INIT(0);
static DEFINE_SPINLOCK(octeontx_pko_devices_lock);
static LIST_HEAD(octeontx_pko_devices);

static struct fpapf_com_s *fpapf;
static struct fpavf_com_s *fpavf;
static struct fpavf *fpa;
static int pko_niff_start(struct pkopf *pko, int vf, u32 max_frame, u32 gbps);

/* In Cavium OcteonTX SoCs, all accesses to the device registers are
 * implicitly strongly ordered.
 * So writeq_relaxed() and readq_relaxed() are safe to use
 * with out any memory barriers.
 */

/* Register read/write APIs */
static void pko_reg_write(struct pkopf *pko, u64 offset, u64 val)
{
	writeq_relaxed(val, pko->reg_base + offset);
}

static u64 pko_reg_read(struct pkopf *pko, u64 offset)
{
	return readq_relaxed(pko->reg_base + offset);
}

static int pko_get_bgx_channel(int bgx, int lmac, int chan)
{
	//NOTE: this is highly 83xx pki centric
	return 0x800 + (0x100 * bgx) + (0x10 * lmac) + chan;
}

static int pko_get_bgx_mac(int bgx, int lmac)
{
	//NOTE: this is highly 83xx pko centric
	return 3 + (0x4 * bgx) + lmac;
}

int pkopf_master_send_message(struct mbox_hdr *hdr,
			      union mbox_data *req,
			      union mbox_data *resp,
			      void *master_data,
			      void *add_data)
{
	struct pkopf *pko = master_data;
	int ret;

	if (hdr->coproc == FPA_COPROC) {
		ret = fpapf->receive_message(
			pko->id, FPA_PKO_DPFI_GMID,
			hdr, req, resp, add_data);
	} else {
		dev_err(&pko->pdev->dev, "PKO message dispatch, wrong VF type\n");
		ret = -1;
	}

	return ret;
}

static struct octeontx_master_com_t pko_master_com = {
	.send_message = pkopf_master_send_message
};

static irqreturn_t pko_ecc_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;

	dev_err(&pko->pdev->dev, "ECC received\n");
	return IRQ_HANDLED;
}

static irqreturn_t pko_peb_err_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg;

	dev_err(&pko->pdev->dev, "peb err received\n");
	reg = pko_reg_read(pko, PKO_PF_PEB_PAD_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb pad err info:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PEB_PSE_FIFO_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb pse fifo err info:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PEB_SUBD_ADDR_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb subd addr err info:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PEB_SUBD_SIZE_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb subd size err info:%llx\n", reg);
	return IRQ_HANDLED;
}

static irqreturn_t pko_pq_drain_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;

	dev_err(&pko->pdev->dev, "pq drain received\n");
	pko_reg_write(pko, PKO_PF_PQ_DRAIN_W1C, 0x1);
	return IRQ_HANDLED;
}

static irqreturn_t pko_pdm_sts_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg;

	dev_err(&pko->pdev->dev, "pdm sts received\n");
	reg = pko_reg_read(pko, PKO_PF_PDM_STS_INFO);
	dev_err(&pko->pdev->dev, "sts info: %llx\n", reg);

	reg = pko_reg_read(pko, PKO_PF_PDM_STS_W1C);
	dev_err(&pko->pdev->dev, "sts w1c: %llx\n", reg);
	pko_reg_write(pko, PKO_PF_PDM_STS_W1C, reg);

	return IRQ_HANDLED;
}

static irqreturn_t pko_pdm_ncb_int_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg;

	dev_err(&pko->pdev->dev, "pdm ncb int received\n");
	reg = pko_reg_read(pko, PKO_PF_PDM_NCB_MEM_FAULT);
	dev_err(&pko->pdev->dev, "pdm ncb mem fualt:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PDM_NCB_TX_ERR_WORD);
	dev_err(&pko->pdev->dev, "pdm ncb err word:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PDM_NCB_TX_ERR_INFO);
	dev_err(&pko->pdev->dev, "pdm ncb err info:%llx\n", reg);
	return IRQ_HANDLED;
}

static irqreturn_t pko_peb_ncb_int_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg;

	dev_err(&pko->pdev->dev, "peb ncb int received\n");
	reg = pko_reg_read(pko, PKO_PF_PEB_NCB_MEM_FAULT);
	dev_err(&pko->pdev->dev, "peb ncb mem fualt:%llx\n", reg);
	return IRQ_HANDLED;
}

static struct intr_hand intr[] = {
	{0x8000000000000000ULL, "pko lut ecc sbe",
		PKO_PF_LUT_ECC_SBE_INT_ENA_W1C,
		PKO_PF_LUT_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0x8000000000000000ULL, "pko lut ecc dbe",
		PKO_PF_LUT_ECC_DBE_INT_ENA_W1C,
		PKO_PF_LUT_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko dq ecc sbe", PKO_PF_DQ_ECC_SBE_INT_ENA_W1C,
		PKO_PF_DQ_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko dq ecc dbe", PKO_PF_DQ_ECC_DBE_INT_ENA_W1C,
		PKO_PF_DQ_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l2 ecc sbe", PKO_PF_L2_ECC_SBE_INT_ENA_W1C,
		PKO_PF_L2_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l2 ecc dbe", PKO_PF_L2_ECC_DBE_INT_ENA_W1C,
		PKO_PF_L2_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l3 ecc sbe", PKO_PF_L3_ECC_SBE_INT_ENA_W1C,
		PKO_PF_L3_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l3 ecc dbe", PKO_PF_L3_ECC_DBE_INT_ENA_W1C,
		PKO_PF_L3_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdm ecc sbe",
		PKO_PF_PDM_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PDM_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdm ecc dbe",
		PKO_PF_PDM_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PDM_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l1 ecc sbe", PKO_PF_L1_ECC_SBE_INT_ENA_W1C,
		PKO_PF_L1_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l1 ecc dbe", PKO_PF_L1_ECC_DBE_INT_ENA_W1C,
		PKO_PF_L1_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pq ecc sbe", PKO_PF_PQ_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PQ_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pq ecc dbe", PKO_PF_PQ_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PQ_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdmncb eccsbe",
		PKO_PF_PDM_NCB_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PDM_NCB_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdmncb eccdbe",
		PKO_PF_PDM_NCB_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PDM_NCB_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko peb sbe", PKO_PF_PEB_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PEB_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko peb dbe", PKO_PF_PEB_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PEB_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0x3ffULL, "pko peb err", PKO_PF_PEB_ERR_INT_ENA_W1C,
		PKO_PF_PEB_ERR_INT_ENA_W1S, pko_peb_err_intr_handler},
	{0x1ULL, "pko pq drain", PKO_PF_PQ_DRAIN_INT_ENA_W1C,
		PKO_PF_PQ_DRAIN_INT_ENA_W1S, pko_pq_drain_intr_handler},
	{0x270200C209ULL, "pko pdm sts", PKO_PF_PDM_STS_INT_ENA_W1C,
		PKO_PF_PDM_STS_INT_ENA_W1S, pko_pdm_sts_intr_handler},
	{0x7ULL, "pko pdm ncb int", PKO_PF_PDM_NCB_INT_ENA_W1C,
		PKO_PF_PDM_NCB_INT_ENA_W1S, pko_pdm_ncb_int_intr_handler},
	{0x4ULL, "pko peb ncb int", PKO_PF_PEB_NCB_INT_ENA_W1C,
		PKO_PF_PEB_NCB_INT_ENA_W1S, pko_peb_ncb_int_intr_handler},
	{0xffffffffffffffffULL, "pko pebncb eccsbe",
		PKO_PF_PEB_NCB_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PEB_NCB_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pebncb eccdbe",
		PKO_PF_PEB_NCB_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PEB_NCB_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler}

};

static int pko_pf_remove_domain(u32 id, u16 domain_id)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;
	int i;

	spin_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko) {
		spin_unlock(&octeontx_pko_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.in_use &&
		    pko->vf[i].domain.domain_id == domain_id) {
			pko->vf[i].domain.in_use = false;
			iounmap(pko->vf[i].domain.reg_base);
		}
	}

	spin_unlock(&octeontx_pko_devices_lock);

	return 0;
}

static void identify(struct pkopf_vf *vf, u16 domain_id,
		     u16 subdomain_id)
{
	u64 reg = (((u64)subdomain_id << 16) | (domain_id)) << 7;

	writeq_relaxed(reg, vf->domain.reg_base + PKO_VF_DQ_FC_CONFIG);
}

static void pko_pf_gmctl_init(struct pkopf *pf, int vf, u16 gmid)
{
	u64 reg;

	/* Write stream and GMID settings */
	reg = PKO_PF_VFX_GMCTL_GET_BE(
	      pko_reg_read(pf, PKO_PF_VFX_GMCTL(vf)));
	reg = PKO_PF_VFX_GMCTL_BE(reg) |
	      PKO_PF_VFX_GMCTL_GMID(gmid) |
	      PKO_PF_VFX_GMCTL_STRM(vf + 1);
	pko_reg_write(pf, PKO_PF_VFX_GMCTL(vf), reg);
	reg = pko_reg_read(pf, PKO_PF_VFX_GMCTL(vf));
}

static int pko_pf_create_domain(u32 id, u16 domain_id, u32 num_dqs,
				struct octtx_bgx_port *port, void *master,
				void *master_data, struct kobject *kobj,
				char *g_name)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;
	u64 i;
	int vf_idx = 0;
	resource_size_t vf_start;
	u32 gbps;
	const u32 max_frame = 0xffff; // FIXME: consider new pko domain param
	struct pci_dev *virtfn;

	if (!kobj || !g_name)
		return -EINVAL;

	spin_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko) {
		spin_unlock(&octeontx_pko_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.in_use) {
			continue;
		} else {
			pko->vf[i].domain.domain_id = domain_id;
			pko->vf[i].domain.subdomain_id = vf_idx;
			pko->vf[i].domain.gmid = get_gmid(domain_id);

			pko->vf[i].domain.in_use = true;
			pko->vf[i].domain.master = master;
			pko->vf[i].domain.master_data = master_data;

			vf_start = pci_resource_start(pko->pdev,
						      PCI_PKO_PF_CFG_BAR);
			vf_start += PKO_VF_OFFSET(i);

			pko->vf[i].domain.reg_base =
				ioremap(vf_start, PKO_VF_CFG_SIZE);

			if (!pko->vf[i].domain.reg_base)
				break;

			pko->vf[i].bgx_mask = port[vf_idx].bgx;
			pko->vf[i].bgx_lmac = port[vf_idx].lmac;

			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr(
					pko->pdev->bus),
					pci_iov_virtfn_bus(pko->pdev, i),
					pci_iov_virtfn_devfn(pko->pdev, i));
			if (!virtfn)
				break;
			sysfs_add_link_to_group(kobj, g_name,
						&virtfn->dev.kobj,
				virtfn->dev.kobj.name);

			identify(&pko->vf[i], domain_id, vf_idx);
			pko_pf_gmctl_init(pko, i, get_gmid(domain_id));

			/* Setup the PQ/SQ/DQ */
			/* TODO: Distinguish BGX and LBK ports.*/
			switch (port[vf_idx].lmac_type) {
			case OCTTX_BGX_LMAC_TYPE_40GR:
				gbps = 40;
				break;
			case OCTTX_BGX_LMAC_TYPE_XAUI:
			case OCTTX_BGX_LMAC_TYPE_RXAUI:
			case OCTTX_BGX_LMAC_TYPE_10GR:
				gbps = 10;
				break;
			case OCTTX_BGX_LMAC_TYPE_SGMII:
			default:
				gbps = 1;
				break;
			}
			pko_niff_start(pko, i, max_frame, gbps);

			vf_idx++;
			if (vf_idx == num_dqs) {
				pko->vfs_in_use += num_dqs;
				break;
			}
		}
	}

	spin_unlock(&octeontx_pko_devices_lock);

	if (vf_idx != num_dqs) {
		pko_pf_remove_domain(id, domain_id);
		return -ENODEV;
	}

	return 0;
}

/*caller is responsible for locks
 */
static struct pkopf_vf *get_vf(u32 id, u16 domain_id, u16 subdomain_id,
			       struct pkopf **master)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;
	int i;
	int vf_idx = -1;

	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko)
		return NULL;

	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.domain_id == domain_id &&
		    pko->vf[i].domain.subdomain_id == subdomain_id) {
			vf_idx = i;
			if (master)
				*master = pko;
			break;
		}
	}
	if (vf_idx >= 0)
		return &pko->vf[vf_idx];
	else
		return NULL;
}

static int pko_pf_receive_message(u32 id, u16 domain_id,
				  struct mbox_hdr *hdr,
				  union mbox_data *req,
				  union mbox_data *resp)
{
	struct pkopf_vf *vf;
	struct pkopf *pko = NULL;

	spin_lock(&octeontx_pko_devices_lock);

	vf = get_vf(id, domain_id, hdr->vfid, &pko);

	if (!vf) {
		hdr->res_code = MBOX_RET_INVALID;
		spin_unlock(&octeontx_pko_devices_lock);
		return -ENODEV;
	}

	resp->data = 0;
	hdr->res_code = MBOX_RET_SUCCESS;

	switch (hdr->msg) {
	case IDENTIFY:
		identify(vf, domain_id, hdr->vfid);
		break;
	default:
		hdr->res_code = MBOX_RET_INVALID;
	}

	spin_unlock(&octeontx_pko_devices_lock);
	return 0;
}

static int pko_pf_get_vf_count(u32 id)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;

	spin_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko) {
		spin_unlock(&octeontx_pko_devices_lock);
		return 0;
	}

	spin_unlock(&octeontx_pko_devices_lock);
	return pko->total_vfs;
}

int pko_reset_domain(u32 id, u16 domain_id)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;
	int i;

	spin_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko) {
		spin_unlock(&octeontx_pko_devices_lock);
		return 0;
	}

	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.in_use &&
		    pko->vf[i].domain.domain_id == domain_id) {
			/* TODO: Stop the actual device */
			identify(&pko->vf[i], domain_id,
				 pko->vf[i].domain.subdomain_id);
		}
	}

	spin_unlock(&octeontx_pko_devices_lock);
	return 0;
}

struct pkopf_com_s pkopf_com  = {
	.create_domain = pko_pf_create_domain,
	.free_domain = pko_pf_remove_domain,
	.reset_domain = pko_reset_domain,
	.receive_message = pko_pf_receive_message,
	.get_vf_count = pko_pf_get_vf_count
};
EXPORT_SYMBOL(pkopf_com);

static void pko_irq_free(struct pkopf *pko)
{
	int i;

	/*clear intr */
	for (i = 0; i < PKO_MSIX_COUNT; i++) {
		pko_reg_write(pko, intr[i].coffset, intr[i].mask);
		if (pko->msix_entries[i].vector)
			free_irq(pko->msix_entries[i].vector, pko);
	}
	pci_disable_msix(pko->pdev);
}

static int pko_irq_init(struct pkopf *pko)
{
	int i;
	int ret = 0;

	/*clear intr */
	for (i = 0; i < PKO_MSIX_COUNT; i++)
		pko_reg_write(pko, intr[i].coffset, intr[i].mask);

	pko->msix_entries = devm_kzalloc(&pko->pdev->dev,
			PKO_MSIX_COUNT * sizeof(struct msix_entry), GFP_KERNEL);

	if (!pko->msix_entries)
		return -ENOMEM;

	for (i = 0; i < PKO_MSIX_COUNT; i++)
		pko->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(pko->pdev, pko->msix_entries, PKO_MSIX_COUNT);
	if (ret < 0) {
		dev_err(&pko->pdev->dev, "Enabling msix failed\n");
		return ret;
	}

	for (i = 0; i < PKO_MSIX_COUNT; i++) {
		ret = request_irq(pko->msix_entries[i].vector, intr[i].handler,
				  0, intr[i].name, pko);
		if (ret)
			goto free_irq;
	}

	/*enable intr */
	for (i = 0; i < PKO_MSIX_COUNT; i++)
		pko_reg_write(pko, intr[i].soffset, intr[i].mask);

	return 0;
free_irq:
	for (; i < PKO_MSIX_COUNT; i++)
		pko->msix_entries[i].vector = 0;
	pko_irq_free(pko);
	return ret;
}

static int pko_pf_mac_init(struct pkopf *pko, int mac_num, u32 gbps)
{
	u64 reg, skid, fifo_size, size, rate, fifo;
	int ptgf;

	/* 1. The following parameters have to be consistent (see 83xx HRM):
	 * PKO_MAC(x)_CFG[fifo_num],[skid_max_cnt]
	 * PKO_PTGF(x)_CFG[rate],[size]
	 * PKO_MCI1_MAX_CRED(x)[max_cred_lim]
	 * 2. The combined bit rate among all FIFOs should not exceed
	 * 125 Gbps (80 inflight packets).
	 *
	 * TODO: For now, use simple arithmetic: FIFO number = MAC number.
	 * In general, any MAC may use any FIFO number.
	 */
	ptgf = mac_num / 4;
	switch (gbps) {
	case 40:
		fifo_size = 10000; /* 10KB */
		size = 4; /* {10.0, ---, ---, ---}KB */
		skid = 0x2; /* 64 */
		rate = 0x3; /* 50 Gpbs (48 inflight packets) */
		fifo = ptgf * 4; /* Only the 1st one in group can be used.*/
		break;
	case 10:
		fifo_size = 2500; /* 2.5KB */
		size = 0; /* {2.5, 2.5, 2.5, 2.5}KB */
		skid = 0x0; /* 16 */
		rate = 0x1; /* 12.5 Gpbs (12 inflight packets) */
		fifo = mac_num;
		break;
	default:
		fifo_size = 2500; /* 2.5KB */
		size = 0; /* {2.5, 2.5, 2.5, 2.5}KB */
		skid = 0x0; /* 16 */
		rate = 0x0; /* 6.25 Gpbs (6 inflight packets) */
		fifo = mac_num;
		break;
	}
	reg = fifo | (skid << 5) | (0x0 << 15) | (0x1 << 16);
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_MAC%d_CFG\n",
		reg, mac_num);
	pko_reg_write(pko, PKO_PF_MACX_CFG(mac_num), reg);

	reg = fifo_size / 16; /* MAX_CRED_LIM */
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_MCI1_MAX_CRED%d\n",
		reg, mac_num);
	pko_reg_write(pko, PKO_PF_MCI1_MAX_CREDX(mac_num), reg);

	reg = pko_reg_read(pko, PKO_PF_PTGFX_CFG(ptgf));
	reg = (rate << 3) | size;
	pko_reg_write(pko, PKO_PF_PTGFX_CFG(ptgf), reg);

	reg = (1ull << 63) | 0x10; /* 0x10 -- recommended in HRM.*/
	/* Note: For XFI interface, this value may be big and can create
	 * "underflow" condition in the BGX TX FIFO. If this happens,
	 * use value = 3..6.
	 */
	pko_reg_write(pko, PKO_PF_PTF_IOBP_CFG, reg);
	return 0;
}

static int pko_niff_pq_init(struct pkopf *pko, int vf,
			    int mac_num, u32 max_frame)
{
	u64 reg;
	int l1_sq;
	u64 queue_base = vf * 8;

	l1_sq = pko_reg_read(pko, PKO_PF_L1_CONST);

	l1_sq = l1_sq & 0xffff;

	/* If single child PRIORITY must be 0xF */
	reg = (mac_num << 16) |
		(queue_base << 32) |
		(0xFull << 1);
	pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(mac_num), reg);

	reg = (mac_num << 13);
	pko_reg_write(pko, PKO_PF_L1_SQX_SHAPE(mac_num), reg);

	reg = min(max_frame + 40, (u32)0xffffff);
	pko_reg_write(pko, PKO_PF_L1_SQX_SCHEDULE(mac_num), reg);

	reg = (mac_num | 0ULL) << 44;
	pko_reg_write(pko, PKO_PF_L1_SQX_LINK(mac_num), reg);

	return 0;
}

static u64 pko_lX_get_queue(struct pkopf *pko, int level)
{
	switch (level) {
	case 2:
		return pko_reg_read(pko, PKO_PF_L2_CONST);
	case 3:
		return pko_reg_read(pko, PKO_PF_L3_CONST);
	case 4:
		return pko_reg_read(pko, PKO_PF_L4_CONST);
	case 5:
		return pko_reg_read(pko, PKO_PF_L5_CONST);
	}
	return 0;
}

static void pko_lX_set_schedule(struct pkopf *pko, int level, int q, u64 reg)
{
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_L%d_SQ%d_SCHEDULE\n",
		reg, level, q);
	switch (level) {
	case 2:
		pko_reg_write(pko, PKO_PF_L2_SQX_SCHEDULE(q), reg);
		break;
	case 3:
		pko_reg_write(pko, PKO_PF_L3_SQX_SCHEDULE(q), reg);
		break;
	case 4:
	case 5:
		break;
	}
}

static void pko_lX_set_topology(struct pkopf *pko, int level, int q, u64 reg)
{
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_L%d_SQ%d_TOPOLOGY\n",
		reg, level, q);
	switch (level) {
	case 1:
		pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(q), reg);
		break;
	case 2:
		pko_reg_write(pko, PKO_PF_L2_SQX_TOPOLOGY(q), reg);
		break;
	case 3:
		pko_reg_write(pko, PKO_PF_L3_SQX_TOPOLOGY(q), reg);
		break;
	case 4:
	case 5:
		break;
	}
}

static void pko_lX_set_shape(struct pkopf *pko, int level, int q, u64 reg)
{
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_L%d_SQ%d_SHAPE\n",
		reg, level, q);
	switch (level) {
	case 2:
		pko_reg_write(pko, PKO_PF_L2_SQX_SHAPE(q), reg);
		break;
	case 3:
		pko_reg_write(pko, PKO_PF_L2_SQX_SHAPE(q), reg);
		break;
	case 4:
	case 5:
		break;
	}
}

static int pko_niff_sq_init(struct pkopf *pko, int vf, int level, u32 channel,
			    u32 max_frame, int parent_sq)
{
	int mac_num;
	int queue;
	int channel_level;
	int queue_base;
	u64 reg;

	mac_num = pko_get_bgx_mac(pko->vf[vf].bgx_mask, pko->vf[vf].bgx_lmac);

	queue = pko_lX_get_queue(pko, level);
	channel_level = pko_reg_read(pko, PKO_PF_CHANNEL_LEVEL);
	channel_level += 2;

	queue_base = (vf * 8);

	reg = min(max_frame + 40, (u32)0xffffff);
	pko_lX_set_schedule(pko, level, queue_base, reg);

	reg = 0;
	pko_lX_set_shape(pko, level, queue_base, reg);

	reg = (parent_sq << 16);
	if (level != pko->max_levels) {
		reg |= ((0ULL | queue_base) << 32);
		reg |= (0xf << 1);
	}
	pko_lX_set_topology(pko, level, queue_base, reg);

	if (level == channel_level) {
		reg = ((channel | 0ULL) & 0xffful) << 32;
		pko_reg_write(pko, PKO_PF_L3_L2_SQX_CHANNEL(queue_base), reg);

		reg = (queue_base) | (1Ull << 15) | (mac_num << 9);
		pko_reg_write(pko, PKO_PF_LUTX(channel), reg);
	}

	return queue_base;
}

static int pko_niff_dq_init(struct pkopf *pko, int vf)
{
	int queue_base, i;
	u64 reg;

	queue_base = vf * 8;

	reg = queue_base << 16;
	for (i = 0; i < 8; i++) {
		pko_reg_write(pko, PKO_PF_DQX_TOPOLOGY(queue_base + i), reg);

		/* PRIO = 0, RR_QUANTUM = max */
		pko_reg_write(pko, PKO_PF_DQX_SCHEDULE(queue_base + i),
			      0xffffff);

		pko_reg_write(pko, PKO_PF_DQX_SHAPE(queue_base + i), 0x0);
		pko_reg_write(pko, PKO_PF_PDM_DQX_MINPAD(queue_base + i), 0x1);
	}
	return 0;
}

static int pko_niff_start(struct pkopf *pko, int vf, u32 max_frame, u32 gbps)
{
	int lvl;
	int err;
	int mac_num;
	int channel;

	mac_num = pko_get_bgx_mac(pko->vf[vf].bgx_mask, pko->vf[vf].bgx_lmac);
	channel = pko_get_bgx_channel(pko->vf[vf].bgx_mask,
				      pko->vf[vf].bgx_lmac, 0);

	err = pko_pf_mac_init(pko, mac_num, gbps);
	if (err)
		return -ENODEV;

	err = pko_niff_pq_init(pko, vf, mac_num, max_frame);
	if (err)
		return -ENODEV;

	err = mac_num;
	for (lvl = 2; lvl <= pko->max_levels; lvl++)
		err = pko_niff_sq_init(pko, vf, lvl, channel, max_frame, err);

	err = pko_niff_dq_init(pko, vf);
	if (err)
		return -ENODEV;

	return 0;
}

static int pko_enable(struct pkopf *pko)
{
	u64 reg;
	int retry = 0;

	pko_reg_write(pko, PKO_PF_ENABLE, 0x1);

	while (true) {
		reg = pko_reg_read(pko, PKO_PF_STATUS);
		if (reg & 0x100)
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10)
			return -ENODEV;
	}

	return 0;
}

static int setup_dpfi(struct pkopf *pko)
{
	int err;
	int buffers;
	int retry = 0;
	u64 reg;

	fpapf = try_then_request_module(symbol_get(fpapf_com), "fpapf");
	if (!fpapf)
		return -ENODEV;

	err = fpapf->create_domain(pko->id, FPA_PKO_DPFI_GMID, 1, NULL, NULL);
	if (!err) {
		dev_err(&pko->pdev->dev, "failed to create PKO_DPFI_DOMAIN\n");
		symbol_put(fpapf_com);
		return -ENODEV;
	}

	fpavf = try_then_request_module(symbol_get(fpavf_com), "fpavf");
	if (!fpavf) {
		symbol_put(fpapf_com);
		return -ENODEV;
	}

	fpa = fpavf->get(FPA_PKO_DPFI_GMID, 0, &pko_master_com, pko);
	if (!fpa) {
		dev_err(&pko->pdev->dev, "failed to get fpavf\n");
		symbol_put(fpapf_com);
		symbol_put(fpavf_com);
		return -ENODEV;
	}
	buffers = 80000;

	err = fpavf->setup(fpa, buffers, pko->pdm_buf_size,
			FPA_VF_FLAG_CONT_MEM);
	if (err) {
		dev_err(&pko->pdev->dev, "failed to setup fpavf\n");
		symbol_put(fpapf_com);
		symbol_put(fpavf_com);
		return -ENODEV;
	}

	pko_reg_write(pko, PKO_PF_DPFI_FPA_AURA, 0);
	pko_reg_write(pko, PKO_PF_DPFI_GMCTL, FPA_PKO_DPFI_GMID);
	pko_reg_write(pko, PKO_PF_DPFI_FLUSH, 0);
	pko_reg_write(pko, PKO_PF_DPFI_ENA, 0x1);
	while (true) {
		reg = pko_reg_read(pko, PKO_PF_DPFI_STATUS);
		if (!(reg & 0x2))
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10)
			return -ENODEV;
	}
	return 0;
}

static int pko_init(struct pkopf *pko)
{
	u64 reg;
	int retry = 0;
	int n = 1023;
	int i;

	reg = pko_reg_read(pko, PKO_PF_CONST);

	pko->max_levels = PKO_CONST_GET_LEVELS(reg);
	pko->max_ptgfs = PKO_CONST_GET_PTGFS(reg);
	pko->max_formats = PKO_CONST_GET_FORMATS(reg);
	pko->pdm_buf_size = PKO_CONST_GET_PDM_BUF_SIZE(reg);
	pko->dqs_per_vf = PKO_CONST_GET_DQS_PER_VM(reg);

	while (true) {
		reg = pko_reg_read(pko, PKO_PF_STATUS);
		if (reg & 0x7f)
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10)
			return -ENODEV;
	}

	reg = 0;
	reg = PKO_PDM_CFG_SET_PAD_MINLEN(PKO_PAD_MINLEN) |
		PKO_PDM_CFG_SET_DQ_FC_SKID(n) | PKO_PDM_CFG_SET_EN(1);
	pko_reg_write(pko, PKO_PF_PDM_CFG, reg);

	pko_reg_write(pko, PKO_PF_SHAPER_CFG, 0x1);
	/*use L3 SQs */
	pko_reg_write(pko, PKO_PF_CHANNEL_LEVEL, 0x1);

	n = pko_reg_read(pko, PKO_PF_L2_CONST);
	for (i = 0; i < n; i++)
		pko_reg_write(pko, PKO_PF_L2_SQX_TOPOLOGY(i), 19 << 16);

	for (i = 0; i < pko->max_formats; i++)
		pko_reg_write(pko, PKO_PF_FORMATX_CTL(i), 0x0);
	pko_reg_write(pko, PKO_PF_FORMATX_CTL(1), 0x101);

	return 0;
}

static int pko_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct pkopf *pko = pci_get_drvdata(pdev);
	int ret = -EBUSY;
	int disable = 0;

	if (pko->vfs_in_use != 0)
		return ret;

	ret = 0;
	if (pko->flags & PKO_SRIOV_ENABLED)
		disable = 1;

	if (disable) {
		pci_disable_sriov(pdev);
		pko->flags &= ~PKO_SRIOV_ENABLED;
		pko->total_vfs = 0;
	}

	if (numvfs > 0) {
		ret = pci_enable_sriov(pdev, numvfs);
		if (ret == 0) {
			pko->flags |= PKO_SRIOV_ENABLED;
			pko->total_vfs = numvfs;
			ret = numvfs;
		}
	}
	return ret;
}

static int pko_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct pkopf *pko;
	int err = -ENOMEM;

	pko = devm_kzalloc(dev, sizeof(*pko), GFP_KERNEL);
	if (!pko)
		return err;

	pci_set_drvdata(pdev, pko);
	pko->pdev = pdev;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable PCI device\n");
		return err;
	}

	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		dev_err(dev, "PCI request regions failed\n");
		return err;
	}

	/*Map CFG registers */
	pko->reg_base = pcim_iomap(pdev, PCI_PKO_PF_CFG_BAR, 0);
	if (!pko->reg_base) {
		dev_err(dev, "Can't map CFG space\n");
		err = -ENOMEM;
		return err;
	}

	/*set PKO ID */
	pko->id = atomic_add_return(1, &pko_count);
	pko->id -= 1;

	err = pko_init(pko);
	if (err) {
		dev_err(dev, "Failed to init PKO\n");
		atomic_sub_return(1, &pko_count);
		return err;
	}

	err = setup_dpfi(pko);
	if (err) {
		dev_err(dev, "Failed to init DPFI\n");
		atomic_sub_return(1, &pko_count);
		return err;
	}

	err = pko_irq_init(pko);
	if (err) {
		atomic_sub_return(1, &pko_count);
		dev_err(dev, "failed init irqs\n");
		err = -EINVAL;
		return err;
	}

	err = pko_enable(pko);
	if (err) {
		atomic_sub_return(1, &pko_count);
		dev_err(dev, "failed to enable pko\n");
		err = -EINVAL;
		return err;
	}

	INIT_LIST_HEAD(&pko->list);
	spin_lock(&octeontx_pko_devices_lock);
	list_add(&pko->list, &octeontx_pko_devices);
	spin_unlock(&octeontx_pko_devices_lock);
	return 0;
}

static void pko_remove(struct pci_dev *pdev)
{
	struct pkopf *pko = pci_get_drvdata(pdev);

	if (!pko)
		return;

	pko_irq_free(pko);
	pko_sriov_configure(pdev, 0);
}

/* devices supported */
static const struct pci_device_id pko_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_PKO_PF) },
	{ 0, }  /* end of table */
};

static struct pci_driver pko_driver = {
	.name = DRV_NAME,
	.id_table = pko_id_table,
	.probe = pko_probe,
	.remove = pko_remove,
	.sriov_configure = pko_sriov_configure,
};

MODULE_AUTHOR("Tirumalesh Chalamarla");
MODULE_DESCRIPTION("Cavium OCTEONTX PKO Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, pko_id_table);

static int __init pko_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);

	return pci_register_driver(&pko_driver);
}

static void __exit pko_cleanup_module(void)
{
	pci_unregister_driver(&pko_driver);
}

module_init(pko_init_module);
module_exit(pko_cleanup_module);
