/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/arm-smccc.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/etherdevice.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/mmu_context.h>

#include "octeontx.h"
#include "octeontx_mbox.h"
#include "fpa.h"
#include "fpa.h"
#include "sso.h"
#include "bgx.h"
#include "sli.h"
#include "pko.h"
#include "lbk.h"
#include "tim.h"
#include "pki.h"
#include "dpi.h"
#include "zip.h"

#define DRV_NAME "octeontx"
#define DRV_VERSION "1.0"
#define DEVICE_NAME "octtx-ctr"
#define CLASS_NAME "octeontx-rm"

static struct cdev *octtx_cdev;
static struct device *octtx_device;
static struct class *octtx_class;
static dev_t octtx_dev;

/* Number of milliseconds we wait since last domain reset before we allow
 * domain to be destroyed, this is to account for a time between application
 * opens devices and a time it actually sends RM_START_APP message over
 * mailbox
 */
#define DESTROY_DELAY_IN_MS	1000
#define	MIN_DOMAIN_ID	4
static atomic_t gbl_domain_id = ATOMIC_INIT(MIN_DOMAIN_ID);

static struct bgx_com_s *bgx;
static struct slipf_com_s *slipf;
static struct lbk_com_s *lbk;
static struct fpapf_com_s *fpapf;
static struct ssopf_com_s *ssopf;
static struct pkopf_com_s *pkopf;
static struct timpf_com_s *timpf;
static struct ssowpf_com_s *ssowpf;
static struct pki_com_s *pki;
static struct dpipf_com_s *dpipf;
static struct zippf_com_s *zippf;

struct delayed_work dwork;
struct delayed_work dwork_reset;
struct workqueue_struct *check_link;
struct workqueue_struct *reset_domain;

#define MAX_GPIO 80

struct octtx_domain {
	struct list_head list;
	int node;
	int domain_id;
	int setup;
	int type;
	char name[1024];
	bool in_use;
	ulong last_reset_jiffies;

	int pko_vf_count;
	int fpa_vf_count;
	int sso_vf_count;
	int ssow_vf_count;
	int tim_vf_count;
	int dpi_vf_count;
	int zip_vf_count;

	u64 aura_set;
	u64 grp_mask;

	int bgx_count;
	int lbk_count;
	int sdp_count;
	struct octtx_bgx_port bgx_port[OCTTX_MAX_BGX_PORTS];
	struct octtx_lbk_port lbk_port[OCTTX_MAX_LBK_PORTS];
	struct octtx_sdp_port sdp_port[OCTTX_MAX_SDP_PORTS];

	struct kobject *kobj;
	struct kobject *ports_kobj;
	struct device_attribute sysfs_domain_id;
	struct device_attribute sysfs_domain_in_use;
	bool sysfs_domain_id_created;
	bool sysfs_domain_in_use_created;

	bool fpa_domain_created;
	bool ssow_domain_created;
	bool sso_domain_created;
	bool pki_domain_created;
	bool lbk_domain_created;
	bool bgx_domain_created;
	bool pko_domain_created;
	bool tim_domain_created;
	bool dpi_domain_created;
	bool zip_domain_created;
	bool sdp_domain_created;
};

static int gpio_in_use;
static int gpio_installed[MAX_GPIO];
static struct thread_info *gpio_installed_threads[MAX_GPIO];
static struct task_struct *gpio_installed_tasks[MAX_GPIO];

static DEFINE_MUTEX(octeontx_domains_lock);
static LIST_HEAD(octeontx_domains);

MODULE_AUTHOR("Tirumalesh Chalamarla");
MODULE_DESCRIPTION("Cavium OCTEONTX coprocessor management Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);

static int octeontx_create_domain(const char *name, int type, int sso_count,
				  int fpa_count, int ssow_count, int pko_count,
				  int pki_count, int tim_count, int bgx_count,
				  int lbk_count, int dpi_count, int zip_count,
				  int sdp_count,
				  const long int *bgx_port,
				  const long int *lbk_port,
				  const long int *sdp_port);

static void octeontx_destroy_domain(const char *domain_name);

static void do_destroy_domain(struct octtx_domain *domain);

static int octeontx_reset_domain(void *master_data);

static ssize_t octtx_destroy_domain_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	char tmp_buf[64];
	char *tmp_ptr;
	ssize_t used;

	strlcpy(tmp_buf, buf, 64);
	used = strlen(tmp_buf);
	tmp_ptr = strim(tmp_buf);
	octeontx_destroy_domain(tmp_ptr);

	return used;
}

static ssize_t octtx_create_domain_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	char *start;
	char *end;
	char *ptr;
	char *name;
	char *temp;
	long int type;
	long int sso_count = 0;
	long int fpa_count = 0;
	long int ssow_count = 0;
	long int pko_count = 0;
	long int tim_count = 0;
	long int bgx_count = 0;
	long int lbk_count = 0;
	long int dpi_count = 0;
	long int zip_count = 0;
	long int pki_count = 0;
	long int sdp_count = 0;
	long int lbk_port[OCTTX_MAX_LBK_PORTS];
	long int bgx_port[OCTTX_MAX_BGX_PORTS];
	long int sdp_port[OCTTX_MAX_SDP_PORTS];
	char *errmsg = "Wrong domain specification format.";

	end = kzalloc(PAGE_SIZE, GFP_KERNEL);
	ptr = end;
	memcpy(end, buf, count);

	start = strsep(&end, ";");
	if (!start)
		goto error;

	name = strim(strsep(&start, ":"));
	if (!strcmp(name, ""))
		goto error;
	if (!start)
		type = APP_NET;
	else if (kstrtol(strim(start), 10, &type))
		goto error;

	for (;;) {
		start = strsep(&end, ";");
		if (!start)
			break;
		start = strim(start);
		if (!*start)
			continue;

		if (!strncmp(strim(start), "ssow", sizeof("ssow") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &ssow_count))
				goto error;
		} else if (!strncmp(strim(start), "fpa", sizeof("fpa") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &fpa_count))
				goto error;
		} else if (!strncmp(strim(start), "sso", sizeof("sso") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &sso_count))
				goto error;
		} else if (!strncmp(strim(start), "pko", sizeof("pko") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &pko_count))
				goto error;
		} else if (!strncmp(strim(start), "pki", sizeof("pki") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &pki_count))
				goto error;
		} else if (!strncmp(strim(start), "tim", sizeof("tim") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &tim_count))
				goto error;
		} else if (!strncmp(strim(start), "net", sizeof("net") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &bgx_port[bgx_count]))
				goto error;
			bgx_count++;
		} else if (!strncmp(strim(start), "virt", sizeof("virt") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &lbk_port[lbk_count]))
				goto error;
			lbk_count++;
		} else if (!strncmp(start, "dpi", sizeof("dpi") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(start, 10, &dpi_count))
				goto error;
		} else if (!strncmp(strim(start), "pci", sizeof("pci") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(strim(start), 10, &sdp_port[sdp_count]))
				goto error;
			sdp_count++;
		} else if (!strncmp(start, "zip", sizeof("zip") - 1)) {
			temp = strsep(&start, ":");
			if (!start)
				goto error;
			if (kstrtol(start, 10, &zip_count))
				goto error;
		} else {
			goto error;
		}
	}

	ret = octeontx_create_domain(name, type, sso_count, fpa_count,
				     ssow_count, pko_count, pki_count,
				     tim_count, bgx_count, lbk_count,
				     dpi_count, zip_count,
				     sdp_count, (const long int *)bgx_port,
				     (const long int *)lbk_port,
				     (const long int *)sdp_port);
	if (ret) {
		errmsg = "Failed to create application domain.";
		goto error;
	}

	kfree(ptr);
	return count;
error:
	dev_err(dev, "%s\n", errmsg);
	kfree(ptr);
	return count;
}

static DEVICE_ATTR(create_domain, 0200, NULL, octtx_create_domain_store);
static DEVICE_ATTR(destroy_domain, 0200, NULL, octtx_destroy_domain_store);

static struct attribute *octtx_attrs[] = {
	&dev_attr_create_domain.attr,
	&dev_attr_destroy_domain.attr,
	NULL
};

static struct attribute_group octtx_attr_group = {
	.name = "octtx_attr",
	.attrs = octtx_attrs,
};

int octtx_sysfs_init(struct device *octtx_device)
{
	int ret;

	ret = sysfs_create_group(&octtx_device->kobj, &octtx_attr_group);
	if (ret < 0) {
		dev_err(octtx_device, " create_domain sysfs failed\n");
		return ret;
	}
	return 0;
}

void octtx_sysfs_remove(struct device *octtx_device)
{
	sysfs_remove_group(&octtx_device->kobj, &octtx_attr_group);
}

static int rm_receive_message(struct octtx_domain *domain, struct mbox_hdr *hdr,
			      union mbox_data *resp, void *mdata)
{
	struct mbox_intf_ver *msg = mdata;
	u32 rm_plat, rm_maj, rm_min;
	u32 app_plat, app_maj, app_min;

	switch (hdr->msg) {
	case RM_START_APP:
		domain->in_use = true;
		/* make sure it is flushed to memory because threads
		 * using it might be running on different cores
		 */
		mb();
		break;
	case RM_INTERFACE_VERSION:
		rm_plat = MBOX_INTERFACE_VERSION.platform;
		rm_maj = MBOX_INTERFACE_VERSION.major;
		rm_min = MBOX_INTERFACE_VERSION.minor;
		app_plat = msg->platform;
		app_maj = msg->major;
		app_min = msg->minor;

		/* RM version will be returned to APP */
		msg->platform = rm_plat;
		msg->major = rm_maj;
		msg->minor = rm_min;
		resp->data = sizeof(struct mbox_intf_ver);

		if (rm_plat != app_plat ||
		    rm_maj != app_maj ||
		    rm_min != app_min) {
			dev_err(octtx_device, "MBOX Interface version mismatch. APP ver is %d.%d.%d, RM ver is %d.%d.%d\n",
				app_plat, app_maj, app_min,
				rm_plat, rm_maj, rm_min);
			break;
		}
		break;
	default:
		goto err;
	}

	hdr->res_code = MBOX_RET_SUCCESS;
	return 0;
err:
	hdr->res_code = MBOX_RET_INVALID;
	return -EINVAL;
}

static int octtx_master_receive_message(struct mbox_hdr *hdr,
					union mbox_data *req,
					union mbox_data *resp,
					void *master_data,
					void *add_data)
{
	struct octtx_domain *domain = master_data;

	switch (hdr->coproc) {
	case PKI_COPROC:
		pki->receive_message(0, domain->domain_id, hdr, req,
					resp, add_data);
		break;
	case FPA_COPROC:
		fpapf->receive_message(0, domain->domain_id, hdr, req, resp,
				       add_data);
		break;
	case BGX_COPROC:
		bgx->receive_message(0, domain->domain_id, hdr,
				req, resp, add_data);
		break;
	case LBK_COPROC:
		lbk->receive_message(0, domain->domain_id, hdr,
				req, resp, add_data);
		break;
	case PKO_COPROC:
		pkopf->receive_message(0, domain->domain_id, hdr, req, resp);
		break;
	case TIM_COPROC:
		timpf->receive_message(0, domain->domain_id, hdr,
				req, resp, add_data);
		break;
	case SSO_COPROC:
		if (hdr->msg == SSO_GETDOMAINCFG) {
			struct dcfg_resp *dcfg = add_data;

			dcfg->sso_count = domain->sso_vf_count;
			dcfg->ssow_count = domain->ssow_vf_count;
			dcfg->fpa_count = domain->fpa_vf_count;
			dcfg->pko_count = domain->pko_vf_count;
			dcfg->tim_count = domain->tim_vf_count;
			dcfg->net_port_count = domain->bgx_count;
			dcfg->virt_port_count = domain->lbk_count;
			dcfg->pci_port_count = domain->sdp_count;
			resp->data = sizeof(struct dcfg_resp);
			hdr->res_code = MBOX_RET_SUCCESS;
			break;
		}
	case DPI_COPROC:
		dpipf->receive_message(0, domain->domain_id, hdr,
				       req, resp, add_data);
		break;
	case SDP_COPROC:
		slipf->receive_message(0, domain->domain_id, hdr,
				req, resp, add_data);
		break;
	case ZIP_COPROC:
		zippf->receive_message(0, domain->domain_id, hdr,
				req, resp, add_data);
		break;
	case NO_COPROC:
		rm_receive_message(domain, hdr, resp, add_data);
		break;
	case SSOW_COPROC:
	default:
		dev_err(octtx_device, "invalid mbox message\n");
		hdr->res_code = MBOX_RET_INVALID;
		break;
	}
	return 0;
}

static struct octeontx_master_com_t octtx_master_com = {
	.receive_message = octtx_master_receive_message,
};

void octeontx_destroy_domain(const char *domain_name)
{
	struct octtx_domain *domain = NULL;
	struct octtx_domain *curr;

	mutex_lock(&octeontx_domains_lock);
	list_for_each_entry(curr, &octeontx_domains, list) {
		if (!strcmp(curr->name, domain_name)) {
			domain = curr;
			break;
		}
	}

	if (domain) {
		if (domain->in_use ||
		    time_before(jiffies, domain->last_reset_jiffies +
		    msecs_to_jiffies(DESTROY_DELAY_IN_MS))) {
			dev_err(octtx_device,
				"Error domain %d on node %d is in use.\n",
				domain->domain_id, domain->node);
			goto err_unlock;
		}

		octeontx_reset_domain(domain);
		do_destroy_domain(domain);
		list_del(&domain->list);
		module_put(THIS_MODULE);
		kfree(domain);
	}

err_unlock:
	mutex_unlock(&octeontx_domains_lock);
}

static void do_destroy_domain(struct octtx_domain *domain)
{
	u32 ret, node, i;
	u16 domain_id;
	struct octtx_bgx_port *bgx_port;

	if (!domain)
		return;

	node = domain->node;
	domain_id = domain->domain_id;

	if (domain->bgx_domain_created) {
		for (i = 0; i < domain->bgx_count; i++) {
			bgx_port = &domain->bgx_port[i];
			sysfs_remove_file(&bgx_port->kobj,
					  &bgx_port->sysfs_stats.attr);
		}
		ret = bgx->destroy_domain(node, domain_id, domain->ports_kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove BGX of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->lbk_domain_created) {
		ret = lbk->destroy_domain(node, domain_id, domain->ports_kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove LBK of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->pko_domain_created) {
		ret = pkopf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove PKO of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->pki_domain_created) {
		ret = pki->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove PKI of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->sso_domain_created) {
		ret = ssopf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove SSO of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->ssow_domain_created) {
		ret = ssowpf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove SSOW of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->tim_domain_created) {
		ret = timpf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove TIM of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->fpa_domain_created) {
		ret = fpapf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove FPA of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->dpi_domain_created) {
		ret = dpipf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove dpi of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->zip_domain_created) {
		ret = zippf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove zip of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->sdp_domain_created) {
		ret = slipf->destroy_domain(node, domain_id, domain->kobj);
		if (ret) {
			dev_err(octtx_device,
				"Failed to remove sdp of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->sysfs_domain_in_use_created)
		sysfs_remove_file(domain->kobj,
				  &domain->sysfs_domain_in_use.attr);

	if (domain->sysfs_domain_id_created)
		sysfs_remove_file(domain->kobj, &domain->sysfs_domain_id.attr);

	if (domain->ports_kobj)
		kobject_del(domain->ports_kobj);

	if (domain->kobj)
		kobject_del(domain->kobj);
}

static ssize_t octtx_domain_id_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct octtx_domain *domain;

	domain = container_of(attr, struct octtx_domain, sysfs_domain_id);

	return snprintf(buf, PAGE_SIZE, "%d\n", domain->domain_id);
}

static ssize_t octtx_domain_in_use_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct octtx_domain *domain;

	domain = container_of(attr, struct octtx_domain, sysfs_domain_in_use);

	return snprintf(buf, PAGE_SIZE, "%d\n", domain->in_use);
}

static ssize_t octtx_netport_stats_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct octtx_bgx_port *port;
	int ret;

	port = container_of(kobj, struct octtx_bgx_port, kobj);
	if (!port)
		return 0;

	ret = bgx->get_port_stats(port);
	if (ret)
		return 0;
	ret = pki->get_bgx_port_stats(port);
	if (ret)
		return 0;
	port->stats.rxucast = port->stats.rxpkts -
			      port->stats.rxbcast - port->stats.rxmcast;
	port->stats.txucast = port->stats.txpkts -
			      port->stats.txbcast - port->stats.txmcast;
	return snprintf(buf, PAGE_SIZE,
			"%lld %lld %lld %lld %lld %lld %lld\n"
			"%lld %lld %lld %lld %lld %lld %lld\n",
			port->stats.rxpkts, port->stats.rxbytes,
			port->stats.rxdrop, port->stats.rxerr,
			port->stats.rxucast, port->stats.rxbcast,
			port->stats.rxmcast,
			port->stats.txpkts, port->stats.txbytes,
			port->stats.txdrop, port->stats.txerr,
			port->stats.txucast, port->stats.txbcast,
			port->stats.txmcast);
}

int octeontx_create_domain(const char *name, int type, int sso_count,
			   int fpa_count, int ssow_count, int pko_count,
			   int pki_count, int tim_count, int bgx_count,
			   int lbk_count, int dpi_count, int zip_count,
			   int sdp_count,
			   const long int *bgx_port,
			   const long int *lbk_port,
			   const long int *sdp_port)
{
	void *ssow_ram_mbox_addr = NULL;
	struct octtx_domain *domain;
	struct kobj_attribute *kattr;
	u16 domain_id;
	int ret = -EINVAL;
	int node = 0;
	bool found = false;
	int i, port_count = bgx_count + lbk_count + sdp_count;

	list_for_each_entry(domain, &octeontx_domains, list) {
		if (!strcmp(name, domain->name)) {
			dev_err(octtx_device,
				"Domain name \"%s\" already exists\n", name);
			return -EEXIST;
		}
	}

	if (!sso_count) {
		dev_err(octtx_device, "Domain has to include at least 1 SSO\n");
		return -EINVAL;
	}

	if (!ssow_count) {
		dev_err(octtx_device,
			"Domain has to include at least 1 SSOW\n");
		return -EINVAL;
	}

	if (port_count != 0 && pki_count != 1) {
		dev_err(octtx_device, "Domain has to include exactly 1 PKI if there are BGX or LBK or SDP ports\n");
		return -EINVAL;
	}

	if (pko_count != port_count) {
		dev_err(octtx_device, "Domain has to include as many PKOs as there are BGX and LBK and SDP ports\n");
		return -EINVAL;
	}

	/*get DOMAIN ID */
	while (!found) {
		domain_id = atomic_add_return(1, &gbl_domain_id);
		domain_id -= 1;
		if (domain_id < MIN_DOMAIN_ID)
			continue;
		found = true;
		list_for_each_entry(domain, &octeontx_domains, list) {
			if (domain->domain_id == domain_id) {
				found = false;
				break;
			}
		}
	}

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return -ENOMEM;

	domain->node = node;
	domain->domain_id = domain_id;
	memcpy(domain->name, name, strlen(name));
	domain->type = type;

	domain->kobj = kobject_create_and_add(domain->name,
					      &octtx_device->kobj);
	if (!domain->kobj) {
		ret = -ENOMEM;
		goto error;
	}
	if (port_count) {
		domain->ports_kobj = kobject_create_and_add("ports",
							    domain->kobj);
		if (!domain->ports_kobj) {
			ret = -ENOMEM;
			goto error;
		}
	}

	domain->fpa_vf_count = fpa_count;
	if (domain->fpa_vf_count) {
		domain->aura_set = fpapf->create_domain(node, domain_id,
							domain->fpa_vf_count,
							domain->kobj);
		if (!domain->aura_set) {
			dev_err(octtx_device, "Failed to create FPA domain\n");
			ret = -ENODEV;
			goto error;
		}
		domain->fpa_domain_created = true;
	}

	domain->ssow_vf_count = ssow_count;
	ret = ssowpf->create_domain(node, domain_id, domain->ssow_vf_count,
				    &octtx_master_com, domain, domain->kobj);
	if (ret) {
		dev_err(octtx_device, "Failed to create SSOW domain\n");
		goto error;
	}
	domain->ssow_domain_created = true;

	domain->sso_vf_count = sso_count;
	domain->grp_mask = ssopf->create_domain(node, domain_id,
				domain->sso_vf_count,
				&octtx_master_com, domain, domain->kobj);
	if (!domain->grp_mask) {
		dev_err(octtx_device, "Failed to create SSO domain\n");
		goto error;
	}
	domain->sso_domain_created = true;

	ret = ssowpf->get_ram_mbox_addr(node, domain_id, &ssow_ram_mbox_addr);
	if (ret) {
		dev_err(octtx_device, "Failed to get_ssow_ram_mbox_addr\n");
		goto error;
	}

	ret = ssopf->set_mbox_ram(node, domain_id,
				  ssow_ram_mbox_addr, SSOW_RAM_MBOX_SIZE);
	if (ret) {
		dev_err(octtx_device, "Failed to set_ram_addr\n");
		goto error;
	}

	ret = pki->create_domain(node, domain_id, &octtx_master_com, domain,
				 domain->kobj);
	if (ret) {
		dev_err(octtx_device, "Failed to create PKI domain\n");
		goto error;
	}
	domain->pki_domain_created = true;

	/* OCTEONTX allows to create two internal duplex (from the dataplane
	 * user point of view) ports out of four available LBK devices:
	 * virt0: transferring packets between PKO and PKI (LBK0);
	 * virt1: transferring packets between PKO/PKI and NIC (LBK1 + LBK2).
	 * NOTE: The domain specification validity should be done here.
	 */
	domain->lbk_count = 0;
	for (i = 0; i < lbk_count; i++) {
		if (lbk_port[i] > 1) {
			dev_err(octtx_device, "LBK invalid port g%ld\n",
				lbk_port[i]);
			goto error;
		}

		domain->lbk_port[i].domain_id = domain_id;
		domain->lbk_port[i].dom_port_idx = i;
		domain->lbk_port[i].glb_port_idx = lbk_port[i];
		domain->lbk_port[i].pkind = pki->add_lbk_port(node, domain_id,
							&domain->lbk_port[i]);
		if (domain->lbk_port[i].pkind < 0) {
			dev_err(octtx_device,
				"LBK failed to allocate PKIND for port l%d(g%d)\n",
				domain->lbk_port[i].dom_port_idx,
				domain->lbk_port[i].glb_port_idx);
			goto error;
		}
		domain->lbk_count++;
	}

	if (lbk_count) {
		ret = lbk->create_domain(node, domain_id, domain->lbk_port, i,
					 &octtx_master_com, domain,
					 domain->ports_kobj);
		if (ret) {
			dev_err(octtx_device, "Failed to create LBK domain\n");
			goto error;
		}
		domain->lbk_domain_created = true;
	}

	/* There is a global list of all network (BGX-based) ports
	 * detected by the thunder driver and provided to this driver.
	 * This list is maintained in bgx.c (octeontx_bgx_ports).
	 * In general domain creation, a list of domain local ports
	 * is constructed as a subset of global ports, where mapping
	 * of domain-local to global indexes is provided as follows:
	 * domain->bgx_port[i].port_idx = i; -- domain-local port index.
	 * domain->bgx_port[i].port_gidx = n; -- global port index.
	 * In this, default configuraiton, all available ports are
	 * given to this domain.
	 */
	domain->bgx_count = 0;
	if (bgx_count) {
		for (i = 0; i < bgx_count; i++) {
			domain->bgx_port[i].domain_id = domain_id;
			domain->bgx_port[i].dom_port_idx = i;
			domain->bgx_port[i].glb_port_idx = bgx_port[i];
		}
		ret = bgx->create_domain(node, domain_id, domain->bgx_port, i,
					 &octtx_master_com, domain,
					 domain->ports_kobj);
		if (ret) {
			dev_err(octtx_device, "Failed to create BGX domain\n");
			goto error;
		}
		domain->bgx_domain_created = true;
	}
	/* Now that we know which exact ports we have, set pkinds for them. */
	for (i = 0; i < bgx_count; i++) {
		ret = pki->add_bgx_port(node, domain_id, &domain->bgx_port[i]);
		if (ret < 0) {
			dev_err(octtx_device,
				"BGX failed to allocate PKIND for port l%d(g%d)\n",
				domain->bgx_port[i].dom_port_idx,
				domain->bgx_port[i].glb_port_idx);
			goto error;
		}
		domain->bgx_port[i].pkind = ret;
		ret = bgx->set_pkind(node, domain_id,
				     domain->bgx_port[i].dom_port_idx,
				     domain->bgx_port[i].pkind);
		if (ret < 0) {
			dev_err(octtx_device,
				"BGX failed to set PKIND for port l%d(g%d)\n",
				domain->bgx_port[i].dom_port_idx,
				domain->bgx_port[i].glb_port_idx);
			goto error;
		}
		/* sysfs entry: */
		ret = kobject_init_and_add(&domain->bgx_port[i].kobj,
					   get_ktype(domain->ports_kobj),
					   domain->ports_kobj, "net%d", i);
		if (ret)
			goto error;
		kattr = &domain->bgx_port[i].sysfs_stats;
		kattr->show = octtx_netport_stats_show;
		kattr->attr.name = "stats";
		kattr->attr.mode = 0444;
		sysfs_attr_init(&kattr->attr);
		ret = sysfs_create_file(&domain->bgx_port[i].kobj,
					&kattr->attr);
		if (ret < 0)
			goto error;
		domain->bgx_count++;
	}

	domain->sdp_count = 0;
	if (sdp_count) {
		for (i = 0; i < sdp_count; i++) {
			domain->sdp_port[i].domain_id = domain_id;
			domain->sdp_port[i].dom_port_idx = i;
			domain->sdp_port[i].glb_port_idx = sdp_port[i];
		}
		ret = slipf->create_domain(node, domain_id, domain->sdp_port, i,
				&octtx_master_com, domain, domain->ports_kobj);
		if (ret) {
			dev_err(octtx_device, "Failed to create SDP domain\n");
			goto error;
		}
		domain->sdp_domain_created = true;
	}

	/* Now that we know which exact ports we have, set pkinds for them. */
	for (i = 0; i < sdp_count; i++) {
		ret = pki->add_sdp_port(node, domain_id, &domain->sdp_port[i]);
		if (ret < 0) {
			dev_err(octtx_device,
				"SDP::Failed to allocate PKIND for port l%d(g%d)\n",
				domain->sdp_port[i].dom_port_idx,
				domain->sdp_port[i].glb_port_idx);
			goto error;
		}

		domain->sdp_port[i].pkind = ret;
		ret = slipf->set_pkind(node, domain_id,
				     domain->sdp_port[i].dom_port_idx,
				     domain->sdp_port[i].pkind);
		if (ret < 0) {
			dev_err(octtx_device,
				"SDP::Failed to set PKIND for port l%d(g%d)\n",
				domain->sdp_port[i].dom_port_idx,
				domain->sdp_port[i].glb_port_idx);
			goto error;
		}
		/* TODO: setup sysfs entry for sdp port*/
		domain->sdp_count++;
	}
	if (ret) {
		dev_err(octtx_device, "Failed to create SDP domain\n");
		goto error;
	}

	/* remove this once PKO init extends for LBK. */
	domain->pko_vf_count = port_count;
	if (domain->pko_vf_count) {
		ret = pkopf->create_domain(node, domain_id,
					domain->pko_vf_count,
					domain->bgx_port, domain->bgx_count,
					domain->lbk_port, domain->lbk_count,
					domain->sdp_port, domain->sdp_count,
					&octtx_master_com, domain,
					domain->kobj);
		if (ret) {
			dev_err(octtx_device, "Failed to create PKO domain\n");
			goto error;
		}
		domain->pko_domain_created = true;
	}

	domain->tim_vf_count = tim_count;
	if (domain->tim_vf_count) {
		ret = timpf->create_domain(node, domain_id,
			domain->tim_vf_count, &octtx_master_com, domain,
			domain->kobj);
		if (ret) {
			dev_err(octtx_device, "Failed to create TIM domain\n");
			goto error;
		}
		domain->tim_domain_created = true;
	}

	domain->dpi_vf_count = dpi_count;
	if (domain->dpi_vf_count > 0) {
		ret = dpipf->create_domain(node, domain_id,
					   domain->dpi_vf_count,
					   &octtx_master_com, domain,
					   domain->kobj);
		if (ret) {
			dev_err(octtx_device, "Failed to create DPI domain\n");
			goto error;
		}
		domain->dpi_domain_created = true;
	}

	domain->zip_vf_count = zip_count;
	if (domain->zip_vf_count) {
		ret = zippf->create_domain(node, domain_id,
			domain->zip_vf_count, &octtx_master_com, domain,
			domain->kobj);
		if (ret) {
			dev_err(octtx_device, "Failed to create ZIP domain\n");
			goto error;
		}
		domain->zip_domain_created = true;
	}

	domain->sysfs_domain_id.show = octtx_domain_id_show;
	domain->sysfs_domain_id.attr.name = "domain_id";
	domain->sysfs_domain_id.attr.mode = 0444;
	sysfs_attr_init(&domain->sysfs_domain_id.attr);
	ret = sysfs_create_file(domain->kobj, &domain->sysfs_domain_id.attr);
	if (ret) {
		dev_err(octtx_device, " domain_id sysfs failed\n");
		goto error;
	}
	domain->sysfs_domain_id_created = true;

	domain->sysfs_domain_in_use.show = octtx_domain_in_use_show;
	domain->sysfs_domain_in_use.attr.name = "domain_in_use";
	domain->sysfs_domain_in_use.attr.mode = 0444;
	sysfs_attr_init(&domain->sysfs_domain_in_use.attr);
	ret = sysfs_create_file(domain->kobj,
				&domain->sysfs_domain_in_use.attr);
	if (ret) {
		dev_err(octtx_device, " domain_in_use sysfs failed\n");
		goto error;
	}
	domain->sysfs_domain_in_use_created = true;

	mutex_lock(&octeontx_domains_lock);
	INIT_LIST_HEAD(&domain->list);
	list_add(&domain->list, &octeontx_domains);
	try_module_get(THIS_MODULE);
	mutex_unlock(&octeontx_domains_lock);
	return 0;
error:
	do_destroy_domain(domain);
	kfree(domain);
	return ret;
}

static int octeontx_reset_domain(void *master_data)
{
	struct octtx_domain *domain = master_data;
	void *ssow_ram_mbox_addr = NULL;
	int node = domain->node;
	int ret;

	/* Reset co-processors */
	if (domain->bgx_domain_created) {
		ret = bgx->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset BGX of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->sdp_domain_created) {
		ret = slipf->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset SDP of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->lbk_domain_created) {
		ret = lbk->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset LBK of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->tim_domain_created) {
		ret = timpf->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset TIM of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->pko_domain_created) {
		ret = pkopf->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset PKO of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->pki_domain_created) {
		ret = pki->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset PKI of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->sso_domain_created) {
		ret = ssopf->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset SSO of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->ssow_domain_created) {
		ret = ssowpf->reset_domain(node, domain->domain_id,
					   domain->grp_mask);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset SSOW of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	/* FPA reset should be the last one to call*/
	if (domain->fpa_domain_created) {
		ret = fpapf->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset FPA of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->dpi_domain_created) {
		ret = dpipf->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset DPI of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	if (domain->zip_domain_created) {
		ret = zippf->reset_domain(node, domain->domain_id);
		if (ret) {
			dev_err(octtx_device,
				"Failed to reset ZIP of domain %d on node %d.\n",
				domain->domain_id, node);
		}
	}

	/* Reset mailbox */
	ret = ssowpf->get_ram_mbox_addr(node, domain->domain_id,
					&ssow_ram_mbox_addr);
	if (ret) {
		dev_err(octtx_device,
			"Failed ram_mbox_addr for node (%d): domain (%d)\n",
			node, domain->domain_id);
		return ret;
	}
	ret = ssopf->set_mbox_ram(node, domain->domain_id,
				  ssow_ram_mbox_addr, SSOW_RAM_MBOX_SIZE);
	if (ret) {
		dev_err(octtx_device,
			"Failed to set_ram_addr for node (%d): domain (%d)\n",
			node, domain->domain_id);
		return ret;
	}

	domain->in_use = false;
	domain->last_reset_jiffies = jiffies;

	return 0;
}

static void poll_for_link(struct work_struct *work)
{
	struct octtx_domain *domain;
	int i, node, bgx_idx, lmac;
	int link_up;

	mutex_lock(&octeontx_domains_lock);
	list_for_each_entry(domain, &octeontx_domains, list) {
		/* don't bother if setup is not done */
		if (!domain->setup)
			continue;

		for (i = 0; i < domain->bgx_count; i++) {
			node = domain->bgx_port[i].node;
			bgx_idx = domain->bgx_port[i].bgx;
			lmac = domain->bgx_port[i].lmac;
			link_up = bgx->get_link_status(node, bgx_idx, lmac);
			/* Inform only if link status changed */
			if (link_up == domain->bgx_port[i].link_up)
				continue;

			domain->bgx_port[i].link_up = link_up;
		}
	}
	mutex_unlock(&octeontx_domains_lock);
	queue_delayed_work(check_link, &dwork, HZ * 2);
}

void octtx_reset_domain(struct work_struct *work)
{
	struct octtx_domain *domain;
	int i, master_sso;
	u64 mask = -1;
	u64 val;

	mutex_lock(&octeontx_domains_lock);
	list_for_each_entry(domain, &octeontx_domains, list) {
		/* find first SSO from domain */
		master_sso = __ffs(domain->grp_mask);
		for_each_set_bit(i, (unsigned long *)&domain->grp_mask,
				 sizeof(domain->grp_mask) * 8) {
			val = atomic_read(&octtx_sso_reset[i]);
			if ((master_sso == i) && val) {
				mutex_unlock(&octeontx_domains_lock);
				octeontx_reset_domain(domain);
				mutex_lock(&octeontx_domains_lock);
			}
			atomic_set(&octtx_sso_reset[i], 0);
		}
		mask &= ~domain->grp_mask;
	}

	for_each_set_bit(i, (unsigned long *)&mask, sizeof(mask) * 8) {
		if (atomic_read(&octtx_sso_reset[i]))
			atomic_set(&octtx_sso_reset[i], 0);
	}

	/*make sure the other end receives it*/
	mb();

	mutex_unlock(&octeontx_domains_lock);
	queue_delayed_work(reset_domain, &dwork_reset, 10);
}

static DEFINE_SPINLOCK(el3_inthandler_lock);

static inline int __install_el3_inthandler(unsigned long gpio_num,
					   unsigned long sp,
					   unsigned long cpu,
					   unsigned long ttbr0)
{
	struct arm_smccc_res res;
	unsigned long flags;
	int retval = -1;

	spin_lock_irqsave(&el3_inthandler_lock, flags);
	if (!gpio_installed[gpio_num]) {
		lock_context(current->group_leader->mm, gpio_num);
		arm_smccc_smc(THUNDERX_INSTALL_GPIO_INT, gpio_num,
			      sp, cpu, ttbr0, 0, 0, 0, &res);
		if (res.a0 == 0) {
			gpio_installed[gpio_num] = 1;
			gpio_installed_threads[gpio_num]
				= current_thread_info();
			gpio_installed_tasks[gpio_num]
				= current->group_leader;
			retval = 0;
		} else {
			unlock_context_by_index(gpio_num);
		}
	}
	spin_unlock_irqrestore(&el3_inthandler_lock, flags);
	return retval;
}

static inline int __remove_el3_inthandler(unsigned long gpio_num)
{
	struct arm_smccc_res res;
	unsigned long flags;
	unsigned int retval;

	spin_lock_irqsave(&el3_inthandler_lock, flags);
	if (gpio_installed[gpio_num]) {
		arm_smccc_smc(THUNDERX_REMOVE_GPIO_INT, gpio_num,
			      0, 0, 0, 0, 0, 0, &res);
		gpio_installed[gpio_num] = 0;
		gpio_installed_threads[gpio_num] = NULL;
		gpio_installed_tasks[gpio_num] = NULL;
		unlock_context_by_index(gpio_num);
		retval = 0;
	} else {
		retval = -1;
	}
	spin_unlock_irqrestore(&el3_inthandler_lock, flags);
	return retval;
}

static long octtx_dev_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct octtx_gpio_usr_data gpio_usr;
	u64 gpio_ttbr, gpio_isr_base, gpio_sp, gpio_cpu, gpio_num;
	int ret;
	//struct task_struct *task = current;

	if (!gpio_in_use)
		return -EINVAL;

	if (_IOC_TYPE(cmd) != OCTTX_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_TYPE(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
	case OCTTX_IOC_SET_GPIO_HANDLER: /*Install GPIO ISR handler*/
		ret = copy_from_user(&gpio_usr, (void *)arg, _IOC_SIZE(cmd));
		if (gpio_usr.gpio_num >= MAX_GPIO)
			return -EINVAL;
		if (ret)
			return -EFAULT;
		gpio_ttbr = 0;
		//TODO: reserve a asid to avoid asid rollovers
		asm volatile("mrs %0, ttbr0_el1\n\t" : "=r"(gpio_ttbr));
		gpio_isr_base = gpio_usr.isr_base;
		gpio_sp = gpio_usr.sp;
		gpio_cpu = gpio_usr.cpu;
		gpio_num = gpio_usr.gpio_num;
		ret = __install_el3_inthandler(gpio_num, gpio_sp,
					       gpio_cpu, gpio_isr_base);
		if (ret != 0)
			return -EEXIST;
		break;
	case OCTTX_IOC_CLR_GPIO_HANDLER: /*Clear GPIO ISR handler*/
		gpio_usr.gpio_num = arg;
		if (gpio_usr.gpio_num >= MAX_GPIO)
			return -EINVAL;
		ret = __remove_el3_inthandler(gpio_usr.gpio_num);
		if (ret != 0)
			return -ENOENT;
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

void cleanup_el3_irqs(struct task_struct *task)
{
	int i;

	for (i = 0; i < MAX_GPIO; i++) {
		if (gpio_installed[i] &&
		    gpio_installed_tasks[i] &&
		    ((gpio_installed_tasks[i] == task) ||
			(gpio_installed_tasks[i] == task->group_leader))) {
			pr_alert("Exiting, removing handler for GPIO %d\n",
				 i);
			__remove_el3_inthandler(i);
			pr_alert("Exited, removed handler for GPIO %d\n",
				 i);
		} else {
			if (gpio_installed[i] &&
			    (gpio_installed_threads[i]
			     == current_thread_info()))
				pr_alert(
	    "Exiting, thread info matches, not removing handler for GPIO %d\n",
					 i);
		}
	}
}

static int octtx_dev_open(struct inode *inode, struct file *fp)
{
	gpio_in_use = 1;
	return 0;
}

static int octtx_dev_release(struct inode *inode, struct file *fp)
{
	if (gpio_in_use == 0)
		return -EINVAL;
	gpio_in_use = 0;
	return 0;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = octtx_dev_open,
	.release = octtx_dev_release,
	.unlocked_ioctl = octtx_dev_ioctl
};

static int __init octeontx_init_module(void)
{
	int ret;

	pr_info("%s, ver %s, MBOX IF ver %d.%d.%d\n", DRV_NAME, DRV_VERSION,
		MBOX_INTERFACE_VERSION.platform, MBOX_INTERFACE_VERSION.major,
		MBOX_INTERFACE_VERSION.minor);

	bgx = bgx_octeontx_init();
	if (!bgx)
		return -ENODEV;
	slipf = try_then_request_module(symbol_get(slipf_com), "slipf");
	if (!slipf)
		return -ENODEV;
	lbk = try_then_request_module(symbol_get(lbk_com), "lbk");
	if (!lbk)
		return -ENODEV;
	fpapf = try_then_request_module(symbol_get(fpapf_com), "fpapf");
	if (!fpapf) {
		ret = -ENODEV;
		goto fpapf_err;
	}
	ssopf = try_then_request_module(symbol_get(ssopf_com), "ssopf");
	if (!ssopf) {
		ret = -ENODEV;
		goto ssopf_err;
	}
	ssowpf = try_then_request_module(symbol_get(ssowpf_com), "ssowpf");
	if (!ssowpf) {
		ret = -ENODEV;
		goto ssowpf_err;
	}
	pki = try_then_request_module(symbol_get(pki_com), "pki");
	if (!pki) {
		ret = -ENODEV;
		goto pki_err;
	}
	pkopf = try_then_request_module(symbol_get(pkopf_com), "pkopf");
	if (!pkopf) {
		ret = -ENODEV;
		goto pkopf_err;
	}

	dpipf = try_then_request_module(symbol_get(dpipf_com), "dpi");
	if (!dpipf) {
		ret = -ENODEV;
		goto dpipf_err;
	}

	zippf = try_then_request_module(symbol_get(zippf_com), "zippf");
	if (!zippf) {
		ret = -ENODEV;
		goto zippf_err;
	}

	timpf = try_then_request_module(symbol_get(timpf_com), "timpf");
	if (!timpf) {
		ret = -ENODEV;
		goto timpf_err;
	}

	/* Register a physical link status poll fn() */
	check_link = alloc_workqueue("octeontx_check_link_status",
				     WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!check_link) {
		ret = -ENOMEM;
		goto wq_err;
	}

	/* Register a physical link status poll fn() */
	reset_domain = alloc_workqueue("octeontx_reset_domain",
				       WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!reset_domain) {
		ret = -ENOMEM;
		goto wq_err;
	}

	INIT_DELAYED_WORK(&dwork, poll_for_link);
	INIT_DELAYED_WORK(&dwork_reset, octtx_reset_domain);
	queue_delayed_work(check_link, &dwork, 0);
	queue_delayed_work(reset_domain, &dwork_reset, 0);

	/* Register task cleanup handler */
	ret = task_cleanup_handler_add(cleanup_el3_irqs);
	if (ret != 0) {
		ret = -ENODEV;
		goto cleanup_handler_err;
	}

	/* create a char device */
	ret = alloc_chrdev_region(&octtx_dev, 1, 1, DEVICE_NAME);
	if (ret != 0) {
		ret = -ENODEV;
		goto alloc_chrdev_err;
	}

	octtx_cdev = cdev_alloc();
	if (!octtx_cdev) {
		ret = -ENODEV;
		goto cdev_alloc_err;
	}

	cdev_init(octtx_cdev, &fops);
	ret = cdev_add(octtx_cdev, octtx_dev, 1);
	if (ret < 0) {
		ret = -ENODEV;
		goto cdev_add_err;
	}

	/* create new class for sysfs*/
	octtx_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(octtx_class)) {
		ret = -ENODEV;
		goto class_create_err;
	}

	octtx_device = device_create(octtx_class, NULL, octtx_dev, NULL,
				     DEVICE_NAME);
	if (IS_ERR(octtx_device)) {
		ret = -ENODEV;
		goto device_create_err;
	}

	ret = octtx_sysfs_init(octtx_device);
	if (ret != 0) {
		ret = -ENODEV;
		goto  sysfs_init_err;
	}

	/* Done */
	return 0;

sysfs_init_err:
	device_destroy(octtx_class, octtx_dev);

device_create_err:
	class_destroy(octtx_class);

class_create_err:
cdev_add_err:
	cdev_del(octtx_cdev);

cdev_alloc_err:
	unregister_chrdev_region(octtx_dev, 1);

alloc_chrdev_err:
cleanup_handler_err:
	task_cleanup_handler_remove(cleanup_el3_irqs);

wq_err:
	symbol_put(timpf_com);

timpf_err:
	symbol_put(zippf_com);

zippf_err:
	symbol_put(dpipf_com);

dpipf_err:
	symbol_put(pkopf_com);

pkopf_err:
	symbol_put(pki_com);

pki_err:
	symbol_put(ssowpf_com);

ssowpf_err:
	symbol_put(ssopf_com);

ssopf_err:
	symbol_put(fpapf_com);

fpapf_err:
	symbol_put(lbk_com);
	symbol_put(slipf_com);

	return ret;
}

static void __exit octeontx_cleanup_module(void)
{
	cancel_delayed_work_sync(&dwork);
	cancel_delayed_work_sync(&dwork_reset);
	flush_workqueue(check_link);
	flush_workqueue(reset_domain);
	destroy_workqueue(check_link);
	destroy_workqueue(reset_domain);

	octtx_sysfs_remove(octtx_device);
	device_destroy(octtx_class, octtx_dev);
	class_destroy(octtx_class);

	cdev_del(octtx_cdev);
	unregister_chrdev_region(octtx_dev, 1);

	symbol_put(pki_com);
	symbol_put(ssopf_com);
	symbol_put(ssowpf_com);
	symbol_put(fpapf_com);
	symbol_put(pkopf_com);
	symbol_put(timpf_com);
	symbol_put(zippf_com);
	symbol_put(lbk_com);
	symbol_put(slipf_com);
	symbol_put(thunder_bgx_com);
	task_cleanup_handler_remove(cleanup_el3_irqs);
}

module_init(octeontx_init_module);
module_exit(octeontx_cleanup_module);
