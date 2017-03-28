/*
 *  Copyright (C) 2016 Intel Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */

/*
  ==============================================================================
  ==============================================================================
  Private
  ==============================================================================
  ==============================================================================
*/

#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/linkage.h>
#include <linux/axxia-oem.h>
#include <linux/uaccess.h>

/*
  OEM Calls
*/

struct oem_parameters {
	volatile unsigned long reg0;
	volatile unsigned long reg1;
	volatile unsigned long reg2;
	volatile unsigned long reg3;
};

static void
invoke_oem_fn(struct oem_parameters *p)
{
	asm volatile("mov x0, %x0\n"
		     "mov x1, %x1\n"
		     "mov x2, %x2\n"
		     "mov x3, %x3" : : "r" (p->reg0), "r" (p->reg1),
		     "r" (p->reg2), "r" (p->reg3));
	asm volatile("smc #0");
	asm volatile("mov %x0, x0\n"
		     "mov %x1, x1\n"
		     "mov %x2, x2\n"
		     "mov %x3, x3" : "=r" (p->reg0), "=r" (p->reg1),
		     "=r" (p->reg2), "=r" (p->reg3));

	return;
}

/*
  DSP Cluster Control
*/

static ssize_t
axxia_dspc_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_dspc_get_state());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_dspc_write(struct file *file, const char __user *buffer,
		 size_t count, loff_t *ppos)
{
	char *input;
	unsigned long mask;
	int ret;

	input = kmalloc(count, __GFP_RECLAIMABLE);
	memset(input, 0, count);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count))
		return -EFAULT;

	ret = kstrtoul(input, 0, &mask);
	if (ret) {
		pr_err("Failed axxia_dspc_write mask conversion\n");
		return -EFAULT;
	}

	axxia_dspc_set_state((unsigned int)mask);
	return count;
}

static const struct file_operations axxia_dspc_proc_ops = {
	.read       = axxia_dspc_read,
	.write      = axxia_dspc_write,
	.llseek     = noop_llseek,
};

/*
  ACTLR_EL3 Control
*/

static ssize_t
axxia_actlr_el3_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_actlr_el3_get());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_actlr_el3_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *ppos)
{
	char *input;
	unsigned long data = 0;
	int ret = 0;

	input = kmalloc(count, __GFP_RECLAIMABLE);
	memset(input, 0, count);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count))
		return -EFAULT;

	ret = kstrtoul(input, 0, &data);
	if (ret) {
		pr_err("axxia_actlr_el3_write failed conversion\n");
		return ret;
	}
	axxia_actlr_el3_set(data);

	return count;
}

static const struct file_operations axxia_actlr_el3_proc_ops = {
	.read       = axxia_actlr_el3_read,
	.write      = axxia_actlr_el3_write,
	.llseek     = noop_llseek,
};

/*
  ACTLR_EL2 Control
*/

static ssize_t
axxia_actlr_el2_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_actlr_el2_get());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_actlr_el2_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *ppos)
{
	char *input;
	unsigned long data = 0;
	int ret = 0;

	input = kmalloc(count, __GFP_RECLAIMABLE);
	memset(input, 0, count);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count))
		return -EFAULT;

	ret = kstrtoul(input, 0, &data);
	if (ret) {
		pr_err("axxia_actlr_el2_write failed conversion\n");
		return ret;
	}
	axxia_actlr_el2_set(data);

	return count;
}

static const struct file_operations axxia_actlr_el2_proc_ops = {
	.read       = axxia_actlr_el2_read,
	.write      = axxia_actlr_el2_write,
	.llseek     = noop_llseek,
};

/*
  ==============================================================================
  ==============================================================================
  Public
  ==============================================================================
  ==============================================================================
*/

/*
  ------------------------------------------------------------------------------
  axxia_dspc_get_state
*/

unsigned long
axxia_dspc_get_state(void)
{
	struct oem_parameters parameters;

	parameters.reg0 = 0xc3000000;
	parameters.reg1 = 0;
	parameters.reg2 = 0;
	parameters.reg3 = 0;
	invoke_oem_fn(&parameters);

	if (0 != parameters.reg0)
		pr_warn("Getting the DSP State Failed!\n");

	return parameters.reg1;
}
EXPORT_SYMBOL(axxia_dspc_get_state);

/*
  ------------------------------------------------------------------------------
  axxia_dspc_set_state
*/

void
axxia_dspc_set_state(unsigned long state)
{
	struct oem_parameters parameters;

	parameters.reg0 = 0xc3000001;
	parameters.reg1 = state;
	parameters.reg2 = 0;
	parameters.reg3 = 0;
	invoke_oem_fn(&parameters);

	if (0 != parameters.reg0)
		pr_warn("Setting the DSP State Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_dspc_set_state);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el3_get
*/

unsigned long
axxia_actlr_el3_get(void)
{
	struct oem_parameters parameters;

	parameters.reg0 = 0xc3000002;
	invoke_oem_fn(&parameters);

	if (0 != parameters.reg0)
		pr_warn("Getting ACTLR_EL3 Failed!\n");

	return parameters.reg1;
}
EXPORT_SYMBOL(axxia_actlr_el3_get);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el3_set
*/

void
axxia_actlr_el3_set(unsigned long input)
{
	struct oem_parameters parameters;

	parameters.reg0 = 0xc3000003;
	parameters.reg1 = input;
	invoke_oem_fn(&parameters);

	if (0 != parameters.reg0)
		pr_warn("Setting ACTLR_EL3 Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_actlr_el3_set);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el2_get
*/

unsigned long
axxia_actlr_el2_get(void)
{
	struct oem_parameters parameters;

	parameters.reg0 = 0xc3000004;
	invoke_oem_fn(&parameters);

	if (0 != parameters.reg0)
		pr_warn("Getting ACTLR_EL2 Failed!\n");

	return parameters.reg1;
}
EXPORT_SYMBOL(axxia_actlr_el2_get);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el2_set
*/

void
axxia_actlr_el2_set(unsigned long input)
{
	struct oem_parameters parameters;

	parameters.reg0 = 0xc3000005;
	parameters.reg1 = input;
	invoke_oem_fn(&parameters);

	if (0 != parameters.reg0)
		pr_warn("Setting ACTLR_EL2 Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_actlr_el2_set);

/*
  ------------------------------------------------------------------------------
  axxia_dspc_init
*/

static int
axxia_oem_init(void)
{
	if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		/* Only applicable to the 6700. */
		if (NULL == proc_create("driver/axxia_dspc", S_IWUSR, NULL,
					&axxia_dspc_proc_ops))
			pr_err("Could not create /proc/driver/axxia_dspc!\n");
		else
			pr_info("Axxia DSP Control Initialized\n");
	}

	if (NULL == proc_create("driver/axxia_actlr_el3", S_IWUSR, NULL,
				&axxia_actlr_el3_proc_ops))
		pr_err("Could not create /proc/driver/axxia_actlr_el3!\n");
	else
		pr_info("Axxia ACTLR_EL3 Control Initialized\n");

	if (NULL == proc_create("driver/axxia_actlr_el2", S_IWUSR, NULL,
				&axxia_actlr_el2_proc_ops))
		pr_err("Could not create /proc/driver/axxia_actlr_el2!\n");
	else
		pr_info("Axxia ACTLR_EL3 Control Initialized\n");

	return 0;
}

device_initcall(axxia_oem_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Axxia OEM Control");
