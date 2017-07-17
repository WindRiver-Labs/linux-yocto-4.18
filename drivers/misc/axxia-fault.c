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

#define DEBUG

#include <linux/module.h>
#include <linux/of.h>
#include <linux/signal.h>

#include <asm/system_misc.h>

static int mask_aborts;
module_param(mask_aborts, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(mask_aborts, "Masking Aborts");

/*
  ------------------------------------------------------------------------------
  axxia_x7_fault
*/

#if defined(ARM)

static int
axxia_x7_fault(unsigned long addr, unsigned int fsr, struct pt_regs *regs)
{
	if (0 == mask_aborts)
		return -1;

	pr_warn("Masking System Error...\n");

	return 0;
}

#endif	/* defined(ARM) */

/*
  ------------------------------------------------------------------------------
  axxia_x9xlf_fault

  Return 0 to indicate the error is not a problem (should be ignored)
  or non-zero to indicate that it was not handled.

  This is intended to handle SErrors (Asynchronous Aborts) caused by
  sRIO optional register accesses, which are not supported by the
  Axxia sRIO hardware.  As it isn't possible to determine the cause
  completely, a mechanism is provided to enable or disable masking
  from user space.

  For 5600 (Cortex-A57)

      In the ESR (see ARM documentation for fields), the following
      should be true.

	    esr[31:26] = 0x2f
	    esr[24] = 1       (iss is valid)
	    esr[23:0] = 0x2   (slave error)
*/

asmlinkage int
axxia_x9xlf_fault(struct pt_regs *regs, int el, unsigned long esr)
{
	unsigned int ec;
	unsigned int il;
	unsigned int iss_valid;
	unsigned int iss_value;

	pr_warn("Axxia System Error Handler\n");
	ec = ((esr >> 26) & 0x3f);
	il = ((esr >> 25) & 1);
	iss_valid = ((esr & (1 << 24)) == 0) ? 0 : 1;
	iss_value = (esr & 0xffffff);
	pr_warn("el=%d esr=0x%lx (ec=0x%x il=%d)\n", (int)el, esr, ec, il);

	if (0 == iss_valid) {
		pr_warn("iss=INVALID\n");

		return -1;
	}

	pr_warn("iss=0x%x\n", iss_value);

	if ((0x2f != ec) || (2 != iss_value))
		return -1;

	if (0 == mask_aborts)
		return -1;

	pr_warn("Masking System Error...\n");

	return 0;
}

/*
  ==============================================================================
  ==============================================================================
  Public
  ==============================================================================
  ==============================================================================
*/

/*
  ------------------------------------------------------------------------------
  axxia_fault_get_mask
*/

int
axxia_fault_get_mask(void)
{
	return mask_aborts;
}


/*
  ------------------------------------------------------------------------------
  axxia_fault_set_mask
*/

void
axxia_fault_set_mask(int new_mask)
{
	mask_aborts = new_mask;

	return;
}


/*
  ------------------------------------------------------------------------------
  axxia_fault_init
*/

static int __init
axxia_fault_init(void)
{
#if defined(ARM)
	if (of_find_compatible_node(NULL, NULL, "lsi,axm5500") ||
	    of_find_compatible_node(NULL, NULL, "lsi,axm5516")) {
		mask_aborts = 1;
		hook_fault_code(0x11, axxia_x7_fault, SIGBUS, 0,
				"asynchronous external abort");
		pr_debug("Set up fault handler for Axxia 5500\n");
	}
#elif defined(ARM64)
	if (of_find_compatible_node(NULL, NULL, "lsi,axm5616") ||
	    of_find_compatible_node(NULL, NULL, "lsi,axc6732"))
		pr_debug("Set up fault handler for Axxia 5600/6700\n");
#endif

	return 0;
}

early_initcall(axxia_fault_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Axxia Fault Handlers");
