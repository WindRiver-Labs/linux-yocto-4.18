#ifndef __I2C_AXXIA_H__
#define __I2C_AXXIA_H__

#include <linux/platform_device.h>

/*
 * Version 2 of the I2C peripheral unit has a different register
 * layout and extra registers.  The ID register in the V2 peripheral
 * unit on the AXXIA4430 reports the same ID as the V1 peripheral
 * unit on the AXXIA3530, so we must inform the driver which IP
 * version we know it is running on from platform / cpu-specific
 * code using these constants in the hwmod class definition.
 */

#define AXXIA_I2C_IP_VERSION_1 1                /* ACP34xx */
#define AXXIA_I2C_IP_VERSION_2 2                /* AXM55xx */

/* struct axxia_i2c_bus_platform_data .flags meanings */
#define AXXIA_I2C_FLAGS_NONE            (0x00000000)


/*
 * Maximum byte size of I2C bus name string including null terminator
 */
#define MAX_AXXIA_I2C_HWMOD_NAME_LEN    16


struct axxia_i2c_bus_platform_data {
	struct device_node     *node;
	char                    name[MAX_AXXIA_I2C_HWMOD_NAME_LEN];
	u32		        index;
	u32		        rev;
	u32		        flags;
	u32		        bus_nr;
	struct resource         dev_space;
	struct resource         int_space;
};

#endif
