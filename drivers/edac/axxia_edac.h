/*
 * axxia_edac.h
 *
 *  Created on: Dec 27, 2017
 *      Author: z8cpaul
 */

#ifndef DRIVERS_EDAC_AXXIA_EDAC_H_
#define DRIVERS_EDAC_AXXIA_EDAC_H_


extern void edac_device_handle_multi_ce(struct edac_device_ctl_info *edac_dev,
				int inst_nr, int block_nr, int events,
				const char *msg);

extern void edac_device_handle_multi_ue(struct edac_device_ctl_info *edac_dev,
				int inst_nr, int block_nr, int events,
				const char *msg);


#endif /* DRIVERS_EDAC_AXXIA_EDAC_H_ */
