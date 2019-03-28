/*
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * vpu_encoder_config.h
 *
 * Author Ming Qian<ming.qian@nxp.com>
 */
#ifndef _VPU_ENCODER_CONFIG_H
#define _VPU_ENCODER_CONFIG_H

#define VPU_ENC_WIDTH_MAX		1920
#define VPU_ENC_HEIGHT_MAX		1080
#define VPU_ENC_WIDTH_MIN		64
#define VPU_ENC_HEIGHT_MIN		48
#define VPU_ENC_WIDTH_STEP		16
#define VPU_ENC_HEIGHT_STEP		2

#define VPU_ENC_FRAMERATE_DEFAULT	30

#define VPU_MEM_PATTERN		0x5a5a5a5a

#define VPU_TAIL_SERACH_SIZE		32
#define VPU_STRM_END_PATTERN		{0x0, 0x0, 0x1, 0xb}

#endif
