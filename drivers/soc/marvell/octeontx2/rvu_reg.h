// SPDX-License-Identifier: GPL-2.0
/* Marvell OcteonTx2 RVU Admin Function driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef RVU_REG_H
#define RVU_REG_H

/* Admin function registers */
#define RVU_AF_MSIXTR_BASE                  (0x10)
#define RVU_AF_ECO                          (0x20)
#define RVU_AF_BLK_RST                      (0x30)
#define RVU_AF_PF_BAR4_ADDR                 (0x40)
#define RVU_AF_RAS                          (0x100)
#define RVU_AF_RAS_W1S                      (0x108)
#define RVU_AF_RAS_ENA_W1S                  (0x110)
#define RVU_AF_RAS_ENA_W1C                  (0x118)
#define RVU_AF_GEN_INT                      (0x120)
#define RVU_AF_GEN_INT_W1S                  (0x128)
#define RVU_AF_GEN_INT_ENA_W1S              (0x130)
#define RVU_AF_GEN_INT_ENA_W1C              (0x138)
#define	RVU_AF_AFPF_MBOX0		    (0x02000)
#define	RVU_AF_AFPF_MBOX1		    (0x02008)
#define RVU_AF_AFPFX_MBOXX(a, b)            (0x2000 | (a) << 4 | (b) << 3)
#define RVU_AF_PFME_STATUS                  (0x2800)
#define RVU_AF_PFTRPEND                     (0x2810)
#define RVU_AF_PFTRPEND_W1S                 (0x2820)
#define RVU_AF_PF_RST                       (0x2840)
#define RVU_AF_HWVF_RST                     (0x2850)
#define RVU_AF_PFAF_MBOX_INT                (0x2880)
#define RVU_AF_PFAF_MBOX_INT_W1S            (0x2888)
#define RVU_AF_PFAF_MBOX_INT_ENA_W1S        (0x2890)
#define RVU_AF_PFAF_MBOX_INT_ENA_W1C        (0x2898)
#define RVU_AF_PFFLR_INT                    (0x28a0)
#define RVU_AF_PFFLR_INT_W1S                (0x28a8)
#define RVU_AF_PFFLR_INT_ENA_W1S            (0x28b0)
#define RVU_AF_PFFLR_INT_ENA_W1C            (0x28b8)
#define RVU_AF_PFME_INT                     (0x28c0)
#define RVU_AF_PFME_INT_W1S                 (0x28c8)
#define RVU_AF_PFME_INT_ENA_W1S             (0x28d0)
#define RVU_AF_PFME_INT_ENA_W1C             (0x28d8)

/* Admin function's privileged PF/VF registers */
#define RVU_PRIV_CONST                      (0x8000000)
#define RVU_PRIV_GEN_CFG                    (0x8000010)
#define RVU_PRIV_CLK_CFG                    (0x8000020)
#define RVU_PRIV_ACTIVE_PC                  (0x8000030)
#define RVU_PRIV_PFX_CFG(a)                 (0x8000100 | (a) << 16)
#define RVU_PRIV_PFX_MSIX_CFG(a)            (0x8000110 | (a) << 16)
#define RVU_PRIV_PFX_ID_CFG(a)              (0x8000120 | (a) << 16)
#define RVU_PRIV_PFX_INT_CFG(a)             (0x8000200 | (a) << 16)
#define RVU_PRIV_PFX_NIX_CFG                (0x8000300)
#define RVU_PRIV_PFX_NPA_CFG		    (0x8000310)
#define RVU_PRIV_PFX_SSO_CFG                (0x8000320)
#define RVU_PRIV_PFX_SSOW_CFG               (0x8000330)
#define RVU_PRIV_PFX_TIM_CFG                (0x8000340)
#define RVU_PRIV_PFX_CPT_CFG                (0x8000350)
#define RVU_PRIV_BLOCK_TYPEX_REV(a)         (0x8000400 | (a) << 3)
#define RVU_PRIV_HWVFX_INT_CFG(a)           (0x8001280 | (a) << 16)
#define RVU_PRIV_HWVFX_NIX_CFG              (0x8001300)
#define RVU_PRIV_HWVFX_NPA_CFG              (0x8001310)
#define RVU_PRIV_HWVFX_SSO_CFG              (0x8001320)
#define RVU_PRIV_HWVFX_SSOW_CFG             (0x8001330)
#define RVU_PRIV_HWVFX_TIM_CFG              (0x8001340)
#define RVU_PRIV_HWVFX_CPT_CFG              (0x8001350)

/* RVU PF registers */
#define	RVU_PF_VFX_PFVF_MBOX0		    (0x00000)
#define	RVU_PF_VFX_PFVF_MBOX1		    (0x00008)
#define RVU_PF_VFX_PFVF_MBOXX(a, b)         (0x0 | (a) << 12 | (b) << 3)
#define RVU_PF_VF_BAR4_ADDR                 (0x10)
#define RVU_PF_BLOCK_ADDRX_DISC(a)          (0x200 | (a) << 3)
#define RVU_PF_VFME_STATUSX(a)              (0x800 | (a) << 3)
#define RVU_PF_VFTRPENDX(a)                 (0x820 | (a) << 3)
#define RVU_PF_VFTRPEND_W1SX(a)             (0x840 | (a) << 3)
#define RVU_PF_VFPF_MBOX_INTX(a)            (0x880 | (a) << 3)
#define RVU_PF_VFPF_MBOX_INT_W1SX(a)        (0x8A0 | (a) << 3)
#define RVU_PF_VFPF_MBOX_INT_ENA_W1SX(a)    (0x8C0 | (a) << 3)
#define RVU_PF_VFPF_MBOX_INT_ENA_W1CX(a)    (0x8E0 | (a) << 3)
#define RVU_PF_VFFLR_INTX(a)                (0x900 | (a) << 3)
#define RVU_PF_VFFLR_INT_W1SX(a)            (0x920 | (a) << 3)
#define RVU_PF_VFFLR_INT_ENA_W1SX(a)        (0x940 | (a) << 3)
#define RVU_PF_VFFLR_INT_ENA_W1CX(a)        (0x960 | (a) << 3)
#define RVU_PF_VFME_INTX(a)                 (0x980 | (a) << 3)
#define RVU_PF_VFME_INT_W1SX(a)             (0x9A0 | (a) << 3)
#define RVU_PF_VFME_INT_ENA_W1SX(a)         (0x9C0 | (a) << 3)
#define RVU_PF_VFME_INT_ENA_W1CX(a)         (0x9E0 | (a) << 3)
#define RVU_PF_PFAF_MBOX0                   (0xC00)
#define RVU_PF_PFAF_MBOX1                   (0xC08)
#define RVU_PF_PFAF_MBOXX(a)                (0xC00 | (a) << 3)
#define RVU_PF_INT                          (0xc20)
#define RVU_PF_INT_W1S                      (0xc28)
#define RVU_PF_INT_ENA_W1S                  (0xc30)
#define RVU_PF_INT_ENA_W1C                  (0xc38)
#define RVU_PF_MSIX_VECX_ADDR(a)            (0x000 | (a) << 4)
#define RVU_PF_MSIX_VECX_CTL(a)             (0x008 | (a) << 4)
#define RVU_PF_MSIX_PBAX(a)                 (0xF0000 | (a) << 3)


#define NPA_AF_BLK_RST                  (0x0000)
#define NIX_AF_BLK_RST                  (0x00B0)
#define SSO_AF_BLK_RST                  (0x10f8)
#define TIM_AF_BLK_RST                  (0x10)
#define CPT_AF_BLK_RST                  (0x46000)
#define NDC_AF_BLK_RST                  (0x002F0)
#define NPC_AF_BLK_RST                  (0x00040)

#endif /* RVU_REG_H */
