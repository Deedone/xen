/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ARM GICv4 ITS support
 *
 * Penny Zheng <penny.zheng@arm.com>
 * Copyright (c) 2023 ARM Ltd.
 */

#ifndef ARM_GIC_V4_ITS_H
#define ARM_GIC_V4_ITS_H

#include <xen/spinlock.h>
#include <xen/types.h>

#define GITS_CMD_VMOVI                   0x21
#define GITS_CMD_VMOVP                   0x22
#define GITS_CMD_VSYNC                   0x25
#define GITS_CMD_VMAPP                   0x29
#define GITS_CMD_VMAPTI                  0x2a
#define GITS_CMD_VINVALL                 0x2d

struct its_vpe;

struct its_vm {
    struct its_vpe **vpes;
    /* Number of VPE. */
    unsigned int nr_vpes;
    uint32_t *db_lpi_bases;
    unsigned int nr_db_lpis;
    /* Property table per VM. */
    void *vproptable;
};

struct its_vpe {
    rwlock_t lock;
    uint32_t vpe_id;
    /* Pending table per VCPU. */
    void *vpendtable;
    uint32_t vpe_db_lpi;
    struct its_vm *its_vm;
    unsigned int col_idx;
    bool resident;
    /* Pending VLPIs on schedule out? */
    bool            pending_last;
    struct {
        /* Implementation Defined Area Invalid */
        bool idai;
        /* VPE proxy mapping */
        int vpe_proxy_event;
    };
    /*
     * Ensure mutual exclusion between affinity setting of the vPE
     * and vLPI operations using vpe->col_idx.
     */
    spinlock_t vpe_lock;
};

#endif

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
