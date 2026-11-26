/*
 * Cached MTK_* environment probes for the MT6895 device models.
 *
 * getenv() walks the entire environment block on every call, which costs
 * roughly 2-3 microseconds per call on the Windows build host -- far too much
 * for I/O callbacks that run thousands of times per second.  These probes are
 * cached because the environment does not change while QEMU runs. The cache
 * may be read by more than one vCPU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef QEMU_MTK_ENV_H
#define QEMU_MTK_ENV_H

#include "qemu/atomic.h"

#define MTK_ENV_CACHED(name)                                    \
    ({                                                          \
        static int mtk_env_cache_ = -1;                         \
        int mtk_env_value_ = qatomic_read(&mtk_env_cache_);     \
        if (mtk_env_value_ < 0) {                               \
            mtk_env_value_ = getenv(name) != NULL;              \
            qatomic_set(&mtk_env_cache_, mtk_env_value_);       \
        }                                                       \
        mtk_env_value_;                                         \
    })

#endif /* QEMU_MTK_ENV_H */
