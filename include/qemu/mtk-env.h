/*
 * Cached MTK_* environment probes for the MT6895 device models.
 *
 * getenv() walks the entire environment block on every call, which costs
 * roughly 2-3 microseconds per call on the Windows build host -- far too much
 * for I/O callbacks that run thousands of times per second.  These probes are
 * read-once: the environment does not change while QEMU runs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef QEMU_MTK_ENV_H
#define QEMU_MTK_ENV_H

#define MTK_ENV_CACHED(name)                     \
    ({                                           \
        static int mtk_env_cache_ = -1;          \
        if (mtk_env_cache_ < 0) {                \
            mtk_env_cache_ = getenv(name) ? 1 : 0; \
        }                                        \
        mtk_env_cache_;                          \
    })

#endif /* QEMU_MTK_ENV_H */
