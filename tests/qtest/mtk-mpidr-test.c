/*
 * MT6895 MPIDR board configuration test.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"

static QDict *get_cpu_property(QTestState *qts, const char *path,
                               const char *property)
{
    QDict *reply = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                           "{ 'path': %s, 'property': %s } }", path, property);

    g_assert(qdict_haskey(reply, "return"));
    return reply;
}

static void check_cpu_profile(QTestState *qts, unsigned int ncpus,
                              bool expected_mt)
{
    g_autoptr(QDict) reply = qtest_qmp(qts, "{ 'execute': 'query-cpus-fast' }");
    QList *cpus = qdict_get_qlist(reply, "return");
    const QListEntry *entry;
    unsigned int seen = 0;

    g_assert_nonnull(cpus);
    g_assert_cmpuint(qlist_size(cpus), ==, ncpus);
    QLIST_FOREACH_ENTRY(cpus, entry) {
        QDict *cpu = qobject_to(QDict, qlist_entry_obj(entry));
        int64_t index;
        const char *path;
        g_autoptr(QDict) mt = NULL;
        g_autoptr(QDict) affinity = NULL;

        g_assert_nonnull(cpu);
        index = qdict_get_int(cpu, "cpu-index");
        path = qdict_get_str(cpu, "qom-path");
        g_assert_cmpint(index, >=, 0);
        g_assert_cmpint(index, <, ncpus);
        g_assert_cmphex(seen & (1U << index), ==, 0);
        seen |= 1U << index;

        mt = get_cpu_property(qts, path, "mpidr-mt");
        affinity = get_cpu_property(qts, path, "mp-affinity");
        g_assert_cmpint(qdict_get_bool(mt, "return"), ==, expected_mt);
        g_assert_cmphex(qdict_get_uint(affinity, "return"), ==,
                        (uint64_t)index << 8);
    }
}

static void test_mpidr_board_profile(void)
{
    QTestState *qts;

    /* Verify QOM configuration, not a guest MRS or CPU bring-up. */
    qts = qtest_init("-machine mt6895 -m 256M -smp 8 -nodefaults");
    check_cpu_profile(qts, 8, true);
    qtest_quit(qts);

    /* The same CPU type without the MT6895 board override stays unchanged. */
    qts = qtest_init("-machine none -cpu cortex-a55 -smp 1 -nodefaults");
    check_cpu_profile(qts, 1, false);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/mpidr-board-profile", test_mpidr_board_profile);
    return g_test_run();
}
