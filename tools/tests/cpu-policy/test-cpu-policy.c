#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <err.h>

#include <xen-tools/libs.h>
#include <xen/lib/x86/cpuid.h>
#include <xen/lib/x86/msr.h>
#include <xen/domctl.h>

static unsigned int nr_failures;
#define fail(fmt, ...)                          \
({                                              \
    nr_failures++;                              \
    printf(fmt, ##__VA_ARGS__);                 \
})

static void test_cpuid_serialise_success(void)
{
    static const struct test {
        struct cpuid_policy p;
        const char *name;
        unsigned int nr_leaves;
    } tests[] = {
        {
            .name = "empty policy",
            .nr_leaves = 4,
        },
    };

    printf("Testing CPUID serialise success:\n");

    for ( size_t i = 0; i < ARRAY_SIZE(tests); ++i )
    {
        const struct test *t = &tests[i];
        unsigned int nr = t->nr_leaves;
        xen_cpuid_leaf_t *leaves = malloc(nr * sizeof(*leaves));
        int rc;

        if ( !leaves )
            err(1, "%s() malloc failure", __func__);

        rc = x86_cpuid_copy_to_buffer(&t->p, leaves, &nr);

        if ( rc != 0 )
        {
            fail("  Test %s, expected rc 0, got %d\n",
                 t->name, rc);
            goto test_done;
        }

        if ( nr != t->nr_leaves )
        {
            fail("  Test %s, expected %u leaves, got %u\n",
                 t->name, t->nr_leaves, nr);
            goto test_done;
        }

    test_done:
        free(leaves);
    }
}

static void test_msr_serialise_success(void)
{
    static const struct test {
        struct msr_policy p;
        const char *name;
        unsigned int nr_msrs;
    } tests[] = {
        {
            .name = "empty policy",
            .nr_msrs = MSR_MAX_SERIALISED_ENTRIES,
        },
    };

    printf("Testing MSR serialise success:\n");

    for ( size_t i = 0; i < ARRAY_SIZE(tests); ++i )
    {
        const struct test *t = &tests[i];
        unsigned int nr = t->nr_msrs;
        xen_msr_entry_t *msrs = malloc(nr * sizeof(*msrs));
        int rc;

        if ( !msrs )
            err(1, "%s() malloc failure", __func__);

        rc = x86_msr_copy_to_buffer(&t->p, msrs, &nr);

        if ( rc != 0 )
        {
            fail("  Test %s, expected rc 0, got %d\n",
                 t->name, rc);
            goto test_done;
        }

        if ( nr != t->nr_msrs )
        {
            fail("  Test %s, expected %u msrs, got %u\n",
                 t->name, t->nr_msrs, nr);
            goto test_done;
        }

    test_done:
        free(msrs);
    }
}

static void test_cpuid_deserialise_failure(void)
{
    static const struct test {
        const char *name;
        xen_cpuid_leaf_t leaf;
    } tests[] = {
        {
            .name = "incorrect basic subleaf",
            .leaf = { .leaf = 0, .subleaf = 0 },
        },
        {
            .name = "incorrect hv1 subleaf",
            .leaf = { .leaf = 0x40000000, .subleaf = 0 },
        },
        {
            .name = "incorrect hv2 subleaf",
            .leaf = { .leaf = 0x40000100, .subleaf = 0 },
        },
        {
            .name = "incorrect extd subleaf",
            .leaf = { .leaf = 0x80000000, .subleaf = 0 },
        },
        {
            .name = "OoB basic leaf",
            .leaf = { .leaf = CPUID_GUEST_NR_BASIC },
        },
        {
            .name = "OoB cache leaf",
            .leaf = { .leaf = 0x4, .subleaf = CPUID_GUEST_NR_CACHE },
        },
        {
            .name = "OoB feat leaf",
            .leaf = { .leaf = 0x7, .subleaf = CPUID_GUEST_NR_FEAT },
        },
        {
            .name = "OoB topo leaf",
            .leaf = { .leaf = 0xb, .subleaf = CPUID_GUEST_NR_TOPO },
        },
        {
            .name = "OoB xstate leaf",
            .leaf = { .leaf = 0xd, .subleaf = CPUID_GUEST_NR_XSTATE },
        },
        {
            .name = "OoB extd leaf",
            .leaf = { .leaf = 0x80000000 | CPUID_GUEST_NR_EXTD },
        },
    };

    printf("Testing CPUID deserialise failure:\n");

    for ( size_t i = 0; i < ARRAY_SIZE(tests); ++i )
    {
        const struct test *t = &tests[i];
        uint32_t err_leaf = ~0u, err_subleaf = ~0u;
        int rc;

        /* No writes should occur.  Use NULL to catch errors. */
        rc = x86_cpuid_copy_from_buffer(NULL, &t->leaf, 1,
                                        &err_leaf, &err_subleaf);

        if ( rc != -ERANGE )
        {
            fail("  Test %s, expected rc %d, got %d\n",
                 t->name, -ERANGE, rc);
            continue;
        }

        if ( err_leaf != t->leaf.leaf || err_subleaf != t->leaf.subleaf )
        {
            fail("  Test %s, expected err %08x:%08x, got %08x:%08x\n",
                 t->name, t->leaf.leaf, t->leaf.subleaf,
                 err_leaf, err_subleaf);
            continue;
        }
    }
}

static void test_msr_deserialise_failure(void)
{
    static const struct test {
        const char *name;
        xen_msr_entry_t msr;
        int rc;
    } tests[] = {
        {
            .name = "bad msr index",
            .msr = { .idx = 0xdeadc0de },
            .rc = -ERANGE,
        },
        {
            .name = "nonzero flags",
            .msr = { .idx = 0xce, .flags = 1 },
            .rc = -EINVAL,
        },
        {
            .name = "truncated val",
            .msr = { .idx = 0xce, .val = ~0ull },
            .rc = -EOVERFLOW,
        },
    };

    printf("Testing MSR deserialise failure:\n");

    for ( size_t i = 0; i < ARRAY_SIZE(tests); ++i )
    {
        const struct test *t = &tests[i];
        uint32_t err_msr = ~0u;
        int rc;

        /* No writes should occur.  Use NULL to catch errors. */
        rc = x86_msr_copy_from_buffer(NULL, &t->msr, 1, &err_msr);

        if ( rc != t->rc )
        {
            fail("  Test %s, expected rc %d, got %d\n",
                 t->name, t->rc, rc);
            continue;
        }

        if ( err_msr != t->msr.idx )
        {
            fail("  Test %s, expected err_msr %#x, got %#x\n",
                 t->name, t->msr.idx, err_msr);
            continue;
        }
    }
}

int main(int argc, char **argv)
{
    printf("CPU Policy unit tests\n");

    test_cpuid_serialise_success();
    test_msr_serialise_success();

    test_cpuid_deserialise_failure();
    test_msr_deserialise_failure();

    if ( nr_failures )
        printf("Done: %u failures\n", nr_failures);
    else
        printf("Done: all ok\n");

    return !!nr_failures;
}
