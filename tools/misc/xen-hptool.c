#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <xenevtchn.h>
#include <xenctrl.h>
#include <xenguest.h>
#include <xenstore.h>
#include <xen-tools/common-macros.h>
#include "xen-hptool.h"


void show_help(void)
{
    fprintf(stderr,
            "xen-hptool: Xen CPU/memory hotplug tool\n"
            "Usage: xen-hptool <command> [args]\n"
            "Commands:\n"
            "  help                     display this help\n"
            "  cpu-online    <cpuid>    online CPU <cpuid>\n"
            "  cpu-offline   <cpuid>    offline CPU <cpuid>\n"
#if defined(__i386__) || defined(__x86_64__)
            "  mem-online    <mfn>      online MEMORY <mfn>\n"
            "  mem-offline   <mfn>      offline MEMORY <mfn>\n"
            "  mem-status    <mfn>      query Memory status<mfn>\n"
            "  smt-enable               onlines all SMT threads\n"
            "  smt-disable              offlines all SMT threads\n"
#endif
           );
}

/* wrapper function */
static int help_func(int argc, char *argv[], xc_interface *xch)
{
    show_help();
    return 0;
}

static int exec_cpu_hp_fn(int (*hp_fn)(xc_interface *, int), int cpu,
                          xc_interface *xch)
{
    int ret;

    for ( ; ; )
    {
        ret = (*hp_fn)(xch, cpu);
        if ( (ret >= 0) || (errno != EBUSY) )
            break;
        usleep(100000); /* 100ms */
    }

    return ret;
}

static int hp_cpu_online_func(int argc, char *argv[], xc_interface *xch)
{
    int cpu, ret;

    if ( argc != 1 )
    {
        show_help();
        return -1;
    }

    cpu = atoi(argv[0]);
    printf("Prepare to online CPU %d\n", cpu);
    ret = exec_cpu_hp_fn(xc_cpu_online, cpu, xch);
    if (ret < 0)
        fprintf(stderr, "CPU %d online failed (error %d: %s)\n",
                cpu, errno, strerror(errno));
    else
        printf("CPU %d onlined successfully\n", cpu);

    return ret;

}
static int hp_cpu_offline_func(int argc, char *argv[], xc_interface *xch)
{
    int cpu, ret;

    if (argc != 1 )
    {
        show_help();
        return -1;
    }
    cpu = atoi(argv[0]);
    printf("Prepare to offline CPU %d\n", cpu);
    ret = exec_cpu_hp_fn(xc_cpu_offline, cpu, xch);
    if (ret < 0)
        fprintf(stderr, "CPU %d offline failed (error %d: %s)\n",
                cpu, errno, strerror(errno));
    else
        printf("CPU %d offlined successfully\n", cpu);

    return ret;
}

struct {
    const char *name;
    int (*function)(int argc, char *argv[], xc_interface *xch);
} main_options[] = {
    { "help", help_func },
    { "cpu-online", hp_cpu_online_func },
    { "cpu-offline", hp_cpu_offline_func },
#if defined(__i386__) || defined(__x86_64__)
    { "mem-status", hp_mem_query_func},
    { "mem-online", hp_mem_online_func},
    { "mem-offline", hp_mem_offline_func},
    { "smt-enable", main_smt_enable },
    { "smt-disable", main_smt_disable },
#endif
};


int main(int argc, char *argv[])
{
    int i, ret;
    xc_interface *xch;

    if (argc < 2)
    {
        show_help();
        return 0;
    }

    xch = xc_interface_open(0,0,0);
    if ( !xch )
    {
        fprintf(stderr, "failed to get the handler\n");
        return 0;
    }

    for ( i = 0; i < ARRAY_SIZE(main_options); i++ )
        if (!strncmp(main_options[i].name, argv[1], strlen(argv[1])))
            break;
    if ( i == ARRAY_SIZE(main_options) )
    {
        fprintf(stderr, "Unrecognised command '%s' -- try "
                "'xen-hptool help'\n", argv[1]);
        return 1;
    }

    ret = main_options[i].function(argc -2, argv + 2, xch);

    xc_interface_close(xch);

    return !!ret;
}
