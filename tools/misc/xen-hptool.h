#ifndef __XEN_HPTOOL_H__
#define __XEN_HPTOOL_H__

#if defined(__i386__) || defined(__x86_64__)
int hp_mem_online_func(int argc, char *argv[], xc_interface *xch);
int hp_mem_query_func(int argc, char *argv[], xc_interface *xch);
int hp_mem_offline_func(int argc, char *argv[], xc_interface *xch);
int main_smt_enable(int argc, char *argv[], xc_interface *xch);
int main_smt_disable(int argc, char *argv[], xc_interface *xch);
#endif

void show_help(void);

#endif /* __XEN_HPTOOL_H__ */
