#ifndef __MACROS_H__
#define __MACROS_H__

#define ROUNDUP(x, a) (((x) + (a) - 1) & ~((a) - 1))

#define IS_ALIGNED(val, align) (!((val) & ((align) - 1)))

#define DIV_ROUND(n, d) (((n) + (d) / 2) / (d))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define MASK_EXTR(v, m) (((v) & (m)) / ((m) & -(m)))
#define MASK_INSR(v, m) (((v) * ((m) & -(m))) & (m))

#define count_args_(dot, a1, a2, a3, a4, a5, a6, a7, a8, x, ...) x
#define count_args(args...) \
    count_args_(., ## args, 8, 7, 6, 5, 4, 3, 2, 1, 0)

/* Indirect macros required for expanded argument pasting. */
#define PASTE_(a, b) a ## b
#define PASTE(a, b) PASTE_(a, b)

#define __STR(...) #__VA_ARGS__
#define STR(...) __STR(__VA_ARGS__)

#endif /* __MACROS_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
