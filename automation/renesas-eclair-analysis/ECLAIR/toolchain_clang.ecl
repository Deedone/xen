# Compilers.
-file_tag+={Clang_LLVM,"^/opt/atfe/bin/clang-22$"}

# Manuals.
-setq=Clang_LLVM_MANUAL,"https://releases.llvm.org/22.1.0/tools/clang/docs/UsersManual.html"
-setq=Clang_CMD_MANUAL,"https://releases.llvm.org/22.1.0/tools/clang/docs/ClangCommandLineReference.html"
-setq=Clang_LANG_EXT,"https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html"
-setq=ARM64_ABI_MANUAL,"https://github.com/ARM-software/abi-aa/blob/60a8eb8c55e999d74dac5e368fc9d7e36e38dda4/aapcs64/aapcs64.rst"
-setq=ARM64_LIBC_MANUAL,"https://www.gnu.org/software/libc/manual/pdf/libc.pdf"
-setq=C99_STD,"ISO/IEC 9899:1999"

-doc_begin="
    __alignof__, __alignof: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#implementation-defined-keywords
    asm, __asm__: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#implementation-defined-keywords
    __attribute__: Clang supports GCC’s gnu attribute namespace. See https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#non-standard-c-11-attributes
    __auto_type: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#auto-type
    __builtin_offsetof: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#builtin-offsetof
    __builtin_types_compatible_p: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#builtin-functions
    __builtin_va_arg: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#variadic-function-builtins
    __const__, __inline__, __inline: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#implementation-defined-keywords
    _Static_assert: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#c-keywords-supported-in-all-language-modes
    typeof, __typeof__: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#typeof-typeof-typeof-unqual-typeof-unqual
    __volatile__: see https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#const-const-volatile-volatile-restrict-restrict
"
-name_selector+={alignof, "^(__alignof__|__alignof)$"}
-name_selector+={asm, "^(__asm__|asm)$"}
-name_selector+={attribute, "^__attribute__$"}
-name_selector+={auto_type, "^__auto_type$"}
-name_selector+={builtin_offsetof, "^__builtin_offsetof$"}
-name_selector+={builtin_types_p, "^__builtin_types_compatible_p$"}
-name_selector+={builtin_va_arg, "^__builtin_va_arg$"}
-name_selector+={const, "^__const__$"}
-name_selector+={inline, "^(__inline__|__inline)$"}
-name_selector+={static_assert, "^_Static_assert$"}
-name_selector+={typeof, "^(__typeof__|typeof)$"}
-name_selector+={volatile, "^__volatile__$"}

-config=STD.tokenext,behavior+={c99,Clang_LLVM,
"alignof||
asm||
attribute||
auto_type||
builtin_offsetof||
builtin_types_p||
builtin_va_arg||
const||
inline||
static_assert||
typeof||
volatile"
}
-doc_end

-doc_begin="Clang aims to support a broad range of GCC extensions.
    See https://releases.llvm.org/22.1.0/tools/clang/docs/LanguageExtensions.html#introduction"
-config=STD.vptrarth,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.stmtexpr,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.anonstct,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.arayzero,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.bincondl,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.caseuplw,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.anonfild,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.adrslabl,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Non-documented extension (supported for GCC compatibility)."
-config=STD.emptinit,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.emptdecl,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.emptenum,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.pteincmp,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.funojptr,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.ltrlbin,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="
    ext_auto_type: Extension supported by Clang.
    ext_c_missing_varargs_arg: Extension supported by Clang.
    ext_enum_value_not_int: Extension supported by Clang.
    ext_flexible_array_in_array: Extension supported by Clang.
    ext_flexible_array_in_struct: Extension supported by Clang.
    ext_forward_ref_enum_def: Extension supported by Clang.
    ext_gnu_array_range: Extension supported by Clang.
    ext_gnu_statement_expr_macro: Extension supported by Clang.
    ext_named_variadic_macro: Extension supported by Clang.
    ext_paste_comma: Extension supported by Clang.
    ext_return_has_void_expr: Extension supported by Clang.
    ext_sizeof_alignof_void_type: Extension supported by Clang.
"
-name_selector+={ext_auto_type, "^ext_auto_type$"}
-name_selector+={ext_c_missing_varargs_arg, "^ext_c_missing_varargs_arg$"}
-name_selector+={ext_enum_value_not_int, "^ext_enum_value_not_int$"}
-name_selector+={ext_flexible_array_in_array, "^ext_flexible_array_in_array$"}
-name_selector+={ext_flexible_array_in_struct, "^ext_flexible_array_in_struct$"}
-name_selector+={ext_forward_ref_enum_def, "^ext_forward_ref_enum_def$"}
-name_selector+={ext_gnu_array_range, "^ext_gnu_array_range$"}
-name_selector+={ext_gnu_statement_expr_macro, "^ext_gnu_statement_expr_macro$"}
-name_selector+={ext_named_variadic_macro, "^ext_named_variadic_macro$"}
-name_selector+={ext_paste_comma, "^ext_paste_comma$"}
-name_selector+={ext_return_has_void_expr, "^ext_return_has_void_expr$"}
-name_selector+={ext_sizeof_alignof_void_type, "^ext_sizeof_alignof_void_type$"}

-config=STD.diag,behavior+={c99,Clang_LLVM,
"ext_auto_type||
ext_c_missing_varargs_arg||
ext_forward_ref_enum_def||
ext_gnu_array_range||
ext_gnu_statement_expr_macro||
ext_named_variadic_macro||
ext_paste_comma||
ext_return_has_void_expr||
ext_sizeof_alignof_void_type"
}
-doc_end

-doc_begin="No implementation limit for object size is documented in Clang."
-config=STD.byteobjt,behavior+={c99,Clang_LLVM,"unlimited"}
-doc_end

-doc_begin="No implementation limit for the number of characters in a logical source line is documented in Clang."
-config=STD.charline,behavior+={c99,Clang_LLVM,"unlimited"}
-doc_end

-doc_begin="No implementation limit for nesting levels of #include directives is documented in Clang."
-config=STD.inclnest,behavior+={c99,Clang_LLVM,"unlimited"}
-doc_end

-doc_begin="No implementation limit for simultaneously defined macro identifiers is documented in Clang."
-config=STD.macident,behavior+={c99,Clang_LLVM,"unlimited"}
-doc_end

-doc_begin="See Section \"8.1.8 Bit-fields\" of ARM64_ABI_MANUAL.
    Clang follows the AArch64 ABI bit-field layout rules and supports GCC-compatible bit-field types."
-config=STD.bitfldtp,behavior+={c99,Clang_LLVM,"unsigned char;unsigned short;unsigned long;unsigned long long"}
-doc_end

-doc_begin="Clang supports #pragma pack for GCC/MSVC compatibility.
    Clang also supports #pragma GCC visibility push/pop."
-config=STD.nonstdc,behavior+={c99,Clang_LLVM,"^(pack\\(|GCC visibility (push|pop)).*$"}
-doc_end

-doc_begin="Clang supports UTF-8 source encoding."
-config=STD.charset,behavior+={c99,Clang_LLVM,"utf8"}
-doc_end

-doc_begin="No implementation-defined limit on significant characters of external identifiers is documented by Clang."
-config=STD.extidsig,behavior+={c99,Clang_LLVM, "0"}
-doc_end

-doc_begin="Clang supports UTF-8 execution and source character sets."
-config=STD.bytebits,behavior+={c99,Clang_LLVM,"8"}
-config=STD.charsobj,behavior+={c99,Clang_LLVM,"utf8"}
-config=STD.charsval,behavior+={c99,Clang_LLVM,"utf8"}
-config=STD.charsmap,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.charsmem,behavior+={c99,Clang_LLVM,"utf8"}
-config=STD.execvals,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Clang provides diagnostic identifiers in the form of warning flags (e.g. -Wabc)."
-config=STD.diagidnt,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Given that Xen is compiled in hosted mode, ECLAIR cannot exclude the independency from program termination implementation-defined behavior. See \"Section 25.7 Program Termination\" of "ARM64_LIBC_MANUAL"."
-config=STD.exitstat,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Clang implements include file search according to its driver model."
-config=STD.inclangl,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.inclfile,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.inclhead,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Clang documents implementation-defined behavior through target ABI and LLVM behavior."
-config=STD.signdint,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.stringfy,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.volatltp,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.widestng,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="See Chapter \"5   Data types and alignment\" of "ARM64_ABI_MANUAL"."
-config=STD.objbytes,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Bit-field layout and ordering are defined by the AArch64 ABI.
    Clang follows the ABI bit-field allocation and does not define independent rules for placement or ordering."
-config=STD.bitfldby,behavior+={c99,Clang_LLVM,"specified"}
-config=STD.bitfldor,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Values of macros in <limits.h>, <stdint.h>, and <float.h> are defined by the C standard library and the AArch64 ABI."
-config=STD.stdmacro,behavior+={c99,Clang_LLVM,"specified"}
-doc_end

-doc_begin="Clang supports GCC-compatible pragma directives."
-config=STD.pragmdir,behavior+={c99,Clang_LLVM,"^(pack\\(|GCC visibility (push|pop)).*$"}
-doc_end
