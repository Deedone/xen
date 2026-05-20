-project_name=getenv("ECLAIR_PROJECT_NAME")
-project_root=getenv("ECLAIR_PROJECT_ROOT")

setq(data_dir,getenv("ECLAIR_DATA_DIR"))
setq(analysis_kind,getenv("ANALYSIS_KIND"))
setq(analysis_set,getenv("SET"))

-enable=B.REPORT.ECB
-config=B.REPORT.ECB,output=join_paths(data_dir,"FRAME.@FRAME@.ecb")
-config=B.REPORT.ECB,preprocessed=show
-config=B.REPORT.ECB,macros=10

-enable=B.EXPLAIN

-doc_begin="Do not analyze intermediate linking artifacts, as they do not differ from their final
counterparts for the purposes of MISRA C static analysis."
-file_tag+={xen_efi_tmp, "^xen/\\.xen\\.efi\\..*$"}
-file_tag+={xen_syms_tmp, "^xen/\\.xen-syms\\..*$"}
-frames+={hide, "kind(program)&&target(xen_syms_tmp||xen_efi_tmp)"}
-doc_end

-eval_file=getenv("TOOLCHAIN_FILE")
-eval_file=public_APIs.ecl

-doc="Initially, there are no files tagged as adopted."
-file_tag+={adopted,"none()"}

-eval_file=adopted.ecl
-eval_file=out_of_scope.ecl

-eval_file=deviations.ecl
-eval_file=call_properties.ecl
-eval_file=tagging.ecl
-eval_file=concat(analysis_set,".ecl")

-doc="Hide reports in external code."
-reports+={hide,all_exp_external}
