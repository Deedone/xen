#!/bin/bash
# Turn brtrace.dat files produced by a plugin-instrumented QEMU run into an
# LCOV MC/DC report. Consumes the xen-dwarf.pickle emitted by mcdc-build.sh.

set -ex -o pipefail

REPORT_NAME="combined_report"

while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--name)
            REPORT_NAME="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $(basename "$0") [-n|--name REPORT_NAME] trace1.dat [trace2.dat ...]"
            exit 0
            ;;
        *)
            break
            ;;
    esac
done

XEN_ROOT="${XEN_ROOT:-${PWD}}"
WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

MCDC_TOOL_DIR="${MCDC_TOOL_DIR:-${XEN_ROOT}/clang-qemu-mcdc}"
MCDC_DWARF="${MCDC_DWARF:-${WORKDIR}/xen-dwarf.pickle}"

if [ $# -eq 0 ]; then
    TRACE_FILES=("${MCDC_TRACE:-${XEN_ROOT:-${PWD}}/brtrace.dat}")
else
    TRACE_FILES=("$@")
fi

OUT_DIR="${XEN_ROOT}/coverage_data/${REPORT_NAME}"
mkdir -p "${OUT_DIR}"

for trace_file in "${TRACE_FILES[@]}"; do
    if [ ! -s "${trace_file}" ]; then
        echo "MC/DC trace ${trace_file} is missing or empty" >&2
        exit 1
    fi
done

INFO="${OUT_DIR}/${REPORT_NAME}.info"
python3 "${MCDC_TOOL_DIR}/mcdc_coverage_gen.py" \
    --dwarf "${MCDC_DWARF}" --lcov "${INFO}" "${TRACE_FILES[@]}"

for trace_file in "${TRACE_FILES[@]}"; do
    DEST_NAME=$(basename "${trace_file}")
    cp "${trace_file}" "${OUT_DIR}/${DEST_NAME}"
done

cd ${XEN_ROOT}/xen

genhtml --branch-coverage --mcdc-coverage -o "${OUT_DIR}/html" "${INFO}"
