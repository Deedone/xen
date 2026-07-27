#!/bin/bash
# Turn a brtrace.dat produced by a plugin-instrumented QEMU run into an LCOV
# MC/DC report. Consumes the xen-dwarf.pickle emitted by mcdc-build.sh.

set -ex -o pipefail

APP_NAME="${1:?usage: $(basename "$0") TEST-NAME}"
XEN_ROOT="${XEN_ROOT:-${PWD}}"
WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

MCDC_TOOL_DIR="${MCDC_TOOL_DIR:-${XEN_ROOT}/clang-qemu-mcdc}"
MCDC_DWARF="${MCDC_DWARF:-${WORKDIR}/xen-dwarf.pickle}"
MCDC_TRACE="${MCDC_TRACE:-${XEN_ROOT}/brtrace.dat}"

OUT_DIR="${XEN_ROOT}/coverage_data/${APP_NAME}"
mkdir -p "${OUT_DIR}"

if [ ! -s "${MCDC_TRACE}" ]; then
    echo "MC/DC trace ${MCDC_TRACE} is missing or empty" >&2
    exit 1
fi

INFO="${OUT_DIR}/${APP_NAME}.info"
python3 "${MCDC_TOOL_DIR}/mcdc_coverage_gen.py" \
    --dwarf "${MCDC_DWARF}" --lcov "${INFO}" "${MCDC_TRACE}"

cd ${XEN_ROOT}/xen

genhtml --branch-coverage --mcdc-coverage -o "${OUT_DIR}/html" "${INFO}"
cp "${MCDC_TRACE}" "${OUT_DIR}/brtrace.dat"
