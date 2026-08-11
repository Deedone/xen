#!/bin/bash
# Build an MC/DC table Xen and pre-compute the branch tracing config
# consumed by the QEMU brtrace plugin.

set -ex -o pipefail

XEN_ROOT="${PWD}"
WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

MCDC_TOOL_URI="${MCDC_TOOL_URI:-https://github.com/xen-troops/clang-qemu-mcdc.git}"
MCDC_TOOL_BRANCH="${MCDC_TOOL_BRANCH:-master}"
MCDC_TOOL_DIR="${MCDC_TOOL_DIR:-${XEN_ROOT}/clang-qemu-mcdc}"

if [ ! -d "${MCDC_TOOL_DIR}" ]; then
    git clone --depth 1 -b "${MCDC_TOOL_BRANCH}" "${MCDC_TOOL_URI}" "${MCDC_TOOL_DIR}"
fi

mkdir -p binaries

cd "${XEN_ROOT}/xen"

make defconfig

debug="${debug:-n}"
scripts/config --file .config -${debug} DEBUG

if [[ -n "${EXTRA_XEN_CONFIG}" ]]; then
    echo "${EXTRA_XEN_CONFIG}" >> .config
fi

make olddefconfig
cp .config "${XEN_ROOT}/xen-config"

# Build while capturing compile_commands.json for the source parser.
rm -f compile_commands.json
bear --append -- make -j"$(nproc)"

# Source-level pass
python3 "${MCDC_TOOL_DIR}/mcdc_tool_parser.py" \
    xen.pickle compile_commands.json

# Binary-level pass
python3 "${MCDC_TOOL_DIR}/mcdc_tool_dwarf.py" \
    xen-syms xen.pickle xen-dwarf.pickle plugin.conf

cd "${XEN_ROOT}"
cp xen/xen           "${WORKDIR}/xen"
cp xen/xen-syms      "${WORKDIR}/xen-syms"
cp xen/xen-dwarf.pickle "${WORKDIR}/xen-dwarf.pickle"
cp xen/plugin.conf   "${WORKDIR}/plugin.conf"
