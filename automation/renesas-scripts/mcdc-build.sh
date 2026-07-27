#!/bin/bash
# Build an MC/DC table Xen and pre-compute the branch tracing config
# consumed by the QEMU brtrace plugin.

set -ex -o pipefail

XEN_ROOT="${PWD}"
WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

MCDC_TOOL_URI="${MCDC_TOOL_URI:-https://github.com/xen-troops/clang-qemu-mcdc.git}"
MCDC_TOOL_BRANCH="${MCDC_TOOL_BRANCH:-master}"
MCDC_TOOL_DIR="${MCDC_TOOL_DIR:-${XEN_ROOT}/clang-qemu-mcdc}"

mkdir -p "${WORKDIR}"

if [ ! -d "${MCDC_TOOL_DIR}" ]; then
    git clone --depth 1 -b "${MCDC_TOOL_BRANCH}" "${MCDC_TOOL_URI}" "${MCDC_TOOL_DIR}"
fi

cd "${XEN_ROOT}/xen"

make XEN_TARGET_ARCH=arm64 rel_arm64_defconfig
cat >> .config <<EOF
CONFIG_EXPERT=y
CONFIG_NO_OPTIMIZE=y
CONFIG_FRAME_POINTER=y
CONFIG_DEBUG_INFO=y
EOF
make XEN_TARGET_ARCH=arm64 olddefconfig
cp .config "${XEN_ROOT}/xen-mcdc-config"

# Build while capturing compile_commands.json for the source parser.
rm -f compile_commands.json
bear --append -- \
    make XEN_TARGET_ARCH=arm64 clang=y llvm=y HOSTCC=gcc -j"$(nproc)"

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
