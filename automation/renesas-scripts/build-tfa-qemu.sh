#!/bin/bash

set -ex -o pipefail

TFA_ROOT="${PWD}/trusted-firmware-a"
TFA_BUILD="${TFA_ROOT}/build/qemu/release"
WORKDIR="${PWD}/binaries"

git clone --depth 1 --single-branch --branch safety-v2.15.0 \
    https://github.com/xen-troops/arm-trusted-firmware.git "${TFA_ROOT}"

# Keep TF-A debug disabled: its Cortex-A710 errata report reads
# CLUSTERIDR_EL1, which QEMU does not model, and BL1 traps before boot.
make -C "${TFA_ROOT}" -j"${JOBS:-$(nproc)}" \
    PLAT=qemu \
    CC=gcc \
    QEMU_USE_GIC_DRIVER=QEMU_GICV3 \
    GIC_ENABLE_V4_EXTN=1 \
    PRELOADED_BL33_BASE=0x40080000 \
    ARM_LINUX_KERNEL_AS_BL33=1 \
    qemu_fw.bios

mkdir -p "${WORKDIR}"
cp "${TFA_BUILD}/qemu_fw.bios" "${WORKDIR}"
