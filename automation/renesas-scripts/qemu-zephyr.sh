#!/bin/bash

set -ex -o pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $(basename $0) TEST-NAME"
    exit 0
fi

export APP_NAME="$1"
export XEN_ROOT="${PWD}"
export WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"
export QEMU_PREFIX="${QEMU_PREFIX:-/usr/local/bin/}"
export ZTESTS_ROOT=${XEN_ROOT}/zephyr_tests

export QEMU_LOG="${QEMU_LOG:-${XEN_ROOT}/qemu.serial}"

export PASSED="${PASSED:-Test result: SUCCESS}"

# Set artifacts path (replacing prebuilt images)
export PREBUILT_IMAGES=${WORKDIR}

rm -f ${QEMU_LOG}

git clone --depth 1 https://gitlab-ci-token:${CI_JOB_TOKEN}@gitpct.epam.com/rec-fusa/zephyr_tests.git -b safety-staging

cd ${ZEPHYR_SDK_INSTALL_DIR}

# DomU builds: use xenvm board for Xen virtual machines
west build -p always -b xenvm ${ZTESTS_ROOT}/testcases/domu-basic

cp build/zephyr/zephyr.bin ${WORKDIR}/domu-basic.bin
cp build/zephyr/zephyr.elf ${WORKDIR}/domu-basic.elf
# Copy DTB if it exists (for domu-basic)
if [ -f "build/domu-basic.dtb" ]; then
    cp build/domu-basic.dtb ${WORKDIR}/domu-basic.dtb
fi

# Dom0 builds: use qemu_cortex_a53 with xen_dom0 snippet for privileged domain features
# and use application level DTS overlay xen_dom0_overlay snippet, which adds "hypervisor" node
west build -p always -b qemu_cortex_a53 -S xen_dom0 -S xen_dom0_overlay ${ZTESTS_ROOT}/testcases/${APP_NAME}

cp build/zephyr/zephyr.bin ${WORKDIR}/${APP_NAME}.bin
cp build/zephyr/zephyr.elf ${WORKDIR}/${APP_NAME}.elf

# Recompile xen.dtb from xen.dts to ensure it's up-to-date
dtc -I dts -O dtb ${ZTESTS_ROOT}/device-tree/xen.dts -o ${WORKDIR}/xen.dtb

REG_ADDR=0x41000000
REG_SIZE=$(printf "0x%x" "$(stat -c '%s' "${WORKDIR}/${APP_NAME}.bin")")

fdtput -t x ${WORKDIR}/xen.dtb /chosen/module@41000000 reg ${REG_ADDR} ${REG_SIZE}
fdtget -t x ${WORKDIR}/xen.dtb /chosen/module@41000000 reg

# Run QEMU
${QEMU_PREFIX}qemu-system-aarch64 \
    -cpu cortex-a57 \
    -machine virt,virtualization=true,gic-version=3,iommu=smmuv3 \
    -m 2048 \
    -smp 2 \
    -no-reboot \
    -nodefaults \
    -display none \
    -monitor none \
    -serial stdio \
    -device loader,file=${WORKDIR}/${APP_NAME}.bin,addr=${REG_ADDR} \
    -kernel ${WORKDIR}/xen -dtb ${WORKDIR}/xen.dtb > ${QEMU_LOG} 2>&1

#Print the captured logs to the job output
cat ${QEMU_LOG} || true

# Test validation
grep -qF "${PASSED}" "${QEMU_LOG}" && { echo -e "\e[32m***FOUND EXPECTED TEST STRING***\e[0m"; exit 0; }
echo -e "\e[31m***NOT FOUND EXPECTED TEST STRING***\e[0m"
exit 1
