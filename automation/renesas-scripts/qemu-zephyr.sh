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

export PASSED="${PASSED:-TESTSUITE .* succeeded}"

# Xen image to boot. MC/DC runs point this at the instrumented build.
export XEN_BIN="${XEN_BIN:-${WORKDIR}/xen}"

# When MCDC_CONF is set, attach the brtrace QEMU plugin and generate an MC/DC
# report from the collected trace after the run.
QEMU_PLUGIN_ARGS=""
if [ -n "${MCDC_CONF:-}" ]; then
    export MCDC_PLUGIN="${MCDC_PLUGIN:-/usr/local/lib/qemu-plugins/libbrtrace.so}"
    export MCDC_TRACE="${MCDC_TRACE:-${XEN_ROOT}/brtrace.dat}"
    QEMU_PLUGIN_ARGS="-plugin ${MCDC_PLUGIN},config=${MCDC_CONF},tracefile=${MCDC_TRACE}"
    rm -f "${MCDC_TRACE}"
fi

# Set artifacts path (replacing prebuilt images)
export PREBUILT_IMAGES=${WORKDIR}

rm -f ${QEMU_LOG}

git clone --depth 1 https://gitlab-ci-token:${CI_JOB_TOKEN}@gitpct.epam.com/rec-fusa/zephyr_tests.git -b "${ZEPHYR_BRANCH:-safety-staging}"

cd ${ZEPHYR_SDK_INSTALL_DIR}

do_zephyr_fetch()
{
    pushd $1
    git fetch --depth 1 origin $2
    git checkout $2
    popd
}

do_zephyr_fetch zephyr zephyr-v4.4.0-xt

do_zephyr_fetch zephyr-xenlib main

# Auto-detect and build DomU dependencies
DOMAIN_BINS_S="${ZTESTS_ROOT}/testcases/${APP_NAME}/src/domain_bins.S"
if [ -f "${DOMAIN_BINS_S}" ]; then
    EXTRA_DOMUS=$(grep '\.incbin.*"domu-[^"]*\.bin"' "${DOMAIN_BINS_S}" | \
        sed -n 's/.*\.incbin[[:space:]]*"\(domu-[^"]*\)\.bin".*/\1/p' | \
        sort -u || true)

    for domu in ${EXTRA_DOMUS}; do
        west build -p always -b xenvm/xenvm/gicv3 "${ZTESTS_ROOT}/testcases/${domu}"
        cp build/zephyr/zephyr.bin "${WORKDIR}/${domu}.bin"
        if [ -f "build/${domu}.dtb" ]; then
            cp "build/${domu}.dtb" "${WORKDIR}/${domu}.dtb"
        fi
    done
fi

# Dom0 builds: use qemu_cortex_a53 with xen_dom0 snippet for privileged domain features
# and use application level DTS overlay xen_dom0_overlay snippet, which adds "hypervisor" node
west build -p always -b qemu_cortex_a53 -S xen_dom0 -S xen_dom0_overlay ${ZTESTS_ROOT}/testcases/${APP_NAME}
cp build/zephyr/zephyr.bin ${WORKDIR}/${APP_NAME}.bin

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
    ${QEMU_PLUGIN_ARGS} \
    -kernel ${XEN_BIN} -dtb ${WORKDIR}/xen.dtb > ${QEMU_LOG} 2>&1

#Print the captured logs to the job output
cat ${QEMU_LOG} || true

# Generate MCDC report only if test was passed
do_mcdc_report() {
    # MC/DC report generation from the trace collected during this run.
    if [ -n "${MCDC_CONF:-}" ]; then
        ( cd "${XEN_ROOT}" &&
        ./automation/renesas-scripts/mcdc-report.sh "${APP_NAME}" ) || true
    fi
}

# Test validation
grep -qE "${PASSED}" "${QEMU_LOG}" && { do_mcdc_report; echo -e "\e[32m***FOUND EXPECTED TEST STRING***\e[0m"; exit 0; }
echo -e "\e[31m***NOT FOUND EXPECTED TEST STRING***\e[0m"
exit 1
