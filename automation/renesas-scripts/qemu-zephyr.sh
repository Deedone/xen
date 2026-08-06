#!/bin/bash

set -ex -o pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $(basename $0) TEST-NAME"
    exit 0
fi

export APP_NAME="${1//\//-}"
export XEN_ROOT="${PWD}"
export WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"
export QEMU_PREFIX="${QEMU_PREFIX:-/usr/local/bin/}"
export ZTESTS_ROOT=${XEN_ROOT}/zephyr_tests
export TFA_BIN="${TFA_BIN:-${WORKDIR}/qemu_fw.bios}"

# TF-A/QEMU Zephyr test memory layout:
#   0x40000000              device tree
#   0x40080000              Xen (preloaded BL33)
#   0x41000000              Zephyr control-domain boot module
#   0x42000000 and above    DomU boot modules, 16 MiB apart
#   0x58000000-0x60000000   static control-domain RAM

export QEMU_LOG="${QEMU_LOG:-${XEN_ROOT}/qemu.serial}"

export PASSED="${PASSED:-TESTSUITE .* succeeded}"

# Xen image to boot. MC/DC runs point this at the instrumented build.
export XEN_BIN="${XEN_BIN:-${WORKDIR}/xen}"

# When MCDC_CONF is set, attach the brtrace QEMU plugin and generate an MC/DC
# report from the collected trace after the run.
QEMU_PLUGIN_ARGS=""
if [ -n "${MCDC_CONF:-}" ]; then
    export MCDC_PLUGIN="${MCDC_PLUGIN:-/usr/local/lib/qemu-plugins/libbrtrace.so}"
    export MCDC_TRACE="${MCDC_TRACE:-${XEN_ROOT}/${APP_NAME}-brtrace.dat}"
    QEMU_PLUGIN_ARGS="-plugin ${MCDC_PLUGIN},config=${MCDC_CONF},tracefile=${MCDC_TRACE}"
    rm -f "${MCDC_TRACE}"
fi

if [ "$RUN_COVERAGE" == "true" ]; then
    echo "Running QEMU with coverage plugin."

    export LLDB_COV_LOG="${LLDB_COV_LOG:-${XEN_ROOT}/lldb_coverage.log}"
    export QEMU_COV_TRACE="${QEMU_COV_TRACE:-${XEN_ROOT}/trace.drcov}"
    export COVERAGE_OUT="${COVERAGE_OUT:-${XEN_ROOT}/coverage_data}"

    START_HEX=$(readelf -s "${WORKDIR}/xen-syms" | grep ' _stext$' | awk '{print $2}')
    END_HEX=$(readelf -s "${WORKDIR}/xen-syms" | awk '$NF == "_einittext" {print $2}')

    START_CODE="0x${START_HEX}"
    END_CODE="0x${END_HEX}"

    QEMU_PLUGIN_ARGS+=" -plugin /usr/local/lib/qemu-plugins/libdrcov.so"
    QEMU_PLUGIN_ARGS+=",filename=${QEMU_COV_TRACE}"
    QEMU_PLUGIN_ARGS+=",start_code=${START_CODE}"
    QEMU_PLUGIN_ARGS+=",end_code=${END_CODE}"
    QEMU_PLUGIN_ARGS+=",bin_path=${WORKDIR}/xen-syms "

    rm -f ${QEMU_COV_TRACE}

    mkdir -p ${COVERAGE_OUT}
fi

export QEMU_TRACE="${QEMU_TRACE:-${XEN_ROOT}/qemu.trace}"
rm -f ${QEMU_TRACE}

export TEST_DIR="${ZTESTS_ROOT}/testcases/${1}"

# Number of pCPUs exposed to QEMU. A test needing more can raise it in test.env.
export SMP="${SMP:-2}"

# Boot directly unless a test opts into TF-A in test.env.
export USE_TFA="${USE_TFA:-false}"

# Set artifacts path (replacing prebuilt images)
export PREBUILT_IMAGES=${WORKDIR}

rm -f ${QEMU_LOG}

git clone --depth 1 https://gitlab-ci-token:${CI_JOB_TOKEN}@gitpct.epam.com/rec-fusa/zephyr_tests.git -b "${ZEPHYR_BRANCH:-safety-staging}"

# Per-test overrides (SMP, TF-A, Dom0/DomU build source and arguments, ...)
DOM0_BUILD_SOURCE="${TEST_DIR}"
DOM0_BUILD_ARGS=()
DOMU_BUILD_SOURCE=""
DOMU_BUILD_ARGS=()
if [ -f "${TEST_DIR}/test.env" ]; then
    source "${TEST_DIR}/test.env"
fi

cd ${ZEPHYR_SDK_INSTALL_DIR}

do_zephyr_fetch()
{
    pushd $1
    git fetch --depth 1 origin $2
    git checkout $2
    popd
}

do_zephyr_fetch zephyr zephyr-v4.4.0-xt

do_zephyr_fetch zephyr-xenlib safety-staging

# Auto-detect and build DomU dependencies
DOMAIN_BINS_S="${DOM0_BUILD_SOURCE}/src/domain_bins.S"
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
west build -p always -b qemu_cortex_a53 -S xen_dom0 -S xen_dom0_overlay \
    "${DOM0_BUILD_SOURCE}" "${DOM0_BUILD_ARGS[@]}"
cp build/zephyr/zephyr.bin ${WORKDIR}/${APP_NAME}.bin

# Recompile xen.dtb from xen.dts to ensure it's up-to-date. A test may ship an
# xen.overlay (extra pCPUs, boot-time cpupools, ...) applied on top of it.
dtc -@ -I dts -O dtb ${ZTESTS_ROOT}/device-tree/xen.dts -o ${WORKDIR}/xen-base.dtb
dtc -@ -I dts -O dtb ${ZTESTS_ROOT}/device-tree/control-domain.overlay \
    -o ${WORKDIR}/control-domain.dtbo
if [ -f "${TEST_DIR}/xen.overlay" ]; then
    dtc -@ -I dts -O dtb ${TEST_DIR}/xen.overlay -o ${WORKDIR}/xen.dtbo
    fdtoverlay -i ${WORKDIR}/xen-base.dtb -o ${WORKDIR}/xen.dtb \
        ${WORKDIR}/xen.dtbo ${WORKDIR}/control-domain.dtbo
else
    fdtoverlay -i ${WORKDIR}/xen-base.dtb -o ${WORKDIR}/xen.dtb \
        ${WORKDIR}/control-domain.dtbo
fi

QEMU_BOOT_ARGS=(-kernel "${XEN_BIN}")

if [ "${USE_TFA}" = "true" ]; then
    QEMU_BOOT_ARGS=(
        -device "loader,file=${XEN_BIN},addr=0x40080000,force-raw=on"
        -bios "${TFA_BIN}"
    )
fi

REG_ADDR=0x41000000
REG_SIZE=$(printf "0x%x" "$(stat -c '%s' "${WORKDIR}/${APP_NAME}.bin")")

CONTROL_NODE=/chosen/control-domain
fdtput -t i ${WORKDIR}/xen.dtb ${CONTROL_NODE} cpus "${CONTROL_VCPUS:-1}"
fdtput -t x ${WORKDIR}/xen.dtb ${CONTROL_NODE}/module@41000000 reg \
    ${REG_ADDR} ${REG_SIZE}
fdtget -t x ${WORKDIR}/xen.dtb ${CONTROL_NODE}/module@41000000 reg

# dom0less domains: DOM0LESS_DOMUS lists one kernel per /chosen/domU<n> node
# of the test's xen.dts, in ascending <n> order. Each kernel is loaded at
# DOM0LESS_ADDR + <n> * DOM0LESS_STEP and its module node is pointed at it.
DOM0LESS_ADDR=$((0x42000000))
DOM0LESS_STEP=$((0x01000000))
DOM0LESS_LOADERS=()
BUILT_DOMUS=""
IDX=1

for domu in ${DOM0LESS_DOMUS}; do
    DOMU_SOURCE="${DOMU_BUILD_SOURCE:-${ZTESTS_ROOT}/testcases/${domu}}"
    if [[ " ${BUILT_DOMUS} " != *" ${domu} "* ]]; then
        west build -p always -b xenvm/xenvm/gicv3 \
            "${DOMU_SOURCE}" "${DOMU_BUILD_ARGS[@]}"
        cp build/zephyr/zephyr.bin "${WORKDIR}/${domu}.bin"
        BUILT_DOMUS="${BUILT_DOMUS} ${domu}"
    fi

    ADDR=$(printf "0x%x" $((DOM0LESS_ADDR + (IDX - 1) * DOM0LESS_STEP)))
    SIZE=$(printf "0x%x" "$(stat -Lc '%s' "${WORKDIR}/${domu}.bin")")

    fdtput -t x ${WORKDIR}/xen.dtb /chosen/domU${IDX}/module@${ADDR#0x} reg \
        ${ADDR} ${SIZE}
    DOM0LESS_LOADERS+=(-device "loader,file=${WORKDIR}/${domu}.bin,addr=${ADDR}")
    IDX=$((IDX + 1))
done

# A testcase may ship a qemu-extra-args file (extra QEMU arguments, one per
# line) and a check-trace.py (host-side validation of what QEMU recorded), for
# properties that cannot be observed from inside the guest.

QEMU_EXTRA_ARGS=()
if [ -f "${TEST_DIR}/qemu-extra-args" ]; then
    while IFS= read -r line; do
        case "${line}" in ''|\#*) continue ;; esac
        eval "QEMU_EXTRA_ARGS+=(${line})"
    done < "${TEST_DIR}/qemu-extra-args"
fi

if [ -n "${LLDB_SCRIPT:-}" ]; then
    # LLDB needs QEMU's GDB stub listening before any vCPU starts.
    QEMU_EXTRA_ARGS+=(-s -S)
fi

run_qemu()
{
    if [ -z "${LLDB_SCRIPT:-}" ]; then
        "$@"
        return
    fi

    if [ -z "${LLDB_PASSED:-}" ]; then
        echo "LLDB_PASSED must be set when LLDB_SCRIPT is used"
        return 1
    fi

    (
        local lldb_log="${LLDB_LOG:-${XEN_ROOT}/lldb.serial}"
        local lldb_rc=0 qemu_pid qemu_rc=0

        "$@" </dev/null &
        qemu_pid=$!
        trap 'kill "${qemu_pid}" 2>/dev/null || true;
              wait "${qemu_pid}" 2>/dev/null || true' EXIT
        sleep 1

        PYTHONPATH="${XEN_ROOT}/automation/renesas-scripts/lldb/:${PYTHONPATH:-}" \
            XEN_ELF="${WORKDIR}/xen-syms" XEN_PORT="1234" \
            lldb --batch -o "command script import ${LLDB_SCRIPT}" \
            > "${lldb_log}" 2>&1 || lldb_rc=$?
        cat "${lldb_log}" || true

        if [ "${lldb_rc}" -ne 0 ] || \
           ! grep -qF -- "${LLDB_PASSED}" "${lldb_log}"; then
            [ "${lldb_rc}" -ne 0 ] || lldb_rc=1
            exit "${lldb_rc}"
        fi

        wait "${qemu_pid}" || qemu_rc=$?
        trap - EXIT
        exit "${qemu_rc}"
    )
}

# Run QEMU
run_qemu "${QEMU_PREFIX}qemu-system-aarch64" \
    -cpu cortex-a710 \
    -machine virt,secure=${USE_TFA},virtualization=true,gic-version=4,iommu=smmuv3 \
    -m 2048 \
    -smp ${SMP} \
    -no-reboot \
    -nodefaults \
    -display none \
    -monitor none \
    -serial stdio \
    -netdev user,id=net1,hostfwd=tcp::2223-:22 -device igb,netdev=net1 \
    -netdev user,id=net2,hostfwd=tcp::2224-:23 -device e1000e,netdev=net2,romfile= \
    -device edu \
    -device loader,file=${WORKDIR}/${APP_NAME}.bin,addr=${REG_ADDR} \
    "${QEMU_BOOT_ARGS[@]}" \
    ${QEMU_PLUGIN_ARGS} \
    "${DOM0LESS_LOADERS[@]}" \
    "${QEMU_EXTRA_ARGS[@]}" \
    -dtb ${WORKDIR}/xen.dtb 2>&1 | tee "${QEMU_LOG}"

# Generate coverage report only if test was passed
do_coverage_report() {
    # MC/DC report generation from the trace collected during this run.
    if [ -n "${MCDC_CONF:-}" ]; then
        ( cd "${XEN_ROOT}" &&
        ./automation/renesas-scripts/mcdc-report.sh -n "${APP_NAME}" ) || true
    fi

    if [ "$RUN_COVERAGE" == "true" ]; then
        ELF="${WORKDIR}/xen-syms" COV_INPUT=${QEMU_COV_TRACE} \
        LCOV_OUT=${COVERAGE_OUT}/${APP_NAME}.cov.info \
        lldb --batch -o "command script import ${XEN_ROOT}/automation/renesas-scripts/lldb_coverage.py" \
        > ${LLDB_COV_LOG} 2>&1
    fi
}

if [ -f "${TEST_DIR}/check-trace.py" ]; then
    if ! python3 "${TEST_DIR}/check-trace.py" "${QEMU_LOG}" "${QEMU_TRACE}"; then
        echo -e "\e[31m***TRACE VALIDATION FAILED***\e[0m"
        exit 1
    fi
    echo -e "\e[32m***TRACE VALIDATION PASSED***\e[0m"
fi

# Test validation
grep -qE "${PASSED}" "${QEMU_LOG}" && { do_coverage_report; echo -e "\e[32m***FOUND EXPECTED TEST STRING***\e[0m"; exit 0; }
echo -e "\e[31m***NOT FOUND EXPECTED TEST STRING***\e[0m"
exit 1
