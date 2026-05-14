#!/bin/bash

set -ex -o pipefail

export XEN_ROOT="${PWD}"
export WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"
export FUZZER_PREFIX="${FUZZER_PREFIX:-/root/}"
export TIMEOUT_FUZZER="${TIMEOUT_FUZZER:-300}"
export PASSED="${PASSED:-No objectives found, all good!}"
export FUZZER_LOG="${FUZZER_LOG:-${XEN_ROOT}/fuzzer.serial}"

mkdir -p ${XEN_ROOT}/corpus
cp ${FUZZER_PREFIX}corpus/* ${XEN_ROOT}/corpus
mkdir -p ${XEN_ROOT}/gcov_reports
rm -f ${FUZZER_LOG}

# Run Fuzzer
${FUZZER_PREFIX}xen_fuzzer --version

echo "Run Xen fuzzer with timeout ${TIMEOUT_FUZZER} seconds..."
${FUZZER_PREFIX}xen_fuzzer -t ${TIMEOUT_FUZZER} raw \
    -accel tcg \
    -machine virt,virtualization=yes,acpi=off,gic-version=3 \
    -m 4G \
    -cpu max \
    -no-reboot \
    -nodefaults \
    -display none \
    -monitor none \
    -serial stdio \
    -L ${FUZZER_PREFIX} \
    -append 'dom0_mem=512M loglvl=all guest_loglvl=none console=dtuart' \
    -kernel ${WORKDIR}/xen \
    -device guest-loader,addr=0x42000000,kernel=${FUZZER_PREFIX}test-mmu64le-arm-structured-fuzzer,bootargs="none" \
    -snapshot > ${FUZZER_LOG} 2>&1 || true

# Debug found crashes
pushd ${FUZZER_PREFIX}
for f in ${XEN_ROOT}/crashes/*; do
    [ -f "$f" ] || continue
    echo "Processing found crash: $(basename "$f")"
    ${FUZZER_PREFIX}xen_fuzzer -r "$f" run ${WORKDIR}/xen ${FUZZER_PREFIX}test-mmu64le-arm-structured-fuzzer || true
done

# Run validation
grep -qF "${PASSED}" "${FUZZER_LOG}" && { echo -e "\e[32m***No objectives found, all good!***\e[0m"; exit 0; }
echo -e "\e[31m***Found objectives (crashes)!***\e[0m"
exit 1
