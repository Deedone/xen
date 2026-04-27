#!/bin/bash

set -o pipefail # Propagate pipeline errors
set -x # Exit immediately if any command returns a non-zero status
set -e # Print each command before execution (with expansion applied)

if [ $# -lt 1 ]; then
    echo "Usage: $(basename $0) LLDB-SCRIPT"
    exit 0
fi

export LLDB_SCRIPT="$1"
export XEN_ROOT="${PWD}"
export WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

export QEMU_LOG="${QEMU_LOG:-${XEN_ROOT}/qemu.serial}"
export LLDB_LOG="${LLDB_LOG:-${XEN_ROOT}/lldb.serial}"
export XEN_LOG="${XEN_LOG:-${XEN_ROOT}/xen.serial}"

export PASSED="${PASSED:-[SUCCESS]}"

export XEN_CMDLINE="${XEN_CMDLINE:-loglvl=all noreboot console_timestamps=boot console=dtuart}"

# Add directory with lldb_automation library
export PYTHONPATH="${XEN_ROOT}/automation/renesas-scripts/lldb/:$PYTHONPATH"

rm -f ${QEMU_LOG}
rm -f ${LLDB_LOG}
rm -f ${XEN_LOG}

# Generate base device tree from QEMU
qemu-system-aarch64 \
    -cpu cortex-a57 \
    -machine virt,virtualization=true,gic-version=3 \
    -m 2048 \
    -smp 2 \
    -machine dumpdtb=${WORKDIR}/virt-gicv3.dtb

# Add cmdline to chosen node
fdtput -c ${WORKDIR}/virt-gicv3.dtb /chosen 2>/dev/null || true
fdtput -t s ${WORKDIR}/virt-gicv3.dtb /chosen xen,xen-bootargs "${XEN_CMDLINE}"

# Run QEMU in background, LLDB conflicts with "-serial stdio", so write Xen logs into file
qemu-system-aarch64 \
    -s -S \
    -cpu cortex-a57 \
    -machine virt,virtualization=true,gic-version=3 \
    -m 2048 \
    -smp 2 \
    -no-reboot \
    -nodefaults \
    -display none \
    -monitor none \
    -serial file:${XEN_LOG} \
    -kernel ${WORKDIR}/xen -dtb ${WORKDIR}/virt-gicv3.dtb > ${QEMU_LOG} 2>&1 &

QEMU_PID=$!
sleep 1

XEN_ELF="${WORKDIR}/xen-syms" XEN_PORT="1234" \
    lldb --batch -o "command script import ${XEN_ROOT}/automation/renesas-scripts/lldb/${LLDB_SCRIPT}" \
    > ${LLDB_LOG} 2>&1

#Stopping QEMU
kill $QEMU_PID || true
wait $QEMU_PID 2>/dev/null || true
sync || true

#Print the captured logs to the job output
cat ${XEN_LOG} || true
cat ${QEMU_LOG} || true
cat ${LLDB_LOG} || true

# Test validation
grep -qF "${PASSED}" "${LLDB_LOG}" && { echo -e "\e[32m***FOUND EXPECTED TEST STRING***\e[0m"; exit 0; }
echo -e "\e[31m***NOT FOUND EXPECTED TEST STRING***\e[0m"
exit 1
