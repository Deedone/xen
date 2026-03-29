#!/bin/bash

set -o pipefail # Propagate pipeline errors
set -x # Exit immediately if any command returns a non-zero status
set -e # Print each command before execution (with expansion applied)

if [ $# -lt 1 ]; then
    echo "Usage: $(basename $0) GDB-SCRIPT"
    exit 0
fi

export GDB_SCRIPT="$1"
export XEN_ROOT="${PWD}"
export WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

export QEMU_LOG="${QEMU_LOG:-${XEN_ROOT}/qemu.serial}"
export GDB_LOG="${GDB_LOG:-${XEN_ROOT}/gdb.serial}"

export PASSED="${PASSED:-Test result: SUCCESS}"

export XEN_CMDLINE="${XEN_CMDLINE:-loglvl=all noreboot console_timestamps=boot console=dtuart}"

rm -f ${QEMU_LOG}
rm -f ${GDB_LOG}

# Generate base device tree from QEMU
qemu-system-aarch64 -cpu cortex-a57 -machine virt,virtualization=true,gic-version=3 \
    -m 2G -smp 2 -display none -machine dumpdtb=${WORKDIR}/virt-gicv3.dtb

# Add cmdline to chosen node
fdtput -c ${WORKDIR}/virt-gicv3.dtb /chosen || true
fdtput -t s ${WORKDIR}/virt-gicv3.dtb /chosen xen,xen-bootargs "${XEN_CMDLINE}"

qemu-system-aarch64 -s -S -cpu cortex-a57 -machine virt,virtualization=true,gic-version=3 \
    -m 2G -smp 2 -no-reboot -nographic -monitor none -nodefaults -serial stdio -display none \
    -kernel ${WORKDIR}/xen -dtb ${WORKDIR}/virt-gicv3.dtb \
    > ${QEMU_LOG} 2>&1 &

QEMU_PID=$!
sleep 1

gdb-multiarch -q -x ${XEN_ROOT}/automation/renesas-scripts/gdb/${GDB_SCRIPT} \
    > ${GDB_LOG} 2>&1

#Stopping QEMU
kill $QEMU_PID || true
wait $QEMU_PID 2>/dev/null || true
sleep 1

#Print the captured logs to the job output
cat ${QEMU_LOG} || true
cat ${GDB_LOG} || true

# Test validation
grep -q "${PASSED}" "${GDB_LOG}" && exit 0
exit 1
