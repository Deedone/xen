#!/bin/bash
#
# XTF test runner (QEMU).
#

set -e -o pipefail

if [ $# -lt 3 ]; then
    echo "Usage: $(basename $0) ARCH XTF-VARIANT XTF-NAME"
    exit 0
fi

export ARCH="$1"
XTF_NAME_ARG="$3"
shift

set -x


export XEN_ROOT="${PWD}"

# DomU Busybox
cd binaries
mkdir -p initrd
mkdir -p initrd/bin
mkdir -p initrd/sbin
mkdir -p initrd/etc
mkdir -p initrd/dev
mkdir -p initrd/proc
mkdir -p initrd/sys
mkdir -p initrd/lib
mkdir -p initrd/var
mkdir -p initrd/mnt
cp /bin/busybox initrd/bin/busybox
initrd/bin/busybox --install initrd/bin
echo "#!/bin/sh

mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev
/bin/sh" > initrd/init
chmod +x initrd/init
cd initrd
find . | cpio -H newc -o | gzip > ../domU-rootfs.cpio.gz
cd .. # binaries
cd .. # XEN_ROOT
cd $(dirname $0)

source include/xtf-runner

if [ ! -f "include/xtf-${ARCH}" ]; then
    die "unsupported architecture '${ARCH}'"
fi
source include/xtf-${ARCH}

# Per-test environment overrides
if [[ "${XTF_NAME_ARG}" == "hyp-domctl-sched-rtds-edf" ]]; then
    export XTF_NUM_DOMUS=2
fi

xtf_test $@

# Post-test verification: parse Xen serial log for RTDS EDF ordering.
if [[ "${XTF_NAME_ARG}" == "hyp-domctl-sched-rtds-edf" ]]; then
    echo "--- Verifying RTDS EDF ordering from serial log ---"
    rtds_dump=$(sed 's/\r//g' < "${XEN_ROOT}/smoke.serial" | \
        sed -n '/Global RunQueue info:/,/Global DepletedQueue info:/p')

    d1=$(printf '%s\n' "${rtds_dump}" | \
        sed -n 's/.*\[\s*1\.[0-9]\+\s*\].*cur_d=\([0-9][0-9]*\).*/\1/p' | \
        head -n1 | tr -cd '0-9')
    d2=$(printf '%s\n' "${rtds_dump}" | \
        sed -n 's/.*\[\s*2\.[0-9]\+\s*\].*cur_d=\([0-9][0-9]*\).*/\1/p' | \
        head -n1 | tr -cd '0-9')

    echo "DomU1 cur_deadline=${d1:-<not found>}  DomU2 cur_deadline=${d2:-<not found>}"

    if [ -n "${d1}" ] && [ -n "${d2}" ] && [ "${d1}" -le "${d2}" ]; then
        echo "rtds_sched_edf test passed"
    else
        echo "FAIL: EDF ordering not observed (d1=${d1:-?} d2=${d2:-?})"
        exit 1
    fi
fi
