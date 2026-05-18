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

if [[ "${XTF_NAME_ARG}" == "hyp-sysctl-sched-rtds-edf" ]]; then
    domu_check="
taskset -c 0 sh -c 'while :; do :; done' &
taskset -c 1 sh -c 'while :; do :; done' &
wait
"
elif [[ "${XTF_NAME_ARG}" == "hyp-sysctl-sched-rtds-extratime" ]]; then
    domu_check="
taskset -c 0 sh -c 'while :; do :; done' &
wait
"
fi

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
${domu_check}
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
if [[ "${XTF_NAME_ARG}" == "hyp-sysctl-sched-rtds-edf" ]]; then
    export XTF_NUM_DOMUS=1
    export XTF_DOMU_VCPUS=2
elif [[ "${XTF_NAME_ARG}" == "hyp-sysctl-sched-rtds-extratime" ]]; then
    export XTF_NUM_DOMUS=1
    export XTF_DOMU_VCPUS=1
fi

xtf_test $@
