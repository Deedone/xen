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

xtf_test $@
