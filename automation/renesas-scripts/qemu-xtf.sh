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

# DomU Busybox (default DomU for XTF tests that use a Linux guest)
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

# Auto-load per-test overrides (DomU type, vCPU count, Zephyr config, etc.)
if [ -f "include/tests/${XTF_NAME_ARG}" ]; then
    source "include/tests/${XTF_NAME_ARG}"
fi

# Build Zephyr DomU if the per-test config requests it
if [[ -n "${XTF_DOMU_ZEPHYR_APP}" ]]; then
    _scripts_dir="${PWD}"
    if [[ ! -d "${XEN_ROOT}/zephyr_tests" ]]; then
        git clone --depth 1 \
            "https://gitlab-ci-token:${CI_JOB_TOKEN}@gitpct.epam.com/rec-fusa/zephyr_tests.git" \
            -b "${XTF_DOMU_ZEPHYR_BRANCH:-safety-staging}" "${XEN_ROOT}/zephyr_tests"
    fi
    cd "${ZEPHYR_SDK_INSTALL_DIR}"
    west build -p always -b "${XTF_DOMU_ZEPHYR_BOARD:-xenvm/xenvm/gicv3}" \
        "${XEN_ROOT}/zephyr_tests/testcases/${XTF_DOMU_ZEPHYR_APP}"
    cp build/zephyr/zephyr.bin "${XEN_ROOT}/binaries/${XTF_DOMU_KERNEL}"
    cd "${_scripts_dir}"
fi

xtf_test $@
