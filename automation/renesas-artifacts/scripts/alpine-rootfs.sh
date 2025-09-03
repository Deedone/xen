#!/bin/bash

set -eu

WORKDIR="${PWD}"
COPYDIR="${WORKDIR}/binaries"
UNAME=$(uname -m)

apk --no-cache upgrade

PKGS=(
    # System
    libgcc
    openrc
    udev
    util-linux

    # Xen toolstack runtime deps
    libbz2
    libuuid
    lzo
    xz
    yajl

    # Xen Test Framework
    python3

    # QEMU
    glib
    libaio
    pixman
    )

case $UNAME in
    x86_64)
        PKGS+=(
            # System
            pciutils

            # QEMU
            libelf
            )
        ;;

    aarch64)
        PKGS+=(
            # Xen
            libfdt
            )
        ;;
esac

apk add --no-cache "${PKGS[@]}"

# Xen
cd /
# Minimal ramdisk environment in case of cpio output
rc-update add udev
rc-update add udev-trigger
rc-update add udev-settle
rc-update add loopback sysinit
rc-update add bootmisc boot
rc-update add devfs sysinit
rc-update add dmesg sysinit
rc-update add hostname boot
rc-update add hwclock boot
rc-update add hwdrivers sysinit
rc-update add killprocs shutdown
rc-update add mount-ro shutdown
rc-update add savecache shutdown
rc-update add local default
cp -a /sbin/init /init
echo "ttyS0" >> /etc/securetty
echo "hvc0" >> /etc/securetty
echo "ttyS0::respawn:/sbin/getty -L ttyS0 115200 vt100" >> /etc/inittab
echo "hvc0::respawn:/sbin/getty -L hvc0 115200 vt100" >> /etc/inittab
echo "rc_verbose=yes" >> /etc/rc.conf
echo > /etc/modules
passwd -d "root" root

# Create rootfs
cd /
{
    PATHS="bin etc home init lib mnt opt root sbin srv usr var"
    find $PATHS -print0
    echo -ne "dev\0proc\0run\0sys\0"
} | cpio -0 -R 0:0 -H newc -o | gzip > "${COPYDIR}/rootfs.cpio.gz"

# Print the contents for the build log
zcat "${COPYDIR}/rootfs.cpio.gz" | cpio -tv
