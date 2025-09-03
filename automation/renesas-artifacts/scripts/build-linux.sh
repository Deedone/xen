#!/usr/bin/env bash

if test -z "${LINUX_VERSION}"
then
    >&2 echo "LINUX_VERSION must be set"; exit 1
fi

set -ex -o pipefail

WORKDIR="${PWD}"
COPYDIR="${WORKDIR}/binaries"
UNAME=$(uname -m)

# Build Linux
MAJOR=${LINUX_VERSION%%.*}
curl -fsSLO \
    https://cdn.kernel.org/pub/linux/kernel/v"${MAJOR}".x/linux-"${LINUX_VERSION}".tar.xz
tar xf linux-"${LINUX_VERSION}".tar.xz
cd linux-"${LINUX_VERSION}"

make defconfig
./scripts/config --enable BRIDGE
./scripts/config --enable IGC
./scripts/config --enable IPV6
./scripts/config --enable TUN

case $UNAME in
    x86_64)
        make xen.config
        cp .config .config.orig
        cat .config.orig \
            | grep 'XEN' \
            | grep '=m' \
            | sed 's/=m/=y/g' >> .config
        ;;

    aarch64)
        ./scripts/config --enable XEN_NETDEV_BACKEND
        ;;
esac

make olddefconfig

case $UNAME in
    x86_64)
        make -j$(nproc) bzImage
        cp arch/x86/boot/bzImage "${COPYDIR}"

        # Build argo if requested
        if [[ -n "${ARGO_SHA}" ]]; then
            make modules_prepare
            . "${WORKDIR}/scripts/build-argo.sh"
        fi
        ;;

    aarch64)
        make -j$(nproc) Image
        cp arch/arm64/boot/Image "${COPYDIR}"
        ;;
esac
