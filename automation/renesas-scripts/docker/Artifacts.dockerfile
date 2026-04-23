# Dockerfile for Xen test artifacts, use Alpine

# Builder image
FROM --platform=linux/arm64/v8 registry.gitlab.com/xen-project/hardware/test-artifacts/alpine:3.18-arm64-build AS kernel-builder

ENV LINUX_VERSION=6.6.86
WORKDIR /build

RUN set -e && curl -fsSLO https://cdn.kernel.org/pub/linux/kernel/v6.x/linux-${LINUX_VERSION}.tar.xz && \
    tar xJf linux-${LINUX_VERSION}.tar.xz && \
    cd linux-${LINUX_VERSION} && \
    make defconfig && \
    ./scripts/config --enable BRIDGE && \
    ./scripts/config --enable IGC && \
    ./scripts/config --enable IPV6 && \
    ./scripts/config --enable TUN && \
    ./scripts/config --enable XEN_NETDEV_BACKEND && \
    make olddefconfig && \
    make -j4 Image && \
    cp arch/arm64/boot/Image /build/Image

# Builder rootfs
FROM --platform=linux/arm64/v8 registry.gitlab.com/xen-project/hardware/test-artifacts/alpine:3.18-arm64-base AS rootfs-builder

# Build depends
RUN apk --no-cache upgrade && apk --no-cache add \
    bash \
    libgcc \
    openrc \
    udev \
    util-linux \
    libbz2 \
    libc-utils \
    libncursesw \
    libuuid \
    lzo \
    xz \
    yajl \
    python3 \
    readline \
    glib \
    libaio \
    pixman \
    libfdt

WORKDIR /build

RUN set -e && cd /  && \
    rc-update add udev && \
    rc-update add udev-trigger && \
    rc-update add udev-settle && \
    rc-update add loopback sysinit && \
    rc-update add bootmisc boot && \
    rc-update add devfs sysinit && \
    rc-update add dmesg sysinit && \
    rc-update add hostname boot && \
    rc-update add hwclock boot && \
    rc-update add hwdrivers sysinit && \
    rc-update add killprocs shutdown && \
    rc-update add mount-ro shutdown && \
    rc-update add savecache shutdown && \
    rc-update add local default && \
    cp -a /sbin/init /init && \
    echo "ttyS0" >> /etc/securetty && \
    echo "hvc0" >> /etc/securetty && \
    echo "ttyS0::respawn:/sbin/getty -L ttyS0 115200 vt100" >> /etc/inittab && \
    echo "hvc0::respawn:/sbin/getty -L hvc0 115200 vt100" >> /etc/inittab && \
    echo "rc_verbose=yes" >> /etc/rc.conf && \
    echo > /etc/modules && \
    passwd -d "root" root && \
    cd / && \
    { PATHS="bin etc home init lib mnt opt root sbin srv usr var"; find $PATHS -print0; echo -ne "dev\0proc\0run\0sys\0"; } | cpio -0 -R 0:0 -H newc -o | gzip > /build/rootfs.cpio.gz && \
    zcat /build/rootfs.cpio.gz | cpio -tv

# Final artifacts export image
FROM --platform=linux/arm64/v8 alpine:3.22.4@sha256:310c62b5e7ca5b08167e4384c68db0fd2905dd9c7493756d356e893909057601 AS final

WORKDIR /artifacts

COPY --from=kernel-builder /build/Image              /artifacts/Image
COPY --from=rootfs-builder /build/rootfs.cpio.gz     /artifacts/rootfs.cpio.gz
