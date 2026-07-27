# Dockerfile for building pre-built QEMU image
ARG TARGETPLATFORM=linux/arm64

FROM --platform=$TARGETPLATFORM debian:bookworm@sha256:d01662367b48fc3bd42f389af59f2b39e20652b8f4be4130f80d1ac223d7eb27 AS builder
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get --quiet --yes --no-install-recommends install \
    bzip2=1.0.8* \
    ca-certificates=20230311* \
    cmake=3.25.1* \
    gcc=4:12.2.0* \
    g++=4:12.2.0* \
    git=1:2.39.5* \
    libc6-dev=2.36* \
    libglib2.0-dev=2.74.6* \
    libslirp-dev=4.7.0* \
    make=4.3* \
    meson=1.0.1* \
    ninja-build=1.11.1* \
    pkg-config=1.8.1* \
    python3-dev=3.11.2* \
    python3-venv=3.11.2* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists* /tmp/* /var/tmp/*

ARG QEMU_VERSION=v11.0.2-xt
ARG QEMU_URL=https://github.com/xen-troops/qemu.git

WORKDIR /tmp

RUN git clone -b ${QEMU_VERSION} --single-branch ${QEMU_URL} \
    && cd qemu \
    && ./configure \
        --prefix=/opt/qemu \
        --target-list=aarch64-softmmu \
        --cpu=host \
        --disable-containers \
        --disable-debug-info \
        --disable-hexagon-idef-parser \
        --disable-qom-cast-debug  \
        --enable-fdt=internal \
        --enable-strip \
        --disable-alsa \
        --disable-bochs \
        --disable-bpf \
        --disable-brlapi \
        --disable-bzip2 \
        --disable-canokey \
        --disable-cap-ng \
        --disable-capstone \
        --disable-cloop \
        --disable-cocoa \
        --disable-colo-proxy \
        --disable-coreaudio \
        --disable-crypto-afalg \
        --disable-curl \
        --disable-curses \
        --disable-dbus-display \
        --disable-dmg \
        --disable-docs \
        --disable-dsound \
        --disable-fuse \
        --disable-gcrypt \
        --disable-gettext \
        --disable-gio \
        --disable-glusterfs \
        --disable-gnutls \
        --disable-gtk \
        --disable-guest-agent \
        --disable-guest-agent-msi \
        --disable-hv-balloon \
        --disable-hvf \
        --disable-jack \
        --disable-keyring \
        --disable-kvm \
        --disable-l2tpv3 \
        --disable-lzfse \
        --disable-modules \
        --disable-opengl \
        --disable-oss \
        --disable-pa \
        --disable-pipewire \
        --disable-pixman \
        --enable-plugins \
        --disable-png \
        --disable-pvg \
        --disable-qatzip \
        --disable-qcow1 \
        --disable-qed \
        --disable-qga-vss \
        --disable-replication  \
        --disable-rust \
        --disable-rutabaga-gfx \
        --disable-sdl \
        --disable-selinux \
        --enable-slirp \
        --disable-smartcard \
        --disable-snappy \
        --disable-sndio \
        --disable-spice \
        --enable-tcg \
        --disable-tools \
        --disable-tpm \
        --disable-valgrind \
        --disable-vdi \
        --disable-libvduse \
        --disable-vduse-blk-export \
        --disable-vfio-user-server \
        --disable-vhdx \
        --disable-vhost-crypto \
        --disable-vhost-kernel \
        --disable-vhost-net \
        --disable-vhost-user \
        --disable-vhost-user-blk-server \
        --disable-vhost-vdpa \
        --disable-virglrenderer \
        --disable-virtfs \
        --disable-vmdk \
        --disable-vmnet \
        --disable-vnc \
        --disable-vpc \
        --disable-vte \
        --disable-vvfat \
        --disable-werror \
        --disable-whpx \
        --disable-xen \
        --disable-xkbcommon \
        --disable-zstd \
        --enable-system \
        --disable-user \
        --disable-linux-user \
        --disable-bsd-user \
    && make -j4 \
    && make install \
    && mkdir -p /opt/qemu/lib/qemu-plugins \
    && find build/contrib/plugins -name "*.so" -exec cp {} /opt/qemu/lib/qemu-plugins \; \
    && cd /tmp && rm -fr /tmp/*

FROM scratch
COPY --from=builder /opt/qemu /opt/qemu
