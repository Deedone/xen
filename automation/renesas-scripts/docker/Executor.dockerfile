# Dockerfile for Xen certification tasks, use Debian Bookworm

#Builder image
FROM --platform=linux/arm64/v8 debian:bookworm@sha256:d01662367b48fc3bd42f389af59f2b39e20652b8f4be4130f80d1ac223d7eb27 AS builder
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get --quiet --yes --no-install-recommends install \
    bzip2=1.0.8-5+b1 \
    ca-certificates=20230311+deb12u1 \
    cmake=3.25.1-1 \
    gcc=4:12.2.0-3 \
    g++=4:12.2.0-3 \
    git=1:2.39.5-0+deb12u3 \
    libc6-dev=2.36-9+deb12u13 \
    libglib2.0-dev=2.74.6-2+deb12u8 \
    libslirp-dev=4.7.0-1 \
    libxml2-dev=2.9.14+dfsg-1.3~deb12u5 \
    make=4.3-4.1 \
    meson=1.0.1-5 \
    ninja-build=1.11.1-2~deb12u1 \
    patch=2.7.6-7 \
    pkg-config=1.8.1-1 \
    python3-dev=3.11.2-1+b1 \
    python3-venv=3.11.2-1+b1 \
    swig=4.1.0-0.2 \
    wget=1.21.3-1+deb12u1 \
    xz-utils=5.4.1-1 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists* /tmp/* /var/tmp/*

########################################################################

FROM builder AS qemu_build
ENV DEBIAN_FRONTEND=noninteractive

ARG QEMU_VERSION=10.2.2
ARG QEMU_URL=https://download.qemu.org/qemu-${QEMU_VERSION}.tar.xz

WORKDIR /tmp

COPY 0001-contrib-plugins-drcov-add-support-system-mode.patch /tmp

RUN wget --progress=bar:force:noscroll ${QEMU_URL} \
    && tar xf qemu-${QEMU_VERSION}.tar.xz \
    && cd qemu-${QEMU_VERSION} \
    && patch -p1 < /tmp/0001-contrib-plugins-drcov-add-support-system-mode.patch \
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

########################################################################

FROM builder AS atfe_build
ENV DEBIAN_FRONTEND=noninteractive

ARG ATfE_VERSION=22.1.0
ARG ATfE_URL=https://github.com/arm/arm-toolchain/archive/refs/tags/release-${ATfE_VERSION}-ATfE.tar.gz

WORKDIR /tmp

RUN wget --progress=bar:force:noscroll ${ATfE_URL} \
    && tar xf release-${ATfE_VERSION}-ATfE.tar.gz \
    && cd arm-toolchain-release-${ATfE_VERSION}-ATfE \
    && mkdir build && cd build \
    && cmake -G Ninja ../llvm \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/atfe \
        -DLLVM_ENABLE_PROJECTS="clang;lld;lldb" \
        -DLLVM_TARGETS_TO_BUILD="AArch64" \
        -DLLVM_PARALLEL_LINK_JOBS=1 \
        -DLLDB_ENABLE_PYTHON=ON \
        -DLLVM_ENABLE_LIBXML2=ON \
    && ninja -j4 \
    && ninja install \
    && cd /tmp && rm -fr /tmp/*

########################################################################

FROM builder AS zephyr_build
ENV DEBIAN_FRONTEND=noninteractive

ARG ZEPHYR_SDK_VERSION=1.0.1
ARG ZEPHYR_SDK_URL=https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-aarch64_gnu.tar.xz

WORKDIR /tmp

RUN mkdir -p /opt && mkdir -p /opt/zephyr \
    && wget --progress=bar:force:noscroll ${ZEPHYR_SDK_URL} \
    && tar xf zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-aarch64_gnu.tar.xz \
    --strip-components=1 \
    -C /opt/zephyr \
    --wildcards \
    '*/gnu/aarch64-zephyr-elf/*' \
    '*/cmake/*' \
    '*/sdk_version' \
    && rm -f /tmp/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-aarch64_gnu.tar.xz

########################################################################

#Final image
FROM --platform=linux/arm64/v8 python:3.12-bookworm@sha256:49de7aa80568e7a112035322a6a961cd55774862b35198886f7508223abf6ca1 AS runner
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get --quiet --yes --no-install-recommends install \
    acpica-tools=20200925-8 \
    bison=2:3.8.2+dfsg-1+b1 \
    busybox-static=1:1.35.0-4+b7 \
    ca-certificates=20230311+deb12u1 \
    checkpolicy=3.4-1+b2 \
    cmake=3.25.1-1 \
    cpio=2.13+dfsg-7.1 \
    curl=7.88.1-10+deb12u14 \
    device-tree-compiler=1.6.1-4+b1 \
    expect=5.45.4-2+b1 \
    file=1:5.44-3 \
    flex=2.6.4-8.2 \
    gcc=4:12.2.0-3 \
    g++=4:12.2.0-3 \
    gdb-multiarch=13.1-3 \
    git=1:2.39.5-0+deb12u3 \
    grep=3.8-5 \
    lcov=1.16-1 \
    libfdt-dev=1.6.1-4+b1 \
    libglib2.0-dev=2.74.6-2+deb12u8 \
    liblzma-dev=5.4.1-1 \
    libnl-3-dev=3.7.0-0.2+b1 \
    libnl-route-3-dev=3.7.0-0.2+b1 \
    libncurses-dev=6.4-4 \
    libpixman-1-dev=0.42.2-1 \
    libslirp-dev=4.7.0-1 \
    libyajl-dev=2.1.0-3+deb12u2 \
    markdown=1.0.1-12 \
    make=4.3-4.1 \
    ninja-build=1.11.1-2~deb12u1 \
    pandoc=2.17.1.1-2~deb12u1 \
    pkg-config=1.8.1-1 \
    u-boot-qemu=2023.01+dfsg-2+deb12u2 \
    u-boot-tools=2023.01+dfsg-2+deb12u2 \
    unzip=6.0-28 \
    uuid-dev=2.38.1-5+deb12u3 \
    wget=1.21.3-1+deb12u1 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists* /tmp/* /var/tmp/*

# Copy QEMU
COPY --from=qemu_build /opt/qemu/bin/qemu-system-aarch64 /usr/local/bin/qemu-system-aarch64
COPY --from=qemu_build /opt/qemu/share/qemu/efi-virtio.rom /usr/local/share/qemu/efi-virtio.rom
COPY --from=qemu_build /opt/qemu/lib/qemu-plugins /usr/local/lib/qemu-plugins

# Copy ATfE
COPY --from=atfe_build /opt/atfe /usr/local

# Copy Zephyr
COPY --from=zephyr_build /opt/zephyr /usr/local/zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/usr/local/zephyr
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
RUN pip3 install --break-system-packages \
    west \
    pyelftools \
    PyYAML \
    packaging \
    pykwalify \
    jsonschema \
    kconfiglib \
    junitparser

WORKDIR /usr/local/zephyr
COPY manifest /usr/local/zephyr/manifest
RUN west init -l manifest \
    && west update --fetch-opt=--depth=1 --fetch-opt=--no-tags \
    && west zephyr-export

WORKDIR /build
