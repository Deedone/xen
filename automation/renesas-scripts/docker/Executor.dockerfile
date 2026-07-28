# Dockerfile for Xen certification tasks, use Debian Bookworm
ARG TARGETPLATFORM=linux/arm64

ARG QEMU_PREBUILT_IMAGE=xentroops/xen_executor_qemu:v11.0.2-xt
ARG ATFE_PREBUILT_IMAGE=xentroops/xen_executor_atfe:22.1.0
ARG ZEPHYR_PREBUILT_IMAGE=xentroops/xen_executor_zephyr:1.0.1

# Resolution of pre-built component sources
FROM ${QEMU_PREBUILT_IMAGE} AS qemu_source
FROM ${ATFE_PREBUILT_IMAGE} AS atfe_source
FROM ${ZEPHYR_PREBUILT_IMAGE} AS zephyr_source

# Final runner image
FROM --platform=$TARGETPLATFORM python:3.12-bookworm@sha256:49de7aa80568e7a112035322a6a961cd55774862b35198886f7508223abf6ca1 AS runner
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get --quiet --yes --no-install-recommends install \
    acpica-tools=20200925* \
    bear=3.1* \
    bison=2:3.8.2* \
    busybox-static=1:1.35.0* \
    ca-certificates=20230311* \
    checkpolicy=3.4* \
    cmake=3.25.1* \
    cpio=2.13* \
    cppcheck=2.10* \
    curl=7.88.1* \
    device-tree-compiler=1.6.1* \
    expect=5.45.4* \
    file=1:5.44* \
    flex=2.6.4* \
    gcc=4:12.2.0* \
    g++=4:12.2.0* \
    gdb-multiarch=13.1* \
    git=1:2.39.5* \
    grep=3.8* \
    libfdt-dev=1.6.1* \
    libglib2.0-dev=2.74.6* \
    liblzma-dev=5.4.1* \
    libnl-3-dev=3.7.0* \
    libnl-route-3-dev=3.7.0* \
    libncurses-dev=6.4* \
    libpixman-1-dev=0.42.2* \
    libslirp-dev=4.7.0* \
    libyajl-dev=2.1.0* \
    libcapture-tiny-perl=0.48* \
    libdatetime-perl=2:1.59* \
    libdevel-cover-perl=1.38* \
    libjson-perl=4.10000* \
    libtimedate-perl=2.3300* \
    libmodule-load-conditional-perl=0.74* \
    markdown=1.0.1* \
    make=4.3* \
    ninja-build=1.11.1* \
    pandoc=2.17.1.1* \
    pkg-config=1.8.1* \
    u-boot-qemu=2023.01* \
    u-boot-tools=2023.01* \
    unzip=6.0* \
    uuid-dev=2.38.1* \
    wget=1.21.3* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists* /tmp/* /var/tmp/*

# Copy QEMU artifacts from pre-built image
COPY --from=qemu_source /opt/qemu/bin/qemu-system-aarch64 /usr/local/bin/qemu-system-aarch64
COPY --from=qemu_source /opt/qemu/share/qemu/efi-virtio.rom /usr/local/share/qemu/efi-virtio.rom
COPY --from=qemu_source /opt/qemu/lib/qemu-plugins /usr/local/lib/qemu-plugins
# qemu-plugin.h header so the MC/DC brtrace plugin can be built against this QEMU
COPY --from=qemu_source /opt/qemu/include/qemu-plugin.h /usr/include/qemu-plugin.h

# Copy ATfE artifacts from pre-built image
COPY --from=atfe_source /opt/atfe /usr/local

# Copy Zephyr artifacts from pre-built image
COPY --from=zephyr_source /opt/zephyr /usr/local/zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/usr/local/zephyr
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr

RUN pip3 install --break-system-packages \
    west \
    pyelftools \
    capstone \
    PyYAML \
    packaging \
    pykwalify \
    jsonschema \
    kconfiglib \
    junitparser

# Install LCOV
ARG LCOV_BRANCH=v2.4
ARG LCOV_GIT=https://github.com/linux-test-project/lcov.git
WORKDIR /tmp
RUN cd /tmp && git clone -b "${LCOV_BRANCH}" --single-branch --depth 1 "${LCOV_GIT}" \
    && cd lcov && make install && cd /tmp && rm -fr /tmp/*

# Install Cppcheck 2.7
ARG CPPCHECK_URL=https://github.com/danmar/cppcheck/archive/2.7.tar.gz
WORKDIR /tmp
RUN cd /tmp && wget --progress=bar:force:noscroll ${CPPCHECK_URL} \
    && tar xf 2.7.tar.gz && cd cppcheck-2.7 && mkdir build && cd build \
    && cmake .. && cmake --build . && make install && cd /tmp && rm -fr /tmp/*

# Copy manifest and initialize Zephyr workspace
WORKDIR /usr/local/zephyr
COPY manifest /usr/local/zephyr/manifest
RUN west init -l manifest \
    && west update --fetch-opt=--depth=1 --fetch-opt=--no-tags \
    && west zephyr-export

WORKDIR /build
