# Dockerfile for building pre-built ATfE image
ARG TARGETPLATFORM=linux/arm64

FROM --platform=$TARGETPLATFORM debian:bookworm@sha256:d01662367b48fc3bd42f389af59f2b39e20652b8f4be4130f80d1ac223d7eb27 AS builder
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get --quiet --yes --no-install-recommends install \
    ca-certificates=20230311* \
    cmake=3.25.1* \
    gcc=4:12.2.0* \
    g++=4:12.2.0* \
    git=1:2.39.5* \
    libc6-dev=2.36* \
    libxml2-dev=2.9.14* \
    make=4.3* \
    ninja-build=1.11.1* \
    python3-dev=3.11.2* \
    swig=4.1.0* \
    wget=1.21.3* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists* /tmp/* /var/tmp/*

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

FROM scratch
COPY --from=builder /opt/atfe /opt/atfe
