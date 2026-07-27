# Dockerfile for building pre-built Zephyr SDK image
ARG TARGETPLATFORM=linux/arm64

FROM --platform=$TARGETPLATFORM debian:bookworm@sha256:d01662367b48fc3bd42f389af59f2b39e20652b8f4be4130f80d1ac223d7eb27 AS builder
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get --quiet --yes --no-install-recommends install \
    ca-certificates=20230311* \
    wget=1.21.3* \
    xz-utils=5.4.1* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists* /tmp/* /var/tmp/*

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

FROM scratch
COPY --from=builder /opt/zephyr /opt/zephyr
