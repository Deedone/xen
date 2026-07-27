## Structure

```
docker/
├── Artifacts.dockerfile                                                # Dockerfile for Xen test artifacts
├── Executor.dockerfile                                                 # Dockerfile for Xen certification tasks
└── README.md                                                           # This file
```

## Quick Start

### Build the Artifacts Image

This Artifacts image builds minimal Xen test artifacts for ARM64:

 - Linux kernel v6.6.86
 - Alpine-based initramfs (rootfs.cpio.gz)

Built on top of Alpine Linux.

```bash
DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --progress=plain -t xentroops/xen_artifacts_rel -f Artifacts.dockerfile .
```

### Build the Executor Image

The Executor image is a full ARM64 CI/testing environment based on Debian.
It integrates:

 - xen-troops QEMU v11.0.2-xt (system-mode + plugins + system SMMUv3 model)
 - LLVM toolchain (Clang, LLD, LLDB via Arm Toolchain 22.1.0)
 - Utilities for Xen running/testing/debugging

The image is designed for reproducible CI execution and Xen certification
tasks.

Build uses a multi-stage Docker pipeline:

```bash
DOCKER_BUILDKIT=1 docker build --target builder --platform linux/arm64 --progress=plain -t executor:builder -f Executor.dockerfile .
DOCKER_BUILDKIT=1 docker build --target qemu_build --platform linux/arm64 --progress=plain -t executor:qemu_build -f Executor.dockerfile .
DOCKER_BUILDKIT=1 docker build --target atfe_build --platform linux/arm64 --progress=plain -t executor:atfe_build -f Executor.dockerfile .
DOCKER_BUILDKIT=1 docker build --target zephyr_build --platform linux/arm64 --progress=plain -t executor:zephyr_build -f Executor.dockerfile .
DOCKER_BUILDKIT=1 docker build --target runner --platform linux/arm64 --progress=plain -t executor:runner -f Executor.dockerfile .
```

Tag final runtime image:

```bash
docker tag executor:runner xentroops/xen_rel:latest
```

### Push Image

Push final runtime image:

```bash
echo "<TOKEN>" | docker login -u "xentroops" --password-stdin
docker push xentroops/<image_name>:<tag>
```
