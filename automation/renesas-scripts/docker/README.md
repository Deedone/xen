## Structure

```
docker/
├── Artifacts.dockerfile                                                # Dockerfile for Xen test artifacts
├── Atfe.dockerfile                                                     # Standalone Dockerfile for pre-built ATfE
├── Executor.dockerfile                                                 # Dockerfile for Xen certification tasks
├── Qemu.dockerfile                                                     # Standalone Dockerfile for pre-built QEMU
├── Zephyr.dockerfile                                                   # Standalone Dockerfile for pre-built Zephyr SDK
└── README.md                                                           # This file
```

## Quick Start

### Build the Artifacts Image

This Artifacts image builds minimal Xen test artifacts for ARM64:

 - Linux kernel v6.6.86
 - Alpine-based initramfs (rootfs.cpio.gz)

Built on top of Alpine Linux.

```bash
DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --progress=plain -t xentroops/xen_artifacts_rel:latest -f Artifacts.dockerfile .
docker push xentroops/xen_artifacts_rel:latest
```

### Build Pre-built Heavy Component Images

Heavy dependencies are built using their respective Dockerfiles and pushed to the registry once:

# Build & push QEMU pre-built image

```bash
DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --progress=plain -t xentroops/xen_executor_qemu:v11.0.2-xt -f Qemu.dockerfile .
docker push xentroops/xen_executor_qemu:v11.0.2-xt
```

# Build & push ATfE pre-built image

```bash
DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --progress=plain -t xentroops/xen_executor_atfe:22.1.0 -f Atfe.dockerfile .
docker push xentroops/xen_executor_atfe:22.1.0
```

# Build & push Zephyr SDK pre-built image

```bash
DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --progress=plain -t xentroops/xen_executor_zephyr:1.0.1 -f Zephyr.dockerfile .
docker push xentroops/xen_executor_zephyr:1.0.1
```

### Build the Executor Image

The Executor image is a full ARM64 CI/testing environment based on Debian.
It integrates:

 - xen-troops QEMU v11.0.2-xt (system-mode + plugins + system SMMUv3 model)
 - LLVM toolchain (Clang, LLD, LLDB via Arm Toolchain 22.1.0)
 - Utilities for Xen running/testing/debugging

The image is designed for reproducible CI execution and Xen certification tasks.
It uses pre-built base images for heavy components (QEMU, ATfE, Zephyr SDK):

# Fast build using pre-built images

```bash
DOCKER_BUILDKIT=1 docker build --platform linux/arm64 --progress=plain -t xentroops/xen_rel:latest -f Executor.dockerfile .
docker push xentroops/xen_rel:latest
```

### Publishing Images to Registry

```bash
echo "<TOKEN>" | docker login -u "xentroops" --password-stdin
docker push xentroops/<image_name>:<tag>
```
