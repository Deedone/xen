#!/bin/bash

set -ex -o pipefail

# DomU Busybox
cd binaries
mkdir -p initrd
mkdir -p initrd/bin
mkdir -p initrd/sbin
mkdir -p initrd/etc
mkdir -p initrd/dev
mkdir -p initrd/proc
mkdir -p initrd/sys
mkdir -p initrd/lib
mkdir -p initrd/var
mkdir -p initrd/mnt
cp /bin/busybox initrd/bin/busybox
initrd/bin/busybox --install initrd/bin
echo "#!/bin/sh

mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev
/bin/sh" > initrd/init
chmod +x initrd/init
cd initrd
find . | cpio -R 0:0 -H newc -o | gzip > ../domU-rootfs.cpio.gz
cd ..

# Dom0 rootfs
cp rootfs.cpio.gz dom0-rootfs.cpio.gz
cat xen-tools.cpio.gz >> dom0-rootfs.cpio.gz

# test-local configuration
mkdir -p rootfs
cd rootfs
mkdir -p etc/local.d root
mv ../domU-rootfs.cpio.gz ./root
cp ../Image ./root
echo "name=\"domU\"
memory=512
vcpus=1
kernel=\"/root/Image\"
ramdisk=\"/root/domU-rootfs.cpio.gz\"
extra=\"console=hvc0 root=/dev/ram0 rdinit=/bin/sh\"
" > root/domU.cfg
echo "#!/bin/bash

bash /etc/init.d/xencommons start

xl list

xl -vvv create -c /root/domU.cfg

" > etc/local.d/xen.start
chmod +x etc/local.d/xen.start

# Fast boot: override inittab to skip OpenRC services entirely.
# The default Alpine init runs dozens of services (udev, networking,
# modules, sysctl, hostname …) before reaching the "local" runlevel
# where xen.start lives.  Under QEMU software emulation this adds
# ~10 minutes of unnecessary work.  Replace it with a minimal init
# that only mounts the filesystems xencommons/xl need and then runs
# the test directly.
mkdir -p sbin
cat > etc/inittab << 'INITTAB'
::sysinit:/sbin/fast-init
tty0::respawn:/bin/sh
INITTAB

cat > sbin/fast-init << 'FASTINIT'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev
mkdir -p /dev/pts /dev/shm
mount -t devpts devpts /dev/pts
mount -t tmpfs tmpfs /dev/shm
mount -t tmpfs tmpfs /run
mkdir -p /tmp
mount -t tmpfs tmpfs /tmp
mkdir -p /var/run /var/log /var/lock
mount -t xenfs xenfs /proc/xen 2>/dev/null || true
bash /etc/local.d/xen.start
FASTINIT
chmod +x sbin/fast-init

find . | cpio -R 0:0 -H newc -o | gzip >> ../dom0-rootfs.cpio.gz
cd ../..

# XXX QEMU looks for "efi-virtio.rom" even if it is unneeded
curl -fsSLO https://github.com/qemu/qemu/raw/v5.2.0/pc-bios/efi-virtio.rom
./binaries/qemu-system-aarch64 \
   -machine virtualization=true \
   -cpu cortex-a57 -machine type=virt,gic-version=3 \
   -m 2048 -smp 2 -display none \
   -nodefaults \
   -machine dumpdtb=binaries/virt-gicv3.dtb

# XXX disable pl061 to avoid Linux crash
fdtput binaries/virt-gicv3.dtb -p -t s /pl061@9030000 status disabled

# ImageBuilder
echo 'MEMORY_START="0x40000000"
MEMORY_END="0xC0000000"

DEVICE_TREE="virt-gicv3.dtb"
XEN="xen"
DOM0_KERNEL="Image"
DOM0_RAMDISK="dom0-rootfs.cpio.gz"
XEN_CMD="console=dtuart dom0_mem=1024M console_timestamps=boot"

NUM_DOMUS=0

LOAD_CMD="tftpb"
UBOOT_SOURCE="boot.source"
UBOOT_SCRIPT="boot.scr"' > binaries/config
rm -rf imagebuilder
git clone --depth 1 https://gitlab.com/xen-project/imagebuilder.git
bash imagebuilder/scripts/uboot-script-gen -t tftp -d binaries/ -c binaries/config


# Run the test
rm -f smoke.serial
export TEST_CMD="./binaries/qemu-system-aarch64 \
    -machine virtualization=true \
    -cpu cortex-a57 -machine type=virt,gic-version=3 \
    -accel tcg,thread=multi \
    -m 2048 -monitor none -serial stdio \
    -smp 2 \
    -no-reboot \
    -device virtio-net-pci,netdev=n0 \
    -netdev user,id=n0,tftp=binaries \
    -bios /usr/lib/u-boot/qemu_arm64/u-boot.bin"

export UBOOT_CMD="virtio scan; dhcp; tftpb 0x40000000 boot.scr; source 0x40000000"
export BOOT_MSG="Latest ChangeSet: "
export TEST_LOG="smoke.serial"
export LOG_MSG="Domain-0"
export PASSED="BusyBox"

./automation/renesas-scripts/console.exp |& sed 's/\r\+$//'
