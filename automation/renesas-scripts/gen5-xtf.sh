#!/bin/bash
#
# Run an XTF test on a physical Gen5 (X5H Ironhide) board.
#
# Usage:
#   gen5-xtf.sh <xtf-variant> <xtf-name>              (CI: BOARD_ID from env)
#   gen5-xtf.sh <board-id> <xtf-variant> <xtf-name>   (direct / manual)
#
#   board-id:    1, 2, ... (maps to /dev/GEN5_CONSOLE<N>); may instead be passed
#                via the BOARD_ID environment variable (CI sets BOARD_ID=1)
#   xtf-variant: e.g. mmu64le
#   xtf-name:    e.g. hyp-domctl-pausedomain
#
# Prerequisites:
#   - binaries/xen       : Xen binary from xen-atfe-arm64 CI artifact
#   - binaries/Image     : Linux kernel from the Linux image-export artifact
#   - binaries/domU-rootfs.cpio.gz : arm64 busybox initrd from arm64-busybox-initrd-export artifact
#   - imagebuilder       : cloned locally or present in CWD
#   - mkimage            : u-boot-tools, for recompiling boot.scr
#   - dtc                : device-tree-compiler
#   - TFTP_BASE writable by CI runner user (e.g. /srv/tftp/ci)
#   - ssh access from the runner to ${RPI_HOST} (default testrpi2), which hosts
#     the board CONTROL ports and /home/testrpi2/bin/x5hctl (the old local x5h_*
#     scripts moved there when the control adapters moved off etest). The console
#     stays local to etest as /dev/GEN5_CONSOLE<N>.
#
# XTF is built from source using include/configs/xtf-arm64-gen5-config.
# A cross-compiler for arm64 must be available (CROSS_COMPILE or in PATH).
# In CI, prefer building XTF on an aws-arm64 Docker runner and passing
# the binary as an artifact rather than cross-compiling on epdefrans.

set -ex -o pipefail

# BOARD_ID may come from the environment (the CI job sets BOARD_ID=1) or as the
# first positional argument for direct/manual runs.
# After this block the remaining args are: <xtf-variant> <xtf-name>.
if [ -n "${BOARD_ID:-}" ]; then
    if [ $# -lt 2 ]; then
        echo "Usage: $(basename "$0") XTF-VARIANT XTF-NAME  (BOARD_ID=${BOARD_ID} from env)"
        exit 1
    fi
else
    if [ $# -lt 3 ]; then
        echo "Usage: $(basename "$0") BOARD-ID XTF-VARIANT XTF-NAME"
        echo "       or set BOARD_ID in the environment and omit BOARD-ID"
        exit 1
    fi
    BOARD_ID="$1"
    shift
fi

# Board CONTROL moved off etest onto the RPi (testrpi2): the CP2102N control
# adapters now live there as /dev/ttyUSB0/1, driven by /home/testrpi2/bin/x5hctl
# (replacing the old local x5h_* scripts). The runner reaches it over ssh; the
# RPi's x5h-ci-dispatch forced command restricts the CI key to exactly these
# calls. Override RPI_HOST / X5HCTL for a different RPi or install path.
RPI_HOST="${RPI_HOST:-testrpi2}"
X5HCTL="${X5HCTL:-/home/testrpi2/bin/x5hctl}"
x5h() { ssh -o BatchMode=yes -o ConnectTimeout=15 "${RPI_HOST}" "${X5HCTL}" "$1" "${BOARD_ID}"; }

# Ensure the board is powered off even if the script exits with an error.
# set -e skips the power-off step on failure, which would leave the board
# running and contaminate the next job that claims this board.
trap 'x5h off || true' EXIT

export ARCH="arm64"
export XEN_ROOT="${PWD}"
export WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

HW_LOG="${HW_LOG:-${XEN_ROOT}/hw.serial}"
export TEST_LOG="${HW_LOG}"
export PASSED="${PASSED:-Test result: SUCCESS}"
export BOOT_MSG="${BOOT_MSG:-Latest ChangeSet: }"
export TEST_TIMEOUT="${TEST_TIMEOUT:-300}"

CONSOLE_DEV="/dev/GEN5_CONSOLE${BOARD_ID}"
: "${TFTP_BASE:?TFTP_BASE must be set (e.g. /srv/tftp/myuser)}"
TFTP_ROOT="${TFTP_ROOT:-/srv/tftp}"

echo "--- runner identity ---"
id
echo "--- TFTP_BASE: ${TFTP_BASE} ---"
ls -la "${TFTP_BASE}/.." | grep "$(basename "${TFTP_BASE}")" || true
echo "--- TFTP_BASE writable: $([ -w "${TFTP_BASE}" ] && echo yes || echo NO) ---"
echo "--- tools ---"
command -v dtc mkimage picocom || true
echo "--- board devices ---"
ls -la /dev/GEN5_CONSOLE${BOARD_ID} 2>&1 || true
echo "--- control port on ${RPI_HOST} ---"
x5h status 2>&1 || true
echo "--- end runner identity ---"

GEN5_BASE_DTS="${GEN5_BASE_DTS:-${XEN_ROOT}/automation/device-tree/r8a78000-ironhide-xen.dts}"

# XTF source settings
export XTF_SRC_CONFIG="${XTF_SRC_CONFIG:-${XEN_ROOT}/automation/renesas-scripts/include/configs/xtf-arm64-gen5-config}"
export XTF_SRC_BRANCH="${XTF_SRC_BRANCH:-safety-staging}"
export XTF_SRC_URI="${XTF_SRC_URI:-https://gitlab-ci-token:${CI_JOB_TOKEN}@gitpct.epam.com/rec-fusa/xtf.git}"
export XTF_SRC_VARIANTS="mmu64le"

export XEN_BINARY="${XEN_BINARY:-${WORKDIR}/xen}"
# Xen cmdline: drive the Gen5 UART by explicit node path /soc/serial@c0710000
# (the pre-real-DTS cmdline used an explicit /serial@... path; prefer that over
# the serial0 alias for Xen's dtuart). dom0_mem=128M matches QEMU XTF baseline.
# No maxcpus cap - the
# real DTS declares GICv3 with 32 redistributor frames, so secondary CPUs come
# up cleanly (the maxcpus=1 workaround was only needed for the wrong GICv4 DTS).
export XEN_CMDLINE="${XEN_CMDLINE:-loglvl=all dom0_mem=128M noreboot console_timestamps=boot console=dtuart dtuart=/soc/serial@c0710000 xsm=flask flask=permissive}"

XTF_NAME_ARG="$2"

cd "$(dirname "$0")"

# ---------------------------------------------------------------------------
# Source shared XTF utilities
# ---------------------------------------------------------------------------
source include/xtf-runner

# ---------------------------------------------------------------------------
# 1. DomU arm64 rootfs - pre-built on aws-arm64 via arm64-busybox-initrd-export
# ---------------------------------------------------------------------------
if [ ! -f "${WORKDIR}/domU-rootfs.cpio.gz" ]; then
    echo "ERROR: ${WORKDIR}/domU-rootfs.cpio.gz not found - add arm64-busybox-initrd-export to CI needs" >&2
    exit 1
fi

# DomU Image must be pre-built (e.g. from the Linux image-export artifact)
if [ ! -f "${WORKDIR}/Image" ]; then
    echo "ERROR: ${WORKDIR}/Image not found - add the Linux image-export job to CI needs" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. Auto-load per-test overrides (DomU type, vCPU count, Zephyr config, etc.)
#    Same mechanism as qemu-xtf.sh: source include/tests/<test-name> if present.
# ---------------------------------------------------------------------------
if [ -f "include/tests/${XTF_NAME_ARG}" ]; then
    source "include/tests/${XTF_NAME_ARG}"
fi

# ---------------------------------------------------------------------------
# 3. XTF binary - use pre-built artifact if present, otherwise build from src
#    In CI the binary comes from a dedicated aws-arm64 build job.
#    For local use a cross-compiler must be available (CROSS_COMPILE env var).
# ---------------------------------------------------------------------------
if [ -f "${WORKDIR}/xtf-test" ]; then
    export XTF_NAME="${XTF_NAME_ARG}"
    export XTF_VARIANT="$1"
    # Apply any per-test XEN_CMDLINE additions (e.g. sched=rtds)
    xtf_build_cmdline "$1" "${XTF_NAME_ARG}"
else
    xtf_build_test "$@"
    cp "${XTF_BINARY}" "${WORKDIR}/xtf-test"
fi

# ---------------------------------------------------------------------------
# 4. Compile Gen5 base DTB
# ---------------------------------------------------------------------------
dtc -I dts -O dtb "${GEN5_BASE_DTS}" -o "${WORKDIR}/xen-base.dtb"

# ---------------------------------------------------------------------------
# 5. Generate imagebuilder boot script
# ---------------------------------------------------------------------------
if [ ! -d imagebuilder ]; then
    git clone --depth 1 https://gitlab.com/xen-project/imagebuilder.git
fi

local_num_domus="${XTF_NUM_DOMUS:-1}"
local_domu_vcpus="${XTF_DOMU_VCPUS:-1}"

cat > "${WORKDIR}/ib-config" <<IBEOF
MEMORY_START="0x48000000"
MEMORY_END="0xC0000000"

XEN="xen"
XEN_CMD="${XEN_CMDLINE}"

DEVICE_TREE="xen-base.dtb"

DOM0_KERNEL="xtf-test"

NUM_DOMUS=${local_num_domus}

LOAD_CMD="tftpb"
UBOOT_SOURCE="boot.source"
UBOOT_SCRIPT="boot.scr"
IBEOF

for i in $(seq 0 $((local_num_domus - 1))); do
    cat >> "${WORKDIR}/ib-config" <<DOMUEOF
DOMU_KERNEL[$i]="Image"
DOMU_RAMDISK[$i]="domU-rootfs.cpio.gz"
DOMU_MEM[$i]=128
DOMU_VCPUS[$i]=${local_domu_vcpus}
DOMUEOF
done

# XSM/Flask policy (optional). Provide binaries/xenpolicy via a CI artifact to
# load a real policy module; otherwise xsm=flask runs permissive with the
# built-in bootstrap policy. Override the source path with XEN_POLICY_FILE.
if [ -n "${XEN_POLICY_FILE:-}" ] && [ -f "${XEN_POLICY_FILE}" ]; then
    cp -f "${XEN_POLICY_FILE}" "${WORKDIR}/xenpolicy"
fi
if [ -f "${WORKDIR}/xenpolicy" ]; then
    echo 'XEN_POLICY="xenpolicy"' >> "${WORKDIR}/ib-config"
fi

bash -x imagebuilder/scripts/uboot-script-gen \
    -t tftp \
    -d "${WORKDIR}/" \
    -c "${WORKDIR}/ib-config"

# ---------------------------------------------------------------------------
# 6. Patch boot.source: prefix all tftpb filenames with TFTP subdir
#    imagebuilder generates bare filenames; TFTP server expects a subdir path.
# ---------------------------------------------------------------------------
_rel="${TFTP_BASE#${TFTP_ROOT}}"; _rel="${_rel#/}"
TFTP_SUBDIR="${_rel:+${_rel}/}xtf-${XTF_NAME}"

sed -i -E \
    "s|tftpb (0x[0-9a-fA-F]+) ([^/[:space:]])|tftpb \1 ${TFTP_SUBDIR}/\2|g" \
    "${WORKDIR}/boot.source"

# Recompile the patched source into a U-Boot script image
mkimage -A arm64 -T script -C none -n "gen5 xtf boot" \
    -d "${WORKDIR}/boot.source" "${WORKDIR}/boot.scr"

# ---------------------------------------------------------------------------
# 7. Deploy to TFTP
# ---------------------------------------------------------------------------
TFTP_DIR="${TFTP_BASE}/xtf-${XTF_NAME}"
mkdir -p "${TFTP_DIR}"

cp "${XEN_BINARY}"                  "${TFTP_DIR}/xen"
cp "${WORKDIR}/xen-base.dtb"        "${TFTP_DIR}/xen-base.dtb"
cp "${WORKDIR}/xtf-test"            "${TFTP_DIR}/xtf-test"
cp "${WORKDIR}/Image"               "${TFTP_DIR}/Image"
cp "${WORKDIR}/domU-rootfs.cpio.gz" "${TFTP_DIR}/domU-rootfs.cpio.gz"
cp "${WORKDIR}/boot.scr"            "${TFTP_DIR}/boot.scr"
[ -f "${WORKDIR}/xenpolicy" ] && cp "${WORKDIR}/xenpolicy" "${TFTP_DIR}/xenpolicy"

# ---------------------------------------------------------------------------
# 8. Power-cycle the board
#    x5h_boot = POWER#OFF + MD#26149 (normal boot mode) + POWER#ON + I2C init
# ---------------------------------------------------------------------------
x5h ctrl_conf
x5h boot

# ---------------------------------------------------------------------------
# 9. Watch console - U-Boot loads boot.scr directly from TFTP
#    No pre-registered env var needed; boot.scr is self-contained.
# ---------------------------------------------------------------------------
export TEST_CMD="picocom -b 1843200 --imap lfcrlf ${CONSOLE_DEV}"
# Board has no persistent U-Boot env (bad CRC); inject network vars every boot.
# MAC is hardware-specific per board (Renesas OUI 74:90:50:48:DF:xx). The IP is
# derived per board (board k -> 10.13.64.(210+k): board1=.211, board2=.212) so
# that boards booting in parallel never share an address during TFTP. To add a
# board, add its MAC here; the IP follows automatically. Override via BOARD_MAC
# / BOARD_IP if needed.
# UBOOT_NET_CMD is sent first so the combined command stays within U-Boot's
# ~137-character input buffer limit (all-in-one string gets truncated).
case "${BOARD_ID}" in
    1) _default_mac="74:90:50:48:df:64" ;;
    2) _default_mac="74:90:50:48:df:14" ;;
    *) _default_mac="" ;;
esac
BOARD_IP="${BOARD_IP:-10.13.64.$((210 + BOARD_ID))}"
SERVER_IP="${SERVER_IP:-10.13.64.194}"
BOARD_MAC="${BOARD_MAC:-${_default_mac}}"
export UBOOT_NET_CMD="setenv ipaddr ${BOARD_IP}; setenv serverip ${SERVER_IP}; setenv ethaddr ${BOARD_MAC}"
export UBOOT_CMD="tftpb 0x50000000 ${TFTP_SUBDIR}/boot.scr; source 0x50000000"

# Drive the board console with expect (console.exp spawns picocom at 1843200,
# interrupts U-Boot autoboot, sends UBOOT_NET_CMD then UBOOT_CMD, and matches
# BOOT_MSG / PASSED).
./console.exp |& sed 's/\r\+$//'

# ---------------------------------------------------------------------------
# 10. Power off
# ---------------------------------------------------------------------------
x5h off

echo "XTF test ${XTF_NAME} (${XTF_VARIANT}) on board ${BOARD_ID}: PASSED"
