#!/bin/bash
#
# Run a Zephyr-based Xen test on a physical Gen5 (X5H Ironhide) board.
#
# Usage:
#   gen5-zephyr.sh <test-name>              (CI: BOARD_ID from env)
#   gen5-zephyr.sh <board-id> <test-name>   (direct / manual)
#
#   board-id:  1, 2, ... (maps to /dev/GEN5_CONSOLE<N> on etest; board control
#              is on the RPi via x5hctl); may instead be passed via the BOARD_ID
#              environment variable (CI sets BOARD_ID=1)
#   test-name: e.g. hyp-domctl-pausedomain
#
# Prerequisites:
#   - binaries/xen      : Xen binary from xen-atfe-arm64 CI artifact
#   - binaries/dom0.bin : Zephyr Dom0 from zephyr-gen5-build-<test-name> artifact
#                         Built on aws-arm64 with:
#                           west build -b qemu_cortex_a53 \
#                             -S xen_dom0 -S xen_dom0_overlay \
#                             -S xen_dom0_overlay_gen5_ironhide \
#                             zephyr_tests/testcases/<test-name>
#   - imagebuilder      : cloned locally or present in CWD
#   - mkimage, dtc      : u-boot-tools, device-tree-compiler
#   - TFTP_BASE writable by CI runner user (e.g. /srv/tftp/ci)
#   - ssh access from the runner to ${RPI_HOST} (default testrpi2), which hosts
#     the board CONTROL ports and /home/testrpi2/bin/x5hctl (the old local x5h_*
#     scripts moved there). The console stays local to etest as
#     /dev/GEN5_CONSOLE<N>.

set -ex -o pipefail

# BOARD_ID may come from the environment (the CI job sets BOARD_ID=1) or as the
# first positional argument for direct/manual runs.
if [ -n "${BOARD_ID:-}" ]; then
    if [ $# -lt 1 ]; then
        echo "Usage: $(basename "$0") TEST-NAME  (BOARD_ID=${BOARD_ID} from env)"
        exit 1
    fi
    TEST_NAME="$1"
else
    if [ $# -lt 2 ]; then
        echo "Usage: $(basename "$0") BOARD-ID TEST-NAME"
        echo "       or set BOARD_ID in the environment and omit BOARD-ID"
        exit 1
    fi
    BOARD_ID="$1"
    TEST_NAME="$2"
fi

# Board CONTROL moved off etest onto the RPi (testrpi2): the CP2102N control
# adapters now live there as /dev/ttyUSB0/1, driven by /home/testrpi2/bin/x5hctl
# (replacing the old local x5h_* scripts). The runner reaches it over ssh; the
# RPi's x5h-ci-dispatch forced command restricts the CI key to exactly these
# calls. Override RPI_HOST / X5HCTL for a different RPi or install path.
RPI_HOST="${RPI_HOST:-testrpi2}"
X5HCTL="${X5HCTL:-/home/testrpi2/bin/x5hctl}"
x5h() { ssh -o BatchMode=yes -o ConnectTimeout=15 "${RPI_HOST}" "${X5HCTL}" "$1" "${BOARD_ID}"; }

# Power the board off even if the script exits early, so a failed run does not
# leave it running and contaminate the next job that claims this board.
trap 'x5h off || true' EXIT

export XEN_ROOT="${PWD}"
export WORKDIR="${WORKDIR:-${XEN_ROOT}/binaries}"

HW_LOG="${HW_LOG:-${XEN_ROOT}/hw.serial}"
export TEST_LOG="${HW_LOG}"
# Derive ztest suite name: strip optional hyp- prefix, hyphens -> underscores
_ztest_suite="${TEST_NAME#hyp-}"; _ztest_suite="${_ztest_suite//-/_}"
export PASSED="${PASSED:-TESTSUITE ${_ztest_suite} succeeded}"
export BOOT_MSG="${BOOT_MSG:-Latest ChangeSet: }"
export TEST_TIMEOUT="${TEST_TIMEOUT:-300}"

CONSOLE_DEV="/dev/GEN5_CONSOLE${BOARD_ID}"
: "${TFTP_BASE:?TFTP_BASE must be set (e.g. /srv/tftp/ci)}"
TFTP_ROOT="${TFTP_ROOT:-/srv/tftp}"

GEN5_BASE_DTS="${GEN5_BASE_DTS:-${XEN_ROOT}/automation/device-tree/r8a78000-ironhide-xen.dts}"

export XEN_BINARY="${XEN_BINARY:-${WORKDIR}/xen}"
# Drive the Gen5 UART by explicit node path /soc/serial@c0710000 (prefer the
# explicit path over the serial0 alias for Xen's dtuart). No maxcpus cap - the
# real DTS declares GICv3 with 32 redistributor frames.
export XEN_CMDLINE="${XEN_CMDLINE:-loglvl=all dom0_mem=128M noreboot console_timestamps=boot console=dtuart dtuart=/soc/serial@c0710000 xsm=flask flask=permissive}"

cd "$(dirname "$0")"

# ---------------------------------------------------------------------------
# 1. Verify pre-built Zephyr Dom0 artifact is present
#    The binary is produced by the zephyr-gen5-build-<test-name> CI job
#    running on aws-arm64 (Zephyr SDK required; not available on epdefrans).
# ---------------------------------------------------------------------------
if [ ! -f "${WORKDIR}/dom0.bin" ]; then
    echo "ERROR: ${WORKDIR}/dom0.bin not found - add zephyr-gen5-build-${TEST_NAME} to CI needs" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. Compile Gen5 base DTB
# ---------------------------------------------------------------------------
dtc -I dts -O dtb "${GEN5_BASE_DTS}" -o "${WORKDIR}/xen-base.dtb"

# ---------------------------------------------------------------------------
# 3. Generate boot script with fixed load addresses.
#
# Xen is loaded at 0x4b200000 so that after Xen self-relocates, the freed
# pages become xenheap and the grant table is allocated at 0x4b200000 -
# matching the hardcoded address in zephyr_tests xen_dom0_overlay_gen5_ironhide.
# Letting imagebuilder pick addresses dynamically shifts the grant table and
# breaks Zephyr's static DTS.
#
#   dom0.bin  -> 0x48400000  (Zephyr Dom0 kernel, ~72 KB)
#   xen       -> 0x4b200000  (Xen hypervisor; grant table lands here)
#   xen-base.dtb -> 0x4d000000  (above the extended region boundary 0x4b400000)
#   boot.scr  -> 0x50000000  (loaded by UBOOT_CMD before sourcing)
# ---------------------------------------------------------------------------
_rel="${TFTP_BASE#${TFTP_ROOT}}"; _rel="${_rel#/}"
TFTP_SUBDIR="${_rel:+${_rel}/}zephyr-${TEST_NAME}"

DOM0_SIZE=$(stat -L --printf=%s "${WORKDIR}/dom0.bin")

# Optional XSM/Flask policy module. Loaded at 0x49000000 (between dom0 at
# 0x48400000 and Xen at 0x4b200000). Guarded: a no-op unless a policy binary is
# present - provide binaries/xenpolicy via a CI artifact, or set XEN_POLICY_FILE.
# Without it, xsm=flask runs permissive with the built-in bootstrap policy.
if [ -n "${XEN_POLICY_FILE:-}" ] && [ -f "${XEN_POLICY_FILE}" ]; then
    cp -f "${XEN_POLICY_FILE}" "${WORKDIR}/xenpolicy"
fi
_policy_tftpb=""
_policy_fdt=""
if [ -f "${WORKDIR}/xenpolicy" ]; then
    POLICY_SIZE=$(stat -L --printf=%s "${WORKDIR}/xenpolicy")
    _policy_tftpb="tftpb 0x49000000 ${TFTP_SUBDIR}/xenpolicy"
    _policy_fdt="fdt mknod /chosen xsm-policy@49000000
fdt set /chosen/xsm-policy@49000000 compatible  \"xen,xsm-policy\" \"xen,multiboot-module\" \"multiboot,module\"
fdt set /chosen/xsm-policy@49000000 reg <0x0 0x49000000 0x0 $(printf '0x%x' "${POLICY_SIZE}") >"
fi

cat > "${WORKDIR}/boot.source" <<BSEOF
tftpb 0x48400000 ${TFTP_SUBDIR}/dom0.bin
tftpb 0x4b200000 ${TFTP_SUBDIR}/xen
tftpb 0x4d000000 ${TFTP_SUBDIR}/xen-base.dtb
${_policy_tftpb}
fdt addr 0x4d000000
fdt resize 1024
fdt set /chosen \\#address-cells <0x2>
fdt set /chosen \\#size-cells <0x2>
fdt set /chosen xen,xen-bootargs "${XEN_CMDLINE}"
fdt mknod /chosen dom0@48400000
fdt set /chosen/dom0@48400000 compatible  "xen,linux-zimage" "xen,multiboot-module" "multiboot,module"
fdt set /chosen/dom0@48400000 reg <0x0 0x48400000 0x0 $(printf '0x%x' "${DOM0_SIZE}") >
fdt set /chosen xen,dom0-bootargs "console=hvc0"
${_policy_fdt}
setenv fdt_high 0xffffffffffffffff
booti 0x4b200000 - 0x4d000000
BSEOF

mkimage -A arm64 -T script -C none -n "gen5 zephyr boot" \
    -d "${WORKDIR}/boot.source" "${WORKDIR}/boot.scr"

# ---------------------------------------------------------------------------
# 5. Deploy to TFTP
# ---------------------------------------------------------------------------
TFTP_DIR="${TFTP_BASE}/zephyr-${TEST_NAME}"
mkdir -p "${TFTP_DIR}"

cp "${XEN_BINARY}"            "${TFTP_DIR}/xen"
cp "${WORKDIR}/xen-base.dtb" "${TFTP_DIR}/xen-base.dtb"
cp "${WORKDIR}/dom0.bin"      "${TFTP_DIR}/dom0.bin"
cp "${WORKDIR}/boot.scr"      "${TFTP_DIR}/boot.scr"
[ -f "${WORKDIR}/xenpolicy" ] && cp "${WORKDIR}/xenpolicy" "${TFTP_DIR}/xenpolicy"

# ---------------------------------------------------------------------------
# 6. Power-cycle the board
#    x5h_boot = POWER#OFF + MD#26149 (normal boot mode) + POWER#ON + I2C init
# ---------------------------------------------------------------------------
x5h ctrl_conf
x5h boot

# ---------------------------------------------------------------------------
# 7. Watch console - U-Boot loads boot.scr directly from TFTP
# ---------------------------------------------------------------------------
export TEST_CMD="picocom -b 1843200 --imap lfcrlf ${CONSOLE_DEV}"
# MAC is hardware-specific per board; the IP is derived per board
# (board k -> 10.13.64.(210+k): board1=.211, board2=.212) so boards booting in
# parallel never share an address during TFTP. Add a board's MAC here to extend;
# the IP follows automatically. Override via BOARD_MAC / BOARD_IP if needed.
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
# 8. Power off
# ---------------------------------------------------------------------------
x5h off

echo "Zephyr test ${TEST_NAME} on board ${BOARD_ID}: PASSED"
