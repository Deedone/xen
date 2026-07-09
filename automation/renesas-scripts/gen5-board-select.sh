#!/bin/bash
#
# Pick a healthy Gen5 (X5H Ironhide) board and emit BOARD_ID for the gen5 HW
# test jobs to consume. Detects wedged (deaf) boards and recovers them with a
# two-tier, guarded PDU escalation, then writes BOARD_ID to a dotenv artifact.
#
# Runs in the *build* stage (needs no build artifacts) so its probe/recover
# latency overlaps the Xen/Linux/XTF builds and the cloud QEMU jobs. The gen5
# test jobs add `needs: [gen5-board-select]` and inherit BOARD_ID via dotenv.
#
# Usage:
#   gen5-board-select.sh                 (CI: BOARDS from env, default "1 2")
#   BOARDS="2" gen5-board-select.sh      (pin to a single board)
#
# Detection: a board is "alive" if, after a soft power cycle (x5hctl boot), its
# console (/dev/GEN5_CONSOLE<id>) emits an early sign-of-life string within
# PROBE_TIMEOUT. This reuses console.exp exactly as the real tests do.
#
# Recovery ladder (per board, escalating only as needed):
#   tier 0  soft  - x5hctl boot already power-cycles POWER#OFF/ON (part of probe)
#   tier 1  PDU targeted   - cut just this board's PDU outlet (ih0=1 / ih1=3),
#                            leave testrpi2 up. Resets the board AP; no collateral.
#   tier 2  PDU bench-wide - cut board outlet + testrpi2 outlet 8 (~90s dwell,
#                            RPi back on first, wait for ssh). The ONLY cure for a
#                            wedged, USB-powered control SoC. Disruptive (drops
#                            control for BOTH boards) so it is GUARDED by an flock
#                            and fires at most once per run.
#
# Power/control planes (see automation/renesas-scripts/gen5-{xtf,zephyr}.sh):
#   - Wall power (hard) : /usr/local/bin/board {ih0|ih1|rpi2} {on|off|status}
#                         (PDU pduboard1 @ 10.13.64.210). id 1 -> ih0, 2 -> ih1.
#   - Soft power        : ssh ${RPI_HOST} ${X5HCTL} <verb> <id>  (POWER#/MD#/I2C)
#   - Console (observe) : /dev/GEN5_CONSOLE<id> @ 1843200 (local to etest)

# NOTE: deliberately NOT `set -e`: probe/recover failures are expected and are
# handled explicitly via return codes.
set -u -o pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BOARDS="${BOARDS:-1 2}"

# RECOVER=0 -> probe only, skip the PDU recovery tiers (safe non-disruptive
# diagnostics on the shared bench). Default 1 = full detect + recover.
RECOVER="${RECOVER:-1}"

RPI_HOST="${RPI_HOST:-testrpi2}"
X5HCTL="${X5HCTL:-/home/testrpi2/bin/x5hctl}"
BOARD_CTL="${BOARD_CTL:-/usr/local/bin/board}"

# Early boot markers, any of which proves the board is alive: RSIP-M crypto fw
# (prints before BL31/U-Boot), the U-Boot banner, or its autoboot prompt.
SIGN_OF_LIFE="${SIGN_OF_LIFE:-I_RSIPM|RSIP-M|U-Boot 20|Hit any key to stop autoboot|Renesas Ironhide}"
PROBE_TIMEOUT="${PROBE_TIMEOUT:-60}"      # seconds to reach sign-of-life

# PDU dwell/settle timings.
PDU_OFF_DWELL="${PDU_OFF_DWELL:-5}"       # tier-1 targeted outlet off dwell
PDU_ON_SETTLE="${PDU_ON_SETTLE:-10}"      # after outlet on, before soft boot
BENCH_DWELL="${BENCH_DWELL:-90}"          # tier-2 cold dwell (control SoC reset)
RPI_WAIT="${RPI_WAIT:-180}"               # max wait for testrpi2 ssh to return

STATE_DIR="${STATE_DIR:-/tmp/gen5-bench-state}"
BENCH_LOCK="${BENCH_LOCK:-${STATE_DIR}/bench.lock}"
# Persistent per-board health history lives on the runner ($HOME survives across
# pipelines on the shell executor); per-run debug serials go to the workspace so
# the job captures them as artifacts. Both absolute so they survive the cd below.
STATS_DIR="${GEN5_STATS_DIR:-${HOME}/gen5-ci-stats}"
STATS_FILE="${STATS_DIR}/board-health.jsonl"
SERIAL_DIR="${SERIAL_DIR:-${CI_PROJECT_DIR:-${PWD}}}"
# The dotenv artifact is resolved relative to CI_PROJECT_DIR.
DOTENV="${DOTENV:-${CI_PROJECT_DIR:-${PWD}}/build.env}"

mkdir -p "${STATE_DIR}" "${STATS_DIR}" 2>/dev/null || true

cd "$(dirname "$0")"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
# Soft control on testrpi2 over ssh. Args: <verb> <board-id>.
x5h() { ssh -o BatchMode=yes -o ConnectTimeout=15 "${RPI_HOST}" "${X5HCTL}" "$1" "$2"; }

# PDU wall power. Args: <ih0|ih1|rpi2> <on|off|status>.
pdu() { "${BOARD_CTL}" "$1" "$2"; }

# id 1/2 -> PDU board name ih0/ih1.
board_name() { case "$1" in 1) echo ih0 ;; 2) echo ih1 ;; *) echo "ih$(( $1 - 1 ))" ;; esac; }

# Append one JSON record to the persistent per-board health log.
# Args: <board> <phase> <result> [boot_s]
log_stat() {
    local board="$1" phase="$2" result="$3" boot_s="${4:-null}"
    printf '{"ts":"%s","pipeline":"%s","job":"%s","board":"%s","phase":"%s","result":"%s","boot_s":%s}\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${CI_PIPELINE_ID:-local}" "${CI_JOB_ID:-local}" \
        "${board}" "${phase}" "${result}" "${boot_s}" >> "${STATS_FILE}" 2>/dev/null || true
}

# Probe one board for sign-of-life. Includes the soft power cycle (x5hctl boot).
# Returns 0 if alive, 1 if deaf/timeout, 2 if the control path (ssh/x5hctl) failed.
# Args: <board-id> <phase-tag-for-stats>
probe_board() {
    local id="$1" phase="${2:-probe}"
    local dev="/dev/GEN5_CONSOLE${id}"
    local t0 t1

    # Clear any stale picocom holding this console from a previous probe.
    pkill -u "$(id -u)" -f "picocom.*GEN5_CONSOLE${id}\b" 2>/dev/null || true

    x5h ctrl_conf "${id}" || true
    if ! x5h boot "${id}"; then
        echo "board ${id}: control path (x5hctl) failed" >&2
        log_stat "${id}" "${phase}" "ctrl_fail"
        return 2
    fi

    t0=$(date +%s)
    TEST_CMD="picocom -b 1843200 --imap lfcrlf ${dev}" \
    TEST_LOG="${SERIAL_DIR}/probe-b${id}.serial" \
    PASSED="${SIGN_OF_LIFE}" \
    TEST_TIMEOUT="${PROBE_TIMEOUT}" \
    TEST_TIMEOUT_OVERRIDE="${PROBE_TIMEOUT}" \
    ./console.exp >/dev/null 2>&1
    local rc=$?
    t1=$(date +%s)

    # Leave the board powered off after probing; the test job re-boots it anyway.
    x5h off "${id}" || true

    if [ "${rc}" -eq 0 ]; then
        echo "board ${id}: ALIVE (sign-of-life in $((t1 - t0))s)"
        log_stat "${id}" "${phase}" "alive" "$((t1 - t0))"
        return 0
    fi
    echo "board ${id}: DEAF (no sign-of-life within ${PROBE_TIMEOUT}s)"
    log_stat "${id}" "${phase}" "deaf"
    return 1
}

# tier 1: targeted PDU cut of a single board's own outlet (leave testrpi2 up).
pdu_targeted_recover() {
    local id="$1" name
    name="$(board_name "${id}")"
    echo "board ${id}: tier-1 targeted PDU cut (${name})"
    log_stat "${id}" "pdu_targeted" "attempt"
    pdu "${name}" off || true
    sleep "${PDU_OFF_DWELL}"
    pdu "${name}" on || true
    sleep "${PDU_ON_SETTLE}"
}

# tier 2: bench-wide cold cycle (board + testrpi2 outlet 8). Guarded by an
# EXCLUSIVE flock: the gen5 test jobs hold this same lock SHARED while running
# (see test/gen5.yaml), so this exclusive, non-blocking acquire fails and skips
# whenever any board is mid-test -- we never stomp a live run. Recovers every
# board in BOARDS at once; returns 0 if at least one comes back alive.
pdu_benchwide_recover() {
    exec {lockfd}>"${BENCH_LOCK}" 2>/dev/null || { echo "cannot open bench lock" >&2; return 1; }
    if ! flock -x -n "${lockfd}"; then
        echo "bench busy (a gen5 test holds the shared lock): skipping bench-wide recovery" >&2
        return 1
    fi

    echo "tier-2 bench-wide cold cycle: boards ${BOARDS} + testrpi2 (outlet 8)"
    for id in ${BOARDS}; do log_stat "${id}" "pdu_benchwide" "attempt"; done

    # Cut every board outlet, then the RPi that USB-powers the control SoCs.
    for id in ${BOARDS}; do pdu "$(board_name "${id}")" off || true; done
    pdu rpi2 off || true
    sleep "${BENCH_DWELL}"

    # RPi back first so the control SoCs have USB power when the boards come up.
    pdu rpi2 on || true
    echo "waiting up to ${RPI_WAIT}s for testrpi2 control to return..."
    # Probe liveness with an allowed x5hctl verb (x5h status), NOT `ssh testrpi2
    # true`: the CI key is pinned to a forced command that only permits
    # `x5hctl <verb> <board>` and would reject a bare `true`.
    local first_board waited=0
    first_board=$(echo "${BOARDS}" | awk '{print $1}')
    until x5h status "${first_board}" >/dev/null 2>&1; do
        sleep 5; waited=$((waited + 5))
        if [ "${waited}" -ge "${RPI_WAIT}" ]; then
            echo "testrpi2 did not come back within ${RPI_WAIT}s" >&2
            for id in ${BOARDS}; do log_stat "${id}" "pdu_benchwide" "rpi_dead"; done
            flock -u "${lockfd}"; return 1
        fi
    done
    for id in ${BOARDS}; do pdu "$(board_name "${id}")" on || true; done

    # Re-probe every board; first alive wins.
    local alive=""
    for id in ${BOARDS}; do
        x5h ctrl_conf "${id}" || true
        if probe_board "${id}" "reprobe_benchwide"; then alive="${id}"; break; fi
    done
    flock -u "${lockfd}"
    [ -n "${alive}" ] && { SELECTED="${alive}"; return 0; }
    return 1
}

# ---------------------------------------------------------------------------
# Selection
# ---------------------------------------------------------------------------
SELECTED=""

echo "=== gen5-board-select: candidate boards: ${BOARDS} ==="

# tier 0: probe each board (probe includes the soft power cycle).
for id in ${BOARDS}; do
    if probe_board "${id}" "probe"; then SELECTED="${id}"; break; fi
done

# tier 1: targeted PDU recovery for boards still deaf.
if [ -z "${SELECTED}" ] && [ "${RECOVER}" != "0" ]; then
    for id in ${BOARDS}; do
        pdu_targeted_recover "${id}"
        if probe_board "${id}" "reprobe_targeted"; then SELECTED="${id}"; break; fi
    done
fi

# tier 2: guarded bench-wide cold cycle (self-skips via flock if a test is live).
if [ -z "${SELECTED}" ] && [ "${RECOVER}" != "0" ]; then
    pdu_benchwide_recover || true
fi

if [ -z "${SELECTED}" ] && [ "${RECOVER}" = "0" ]; then
    echo "(RECOVER=0: probe-only mode, recovery tiers skipped)" >&2
fi

# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
if [ -z "${SELECTED}" ]; then
    echo "=== gen5-board-select: NO HEALTHY BOARD (all of: ${BOARDS} deaf) ===" >&2
    for id in ${BOARDS}; do log_stat "${id}" "select" "dead"; done
    exit 1
fi

echo "=== gen5-board-select: SELECTED board ${SELECTED} ==="
log_stat "${SELECTED}" "select" "selected"
echo "BOARD_ID=${SELECTED}" > "${DOTENV}"
echo "wrote ${DOTENV}: BOARD_ID=${SELECTED}"
