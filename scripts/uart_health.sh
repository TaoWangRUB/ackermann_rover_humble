#!/usr/bin/env bash
# uart_health.sh — quantify UART byte/message loss for the PX4 ↔ companion link.
#
# Background: Auterion/px4-ros2-interface-lib#165 traced "mode flagged invalid"
# aging at 921600+ baud to UART byte loss when CRTSCTS is not enabled.
# Detecting that loss depends on what the kernel driver exposes:
#
#   * 8250 / FTDI / CP2102 → TIOCGICOUNT counts frame/overrun/parity errors.
#   * CH340 (vendor 1a86)  → driver has no error counters anywhere. Application-
#                            layer measurement is the only option.
#
# Modes:
#   --kernel  (default)  Read serial_icounter_struct via TIOCGICOUNT before and
#                        after a soak. Fails with ENOTTY on CH340 — that's
#                        diagnostic in itself ("this hardware can't tell you").
#   --xrce               Sample the XRCE-DDS arming-check req/reply rates over
#                        the soak. Expected reply Hz = N_modes × request Hz;
#                        the shortfall is your protocol-level loss rate.
#                        Requires MicroXRCEAgent + px4_bringup to be running.
#
# Usage: ./scripts/uart_health.sh [--kernel|--xrce] [PORT] [DURATION_S]
# Default: --kernel /dev/ttyUSB0 60

set -euo pipefail

MODE="kernel"
case "${1:-}" in
  --kernel) MODE="kernel"; shift ;;
  --xrce)   MODE="xrce";   shift ;;
  -h|--help)
    sed -n '2,/^set /p' "$0" | sed 's/^# \?//'; exit 0 ;;
esac

PORT="${1:-/dev/ttyUSB0}"
DURATION="${2:-60}"

run_kernel() {
  if [[ ! -e "${PORT}" ]]; then
    echo "ERROR: ${PORT} not found" >&2; exit 2
  fi

  snapshot() {
    python3 - "$1" <<'PY'
import fcntl, struct, sys, os, errno
TIOCGICOUNT = 0x545D
try:
    fd = os.open(sys.argv[1], os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
except OSError as e:
    print(f"ERR:open:{e.errno}"); sys.exit(0)
buf = bytearray(80)
try:
    fcntl.ioctl(fd, TIOCGICOUNT, buf)
except OSError as e:
    os.close(fd)
    print(f"ERR:ioctl:{e.errno}"); sys.exit(0)
os.close(fd)
fields = struct.unpack('11i', buf[:44])
print(" ".join(str(x) for x in fields))
PY
  }

  first=$(snapshot "${PORT}")
  if [[ "${first}" == ERR:* ]]; then
    case "${first}" in
      ERR:ioctl:25|ERR:ioctl:22)
        cat <<EOF
ERROR: ${PORT} driver does not support TIOCGICOUNT (errno=${first##*:}).
This is normal for CH340 (1a86) USB-UART adapters — the chip and the ch341
kernel driver simply don't track byte-level errors. There is no way to
measure raw byte loss on this hardware.

Options:
  1. Switch detection layer: re-run with --xrce to measure XRCE-DDS protocol
     loss (proves byte loss exists but not the rate). Requires MicroXRCEAgent
     and px4_bringup to be running.
  2. Switch USB-UART chip: FT232 (FTDI), CP2102 (Silicon Labs), or CH9102
     all support TIOCGICOUNT. CH9102 is pin-compatible with CH340 boards.
  3. Use PX4 'serial_test -d /dev/ttyS2 -b 921600' from the FMU nsh console
     with uxrce_dds_client stopped, and a raw reader on the host. Gives a
     ground-truth byte-loss rate at the cost of bringing the link down.
EOF
        exit 3 ;;
      *)
        echo "ERROR: snapshot failed: ${first}" >&2; exit 4 ;;
    esac
  fi

  read _cts0 _dsr0 _rng0 _dcd0 rx0 tx0 fe0 oe0 pe0 brk0 bo0 <<<"${first}"
  t0=$(date +%s.%N)
  echo "Baseline @ $(date +%H:%M:%S)  rx=${rx0} tx=${tx0} frame=${fe0} overrun=${oe0} parity=${pe0} brk=${brk0} buf_overrun=${bo0}"
  echo "Sampling ${PORT} for ${DURATION}s..."
  sleep "${DURATION}"
  read _cts1 _dsr1 _rng1 _dcd1 rx1 tx1 fe1 oe1 pe1 brk1 bo1 < <(snapshot "${PORT}")
  t1=$(date +%s.%N)
  echo "After     @ $(date +%H:%M:%S)  rx=${rx1} tx=${tx1} frame=${fe1} overrun=${oe1} parity=${pe1} brk=${brk1} buf_overrun=${bo1}"
  echo

  python3 - <<PY
elapsed = ${t1} - ${t0}
drx, dtx = ${rx1}-${rx0}, ${tx1}-${tx0}
dfe, doe, dpe = ${fe1}-${fe0}, ${oe1}-${oe0}, ${pe1}-${pe0}
dbrk, dbo = ${brk1}-${brk0}, ${bo1}-${bo0}
errs = dfe + doe + dpe + dbo
print(f"Deltas over {elapsed:.1f}s:")
print(f"  rx        : {drx:>8} bytes  ({drx/elapsed:>7.0f} B/s)")
print(f"  tx        : {dtx:>8} bytes  ({dtx/elapsed:>7.0f} B/s)")
print(f"  frame err : {dfe:>8} events ({dfe/elapsed:>7.3f}/s)")
print(f"  overrun   : {doe:>8} events ({doe/elapsed:>7.3f}/s)")
print(f"  parity err: {dpe:>8}")
print(f"  break     : {dbrk:>8}")
print(f"  buf ovr   : {dbo:>8}")
print()
if errs == 0:
    print("Verdict: clean — no kernel-level UART errors.")
    print("  If XRCE-DDS still drops messages (mode aging), the loss is above")
    print("  the kernel: agent buffer, uxrce_dds_client buffer, or task starvation.")
    print("  Re-run with --xrce to confirm.")
    raise SystemExit(0)
print(f"Verdict: DIRTY — {errs} error events / {elapsed:.0f}s = {errs/elapsed:.3f}/s.")
print("  Apply CRTSCTS hardware flow control (see issue Auterion/px4-ros2-interface-lib#165).")
raise SystemExit(1)
PY
}

run_xrce() {
  CONTAINER="${CONTAINER:-jazzy_slam_x86_64}"

  if ! docker exec "${CONTAINER}" true 2>/dev/null; then
    echo "ERROR: docker container '${CONTAINER}' not running" >&2; exit 2
  fi

  echo "Sampling XRCE arming-check loop for ${DURATION}s (req from PX4 vs reply from modes)..."
  RESULT=$(docker exec "${CONTAINER}" bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    timeout ${DURATION} ros2 topic hz /fmu/out/arming_check_request_v1 2>&1 | tail -3
    echo '---SEP---'
    timeout ${DURATION} ros2 topic hz /fmu/in/arming_check_reply_v1 2>&1 | tail -3
    echo '---SEP---'
    ros2 topic info /fmu/in/arming_check_reply_v1 2>&1 | grep -i 'Publisher count' || echo 'Publisher count: 0'
  " 2>&1 || true)

  echo "${RESULT}"
  echo

  python3 - <<PY
import re
text = """${RESULT}"""
parts = text.split('---SEP---')
def avg(s):
    m = re.search(r'average rate:\s*([0-9.]+)', s)
    return float(m.group(1)) if m else None
req_hz = avg(parts[0]) if len(parts) > 0 else None
rep_hz = avg(parts[1]) if len(parts) > 1 else None
pub = re.search(r'Publisher count:\s*(\d+)', parts[2] if len(parts) > 2 else '')
n_modes = int(pub.group(1)) if pub else 0

if req_hz is None or rep_hz is None:
    print("Verdict: could not measure — is px4_bringup running and PX4 connected?")
    raise SystemExit(4)

expected = req_hz * n_modes
loss = max(0.0, (expected - rep_hz) / expected * 100) if expected > 0 else 0.0
print(f"arming_check_request (PX4 → bridge): {req_hz:.2f} Hz")
print(f"arming_check_reply   (bridge → PX4): {rep_hz:.2f} Hz")
print(f"reply publishers (modes registered): {n_modes}")
print(f"expected reply rate (req × modes):   {expected:.2f} Hz")
print(f"reply loss rate:                     {loss:.1f}%")
print()
if loss < 1.0:
    print("Verdict: clean — modes are replying to ~every request. Aging-out, if any,")
    print("  is caused by something other than serial loss.")
    raise SystemExit(0)
elif loss < 10.0:
    print("Verdict: minor loss — modes may age out occasionally under heavy traffic.")
    raise SystemExit(1)
else:
    print("Verdict: SIGNIFICANT loss — modes will age out repeatedly.")
    print("  Fix: hardware flow control on the UART link (see issue #165).")
    raise SystemExit(1)
PY
}

case "${MODE}" in
  kernel) run_kernel ;;
  xrce)   run_xrce ;;
esac
