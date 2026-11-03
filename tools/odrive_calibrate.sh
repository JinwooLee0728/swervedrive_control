#!/usr/bin/env bash
# Calibrate one ODrive axis over CAN (CAN-Simple, firmware 0.6.x), save the
# result on the ODrive, and verify it can enter closed loop. No USB or
# odrivetool needed.
#
#   tools/odrive_calibrate.sh --status [-i can0] [node_id]   # read-only
#   tools/odrive_calibrate.sh          [-i can0] [node_id]   # calibrate
#
# node_id defaults to 0 (robot1 axis0 in hardware.yaml).
#
# !! The motor BEEPS and TURNS THE WHEEL BY ITSELF for 10-20 s. Lift the wheel
# !! off the ground and keep hands/cables clear. Stop the robot launch first.
#
# Steps: clear errors -> FULL_CALIBRATION_SEQUENCE (state 3) -> wait ->
# save_configuration (Reboot action 1; the ODrive reboots) -> closed-loop
# check at input_vel 0 -> back to IDLE. CTRL-C at any point sends IDLE.

set -euo pipefail

IFACE=can0
STATUS_ONLY=0
NODE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i) IFACE="$2"; shift 2 ;;
    --status) STATUS_ONLY=1; shift ;;
    -h|--help) sed -n 2,17p "$0"; exit 0 ;;
    *) NODE="$1"; shift ;;
  esac
done
[[ "$NODE" =~ ^[0-9]+$ ]] && (( NODE <= 63 )) || { echo "bad node_id: $NODE" >&2; exit 2; }
command -v cansend >/dev/null && command -v candump >/dev/null ||
  { echo "needs can-utils (sudo apt install can-utils)" >&2; exit 2; }

RESULTS=(SUCCESS BUSY CANCELLED DISARMED NO_RESPONSE POLE_PAIR_CPR_MISMATCH
  PHASE_RESISTANCE_OUT_OF_RANGE PHASE_INDUCTANCE_OUT_OF_RANGE UNBALANCED_PHASES
  INVALID_MOTOR_TYPE ILLEGAL_HALL_STATE TIMEOUT HOMING_WITHOUT_ENDSTOP
  INVALID_STATE NOT_CALIBRATED NOT_CONVERGING)
STATES=([0]=UNDEFINED [1]=IDLE [2]=STARTUP_SEQUENCE [3]=FULL_CALIBRATION_SEQUENCE
  [4]=MOTOR_CALIBRATION [6]=ENCODER_INDEX_SEARCH [7]=ENCODER_OFFSET_CALIBRATION
  [8]=CLOSED_LOOP_CONTROL [11]=HOMING [12]=ENCODER_HALL_POLARITY_CALIBRATION
  [13]=ENCODER_HALL_PHASE_CALIBRATION [14]=ANTICOGGING_CALIBRATION)

id() { printf '%03X' $(( NODE << 5 | $1 )); }
send() { cansend "$IFACE" "$(id "$1")#$2"; }
le32() { echo "${1:6:2}${1:4:2}${1:2:2}${1:0:2}"; }   # 8 hex chars, little endian

# Heartbeat (0x001) -> globals ERR (hex), STATE, RESULT. Returns 1 on timeout.
heartbeat() {
  local line data
  line=$(candump -L -n 1 -T 1500 "$IFACE,$(id 0x01):7FF" 2>/dev/null) || true
  [[ -n "$line" ]] || return 1
  data=${line##*#}
  ERR=$(le32 "${data:0:8}")
  STATE=$((16#${data:8:2}))
  RESULT=$((16#${data:10:2}))
}

show() {
  printf '  node %d: state=%s(%d) procedure_result=%s(%d) active_errors=0x%s\n' \
    "$NODE" "${STATES[$STATE]:-?}" "$STATE" "${RESULTS[$RESULT]:-?}" "$RESULT" "$ERR"
}

idle() { send 0x07 01000000 2>/dev/null || true; }

echo "ODrive node $NODE on $IFACE"
heartbeat || { echo "  no heartbeat from node $NODE -- powered? right node_id/bus?" >&2; exit 1; }
show
if (( STATUS_ONLY )); then exit 0; fi

if pgrep -f 'swerve_odrive_bridge|odrive_can_node' >/dev/null; then
  echo "A robot launch (swerve_odrive_bridge / odrive_can_node) is running." >&2
  echo "Stop it first; it would fight this script for the axis." >&2
  exit 1
fi

cat <<EOF

  The motor on node $NODE will beep and SPIN THE WHEEL BY ITSELF for 10-20 s.
  Is that wheel off the ground, with hands and cables clear?
EOF
read -r -p "  Type 'yes' to calibrate: " ans
[[ "$ans" == yes ]] || { echo "aborted"; exit 1; }

trap 'echo; echo "interrupted -> IDLE"; idle; exit 130' INT TERM

echo "Calibrating..."
send 0x18 00                 # Clear_Errors
send 0x07 03000000           # Set_Axis_State FULL_CALIBRATION_SEQUENCE
sleep 1.5
deadline=$((SECONDS + 60))
while :; do
  heartbeat || { echo "  lost heartbeat" >&2; idle; exit 1; }
  if (( STATE == 1 && RESULT != 1 )); then break; fi
  printf '\r  %-36s' "${STATES[$STATE]:-state $STATE}"
  (( SECONDS < deadline )) || { echo; echo "  timed out after 60 s" >&2; idle; exit 1; }
  sleep 0.5
done
echo; show
if (( RESULT != 0 )) || [[ "$ERR" != 00000000 ]]; then
  echo "Calibration FAILED (${RESULTS[$RESULT]:-?}). Nothing saved." >&2
  echo "PHASE_*/UNBALANCED -> check motor wiring; NOT_CONVERGING/CPR mismatch ->" >&2
  echo "check the encoder; wheel touching the ground -> lift it." >&2
  exit 1
fi
echo "Calibration OK."

read -r -p "Save to the ODrive's flash now? It reboots (~3 s). [yes/no] " ans
[[ "$ans" == yes ]] || { echo "Not saved: calibration is lost on power-off."; exit 0; }
send 0x16 01                 # Reboot, action 1 = save_configuration
sleep 3
for _ in 1 2 3 4 5; do heartbeat && break; sleep 1; done
heartbeat || { echo "  no heartbeat after reboot" >&2; exit 1; }
echo "Saved; ODrive back up."; show

echo "Checking it can enter closed loop (input_vel 0, the wheel should not turn)..."
send 0x0B 0200000001000000   # Set_Controller_Mode: VELOCITY, PASSTHROUGH
send 0x0D 0000000000000000   # Set_Input_Vel 0, torque ff 0
send 0x18 00
send 0x07 08000000           # CLOSED_LOOP_CONTROL
sleep 1.2
heartbeat || true
state_cl=$STATE; result_cl=$RESULT
idle
sleep 0.3
heartbeat && show
if (( state_cl == 8 )); then
  echo "PASS: node $NODE entered CLOSED_LOOP_CONTROL after reboot; back in IDLE."
else
  echo "FAIL: closed loop refused after reboot (${RESULTS[$result_cl]:-?})." >&2
  if (( result_cl == 14 )); then
    echo "The calibration was not kept across the reboot. Set" >&2
    echo "motor.config.pre_calibrated / encoder pre_calibrated = True with" >&2
    echo "odrivetool or the ODrive GUI over USB, then save_configuration()." >&2
  fi
  exit 1
fi
