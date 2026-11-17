#!/usr/bin/env bash
# Buddy test suite. Run from anywhere; needs the workspace built and sourced.
#
#   ./run_tests.sh
#
# test_scan_utils.py is pure maths and needs nothing running. The two
# integration tests spin up the real nodes against a synthetic lidar, so they
# use their own ROS_DOMAIN_ID to stay clear of a live robot.
#
# Nodes are started with setsid and killed by process group: `ros2 run` is a
# wrapper, and killing it leaves the actual node alive. Leaked nodes from a
# previous run publish into the same domain and silently corrupt later tests,
# which is confusing enough to be worth the ceremony.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${TEST_DOMAIN_ID:-91}"
rc=0
PGIDS=()

cleanup() {
  for pgid in "${PGIDS[@]:-}"; do
    [ -n "$pgid" ] && kill -TERM -- "-$pgid" 2>/dev/null
  done
  PGIDS=()
  sleep 1
}
trap cleanup EXIT

spawn() {  # spawn <command...> -> records its process group
  setsid "$@" >/dev/null 2>&1 &
  PGIDS+=("$!")
}

run_integration() {
  local name="$1"; shift
  echo; echo "=== integration: $name ==="
  spawn ros2 run tf2_ros static_transform_publisher \
    --frame-id BuddyBaseFootprint_link --child-frame-id laser --yaw 3.14159
  sleep 2
  for n in "$@"; do spawn ros2 run buddy_description "$n"; done
  sleep 5
  python3 "$HERE/test_${name}.py" || rc=1
  cleanup
}

echo "=== unit: scan sector maths ==="
python3 "$HERE/test_scan_utils.py" || rc=1

echo; echo "=== unit: voice intents + dialogue ==="
python3 "$HERE/test_voice.py" || rc=1

echo "=== unit: IMU protocol/calibration ==="
python3 "$HERE/test_imu.py" || rc=1

echo "=== integration: serial/IMU over a fake ESP32 PTY ==="
python3 "$HERE/test_serial_imu_integration.py" || rc=1

run_integration scan_guard_integration buddy_scan_guard.py
run_integration wanderer_integration buddy_scan_guard.py buddy_wanderer.py

echo
if [ $rc -eq 0 ]; then echo "SUITE PASS"; else echo "SUITE FAIL"; fi
exit $rc
