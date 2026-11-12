#!/usr/bin/env bash
# Stop every robot process and prove the serial ports are free.
#
# Why this exists: `pkill -f 'buddy_|ros2 launch'` matches full command lines,
# INCLUDING the shell running the pkill (its argv contains those strings), so it
# kills itself before finishing and silently leaves the robot running. That left
# two and three full stacks racing on one tty on separate occasions, which
# presents as lidar Check Sum errors, impossible /odom jumps, and nonsense from
# the ESP32 - and wasted a lot of debugging.
#
# The fix is the bracket trick: the regex [b]uddy_ matches "buddy_" but this
# file's own command line contains "[b]uddy_", which it does not match.
set -u
PAT='[y]dlidar|[b]uddy_|[f]oxglove_bridge|[r]obot_state_publisher|[s]tatic_transform_publisher|[r]os2 launch|[s]lam_toolbox'

pkill -f "$PAT" 2>/dev/null || true
sleep 3
# second pass for anything that ignored SIGTERM
pgrep -f "$PAT" 2>/dev/null | xargs -r kill -9 2>/dev/null || true
sleep 1

left=$(pgrep -f "$PAT" 2>/dev/null | wc -l)
echo "robot processes remaining: $left"
for d in /dev/ttyUSB0 /dev/ttyUSB1; do
  [ -e "$d" ] || continue
  h=$(fuser "$d" 2>&1 | tr -d '\n')
  echo "  $d holders:${h:- none}"
done
[ "$left" -eq 0 ] || { echo "WARNING: processes still alive"; exit 1; }
