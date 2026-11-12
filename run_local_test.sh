#!/bin/bash
# Run the fake Husky and the health monitor together, entirely on this computer.
#
#   ~/health_ws/src/husky_health_monitor/run_local_test.sh                     # healthy robot
#   ~/health_ws/src/husky_health_monitor/run_local_test.sh gps_1,low_battery   # with faults
#
# Then open http://localhost:8088. Ctrl-C stops both.
# Faults: low_battery estop gps_1 no_fix rtk_float slow_imu imu_zeros rear_camera joystick ps4
#         tilt diagnostics
# To fail a systemd service as well:
#   HUSKY_HEALTH_FAILED_SERVICES=mors@atak_bridge.service ~/health_ws/src/husky_health_monitor/run_local_test.sh

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Cuts this terminal off from the real Husky; the mock must never reach the robot
source "$here/local_test_env.sh" || exit 1

prefix="$(ros2 pkg prefix husky_health_monitor 2>/dev/null)"
if [ -z "$prefix" ]; then
    echo "husky_health_monitor isn't built yet. Run: cd ~/health_ws && colcon build"
    exit 1
fi

# Start the mock directly (not through `ros2 run`) so it can be stopped reliably on exit
"$prefix/lib/husky_health_monitor/mock_husky" ${1:+--faults "$1"} &
mock=$!
trap 'kill -TERM "$mock" 2>/dev/null; wait "$mock" 2>/dev/null' EXIT

echo "Dashboard: http://localhost:8088"
echo "Terminal view (another terminal): python3 $here/husky_health_monitor/cli.py --watch"
echo "Press Ctrl-C to stop."
ros2 launch husky_health_monitor health_monitor.launch.py
