# Source this in each terminal before testing with mock_husky:
#   source ~/health_ws/src/husky_health_monitor/local_test_env.sh
#
# ~/.bashrc points every new terminal at the Husky's discovery server. This cuts the
# current terminal off from the robot and keeps ROS traffic on this computer, so the
# mock's fake topics can't reach the real Husky. Open a new terminal to talk to the
# robot again.

unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=87
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Robot-only systemd services (mors@*.service) show as active instead of NOT FOUND.
# To test a failure: HUSKY_HEALTH_FAILED_SERVICES=mors@atak_bridge.service ros2 launch ...
export HUSKY_HEALTH_SIMULATE_SERVICES=1

source /opt/ros/jazzy/setup.bash
_health_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [ -f "$_health_ws/install/setup.bash" ]; then
    source "$_health_ws/install/setup.bash"
else
    echo "husky_health_monitor isn't built yet: cd $_health_ws && colcon build"
fi
unset _health_ws

echo "Local test mode: ROS_DOMAIN_ID=87, this computer only (not connected to the Husky)."
