#!/bin/bash
# Install the health monitor as a service that starts whenever the Husky boots.
#
# Run it on the robot, in a terminal where `ros2 node list` already shows the Husky's
# nodes: the service gets that terminal's ROS settings. Needs sudo once.
#
#   ~/health_ws/src/husky_health_monitor/install_service.sh            # install and start
#   ~/health_ws/src/husky_health_monitor/install_service.sh --remove   # stop and uninstall
set -euo pipefail

SERVICE=husky-health
UNIT=/etc/systemd/system/$SERVICE.service
ENV_FILE=/etc/$SERVICE.env
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$PKG/../.." && pwd)"

if [ "${1:-}" = "--remove" ]; then
    sudo systemctl disable --now "$SERVICE" 2>/dev/null || true
    sudo rm -f "$UNIT" "$ENV_FILE"
    sudo systemctl daemon-reload
    echo "Removed the $SERVICE service."
    exit 0
fi

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "Build it first: cd $WS && colcon build --packages-select husky_health_monitor"
    exit 1
fi
if [ -n "${HUSKY_HEALTH_SIMULATE_SERVICES:-}" ]; then
    echo "This terminal is set up for local testing (local_test_env.sh). Use a fresh robot terminal."
    exit 1
fi
if [ -z "${ROS_DISCOVERY_SERVER:-}" ]; then
    echo "Note: ROS_DISCOVERY_SERVER is empty in this terminal, so the service won't use one."
    echo "      If 'ros2 node list' doesn't show the Husky's nodes here, stop and fix that first."
fi
if ss -ltn 2>/dev/null | grep -q ':8088 '; then
    echo "Note: something is already using port 8088, probably a monitor started by hand."
    echo "      Stop it (Ctrl-C in its window), or the service's dashboard won't start."
fi

echo "Writing $ENV_FILE with this terminal's ROS settings"
{
    for var in ROS_DOMAIN_ID ROS_DISCOVERY_SERVER RMW_IMPLEMENTATION ROS_AUTOMATIC_DISCOVERY_RANGE \
               ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE \
               CYCLONEDDS_URI; do
        if [ -n "${!var:-}" ]; then
            printf '%s="%s"\n' "$var" "${!var}"
        fi
    done
    echo 'ROS_SUPER_CLIENT="True"'
} | sudo tee "$ENV_FILE" > /dev/null

echo "Writing $UNIT (runs as $USER from $WS)"
sed -e "s|@USER@|$USER|g" -e "s|@WS@|$WS|g" -e "s|@ENV_FILE@|$ENV_FILE|g" \
    "$PKG/systemd/husky-health.service.in" | sudo tee "$UNIT" > /dev/null

sudo systemctl daemon-reload
sudo systemctl enable --now "$SERVICE"
sleep 3
systemctl --no-pager --lines=5 status "$SERVICE" || true

cat <<EOF

The health monitor now starts on every boot.
  Dashboard:     http://<robot-ip>:8088
  Status:        systemctl status $SERVICE
  Logs:          journalctl -u $SERVICE -f
  After updates: sudo systemctl restart $SERVICE
  Uninstall:     $PKG/install_service.sh --remove
EOF
