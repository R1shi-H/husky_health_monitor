# husky_health_monitor

At-a-glance health for the Husky A300 (`a300_00095`) and the MORS Code software.
One ROS 2 node runs onboard and checks power and safety, the drive system,
positioning, perception, operator control, the MORS mission services (ROS topics and
systemd units), the network, ROS itself and the onboard computer. It reports each as
**Healthy / Warning / Error** in three places:

- a **web dashboard** at `http://<robot>:8088`, viewable over Tailscale
- a **terminal readout** (`health_status`), styled after `voxl-inspect-services`
- a `diagnostic_msgs/DiagnosticArray` on `/a300_00095/health_monitor/status`
  (works with `rqt_robot_monitor` and Foxglove)

Every check is defined in a YAML file, so adding a sensor or node means adding a few
lines of config, not code.

```
HUSKY A300 SYSTEM HEALTH
a300_00095 on cpr-a300-00095 at 14:02:11
============================================================
PLATFORM                        STATUS
------------------------------------------------------------
Battery                         ✓ 78%
Emergency stop                  ✓ RELEASED
Motor drivers                   ✓ 50 Hz
...
SENSORS                         STATUS
------------------------------------------------------------
GPS 0                           ✓ 5.0 Hz · RTK
GPS 1                           ✗ STALE
  └ last message 4.2 s ago
...
============================================================
OVERALL SYSTEM STATUS:          ✗ ERROR
```

## Run it on the Husky

The commands below use placeholders for the robot's addresses, which are kept out of
this repo. Get the real values from the setup notes:
`<husky-tailscale-ip>` (also shown by `tailscale status` as `cpr-a300-00095`),
`<husky-ip>` (the robot's onboard network address) and `<robot-user>` (its login).

1. Copy the package to the robot and build it:

   ```bash
   scp -r husky_health_monitor <robot-user>@<husky-tailscale-ip>:~/health_ws/src/
   ssh <robot-user>@<husky-tailscale-ip>
   cd ~/health_ws && source /opt/ros/jazzy/setup.bash
   colcon build --packages-select husky_health_monitor
   source install/setup.bash
   ```

2. Use the same ROS environment as the robot's own terminals, **plus super client
   mode**. Without it the discovery server only tells the monitor about some of the
   graph, and node and topic checks report things as missing.

   ```bash
   export ROS_DISCOVERY_SERVER=<husky-ip>:11811
   export ROS_SUPER_CLIENT=True
   ros2 daemon stop; ros2 daemon start
   ros2 node list        # should show the Husky's nodes before you go on
   ```

3. Start the monitor:

   ```bash
   ros2 launch husky_health_monitor health_monitor.launch.py
   ```

   Launch arguments: `namespace:=a300_00095`, `config:=/path/to/your.yaml`, `web_port:=8088`.

4. View it:
   - Browser, from any machine on Tailscale: `http://<husky-tailscale-ip>:8088`
   - Terminal on the robot: `ros2 run husky_health_monitor health_status --watch`
   - Terminal on a laptop (plain Python, no ROS setup needed):
     `python3 husky_health_monitor/cli.py --url http://<husky-tailscale-ip>:8088 --watch`

   If the page doesn't load over Tailscale, tunnel it through SSH instead:
   `ssh -L 8088:localhost:8088 <robot-user>@<husky-tailscale-ip>`, then open `http://localhost:8088`.

5. Tune the rates. The shipped `expected_hz` values are conservative guesses. With
   the robot running normally, compare them with what's measured:

   ```bash
   ros2 run husky_health_monitor health_status --calibrate
   ```

   Copy the suggested values into `config/a300_00095.yaml` and rebuild (or pass your
   own file with `config:=`). The GPS navigation service is still a commented-out
   placeholder in **Mission Services**; fill in its unit name once it exists.

`health_status` exit codes: 0 healthy, 1 warning, 2 error, 3 monitor unreachable.
That makes it usable from scripts, e.g. a pre-mission check.

## Develop without the robot

One command runs a fake Husky and the monitor together, entirely on a local computer:

```bash
~/health_ws/src/husky_health_monitor/run_local_test.sh gps_1,low_battery
```

Open `http://localhost:8088`. Leave out the faults for a healthy robot; Ctrl-C stops
everything. It works in any terminal, including one set up for the robot by `~/.bashrc`.

The rest of this section is what that script does, if you'd rather run the pieces
separately. `mock_husky` publishes fake versions of every topic in the config, at
realistic rates, and can inject faults. Its topics include `cmd_vel` and the e-stop,
so they must never reach the real robot. `local_test_env.sh` cuts a terminal off from
the Husky's discovery server and keeps ROS traffic on this computer, and the mock
refuses to start without it. Source it in **both** terminals:

```bash
source ~/health_ws/src/husky_health_monitor/local_test_env.sh
ros2 run husky_health_monitor mock_husky --faults gps_1,low_battery     # terminal 1
```

```bash
source ~/health_ws/src/husky_health_monitor/local_test_env.sh
ros2 launch husky_health_monitor health_monitor.launch.py               # terminal 2
```

Then open `http://localhost:8088`, or run `ros2 run husky_health_monitor health_status --watch`
in a third terminal. Stop both with Ctrl-C. To talk to the Husky again, open a new terminal.

Available faults: `low_battery`, `estop`, `gps_1`, `no_fix`, `slow_imu`, `rear_camera`,
`joystick` (GCS PS5 disconnected), `tilt` (30° slope), `diagnostics`. On a laptop the
Clearpath-specific checks (MCU, motors) still work, because the mock publishes standard
stand-in message types.

The MORS systemd services only exist on the robot. In local test mode the monitor shows
any unit that isn't installed on the laptop as active, marked "simulated"; units the
laptop really has, like `tailscaled`, are always checked for real. To see a service fail:

```bash
HUSKY_HEALTH_FAILED_SERVICES=mors@atak_bridge.service ros2 launch husky_health_monitor health_monitor.launch.py
```

Unit tests for the check logic need no ROS graph: `python3 -m pytest test`.

## Configuring checks

Each group in the YAML becomes a card on the dashboard and a section in the terminal
readout, in the order they're listed; the dashboard packs the cards into balanced
columns. A group made only of `system` checks is shown as the tiles at the top. Topic
and node names are relative to `robot_namespace` unless they start with `/`. Node
patterns may use `*`; a pattern starting with `*` matches in any namespace.

| `type` | What it checks | Key options |
|---|---|---|
| `rate` *(default)* | Messages arrive fast enough (raw subscription, nothing deserialized) | `expected_hz`, `warn_below_hz`, `error_below_hz`, `timeout_s`, `idle_ok`, `ok_label` |
| `battery` | `sensor_msgs/BatteryState` charge | `warn_below_percent` (30), `error_below_percent` (15), `percentage_scale` |
| `navsat` | `sensor_msgs/NavSatFix` rate and fix type (-1 NO FIX, 0 GNSS FIX, 1 RTK FLOAT, 2 RTK FIXED) | `expected_hz`, `no_fix_level`, `min_status` (2 = require RTK FIXED), `fix_labels` |
| `bool` | A boolean field, e.g. e-stop | `field` (`data`), `ok_value` (`true`, `false` or `any`), `true_label`, `false_label`, `bad_level` |
| `orientation` | Yaw, roll and pitch from a quaternion, with optional tilt limits; an exactly-zero orientation shows as NOT INITIALIZED | `field` (`pose.pose.orientation`), `warn_tilt_deg`, `error_tilt_deg`, `identity_means_uninitialized` (true) |
| `field` | Any field: numeric thresholds or allowed values | `field` (e.g. `status.temperature`), `warn_above`, `error_above`, `warn_below`, `error_below`, `ok_values`, `unit`, `scale` |
| `diagnostics` | A `DiagnosticStatus` or `DiagnosticArray` (e.g. Clearpath's `diagnostics_agg`) | `include`, `exclude` (glob on names), `stale_level` |
| `node` | Nodes matching a name or pattern are running | `node`, `min_count` |
| `topic` | A topic has publishers/subscribers (graph only, no data) | `min_publishers`, `min_subscribers` |
| `ros_graph` | The monitor can see the ROS graph at all | `min_nodes` |
| `system` | CPU, memory, disk or temperature of the onboard computer | `metric`, `warn_above`, `error_above`, `path` |
| `interface` | A network interface is up with an IPv4 address | `interface` (e.g. `tailscale0`) |
| `ping` | A host answers ping (e.g. 1.1.1.1 for internet, or the ground station) | `host`, `warn_ms`, `period_s` |
| `systemd` | A systemd service is active (`mors@*.service`, `tailscaled`) | `unit`, `user` (a `--user` unit), `period_s`, `status_ok` (status-text prefixes that mean healthy, e.g. `[Connected]` for Tailscale) |

Options every check accepts:

- `critical: false`: errors only raise the overall status to WARNING (cameras, navigation).
- `missing_level: warn`: no data or no node is a warning instead of an error.
- `missing_label`: text shown when data is missing, e.g. `DISCONNECTED`.
- `show_detail: true`: always show the detail line (battery voltage, roll and pitch,
  Tailscale status), not only when something is wrong.
- `idle_ok: true` with `idle_label` (topic checks): no data is a neutral grey state, not
  a fault. Used for cmd_vel, the controllers and target tracking.
- `description`: shown in the dashboard's detail view.
- `msg_type` (topic checks): subscribe before the topic appears in discovery.

Rate checks warn below 80% of `expected_hz`, error below 50%, and go STALE when no
message has arrived for `timeout_s` (default: three periods, at least 2 s). A check that
dropped below a threshold only recovers once the rate is 10% above it again, so a topic
hovering at the threshold doesn't flip back and forth. Topics
that are only published on change (e-stop, Wi-Fi) have no `expected_hz`, so they are
never marked stale. Topics published once and kept (latched, like `robot_description`)
are detected automatically; a transient-local topic that keeps streaming, like
`platform/odom`, is still rate-checked.

Example: watching the MCU temperature once you know its field name (check with
`ros2 topic echo --once /a300_00095/platform/mcu/status/temperature`):

```yaml
- name: MCU temperature
  type: field
  topic: platform/mcu/status/temperature
  field: pcb_temperature
  unit: ' °C'
  warn_above: 60
  error_above: 75
```

## How it stays out of the way

The monitor must not affect normal Husky operation, so:

- It only subscribes. It never publishes on robot topics; its one output topic is
  `health_monitor/status`.
- Subscriptions are best-effort, so publishers never wait for the monitor or resend
  to it. Rate-only checks use raw subscriptions: messages are counted, never decoded.
- For heavy streams it watches the lightest topic that proves the device works:
  the voxel-filtered cloud instead of the full lidar scan, compressed images instead of
  raw. (For the PTZ camera, `image_raw_throttle/compressed` or `camera_info` would be
  lighter still.)
- `systemctl` and ping run every 5 s on a background thread, never on the ROS executor.
- It spins with rclpy's `EventsExecutor`, which handles each message far more cheaply
  than the default executor. In testing against the mock (about 560 messages/s across
  24 topics), the monitor used about 5% of one CPU core and 83 MB of RAM; with the
  default executor it used 26%. Set `executor: single` in the config to switch back.
- Graph queries run every 3 s and checks are evaluated at 1 Hz. The dashboard footer
  shows the monitor's own CPU and memory use.

## Files and outputs

| Path | What |
|---|---|
| `config/a300_00095.yaml` | The checks |
| `husky_health_monitor/checks.py` | Check types and evaluation logic (no ROS dependency) |
| `husky_health_monitor/monitor_node.py` | The node: subscriptions, graph discovery, publishing |
| `husky_health_monitor/web_server.py`, `web/index.html` | Dashboard server and page (no internet needed) |
| `husky_health_monitor/cli.py` | Terminal readout and `--calibrate` |
| `husky_health_monitor/mock_husky.py` | Fake robot for development |
| `~/.ros/husky_health/health-YYYY-MM-DD.jsonl` | History log: every status change, plus a snapshot every 10 s; 7 days kept |

The dashboard's JSON API (`/api/health`, `/api/history?seconds=900`, `/api/graph`) is
read-only.

## Start at boot (optional)

To have the monitor running whenever the Husky is on, add a systemd user service
(`~/.config/systemd/user/husky-health.service`):

```ini
[Unit]
Description=Husky health monitor
After=network-online.target

[Service]
Environment=ROS_DISCOVERY_SERVER=<husky-ip>:11811
Environment=ROS_SUPER_CLIENT=True
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source %h/health_ws/install/setup.bash && exec ros2 launch husky_health_monitor health_monitor.launch.py'
Restart=on-failure

[Install]
WantedBy=default.target
```

Then run `systemctl --user enable --now husky-health` and
`loginctl enable-linger <robot-user>`. Check with the team before adding services to the robot.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| ROS graph: `NO NODES` | Different `ROS_DOMAIN_ID`, wrong `ROS_DISCOVERY_SERVER`, or the robot's ROS stack isn't up |
| Many checks `NO PUBLISHER` but `ros2 topic list` shows them | `ROS_SUPER_CLIENT` isn't `True` in the monitor's shell (the node logs a warning) |
| `NO TYPE` | The message package isn't installed or sourced where the monitor runs (e.g. Clearpath messages on a laptop) |
| Service `NOT FOUND` | The monitor isn't running on the robot, or the unit name is wrong; check `systemctl list-units 'mors@*'` |
| A connected controller shows `NOT CONNECTED` | Its joy node only publishes on input; raise `timeout_s` or set the joy node's `autorepeat_rate` |
| Node or topic check `NOT RUNNING` | The name doesn't match; check `ros2 node list` / `ros2 topic list` |
| Rate checks flicker between OK and WARNING | `expected_hz` is too close to the real rate; use `--calibrate` |
| Dashboard unreachable | Port 8088 blocked or Tailscale down; use the SSH tunnel above |
