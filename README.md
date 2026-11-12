# husky_health_monitor

At-a-glance health for the Husky A300 (`a300_00095`) and the MORS Code software.
One ROS 2 node runs onboard and checks power and safety, the drive system,
positioning, perception, operator control, the MORS mission services (ROS topics and
systemd units), the network, ROS itself and the onboard computer. It reports each as
**Healthy / Warning / Error** in three places:

- a **web dashboard** at `http://<robot>:8088`, viewable over Tailscale
- a **terminal readout** (`health`, see [A short `health` command](#a-short-health-command)),
  styled after `voxl-inspect-services`
- a `diagnostic_msgs/DiagnosticArray` on `/a300_00095/health_monitor/status`
  (works with `rqt_robot_monitor` and Foxglove)

Every check is defined in a YAML file, so adding a sensor or node means adding a few
lines of config, not code.

```
HUSKY A300 SYSTEM HEALTH
a300_00095 on cpr-a300-00095 at 16:32:58
============================================================
POWER & SAFETY                  STATUS
------------------------------------------------------------
Battery                         ✓ 52%
Emergency stop                  ⚠ ENGAGED

DRIVE SYSTEM                    STATUS
------------------------------------------------------------
MCU                             ✓ 1.0 Hz
Motor drivers                   ✓ 50 Hz
Wheel odometry                  ✓ 50 Hz
...
POSITIONING                     STATUS
------------------------------------------------------------
IMU orientation                 ✓ heading 214° SW
GPS 0                           ✓ 5.0 Hz · RTK FIXED
GPS 1                           ⚠ 5.0 Hz · RTK FLOAT
...
============================================================
OVERALL SYSTEM STATUS:          ⚠ WARNING
```

The snapshot is bare by default: one line per check and the overall status. Add
`--details` (`-d`) to see the reason under each warning or error. The dashboard
groups the same checks into cards: Power & Safety, Drive System, Positioning,
Perception, Operator Control, Mission Services, Network, and ROS & Diagnostics, with the
onboard computer's CPU, memory, disk and temperature as tiles at the top and the
overall status as a badge next to the title.

## Run it on the Husky

The commands below use placeholders for the robot's addresses, which are kept out of
this repo. Get the real values from the setup notes:
`<husky-tailscale-ip>` (also shown by `tailscale status` as `cpr-a300-00095`),
`<husky-ip>` (the robot's onboard network address) and `<robot-user>` (its login).

1. Log in to the robot, clone the repo into its own workspace and build it:

   ```bash
   ssh <robot-user>@<husky-tailscale-ip>
   mkdir -p ~/health_ws/src && cd ~/health_ws/src
   git clone https://github.com/R1shi-H/husky_health_monitor.git
   cd ~/health_ws && source /opt/ros/jazzy/setup.bash
   colcon build --packages-select husky_health_monitor
   source install/setup.bash
   ```

   To update later, run `git pull` in `~/health_ws/src/husky_health_monitor`, then the
   same `colcon build` and `source` lines. If the robot has no internet, copy the
   package from a laptop with `scp -r` instead. Keep it in its own workspace rather
   than the team's `mors_ws`, so building or updating it never touches their code.

2. Turn on **super client mode**. Without it the discovery server only tells the
   monitor about part of the graph, and node and topic checks report things as missing.
   The robot's shell already points at its discovery server (`echo $ROS_DISCOVERY_SERVER`
   shows `127.0.0.1:11811;` on a300_00095); only set it if that's empty.

   ```bash
   export ROS_SUPER_CLIENT=True
   ros2 daemon stop; ros2 daemon start
   ros2 node list        # should show the Husky's nodes before you go on
   ```

3. Start the monitor:

   ```bash
   ros2 launch husky_health_monitor health_monitor.launch.py
   ```

   Launch arguments: `namespace:=a300_00095`, `config:=/path/to/your.yaml`, `web_port:=8088`.

   The monitor stops when that terminal closes, so keep it open, or install it as a
   service ([Start at boot](#start-at-boot)) so it always runs. The dashboard footer shows which config
   file is in use; it should start with `/home/<robot-user>/health_ws/`. If it shows
   another workspace, that copy is the one running: source
   `~/health_ws/install/setup.bash` last and relaunch.

4. View it:
   - Browser, from any machine on Tailscale: `http://<husky-tailscale-ip>:8088`
   - Terminal on the robot: `health` (see below), or
     `ros2 run husky_health_monitor health_status`
   - Terminal on a laptop (plain Python, no ROS setup needed):
     `python3 husky_health_monitor/cli.py --url http://<husky-tailscale-ip>:8088 --watch`

   If the page doesn't load over Tailscale, tunnel it through SSH instead:
   `ssh -L 8088:localhost:8088 <robot-user>@<husky-tailscale-ip>`, then open `http://localhost:8088`.

5. Re-check the rates now and then. The `expected_hz` values in the config were
   measured on a300_00095 on 2026-11-09, with the robot e-stopped and charging. Rates
   can change while it's driving, so compare them with what's measured then:

   ```bash
   ros2 run husky_health_monitor health_status --calibrate
   ```

   Copy the suggested values into `config/a300_00095.yaml` and rebuild (or pass your
   own file with `config:=`). The GPS navigation service is still a commented-out
   placeholder in **Mission Services**; fill in its unit name once it exists.

### A short `health` command

The terminal readout is plain Python and only talks to the running monitor, so it
doesn't need ROS set up. On the robot, add a shortcut once:

```bash
grep -q "alias health=" ~/.bashrc || echo "alias health='python3 ~/health_ws/src/husky_health_monitor/husky_health_monitor/cli.py'" >> ~/.bashrc
```

Then, in any new terminal: `health` for a snapshot, `health -w` to keep it updating,
`health -d` to include the reasons. Unlike an alias for `ros2 run`, it works even when
the workspace isn't sourced.

Exit codes: 0 healthy, 1 warning, 2 error, 3 monitor unreachable. That makes it usable
from scripts, e.g. a pre-mission check.

## Notes from testing on a300_00095

Things the config accounts for, from testing on the robot (2026-11-09 and 2026-11-12) and
team review:

- The EKF is disabled, so `platform/odom_filtered` has no publisher; Wheel odometry
  watches `platform/odom`, which uses transient-local QoS but streams at 50 Hz.
- `platform/safety_stop` is an input the platform listens on, not a status, so it isn't
  checked; Emergency stop covers stop state.
- The 3D lidar's voxel-filtered cloud arrives unevenly (about 1.5-4 Hz, gaps up to
  1.1 s; `ros2 topic hz` agrees), so that check averages over 15 s.
- `sensors/ins_0/xvn/odometry_enu` keeps publishing without a fix or base station, but
  every value is 0. The dashboard shows that as NOT INITIALIZED. It reports heading as a
  compass reading (0 = North, 90 = East), converted from ROS's ENU yaw (0 = East).
- GPS colors follow the fix type: RTK FIXED green, RTK FLOAT yellow, GNSS FIX or NO FIX
  red. Without RTK corrections (for example indoors, or with no base station) the
  overall status is ERROR.
- `joy_teleop/joy` (the onboard Bluetooth PS4 controller) publishes even with no
  controller attached; the triggers (axes 2 and 5) rest at 1.0 only when one is.
- IMU 0 runs at 50 Hz (set in `robot.yaml`). Its check also confirms the readings are
  real: the IMU should measure gravity, about 9.8 m/s².
- The Doodle Labs radio's USB network interface (`enx` plus its MAC address) only
  exists while the radio is connected; its `br0` address is permanent, so it isn't used.
- Outdoors the robot uses the radio with Wi-Fi off, so Wi-Fi DISCONNECTED is shown in gray,
  not as a warning.
- Clearpath diagnostics always carry two warnings that aren't faults: the MCU's "new
  firmware available" notice and the Wi-Fi monitor. The config excludes both. The
  controller manager's "high execution jitter" warning is real timing data and is still
  shown; add `'*Hardware Components Activity*'` to `exclude` if it turns out to be constant.
- Not monitored, by request: the 2D lidar, the INS's raw IMU, localization, and cmd_vel.

### What to expect indoors

Indoors the overall status is **ERROR** even when nothing is wrong. Seen on 2026-11-12,
indoors, charging, with the e-stop engaged:

| Check | Shows | Why |
|---|---|---|
| GPS 0, GPS 1 | ✗ NO FIX | No satellites indoors. This alone makes the overall status ERROR |
| IMU orientation | ⚠ NOT INITIALIZED | The INS publishes all zeros until it has a fix |
| Emergency stop | ⚠ ENGAGED | E-stop pressed |
| Clearpath diagnostics | ⚠ several WARN | The Lynx motor drivers report "E-stopped" (and "Frequency too low") while the e-stop is engaged |
| Doodle Labs radio | ⚠ NOT CONNECTED | Whenever the radio isn't connected to the robot |
| Wi-Fi, PS4 and GCS controllers | ○ gray | Off or not connected; gray never counts against the overall status |

Anything else that isn't green indoors is worth a look.

## Develop without the robot

One command runs a fake Husky and the monitor together, entirely on a local computer:

```bash
~/health_ws/src/husky_health_monitor/run_local_test.sh gps_1,low_battery
```

Open `http://localhost:8088`. Leave out the faults for a healthy robot; Ctrl-C stops
everything. It works in any terminal, including one set up for the robot by `~/.bashrc`.

The rest of this section is what that script does, if you'd rather run the pieces
separately. `mock_husky` publishes fake versions of every topic in the config, at
realistic rates, and can inject faults. Its topics include the e-stop and joystick input,
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

Available faults: `low_battery`, `estop`, `gps_1` (stops), `no_fix` (GPS 0),
`rtk_float` (GPS 1), `slow_imu`, `imu_zeros`, `rear_camera`, `ps4` (onboard controller
not connected), `joystick` (GCS PS5 disconnected), `tilt` (30° slope), `diagnostics`. On a laptop the
Clearpath-specific checks (MCU, motors) still work, because the mock publishes standard
stand-in message types.

The MORS systemd services and the Doodle Labs radio only exist on the robot. In local
test mode the monitor shows anything the laptop doesn't have as working, marked
"simulated"; things the laptop really has, like `tailscaled`, are always checked for real. To see a service fail:

```bash
HUSKY_HEALTH_FAILED_SERVICES=mors@atak_bridge.service ros2 launch husky_health_monitor health_monitor.launch.py
```

Unit tests for the check logic need no ROS graph: `python3 -m pytest test`.

In VS Code, open the package folder; **Terminal → Run Task** has Build, Run with fake
robot, Terminal view and Unit tests.

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
| `navsat` | `sensor_msgs/NavSatFix` rate and fix type (-1 NO FIX, 0 GNSS FIX, 1 RTK FLOAT, 2 RTK FIXED) | `expected_hz`, `status_levels` (level per fix type, e.g. `{2: ok, 1: warn, 0: error, -1: error}`), or `no_fix_level` and `min_status`; `fix_labels` |
| `bool` | A boolean field, e.g. e-stop | `field` (`data`), `ok_value` (`true`, `false` or `any`), `true_label`, `false_label`, `bad_level` (`warn`, `error`, or `idle` for gray) |
| `orientation` | Heading, roll and pitch from a quaternion, with optional tilt limits; an exactly-zero orientation shows as NOT INITIALIZED | `field` (`pose.pose.orientation`), `heading` (`compass`: 0 = North, or `enu`), `warn_tilt_deg`, `error_tilt_deg`, `identity_means_uninitialized` (true) |
| `joystick` | A joy topic that publishes even with no controller; connected is read from axes that rest away from 0 only when one is attached | `connected_axes` (`[2, 5]`, the PS4 triggers), `timeout_s` |
| `imu` | `sensor_msgs/Imu` rate, plus readings that aren't all zero and measure gravity | `expected_hz`, `gravity_check` (true), `gravity_tolerance` (3 m/s²) |
| `field` | Any field: numeric thresholds or allowed values | `field` (e.g. `status.temperature`), `warn_above`, `error_above`, `warn_below`, `error_below`, `ok_values`, `unit`, `scale` |
| `diagnostics` | A `DiagnosticStatus` or `DiagnosticArray` (e.g. Clearpath's `diagnostics_agg`) | `include`, `exclude` (glob on names), `stale_level` |
| `node` | Nodes matching a name or pattern are running | `node`, `min_count` |
| `topic` | A topic has publishers/subscribers (graph only, no data) | `min_publishers`, `min_subscribers` |
| `ros_graph` | The monitor can see the ROS graph at all | `min_nodes` |
| `system` | CPU, memory, disk or temperature of the onboard computer | `metric`, `warn_above`, `error_above`, `path` |
| `interface` | A network interface exists and is up, and by default has an IPv4 address | `interface` (a name or glob, e.g. `enx00301a*`), `require_ipv4` (true) |
| `ping` | A host answers ping (e.g. 1.1.1.1 for internet, or the ground station) | `host`, `warn_ms`, `period_s` |
| `systemd` | A systemd service is active (`mors@*.service`, `tailscaled`) | `unit`, `user` (a `--user` unit), `period_s`, `status_ok` (status-text prefixes that mean healthy, e.g. `[Connected]` for Tailscale) |

Options every check accepts:

- `critical: false`: errors only raise the overall status to WARNING (cameras, navigation).
- `missing_level: warn`: no data or no node is a warning instead of an error.
- `missing_label`: text shown when data is missing, e.g. `DISCONNECTED`.
- `show_detail: true`: always show the detail line (battery voltage, roll and pitch,
  Tailscale status), not only when something is wrong.
- `idle_ok: true` with `idle_label` (topic checks): no data is a neutral grey state, not
  a fault. Used for the operator controllers and target tracking.
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
  On a300_00095 (about 100 nodes and 350 topics) it uses 5-7% of one CPU core and
  about 115 MB of RAM.
- Graph queries run every 3 s and checks are evaluated at 1 Hz. The dashboard footer
  shows the monitor's own CPU and memory use.

## Files and outputs

| Path | What |
|---|---|
| `config/a300_00095.yaml` | The checks |
| `husky_health_monitor/checks.py` | Check types and evaluation logic (no ROS dependency) |
| `husky_health_monitor/monitor_node.py` | The node: subscriptions, graph discovery, publishing |
| `husky_health_monitor/web_server.py`, `web/index.html` | Dashboard server and page (no internet needed) |
| `husky_health_monitor/probes.py` | `systemctl` and ping, run on a background thread |
| `husky_health_monitor/cli.py` | Terminal readout and `--calibrate` |
| `husky_health_monitor/mock_husky.py` | Fake robot for development |
| `run_local_test.sh`, `local_test_env.sh` | Run the fake robot and monitor locally, cut off from the real robot |
| `install_service.sh`, `systemd/husky-health.service.in` | Install the monitor as a service that starts on boot |
| `.vscode/` | Editor settings and tasks |
| `~/.ros/husky_health/health-YYYY-MM-DD.jsonl` | History log: every status change, plus a snapshot every 10 s; 7 days kept |

The dashboard's JSON API (`/api/health`, `/api/history?seconds=900`, `/api/graph`) is
read-only.

## Start at boot

To have the monitor running whenever the Husky is on, install it as a system service.
On the robot, in a terminal where `ros2 node list` shows the Husky's nodes (the service
copies that terminal's ROS settings), after stopping any monitor you started by hand:

```bash
~/health_ws/src/husky_health_monitor/install_service.sh
```

It asks for the robot's sudo password once, then starts the service and enables it for
every boot. It runs as the robot user, after the Clearpath stack, and restarts itself if
it crashes.

| Task | Command |
|---|---|
| Status | `systemctl status husky-health` |
| Live logs | `journalctl -u husky-health -f` |
| After `git pull` and `colcon build` | `sudo systemctl restart husky-health` |
| Stop until next boot | `sudo systemctl stop husky-health` |
| Uninstall | `~/health_ws/src/husky_health_monitor/install_service.sh --remove` |

Things to know:

- **Port 8088 belongs to the service.** Starting another copy by hand fails with
  "Address already in use". Stop the service first, or give the manual copy its own
  port and node name:
  `ros2 run husky_health_monitor health_monitor --ros-args -r __ns:=/a300_00095 -r __node:=health_monitor_dev -p web_port:=8089`
- **The dashboard is always on**, on every network the robot is connected to (Wi-Fi,
  Doodle Labs, Tailscale). It's read-only, but if that's too open, set `web.host` in the
  config to the robot's Tailscale address so it only listens there.
- **It costs about 5-7% of one CPU core and about 115 MB of RAM** around the clock, and
  writes a small history log to `~/.ros/husky_health` (7 days kept).
- **ROS settings are copied at install time.** If the robot's discovery server or domain
  changes, run `install_service.sh` again.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| ROS graph: `NO NODES` | Different `ROS_DOMAIN_ID`, wrong `ROS_DISCOVERY_SERVER`, or the robot's ROS stack isn't up |
| Many checks `NO PUBLISHER` but `ros2 topic list` shows them | `ROS_SUPER_CLIENT` isn't `True` in the monitor's shell (the node logs a warning) |
| `NO TYPE` | The message package isn't installed or sourced where the monitor runs (e.g. Clearpath messages on a laptop) |
| Service `NOT FOUND` | The monitor isn't running on the robot, or the unit name is wrong; check `systemctl list-units 'mors@*'` |
| A connected controller shows `NOT CONNECTED` | For an operator controller, its joy node only publishes on input; raise `timeout_s` or set the joy node's `autorepeat_rate`. For the PS4, check `ros2 topic echo --once /a300_00095/joy_teleop/joy` shows axes 2 and 5 at 1.0 |
| Node or topic check `NOT RUNNING` | The name doesn't match; check `ros2 node list` / `ros2 topic list` |
| Rate checks flicker between OK and WARNING | `expected_hz` is too close to the real rate; use `--calibrate`. For a topic that's naturally uneven, add `window_s` to average over longer |
| `could not start the dashboard ... Address already in use` | Another monitor is already running: the service, or one started by hand. Check `systemctl status husky-health` and `ps -eo pid,lstart,args \| grep "[h]ealth_monitor"` |
| Dashboard down after the robot boots | `systemctl status husky-health` and `journalctl -u husky-health -n 50` show why |
| Dashboard shows an old version | The browser is showing the other monitor (see the row above), or another workspace's copy is running; check the config path in the footer |
| Dashboard unreachable | The monitor isn't running, port 8088 is blocked or Tailscale is down; use the SSH tunnel above |
