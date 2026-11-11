"""Fake Husky A300 topics for developing the monitor when the real robot is off.

Publishes the topics config/a300_00095.yaml watches, with plausible types and
rates, plus empty stand-in nodes for the node checks. Clearpath and Fixposition
message types are replaced by std_msgs/Header, which is enough for rate checks.

It refuses to start unless ROS traffic is confined to this computer, because
fake e-stop and joystick messages must never reach the real robot:
    source local_test_env.sh
    ros2 run husky_health_monitor mock_husky --faults gps_1,low_battery

systemd services (mors@*.service) can't be faked from here: local_test_env.sh makes
the monitor show them as active, and HUSKY_HEALTH_FAILED_SERVICES fails chosen ones.
"""

import argparse
import math
import os
import random
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import (BatteryState, CompressedImage, Imu, JointState, Joy, NavSatFix,
                             NavSatStatus, PointCloud2, PointField)
from std_msgs.msg import Bool, Header
from tf2_msgs.msg import TFMessage

FAULTS = {
    'low_battery': 'battery at 12%',
    'estop': 'emergency stop engaged',
    'gps_1': 'GPS 1 stops publishing',
    'no_fix': 'GPS 0 loses its fix',
    'rtk_float': 'GPS 1 drops to RTK float',
    'slow_imu': 'IMU 0 drops to 20 Hz',
    'imu_zeros': 'IMU 0 publishes all-zero readings',
    'rear_camera': 'rear OAK-D stops publishing',
    'joystick': 'GCS PS5 controller disconnected',
    'ps4': 'onboard PS4 controller not connected',
    'tilt': 'robot pitched 30 degrees (steep slope)',
    'diagnostics': 'Clearpath diagnostics report a motor warning',
}


class MockHusky(Node):

    def __init__(self, namespace, faults):
        super().__init__('mock_platform', namespace=namespace)
        self.faults = faults
        self.start = time.monotonic()

        self.stream('platform/bms/state', BatteryState, 1, self.battery)
        self.stream('platform/emergency_stop', Bool, 1, lambda: Bool(data='estop' in faults))
        self.stream('platform/wifi_connected', Bool, 1, lambda: Bool(data=True))
        self.stream('platform/mcu/status', Header, 1, self.header)
        self.stream('platform/motors/feedback', Header, 50, self.header)
        # The real platform/odom streams with transient-local QoS, so the mock does too
        self.stream('platform/odom', Odometry, 50, Odometry, qos=QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.stream('platform/joint_states', JointState, 50, self.joints)
        self.stream('diagnostics_agg', DiagnosticArray, 1, self.diagnostics)
        self.stream('tf', TFMessage, 50, self.tf)

        self.stream('sensors/ins_0/xvn/odometry_enu', Odometry, 10, self.odom)
        gps_0 = NavSatStatus.STATUS_NO_FIX if 'no_fix' in faults else 2  # RTK fixed
        self.stream('sensors/ins_0/gps_0/fix', NavSatFix, 5, lambda: self.fix(gps_0))
        gps_1 = 1 if 'rtk_float' in faults else 2  # RTK float / fixed
        if 'gps_1' not in faults:
            self.stream('sensors/ins_0/gps_1/fix', NavSatFix, 5, lambda: self.fix(gps_1))
        # Rates as measured on a300_00095
        self.stream('sensors/imu_0/data', Imu, 20 if 'slow_imu' in faults else 50, self.imu)
        self.stream('sensors/lidar3d_0/pointcloud_voxel', PointCloud2, 10, self.cloud)
        self.stream('oak_d_pro_w_front/image_color/compressed', CompressedImage, 15, self.image)
        if 'rear_camera' not in faults:
            self.stream('oak_d_pro_w_rear/image_color/compressed', CompressedImage, 15, self.image)
        self.stream('sensors/camera_2/image_raw/compressed', CompressedImage, 30, self.image)

        # The onboard PS4 joy node publishes whether or not a controller is attached
        self.stream('joy_teleop/joy', Joy, 20, lambda: self.ps4('ps4' not in faults))
        # GCS PS5 and Steam Deck 1 connected, Steam Deck 2 not
        if 'joystick' not in faults:
            self.stream('operators/gcs_ps5/joy', Joy, 20, self.joy)
        self.stream('operators/steamdeck_1/joy', Joy, 20, self.joy)

        self.create_publisher(Header, 'autonomy/status', 1)  # advertised, nothing published
        self.stream('object_tracking/enabled', Bool, 1, lambda: Bool(data=True))

    def stream(self, topic, msg_type, hz, make, when=None, qos=10):
        pub = self.create_publisher(msg_type, topic, qos)

        def tick():
            if when is None or when():
                pub.publish(make())
        self.create_timer(1.0 / hz, tick)

    def elapsed(self):
        return time.monotonic() - self.start

    def header(self):
        msg = Header(frame_id='base_link')
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def battery(self):
        pct = 0.12 if 'low_battery' in self.faults else max(0.2, 0.78 - self.elapsed() / 6000)
        return BatteryState(header=self.header(), voltage=24.0 + 4.0 * pct, current=-3.2,
                            percentage=pct, present=True,
                            power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)

    def joints(self):
        names = ['front_left_wheel_joint', 'front_right_wheel_joint',
                 'rear_left_wheel_joint', 'rear_right_wheel_joint']
        return JointState(header=self.header(), name=names, position=[0.0] * 4, velocity=[0.0] * 4)

    def tf(self):
        transform = TransformStamped(header=self.header(), child_frame_id='base_link')
        transform.header.frame_id = 'odom'
        return TFMessage(transforms=[transform])

    def fix(self, status):
        msg = NavSatFix(header=self.header(), latitude=21.3069, longitude=-157.8583, altitude=10.0)
        msg.status.status = status
        return msg

    def odom(self):
        # Turning slowly in place, slightly nose-up; 'tilt' puts it on a steep slope
        yaw = math.radians((self.elapsed() * 3) % 360 - 180)
        pitch = math.radians(30.0 if 'tilt' in self.faults else 2.0)
        roll = math.radians(-1.0)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        msg = Odometry(header=self.header(), child_frame_id='base_link')
        q = msg.pose.pose.orientation
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return msg

    @staticmethod
    def joy():
        return Joy(axes=[0.0] * 6, buttons=[0] * 13)

    @staticmethod
    def ps4(connected):
        # A connected PS4's triggers (axes 2 and 5) rest at 1.0; with no controller all read 0
        axes = [0.0] * 8
        if connected:
            axes[2] = axes[5] = 1.0
        return Joy(axes=axes, buttons=[0] * 13)

    def imu(self):
        msg = Imu(header=self.header())
        if 'imu_zeros' not in self.faults:
            # Level and still: gravity on z, a little sensor noise
            msg.linear_acceleration.z = 9.81 + random.uniform(-0.05, 0.05)
            msg.angular_velocity.z = random.uniform(-0.002, 0.002)
        return msg

    def cloud(self):
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
        return PointCloud2(header=self.header(), height=1, width=2000, fields=fields,
                           point_step=12, row_step=24000, data=bytes(24000), is_dense=True)

    def image(self):
        return CompressedImage(header=self.header(), format='jpeg', data=bytes(30000))

    def diagnostics(self):
        motor_warn = 'diagnostics' in self.faults
        # Like the real robot, the MCU always warns that newer firmware exists (the monitor's
        # config excludes it), so /Platform and /Platform/MCU are always WARN
        entries = [('/Platform', 1, ''),
                   ('/Platform/Battery', 0, 'OK'),
                   ('/Platform/MCU', 1, ''),
                   ('/Platform/MCU/clearpath_diagnostic_updater: MCU Firmware Version', 1,
                    'New firmware available: (3.0.3) -> (3.0.5). Restart service to re-evaluate.'),
                   ('/Platform/Motors', 1 if motor_warn else 0,
                    'Front left motor driver temperature high' if motor_warn else 'OK'),
                   ('/Sensors', 0, ''),
                   ('/Sensors/IMU', 0, 'OK')]
        msg = DiagnosticArray(header=self.header())
        msg.status = [DiagnosticStatus(name=n, level=bytes([lvl]), message=m) for n, lvl, m in entries]
        return msg


def network_risk():
    """Why fake topics from this shell could reach another machine, or '' if they can't."""
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        return f'ROS_DISCOVERY_SERVER is set to {os.environ["ROS_DISCOVERY_SERVER"]}'
    if (os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '').upper() != 'LOCALHOST'
            and os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        return 'ROS discovery is not limited to this computer'
    return ''


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish fake Husky A300 topics.')
    parser.add_argument('--namespace', default='a300_00095')
    parser.add_argument('--faults', default='',
                        help='comma-separated faults to inject: ' + ', '.join(FAULTS))
    parsed, _ = parser.parse_known_args(args)  # leave --ros-args to rclpy
    faults = {f.strip() for f in parsed.faults.split(',') if f.strip()}
    unknown = faults - FAULTS.keys()
    if unknown:
        parser.error(f'unknown fault(s) {", ".join(sorted(unknown))}; choose from {", ".join(FAULTS)}')
    risk = network_risk()
    if risk:
        parser.exit(2, f'mock_husky: not starting: {risk}, so fake topics (including the e-stop '
                       'and joystick input) could reach the real Husky.\n'
                       'Run `source local_test_env.sh` from the package folder first.\n')

    rclpy.init(args=args)
    ns = parsed.namespace
    nodes = [MockHusky(ns, faults),
             Node('controller_manager', namespace=ns),
             Node('robot_state_publisher', namespace=ns),
             Node('fixposition_driver', namespace=f'{ns}/sensors/ins_0'),
             Node('joy_node', namespace=ns),
             Node('teleop_twist_joy_node', namespace=ns)]

    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    nodes[0].get_logger().info(
        f'publishing fake topics under /{ns}' +
        (f' with faults: {", ".join(f"{f} ({FAULTS[f]})" for f in sorted(faults))}' if faults else ''))
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
