"""Differential-drive wheel odometry and velocity commands for the AGV.

  in   /wheel_states  sensor_msgs/JointState  wheel positions (rad) and speeds (rad/s), from the ESP32
  in   /cmd_vel       geometry_msgs/Twist     linear.x (m/s) and angular.z (rad/s)
  out  /odom          nav_msgs/Odometry       pose and speed of base_footprint in odom
  out  /wheel_cmd     sensor_msgs/JointState  wheel speed targets (rad/s), to the ESP32
  out  TF odom -> base_footprint

Kinematics, with r the wheel radius and b the distance between the wheels:
  wheel arc lengths   dl = r * d_angle_left    dr = r * d_angle_right
  robot motion        d = (dr + dl) / 2         d_yaw = (dr - dl) / b
  wheel targets       left = (v - w * b / 2) / r    right = (v + w * b / 2) / r
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    """Rotation about z only: (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.r = self.declare_parameter('wheel_radius', 0.035).value
        self.b = self.declare_parameter('wheel_separation', 0.235).value
        self.joints = (self.declare_parameter('left_joint', 'left_wheel_joint').value,
                       self.declare_parameter('right_joint', 'right_wheel_joint').value)
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        # A wheel cannot turn this much between two messages (50 Hz): the ESP32
        # has restarted and its positions are back at 0.
        self.max_step = self.declare_parameter('max_wheel_step_rad', 1.0).value
        self.cmd_rate = self.declare_parameter('cmd_rate_hz', 20.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout_s', 0.5).value
        self.max_accel = self.declare_parameter('max_wheel_accel', 20.0).value  # rad/s^2

        self.x = self.y = self.yaw = 0.0
        self.last_angles = None
        self.cmd_target = (0.0, 0.0)  # wheel speeds asked by /cmd_vel
        self.cmd_sent = (0.0, 0.0)    # after the acceleration limit
        self.last_cmd_time = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.wheel_cmd_pub = self.create_publisher(JointState, 'wheel_cmd', qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, 'wheel_states', self.on_wheel_states,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / self.cmd_rate, self.send_wheel_cmd)
        self.get_logger().info(f'wheel radius {self.r} m, separation {self.b} m')

    # ---- odometry ----

    def wheel_values(self, msg, field):
        """(left, right) from a JointState field, by joint name."""
        values = getattr(msg, field)
        try:
            return tuple(values[msg.name.index(j)] for j in self.joints)
        except (ValueError, IndexError):
            return None

    def on_wheel_states(self, msg):
        angles = self.wheel_values(msg, 'position')
        speeds = self.wheel_values(msg, 'velocity')
        if angles is None or speeds is None:
            self.get_logger().warn('wheel_states without both wheel joints', throttle_duration_sec=5.0)
            return
        if self.last_angles is None:
            self.last_angles = angles
            return
        d_left = angles[0] - self.last_angles[0]
        d_right = angles[1] - self.last_angles[1]
        self.last_angles = angles
        if abs(d_left) > self.max_step or abs(d_right) > self.max_step:
            self.get_logger().warn('wheel positions jumped (ESP32 restarted?): step skipped')
            return

        dl = self.r * d_left
        dr = self.r * d_right
        d = (dr + dl) / 2.0
        d_yaw = (dr - dl) / self.b
        # Midpoint heading: exact for a straight segment, close for an arc.
        mid_yaw = self.yaw + d_yaw / 2.0
        self.x += d * math.cos(mid_yaw)
        self.y += d * math.sin(mid_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))

        v = self.r * (speeds[0] + speeds[1]) / 2.0
        w = self.r * (speeds[1] - speeds[0]) / self.b
        self.publish_odometry(msg.header.stamp, v, w)

    def publish_odometry(self, stamp, v, w):
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    # ---- velocity commands ----

    def on_cmd_vel(self, msg):
        v, w = msg.linear.x, msg.angular.z
        half = w * self.b / 2.0
        self.cmd_target = ((v - half) / self.r, (v + half) / self.r)
        self.last_cmd_time = self.get_clock().now()

    def send_wheel_cmd(self):
        """Sends the wheel targets at a fixed rate, so the ESP32 watchdog stays fed
        while /cmd_vel is alive, and stops sending when it goes quiet."""
        if self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        target = self.cmd_target if age < self.cmd_timeout else (0.0, 0.0)
        # Acceleration limit: the targets ramp instead of stepping.
        step = self.max_accel / self.cmd_rate
        self.cmd_sent = tuple(s + max(-step, min(step, t - s))
                              for s, t in zip(self.cmd_sent, target))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joints)
        msg.velocity = list(self.cmd_sent)
        self.wheel_cmd_pub.publish(msg)
        if self.cmd_sent == (0.0, 0.0) and age >= self.cmd_timeout:
            self.last_cmd_time = None  # stopped: leave the ESP32 watchdog in charge


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
