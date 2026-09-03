#!/usr/bin/python3
# Hard-coded interpreter, not "env python3": this workspace's PATH puts
# anaconda's python3 first, and rclpy's compiled extension is only importable
# from the system python3 ROS2 was built against.
"""Kinematic driver for the DHL base, for RViz.

Turns /cmd_vel into (a) rolling wheel joints and (b) an odom -> base_footprint
transform, so the robot drives around in RViz. There is no physics here: the
commanded twist is integrated directly, wheels never slip, and nothing pushes
back. For real dynamics the model needs inertials, transmissions and Gazebo.

The four wheels are fixed (no steering joints), so the base is treated as a
skid-steer / differential drive: the two +y wheels turn together, and the two
-y wheels turn together.

This node owns the four wheel joints and nothing else: it publishes only those
four names on /joint_states, while joint_state_publisher_gui publishes the
other 18. robot_state_publisher keeps the last value it saw for each joint
name, so two publishers covering disjoint sets of joints merge cleanly. The GUI
is handed a copy of the description built with wheels_fixed:=true so it does
not offer sliders for wheels that /cmd_vel already drives.
"""

import math

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# The source table names the +y side "R"; keep that naming here.
WHEELS_POS_Y = ["wheel_fr_joint", "wheel_rr_joint"]
WHEELS_NEG_Y = ["wheel_fl_joint", "wheel_rl_joint"]


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class FakeDriver(Node):
    def __init__(self):
        super().__init__("fake_driver")

        self.declare_parameter("wheel_radius", 0.155)
        # geometric track = 2 * 278.3 mm; widen it to trim skid-steer over-rotation
        self.declare_parameter("track_width", 0.5566)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom", True)
        # stop if no command arrives for this long; 0 disables the timeout
        self.declare_parameter("cmd_timeout", 0.5)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.track_width = self.get_parameter("track_width").value
        self.rate_hz = self.get_parameter("rate").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_odom = self.get_parameter("publish_odom").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value

        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd = None
        self.x = self.y = self.th = 0.0
        self.ang_pos_y = 0.0
        self.ang_neg_y = 0.0

        self.js_pub = self.create_publisher(JointState, "joint_states", 1)
        self.odom_pub = (self.create_publisher(Odometry, "odom", 1)
                          if self.publish_odom else None)
        self.tf_pub = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 1)

        self.prev_time = self.get_clock().now()
        self.create_timer(1.0 / self.rate_hz, self.on_timer)

    def on_cmd(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def step(self, dt):
        if (self.cmd_timeout > 0.0 and self.last_cmd is not None
                and (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9
                > self.cmd_timeout):
            self.vx = self.wz = 0.0

        # unicycle pose integration (midpoint on heading)
        self.th += self.wz * dt
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt

        # differential drive: +y side speeds up on a left turn
        half = self.track_width / 2.0
        v_pos_y = self.vx + self.wz * half
        v_neg_y = self.vx - self.wz * half
        self.ang_pos_y = self.wrap(self.ang_pos_y + v_pos_y / self.wheel_radius * dt)
        self.ang_neg_y = self.wrap(self.ang_neg_y + v_neg_y / self.wheel_radius * dt)

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def publish(self, now):
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = WHEELS_POS_Y + WHEELS_NEG_Y
        js.position = [self.ang_pos_y] * 2 + [self.ang_neg_y] * 2
        self.js_pub.publish(js)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = yaw_to_quat(self.th)
        self.tf_pub.sendTransform(tf)

        if self.odom_pub is not None:
            od = Odometry()
            od.header.stamp = now.to_msg()
            od.header.frame_id = self.odom_frame
            od.child_frame_id = self.base_frame
            od.pose.pose.position.x = self.x
            od.pose.pose.position.y = self.y
            od.pose.pose.orientation = yaw_to_quat(self.th)
            od.twist.twist.linear.x = self.vx
            od.twist.twist.angular.z = self.wz
            self.odom_pub.publish(od)

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        if dt <= 0.0:          # bag/sim time jumped backwards
            return
        self.step(dt)
        self.publish(now)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
