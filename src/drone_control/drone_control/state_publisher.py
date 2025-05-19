#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from drone_interfaces.msg import DroneState, PlatformState
import tf_transformations as tf


class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')

        self._drone_state = DroneState()
        self._platform_state = PlatformState()

        self._drone_pub = self.create_publisher(
            DroneState, 'drone/state', 10)
        self._platform_pub = self.create_publisher(
            PlatformState, 'platform/state', 10)

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._on_drone_odom,
            10)

        self.create_subscription(
            Odometry,
            '/platform/odom',
            self._on_platform_odom,
            10)

        self.create_timer(0.02, self._publish_states)

    def _on_drone_odom(self, msg: VehicleOdometry):
        self._drone_state.x, self._drone_state.y, self._drone_state.z = msg.position
        self._drone_state.vx, self._drone_state.vy, self._drone_state.vz = msg.velocity

        # PX4: q = [w x y z]  → tf: [x y z w]
        q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]
        roll, pitch, yaw = tf.euler_from_quaternion(q)
        self._drone_state.roll = roll
        self._drone_state.pitch = pitch
        self._drone_state.yaw = yaw

    def _on_platform_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._platform_state.x = p.x
        self._platform_state.y = p.y
        self._platform_state.z = p.z
        self._platform_state.vx = v.x
        self._platform_state.vy = v.y

    def _publish_states(self):
        self._drone_pub.publish(self._drone_state)
        self._platform_pub.publish(self._platform_state)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
