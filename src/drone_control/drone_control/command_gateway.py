#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleCommand, TrajectorySetpoint


class CommandGateway(Node):
    def __init__(self):
        super().__init__('command_gateway')

        self._cmd_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', 10)
        self._sp_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)


    def _send_cmd(self, cmd: int, p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1_000
        msg.command = int(cmd)
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = msg.source_system = 1
        msg.target_component = msg.source_component = 1
        self._cmd_pub.publish(msg)

    def set_offboard_mode(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                       p1=1 << 7, p2=4)

    def arm(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)

    def disarm(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=0.0)

    def publish_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        sp = TrajectorySetpoint()
        sp.timestamp = self.get_clock().now().nanoseconds // 1_000
        sp.position = [float(x), float(y), float(-z)]
        sp.yaw = float(yaw)
        self._sp_pub.publish(sp)

    def publish_velocity(self, vx: float, vy: float, vz: float, yawspeed: float = 0.0):
        sp = TrajectorySetpoint()
        sp.timestamp = self.get_clock().now().nanoseconds // 1_000
        sp.velocity = [float(vx), float(vy), float(-vz)]
        sp.yawspeed = float(yawspeed)
        self._sp_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = CommandGateway()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
