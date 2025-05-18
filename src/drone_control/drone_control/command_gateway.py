import rclpy
from rclpy.node import Node
from drone_interfaces.srv import SetHoverAltitude, EnablePlatformMotion
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist


class CommandGateway(Node):
    def __init__(self):
        super().__init__('command_gateway')
        self._cmd_pub = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)
        self._vel_pub = self.create_publisher(Twist, 'fmu/in/trajectory_setpoint', 10)
        self._hover_client = self.create_client(SetHoverAltitude, 'set_hover_altitude')
        self._motion_client = self.create_client(EnablePlatformMotion, 'enable_platform_motion')

    def _send_vehicle_command(self, command: int,
                              param1: float = 0.0,
                              param2: float = 0.0,
                              param3: float = 0.0,
                              param4: float = 0.0) -> bool:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        self._cmd_pub.publish(msg)
        return True

    def set_offboard_mode(self) -> bool:
        OFFBOARD_BASE_MODE = 1 << 7            # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        OFFBOARD_CUSTOM_MODE = 4              # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        return self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=OFFBOARD_BASE_MODE,
            param2=OFFBOARD_CUSTOM_MODE
        )

    def arm(self) -> bool:
        return self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    def disarm(self) -> bool:
        return self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    def publish_position(self, x: float, y: float, z: float):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        self._vel_pub.publish(msg)

    def publish_velocity(self, vx: float, vy: float, vz: float):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        self._vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandGateway()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
