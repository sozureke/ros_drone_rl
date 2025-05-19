#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from drone_interfaces.action import LandOnPlatform
from drone_interfaces.msg import DroneState as DroneStateMsg, PlatformState as PlatformStateMsg
from drone_control.command_gateway import CommandGateway
from drone_control.offboard_fsm import OffboardFSM


class FSMActionServer(Node):
    def __init__(self):
        super().__init__('fsm_action_server')

        self._gateway = CommandGateway()
        self._fsm = OffboardFSM(self._gateway)

        self._current_drone_state = None
        self._current_platform_state = None

        self.create_subscription(
            DroneStateMsg, 'drone/state', self._drone_cb, 10)
        self.create_subscription(
            PlatformStateMsg, 'platform/state', self._platform_cb, 10)

        self._action_server = ActionServer(
            self,
            LandOnPlatform,
            'land_on_platform',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self._fb = LandOnPlatform.Feedback()
        self._res = LandOnPlatform.Result()

    def _drone_cb(self, msg):      self._current_drone_state = msg
    def _platform_cb(self, msg):   self._current_platform_state = msg
    def goal_callback(self, _):    return GoalResponse.ACCEPT
    def cancel_callback(self, _):  return CancelResponse.ACCEPT

    def execute_callback(self, handle):
        req = handle.request

        self._fsm.reset(req.platform_x, req.platform_y, req.platform_z)
        self._fsm._hover_altitude = req.hover_altitude

        while self._current_drone_state is None or self._current_platform_state is None:
            time.sleep(0.05)

        done, success = False, False
        while not done:
            _, done, success = self._fsm.step(
                self._current_drone_state,
                self._current_platform_state)

            self._fb.current_state = self._current_drone_state
            self._fb.platform_state = self._current_platform_state
            handle.publish_feedback(self._fb)

            time.sleep(0.05)

        self._res.success = success
        self._res.message = 'Landed' if success else 'Failed'
        handle.succeed()
        return self._res


def main(args=None):
    rclpy.init(args=args)
    node = FSMActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
