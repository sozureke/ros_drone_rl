import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from drone_interfaces.action import LandOnPlatform
from drone_interfaces.msg import DroneState as DroneStateMsg, PlatformState as PlatformStateMsg
from drone_control.command_gateway import CommandGateway
from .offboard_fsm import OffboardFSM


class FSMActionServer(Node):

    def __init__(self):
        super().__init__('fsm_action_server')

        self._gateway = CommandGateway()
        self._fsm = OffboardFSM(self._gateway)

        self._current_drone_state = None
        self._current_platform_state = None

        self.create_subscription(
            DroneStateMsg,
            'drone/state',
            self._drone_state_cb,
            10)
        self.create_subscription(
            PlatformStateMsg,
            'platform/state',
            self._platform_state_cb,
            10)

        self._action_server = ActionServer(
            self,
            LandOnPlatform,
            'land_on_platform',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self._feedback = LandOnPlatform.Feedback()
        self._result = LandOnPlatform.Result()

    def _drone_state_cb(self, msg: DroneStateMsg):
        self._current_drone_state = msg

    def _platform_state_cb(self, msg: PlatformStateMsg):
        self._current_platform_state = msg

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        req = goal_handle.request

        self._fsm.reset(
            req.platform_x,
            req.platform_y,
            req.platform_z
        )
        self._fsm._hover_altitude = req.hover_altitude

        while (
            self._current_drone_state is None
            or self._current_platform_state is None
        ):
            await asyncio.sleep(0.1)

        done = False
        success = False

        while not done:
            feedback_data, done, success = self._fsm.step(
                self._current_drone_state,
                self._current_platform_state
            )

            self._feedback.current_state = self._current_drone_state
            self._feedback.platform_state = self._current_platform_state
            goal_handle.publish_feedback(self._feedback)

            await asyncio.sleep(0.1)

        self._result.success = success
        self._result.message = 'Landed' if success else 'Failed'
        goal_handle.succeed()
        return self._result


def main(args=None):
    rclpy.init(args=args)
    action_server = FSMActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
