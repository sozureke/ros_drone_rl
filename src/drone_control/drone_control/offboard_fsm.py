import time
from enum import Enum, auto

class FSMState(Enum):
    INIT = auto()
    ARM = auto()
    SET_MODE = auto()
    TAKEOFF = auto()
    HOVER = auto()
    ALIGN = auto()
    DESCEND = auto()
    DISARM = auto()
    COMPLETED = auto()
    FAILED = auto()

class OffboardFSM:
    def __init__(self, gateway, hover_altitude: float = 2.0):
        self._gateway = gateway
        self._hover_altitude = hover_altitude
        self._state = FSMState.INIT
        self._start_time = time.time()

    def reset(self, platform_x: float, platform_y: float, platform_z: float):
        self.platform_x = platform_x
        self.platform_y = platform_y
        self.platform_z = platform_z
        self._state = FSMState.INIT
        self._start_time = time.time()

    def step(self, drone_state, platform_state):
        now = time.time()
        if self._state == FSMState.INIT:
            self._state = FSMState.ARM

        elif self._state == FSMState.ARM:
            if self._gateway.arm():
                self._state = FSMState.SET_MODE
            else:
                self._state = FSMState.FAILED

        elif self._state == FSMState.SET_MODE:
            if self._gateway.set_offboard_mode():
                self._state = FSMState.TAKEOFF
                self._start_time = now
            else:
                self._state = FSMState.FAILED

        elif self._state == FSMState.TAKEOFF:
            if abs(drone_state.z - self._hover_altitude) < 0.1:
                self._state = FSMState.HOVER
            elif now - self._start_time > 10.0:
                self._state = FSMState.FAILED
            else:
                self._gateway.publish_position(
                    drone_state.x,
                    drone_state.y,
                    self._hover_altitude
                )

        elif self._state == FSMState.HOVER:
            self._state = FSMState.ALIGN
            self._start_time = now

        elif self._state == FSMState.ALIGN:
            dx = platform_state.x - drone_state.x
            dy = platform_state.y - drone_state.y
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self._state = FSMState.DESCEND
                self._start_time = now
            elif now - self._start_time > 15.0:
                self._state = FSMState.FAILED
            else:
                self._gateway.publish_position(
                    platform_state.x,
                    platform_state.y,
                    drone_state.z
                )

        elif self._state == FSMState.DESCEND:
            if drone_state.z - platform_state.z < 0.05:
                self._state = FSMState.DISARM
            elif now - self._start_time > 20.0:
                self._state = FSMState.FAILED
            else:
                self._gateway.publish_position(
                    platform_state.x,
                    platform_state.y,
                    platform_state.z + 0.05
                )

        elif self._state == FSMState.DISARM:
            if self._gateway.disarm():
                self._state = FSMState.COMPLETED
            else:
                self._state = FSMState.FAILED

        feedback = {
            'state': self._state.name,
            'drone_z': drone_state.z,
            'platform_z': platform_state.z
        }
        done = self._state in (FSMState.COMPLETED, FSMState.FAILED)
        success = (self._state == FSMState.COMPLETED)
        return feedback, done, success
