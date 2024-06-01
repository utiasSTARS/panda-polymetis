import time
import numpy as np
from polymetis import GripperInterface


DEFAULT_OPEN_WIDTH = 0.085


class FakeState:
    def __init__(self, width):
        self.width = width
        pass


class FakeGripperInterface:
    def __init__(self, open_width=DEFAULT_OPEN_WIDTH):
        self.state = FakeState(open_width)

    def get_state(self):
        return self.state

    def goto(self, width, *args, **kwargs):
        self.state.width = width

    def grasp(self, *args, **kwargs):
        self.state.width = 0


class PandaGripperClient:
    # Set up for 2f85
    def __init__(self, server_ip='localhost', open_width=DEFAULT_OPEN_WIDTH, grasp_force=20, grip_speed=0.05,
                 close_force=20, pinch_width=.04, not_open_means_close=True, fake=False):
        self.server_ip = server_ip
        self.open_width = open_width
        self.default_grasp_force = grasp_force
        self.default_speed = grip_speed
        self.default_close_force = close_force
        self.default_pinch_width = pinch_width
        self._max_width = open_width

        # self.gripper = GripperInterface(ip_address=self.server_ip, enforce_version=False)
        if fake:
            self.gripper = FakeGripperInterface(open_width=open_width)
        else:
            self.gripper = GripperInterface(ip_address=self.server_ip)

        self._state = "none"  # in case robot turns on with gripper not open

        if not_open_means_close:
            # check to see if already grasping -- assume grasping if not fully open
            self.get_and_update_state()
            if self._pos < (self._max_width - .001):  # a bit of tolerance
                self._state = "close"

        # internal state tracking
        self._pos = None
        self._error_code = 0

        print("2f85 gripper client initialized.")

    def get_and_update_state(self):
        state = self.gripper.get_state()
        self._pos = state.width
        # self._error_code = state.error_code
        # self._max_width = state.max_width

        return {
            "pos": state.width,
            # "max_width": state.max_width,
            # "is_moving": state.is_moving
        }

    def is_fully_open(self):
        self.get_and_update_state()
        return self._pos >= (self._max_width - .001)

    def send_move_goal(self, width, speed=None, force=None, blocking=False):
        if speed is None: speed = self.default_speed
        if force is None: force = self.default_close_force
        self.gripper.goto(width=width, speed=speed, force=force, blocking=blocking)

        return True  # backwards compatibility

    def open(self, speed=None, blocking=False, timeout=5.0):
        if speed is None: speed = self.default_speed
        if self._state != "open":
            self._state = "open"
            self.send_move_goal(width=self.open_width, speed=speed, blocking=blocking)
        if blocking:
            start_time = time.time()
            while not self.is_fully_open() and time.time() - start_time < timeout:
                time.sleep(0.1)

            if not self.is_fully_open():
                raise ValueError(f"Gripper didn't open after {timeout}s.")

        return True  # backwards compatibility

    def close(self, speed=None, force=None, blocking=False):
        if speed is None: speed = self.default_speed
        if force is None: force = self.default_grasp_force
        if self._state != "close":
            self._state = "close"
            self.grasp(force=force, width=0.0, speed=speed, blocking=blocking)

        return True  # backwards compatibility

    def grasp(self, force, width=0, ep_inner=-1, ep_outer=-1, speed=None, blocking=False):
        if speed is None: speed = self.default_speed

        # self.gripper.grasp(speed=speed, force=force, grasp_width=width, epsilon_inner=ep_inner,
        #                    epsilon_outer=ep_outer, blocking=blocking)
        self.gripper.grasp(speed=speed, force=force, blocking=blocking)

        return True  # backwards compatibility

    def pinch(self, width=None, speed=None, blocking=False):
        if width is None: width = self.default_pinch_width
        if speed is None: speed = self.default_speed
        if self._state != "pinch":
            self._state = "pinch"
            self.send_move_goal(width=width, speed=speed, blocking=blocking)

        return True  # backwards compatibility
