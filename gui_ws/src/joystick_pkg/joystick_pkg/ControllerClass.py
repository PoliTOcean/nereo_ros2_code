from inputs import get_gamepad
import math
import threading
import rclpy
from rclpy.logging import get_logger

class Controller(object):
    MAX_TRIG_VAL = 1.0
    MAX_JOY_VAL = 1.0

    def __init__(self):
        self.LeftJoystickY = 0.0
        self.LeftJoystickX = 0.0
        self.RightJoystickY = 0.0
        self.RightJoystickX = 0.0
        self.LeftTrigger = 0.0
        self.RightTrigger = 0.0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0.0
        self.RightThumb = 0.0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def get_logger(self):
        return get_logger('joystick_pkg.Controller')

    def read(self):
        data = {}
        data['leftJoy'] = [self.LeftJoystickX, self.LeftJoystickY]
        data['rightJoy'] = [self.RightJoystickX, self.RightJoystickY]
        data['middleBtn'] = self.Back # CHECK IF IT IS THE CORRECT ONE
        data['leftTrigger'] = self.LeftTrigger
        data['rightTrigger'] = self.RightTrigger
        data['pad'] = [self.LeftDPad, self.RightDPad, self.DownDPad, self.UpDPad]
        return data
        

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                self.get_logger().info(f'Received event: code={event.code}, state={event.state}')  # Debug log
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / Controller.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / Controller.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state
                elif event.code == 'BTN_WEST':
                    self.X = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'ABS_HAT0X':
                    self.LeftDPad = 1 if event.state == -1 else 0
                    self.RightDPad = 1 if event.state == 1 else 0
                elif event.code == 'ABS_HAT0Y':
                    self.UpDPad = 1 if event.state == -1 else 0
                    self.DownDPad = 1 if event.state == 1 else 0

