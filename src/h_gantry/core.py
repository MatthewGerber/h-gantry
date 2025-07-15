import json
import logging
import math
import os.path
from datetime import timedelta
from typing import Tuple, List

from smbus2 import SMBus

from raspberry_py.gpio import CkPin
from raspberry_py.gpio.adc import ADS7830
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.controls import Joystick
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoUln2003


class HGantry:
    """
    Control for a two-axis gantry using two fixed-position stepper motors.
    """

    def __init__(
            self,
            joystick_z_pin: CkPin,
            left_stepper: Stepper,
            right_stepper: Stepper,
            left_limit_switch_arduino_pin: int,
            right_limit_switch_arduino_pin: int,
            bottom_limit_switch_arduino_pin: int,
            top_limit_switch_arduino_pin: int,
            arduino_serial: LockingSerial,
            timing_pulley_dia_mm: float,
            state_path: str
    ):
        """
        Initialize the gantry.

        :param joystick_z_pin: Joystick z pin.
        :param left_stepper: Left stepper.
        :param right_stepper: Right stepper.
        :param left_limit_switch_arduino_pin: Left limit-switch pin on the Arduino.
        :param right_limit_switch_arduino_pin: Right limit-switch pin on the Arduino.
        :param bottom_limit_switch_arduino_pin: Bottom limit-switch pin on the Arduino.
        :param top_limit_switch_arduino_pin: Top limit-switch pin on the Arduino.
        :param arduino_serial: Arduino serial connection.
        :param timing_pulley_dia_mm: Diameter of the timing pulley.
        :param state_path: Path to file to read/write state information.
        """

        self.joystick_z_pin = joystick_z_pin
        self.left_stepper = left_stepper
        self.right_stepper = right_stepper
        self.left_limit_switch_arduino_pin = left_limit_switch_arduino_pin
        self.right_limit_switch_arduino_pin = right_limit_switch_arduino_pin
        self.bottom_limit_switch_arduino_pin = bottom_limit_switch_arduino_pin
        self.top_limit_switch_arduino_pin = top_limit_switch_arduino_pin
        self.arduino_serial = arduino_serial
        self.timing_pulley_dia_mm = timing_pulley_dia_mm
        self.state_path = state_path

        left_driver = self.left_stepper.driver
        assert isinstance(left_driver, StepperMotorDriverArduinoUln2003)
        self.left_driver = left_driver
        assert self.left_driver.asynchronous

        right_driver = self.right_stepper.driver
        assert isinstance(right_driver, StepperMotorDriverArduinoUln2003)
        self.right_driver = right_driver
        assert self.right_driver.asynchronous

        # calculate timing pulley circumference and steps/mm based on pulley diameter
        self.timing_pulley_circ_mm = math.pi * self.timing_pulley_dia_mm
        self.timing_pulley_mm_per_degree = self.timing_pulley_circ_mm / 360.0
        self.steps_per_mm = self.left_stepper.steps_per_degree / self.timing_pulley_mm_per_degree

        self.cart_width_mm = 50.0
        self.cart_depth_mm = 50.0

        if os.path.exists(self.state_path):
            logging.info(f'Loading state from file:  {self.state_path}')
            with open(self.state_path, 'r') as f:
                state = json.loads(f.read())
            for attribute, value in state.items():
                setattr(self, attribute, value)
        else:
            logging.info(f'No state file exists:  {self.state_path}')
            self.x = 0.0
            self.y = 0.0
            self.left_right_mm = 0.0
            self.bottom_top_mm = 0.0

        # create an a/d converter for the joystick and rescale the digital outputs to be in [-1, 1].
        # joystick_y_ad_channel = 0
        # joystick_x_ad_channel = 1
        # self.adc = ADS7830(
        #     input_voltage=3.3,
        #     bus=SMBus('/dev/i2c-1'),
        #     address=ADS7830.ADDRESS,
        #     command=ADS7830.COMMAND,
        #     channel_rescaled_range={
        #         joystick_y_ad_channel: (-5.0, 5.0),
        #         joystick_x_ad_channel: (-5.0, 5.0)
        #     }
        # )
        #
        # # create a joystick. invert the y-axis values so that pushing forward increases them. center the gantry on
        # # joystick press and move otherwise.
        # self.joystick = Joystick(
        #     adc=self.adc,
        #     x_channel=joystick_x_ad_channel,
        #     y_channel=joystick_y_ad_channel,
        #     z_pin=self.joystick_z_pin,
        #     invert_y=True
        # )
        # self.joystick.event(lambda s: (
        #     self.center(HGantry.get_speed_from_joystick_state(s)) if s.z
        #     else self.move_to_offset(
        #         s.x,
        #         s.y,
        #         HGantry.get_speed_from_joystick_state(s)
        #     )
        # ))

    @staticmethod
    def get_speed_from_joystick_state(
            joystick_state: Joystick.State
    ) -> float:
        """
        Get speed from a joystick state.

        :param joystick_state: State.
        :return: Speed.
        """

        return math.sqrt(joystick_state.x ** 2 + joystick_state.y ** 2)

    def start(
            self
    ):
        """
        Start the gantry.
        """

        # write 3 init commands:  2 steppers and the limit switches
        self.arduino_serial.write_then_read((3).to_bytes(1), 0, False)
        self.left_stepper.start()
        self.right_stepper.start()
        limit_switches_inited = bool(self.arduino_serial.write_then_read(
            (1).to_bytes(1) +  # init
            (2).to_bytes(1) +  # limit switches
            self.left_limit_switch_arduino_pin.to_bytes(1) +
            self.right_limit_switch_arduino_pin.to_bytes(1) +
            self.bottom_limit_switch_arduino_pin.to_bytes(1) +
            self.top_limit_switch_arduino_pin.to_bytes(1),
            1,
            False
        ))
        if not limit_switches_inited:
            raise ValueError('Failed to initialize Arduino limit switches.')

        # self.joystick.start_updating_state(0.5)

    def stop(
            self,
            save_state: bool
    ):
        """
        Stop the gantry.

        :param save_state: Whether to save the gantry's state after stopping.
        """

        # self.joystick.stop_updating_state()
        # self.adc.close()

        self.arduino_serial.write_then_read((2).to_bytes(1), 0, False)
        self.left_stepper.stop()
        self.right_stepper.stop()

        if save_state:
            logging.info(f'Saving state to file:  {self.state_path}')
            with open(self.state_path, 'w') as f:
                f.write(json.dumps({
                    'x': self.x,
                    'y': self.y,
                    'left_right_mm': self.left_right_mm,
                    'bottom_top_mm': self.bottom_top_mm
                }))
        else:
            logging.warning('Not saving gantry state.')

    def move_to_home_limit(
            self,
            mm_per_sec: float
    ):
        """
        Home the gantry to the left-bottom corner.

        :param mm_per_sec: Speed.
        """

        self.move_to_left_limit(mm_per_sec)
        self.move_to_bottom_limit(mm_per_sec)

    def center(
            self,
            mm_per_sec: float
    ):
        """
        Center the gantry.

        :param mm_per_sec: Speed.
        """

        if self.move_to_point(self.left_right_mm / 2.0, self.bottom_top_mm / 2.0, mm_per_sec):
            pass
        else:
            raise ValueError('Centering should never hit a limit switch.')

    def move_to_left_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the left limit.

        :param mm_per_sec: speed.
        """

        while self.move_to_point(self.x - 100.0, self.y, mm_per_sec):
            pass

    def move_to_right_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the right limit.

        :param mm_per_sec: Speed.
        """

        while self.move_to_point(self.x + 100.0, self.y, mm_per_sec):
            pass

    def move_to_bottom_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the bottom limit.

        :param mm_per_sec: Speed.
        """

        while self.move_to_point(self.x, self.y - 100.0, mm_per_sec):
            pass

    def move_to_top_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the top limit.

        :param mm_per_sec: Speed.
        """

        while self.move_to_point(self.x, self.y + 100.0, mm_per_sec):
            pass

    def calibrate(
            self,
            mm_per_sec: float
    ):
        """
        Calibrate the gantry by measuring unknown positions and dimensions.

        :param mm_per_sec: Speed.
        """

        # measure distance between horizontal limits
        self.move_to_left_limit(mm_per_sec)
        left_x = self.x
        self.move_to_right_limit(mm_per_sec)
        right_x = self.x
        self.left_right_mm = self.x = right_x - left_x

        # measure distance between vertical limits
        self.move_to_bottom_limit(mm_per_sec)
        bottom_y = self.y
        self.move_to_top_limit(mm_per_sec)
        top_y = self.y
        self.bottom_top_mm = self.y = top_y - bottom_y

    def get_move_to(
            self,
            x: float,
            y: float
    ) -> Tuple[float, float]:
        """
        Get the movement to a point.

        :param x: Point's x position.
        :param y: Point's y position.
        :return: Movement.
        """

        return x - self.x, y - self.y

    def move_to_point(
            self,
            x: float,
            y: float,
            mm_per_sec: float
    ) -> bool:
        """
        Move to a point.

        :param x: X coordinate to move to.
        :param y: Y coordinate to move to.
        :param mm_per_sec: Speed in mm per second.
        :return: True if move was achieved without hitting a limit switch; False if limit switch was hit before move was
        achieved.
        """

        # calculate x steps (left/right) and y steps (down/up) to move
        move_x_mm, move_y_mm = self.get_move_to(x, y)
        x_steps = move_x_mm * self.steps_per_mm
        y_steps = move_y_mm * self.steps_per_mm

        # assign steps to the left/right motors. consider the following:
        #
        # move left (x_steps < 0):  each stepper takes x_steps
        # move right (x_steps > 0):  each stepper takes x_steps
        # move down (y_steps < 0):  left stepper takes y_steps; right stepper takes -y_steps
        # move up (y_steps > 0):  left stepper takes y_steps; right stepper takes -y_steps
        #
        # the combined x and y movement is achieved by adding x movement to y movement:
        #
        # left stepper steps:  x_steps + y_steps
        # right stepper steps:  x_steps - y_steps
        #
        left_stepper_steps = int(x_steps + y_steps)
        right_stepper_steps = int(x_steps - y_steps)

        # calculate time to step directly to the new point
        distance_mm = math.sqrt(move_x_mm ** 2 + move_y_mm ** 2)
        time_to_step = timedelta(seconds=distance_mm / mm_per_sec)

        # step motors. they're asserted to operate asynchronously, so the return value will be a function that returns
        # the stepper identifier and the number of skipped steps due to limiting.
        left_stepper_has_steps = left_stepper_steps != 0
        right_stepper_has_steps = right_stepper_steps != 0
        num_commands = left_stepper_has_steps + right_stepper_has_steps
        self.arduino_serial.write_then_read(num_commands.to_bytes(1), 0, False)
        get_result_functions = []
        if left_stepper_has_steps:
            get_result_functions.append(self.left_stepper.step(left_stepper_steps, time_to_step))
        else:
            get_result_functions.append(lambda: (self.left_stepper.id, 0.0))
        if right_stepper_has_steps:
            get_result_functions.append(self.right_stepper.step(right_stepper_steps, time_to_step))
        else:
            get_result_functions.append(lambda: (self.right_stepper.id, 0.0))

        # process results, obtaining skipped steps for each stepper. calculate skipped distances from skipped steps.
        left_stepper_skipped_steps, right_stepper_skipped_steps = [
            skipped_steps
            for _, skipped_steps in sorted([  # tuples have stepper id in first tuple element
                get_result()
                for get_result in get_result_functions
            ])
        ]
        skipped_x_mm, skipped_y_mm = self.get_x_mm_y_mm_from_steps(
            left_stepper_skipped_steps,
            right_stepper_skipped_steps
        )

        # advance x, y positions, minus any skipped movement due to limit switches.
        self.x += move_x_mm - skipped_x_mm
        self.y += move_y_mm - skipped_y_mm

        return skipped_x_mm == skipped_y_mm == 0.0

    def get_x_mm_y_mm_from_steps(
            self,
            left_stepper_steps: float,
            right_stepper_steps: float
    ) -> Tuple[float, float]:
        """
        Get x and y travel (mm) from left and right stepper steps.

        :param left_stepper_steps: Left stepper steps.
        :param right_stepper_steps: Right stepper steps.
        :return: 2-tuple of x and y travel (mm).
        """

        # see the analysis here:  https://github.com/MatthewGerber/h-gantry/blob/main/arduino/h_gantry/h_gantry.ino
        #
        # the relevant equations are:
        #
        # x_steps = (lss + rss) / 2
        # y_steps = (lss - rss) / 2
        #
        # thus:
        #
        # x_mm = x_steps / steps_per_mm
        # y_mm = y_steps / steps_per_mm

        return (
            (left_stepper_steps + right_stepper_steps) / (2.0 * self.steps_per_mm),
            (left_stepper_steps - right_stepper_steps) / (2.0 * self.steps_per_mm)
        )

    def move_to_points(
            self,
            points: List[Tuple[float, float]],
            mm_per_sec: float,
            return_to_current_position: bool
    ):
        """
        Trace a list of points.

        :param points: Points.
        :param mm_per_sec: Speed in mm per second.
        :param return_to_current_position: Whether to return to the current position after moving to the points.
        """

        if return_to_current_position:
            points = points.copy()
            points.append((self.x, self.y))

        for x, y in points:
            self.move_to_point(x, y, mm_per_sec)

    def move_to_offset(
            self,
            x_offset_mm: float,
            y_offset_mm: float,
            mm_per_sec: float
    ) -> bool:
        """
        Move to an offset from the current position.

        :param x_offset_mm: X offset.
        :param y_offset_mm: Y offset.
        :param mm_per_sec: Speed in mm per second.
        :return: True if move was achieved without hitting a limit switch; False if limit switch was hit before move was
        achieved.
        """

        return self.move_to_point(self.x + x_offset_mm, self.y + y_offset_mm, mm_per_sec)


def generate_circle_points(
        center_x: float,
        center_y: float,
        radius: float,
        step_angle_deg: float
) -> List[Tuple[float, float]]:
    """
    Generate circle points.

    :param center_x: Center X.
    :param center_y: Center Y.
    :param radius: Radius.
    :param step_angle_deg: Step angle (degrees).
    :return: Points of the circle.
    """

    step_angle_rad = math.radians(step_angle_deg)
    points = []
    curr_angle = 0.0
    while curr_angle <= 2.0 * math.pi:
        x = center_x + radius * math.cos(curr_angle)
        y = center_y + radius * math.sin(curr_angle)
        points.append((x, y))
        curr_angle += step_angle_rad

    return points
