import math
from datetime import timedelta
from typing import Tuple, List

from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoUln2003


class HGantry:
    """
    Control for a two-axis gantry using two fixed-position stepper motors.
    """

    def __init__(
            self,
            left_stepper: Stepper,
            right_stepper: Stepper,
            left_limit_switch_arduino_pin: int,
            right_limit_switch_arduino_pin: int,
            bottom_limit_switch_arduino_pin: int,
            top_limit_switch_arduino_pin: int,
            arduino_serial: LockingSerial,
            timing_pulley_dia_mm: float
    ):
        self.left_stepper = left_stepper
        self.right_stepper = right_stepper
        self.left_limit_switch_arduino_pin = left_limit_switch_arduino_pin
        self.right_limit_switch_arduino_pin = right_limit_switch_arduino_pin
        self.bottom_limit_switch_arduino_pin = bottom_limit_switch_arduino_pin
        self.top_limit_switch_arduino_pin = top_limit_switch_arduino_pin
        self.arduino_serial = arduino_serial
        self.timing_pulley_dia_mm = timing_pulley_dia_mm

        left_driver = self.left_stepper.driver
        assert isinstance(left_driver, StepperMotorDriverArduinoUln2003)
        self.left_driver = left_driver

        right_driver = self.right_stepper.driver
        assert isinstance(right_driver, StepperMotorDriverArduinoUln2003)
        self.right_driver = right_driver

        self.x = 0
        self.y = 0

        self.timing_pulley_circ_mm = self.timing_pulley_dia_mm * math.pi
        self.timing_pulley_mm_per_degree = self.timing_pulley_circ_mm / 360.0
        self.steps_per_mm = self.left_stepper.steps_per_degree / self.timing_pulley_mm_per_degree

        self.left_limit_step = 0
        self.right_limit_step = 0
        self.left_right_steps = 0
        self.left_right_degrees = 0.0
        self.left_right_mm = 0.0
        self.left_right_mm_per_sec = 0.0

        self.bottom_limit_step = 0
        self.top_limit_step = 0
        self.bottom_top_steps = 0
        self.bottom_top_degrees = 0.0
        self.bottom_top_mm = 0.0
        self.bottom_top_mm_per_sec = 0.0
        self.calibrated = False

    def start(
            self
    ):
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
        assert limit_switches_inited

    def stop(
            self
    ):
        self.arduino_serial.write_then_read((2).to_bytes(1), 0, False)
        self.left_stepper.stop()
        self.right_stepper.stop()

    def home(
            self
    ):
        self.move_to_left_limit()
        self.move_to_bottom_limit()

    def move_to_left_limit(
            self
    ):
        self.x = 0

    def move_to_right_limit(
            self
    ):
        pass

    def move_to_bottom_limit(
            self
    ):
        self.y = 0

    def move_to_top_limit(
            self
    ):
        pass

    def calibrate(
            self
    ):
        # self.move_to_left_limit()
        # self.left_limit_step = self.left_stepper.state.step
        # move_start = time()
        # self.move_to_right_limit()
        # move_end = time()
        # self.right_limit_step = self.left_stepper.state.step
        # self.left_right_steps = abs(self.right_limit_step - self.left_limit_step)
        # self.left_right_degrees = self.left_right_steps / self.left_stepper.steps_per_degree
        # self.left_right_mm = self.left_right_degrees * self.timing_pulley_mm_per_degree
        # self.left_right_mm_per_sec = self.left_right_mm (move_end - move_start)

        self.calibrated = True

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
    ):
        """
        Move to a point.

        :param x: X coordinate.
        :param y: Y coordinate.
        :param mm_per_sec: Speed in mm per second.
        """

        if not self.calibrated:
            raise ValueError('Must calibrate before moving.')

        # calculate x and y steps to move
        move_x_mm, move_y_mm = self.get_move_to(x, y)
        x_steps = int(move_x_mm * self.steps_per_mm)
        y_steps = int(move_y_mm * self.steps_per_mm)

        # assign steps to the motors
        left_stepper_steps = x_steps + y_steps
        right_stepper_steps = x_steps - y_steps

        # calculate time to step
        distance_mm = math.sqrt(move_x_mm ** 2 + move_y_mm ** 2)
        time_to_step = timedelta(seconds=distance_mm / mm_per_sec)

        # step motors
        left_stepper_has_steps = left_stepper_steps != 0
        right_stepper_has_steps = right_stepper_steps != 0
        num_commands = left_stepper_has_steps + right_stepper_has_steps
        self.arduino_serial.write_then_read(num_commands.to_bytes(1), 0, False)
        get_result_functions = []
        if left_stepper_has_steps:
            get_result_functions.append(self.left_stepper.step(left_stepper_steps, time_to_step))
        else:
            get_result_functions.append(lambda: f'{self.left_stepper.id},0')
        if right_stepper_has_steps:
            get_result_functions.append(self.right_stepper.step(right_stepper_steps, time_to_step))
        else:
            get_result_functions.append(lambda: f'{self.right_stepper.id},0')

        # process results, obtaining skipped drives/steps if any.
        stepper_id_skipped_steps = sorted([
            (stepper_id, skipped_drives / 2.0)  # each half step uses two drives
            for get_result in get_result_functions
            for stepper_id, skipped_drives in [tuple(int(s) for s in get_result().split(','))]
        ])
        assert len(stepper_id_skipped_steps) == 2
        print(f'Step results:  {stepper_id_skipped_steps}')

        # advance x, y positions, minus any skipped movement due to limit switches.
        skipped_x_mm, skipped_y_mm = self.get_x_mm_y_mm_from_steps(
            stepper_id_skipped_steps[0][1],
            stepper_id_skipped_steps[1][1]
        )
        self.x += move_x_mm - skipped_x_mm
        self.y += move_y_mm - skipped_y_mm

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
