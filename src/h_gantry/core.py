import math
from datetime import timedelta
from typing import Tuple, List

from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoUln2003


class HGantry:
    """
    Control for a two-axis gantry using two fixed-position stepper motors.
    """

    def __init__(
            self,
            left_stepper: Stepper,
            right_stepper: Stepper,
            timing_pulley_tooth_count: int,
            timing_pulley_tooth_pitch_mm: float
    ):
        self.left_stepper = left_stepper
        self.right_stepper = right_stepper
        self.timing_pulley_tooth_count = timing_pulley_tooth_count
        self.timing_pulley_tooth_pitch_mm = timing_pulley_tooth_pitch_mm

        left_driver = self.left_stepper.driver
        assert isinstance(left_driver, StepperMotorDriverArduinoUln2003)
        self.left_driver = left_driver

        right_driver = self.right_stepper.driver
        assert isinstance(right_driver, StepperMotorDriverArduinoUln2003)
        self.right_driver = right_driver

        self.x = 0
        self.y = 0

        self.timing_pulley_mm = self.timing_pulley_tooth_count * self.timing_pulley_tooth_pitch_mm
        self.timing_pulley_mm_per_degree = self.timing_pulley_mm / 360.0

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

        self.steps_per_mm = 0.0
        self.calibrated = False

    def start(
            self
    ):
        self.left_stepper.start()
        self.right_stepper.start()

    def stop(
            self
    ):
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

        self.steps_per_mm = 90.0
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
        get_result_functions = []
        if left_stepper_steps != 0:
            get_result_functions.append(self.left_stepper.step(left_stepper_steps, time_to_step))
        if right_stepper_steps != 0:
            get_result_functions.append(self.right_stepper.step(right_stepper_steps, time_to_step))

        if any(get_result_functions):
            results = [get_result() for get_result in get_result_functions]
            self.x += move_x_mm
            self.y += move_y_mm

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
