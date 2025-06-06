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
        self.mm_per_sec = 0.0
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
        self.mm_per_sec = 10.0
        self.calibrated = True

    def move_x(
            self,
            mm: float
    ):
        if not self.calibrated:
            raise ValueError('Must calibrate before moving.')

        steps = int(mm * self.steps_per_mm)
        if steps != 0:
            time_to_step = timedelta(seconds=abs(mm) / self.mm_per_sec)
            self.left_stepper.step(steps, time_to_step)
            self.right_stepper.step(steps, time_to_step)
            results = sorted([self.left_driver.wait_for_async_result(), self.right_driver.wait_for_async_result()])
            self.x += mm

    def move_y(
            self,
            mm: float
    ):
        if not self.calibrated:
            raise ValueError('Must calibrate before moving.')

        steps = -int(mm * self.steps_per_mm)
        if steps != 0:
            time_to_step = timedelta(seconds=abs(mm) / self.mm_per_sec)
            self.left_stepper.step(-steps, time_to_step)
            self.right_stepper.step(steps, time_to_step)
            results = sorted([self.left_driver.wait_for_async_result(), self.right_driver.wait_for_async_result()])
            self.y += mm

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
            y: float
    ):
        """
        Move to a point.

        :param x: X coordinate.
        :param y: Y coordinate.
        """

        move_x, move_y = self.get_move_to(x, y)
        self.move_x(move_x)
        self.move_y(move_y)

    def trace_points(
            self,
            points: List[Tuple[float, float]]
    ):
        """
        Trace a list of points.

        :param points: Points.
        """

        for x, y in points:
            self.move_to_point(x, y)


def generate_circle_points(
        center_x: float,
        center_y: float,
        radius: float,
        step_angle: float
) -> List[Tuple[float, float]]:
    """
    Generate circle points.

    :param center_x: Center X.
    :param center_y: Center Y.
    :param radius: Radius.
    :param step_angle: Step angle.
    :return: Points.
    """

    step_angle = math.radians(step_angle)
    points = []
    angle = 0.0
    while angle < 2.0 * math.pi:
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        points.append((x, y))
        angle += step_angle

    return points
