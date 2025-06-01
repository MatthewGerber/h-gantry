from datetime import timedelta

from raspberry_py.gpio.motors import Stepper


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

        self.steps_per_mm = 50.0
        self.mm_per_sec = 10.0
        self.calibrated = True

    def move_x(
            self,
            mm: float
    ):
        if not self.calibrated:
            raise ValueError('Must calibrate before moving.')

        steps = int(mm * self.steps_per_mm)
        time_to_step = timedelta(seconds=abs(mm) / self.mm_per_sec)
        self.right_stepper.step(steps, time_to_step)
        self.left_stepper.step(steps, time_to_step)

    def move_y(
            self,
            mm: float
    ):
        if not self.calibrated:
            raise ValueError('Must calibrate before moving.')

        steps = -int(mm * self.steps_per_mm)
        time_to_step = timedelta(seconds=abs(mm) / self.mm_per_sec)
        self.right_stepper.step(steps, time_to_step)
        self.left_stepper.step(-steps, time_to_step)
