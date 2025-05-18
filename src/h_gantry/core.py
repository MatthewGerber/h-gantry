from datetime import timedelta

from raspberry_py.gpio.motors import Stepper


class HGantry:

    def __init__(
            self,
            left_stepper: Stepper,
            right_stepper: Stepper
    ):
        self.left_stepper = left_stepper
        self.right_stepper = right_stepper

        self.x = 0
        self.y = 0
        self.steps_per_mm = 0.0
        self.mm_per_sec = 0.0

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
        pass

    def move_to_bottom_limit(
            self
    ):
        self.y = 0
        pass

    def calibrate(
            self
    ):
        self.steps_per_mm = 1.0
        self.mm_per_sec = 10.0

    def move_x(
            self,
            mm: float
    ):
        steps = -int(mm * self.steps_per_mm)
        time_to_step = timedelta(seconds=mm / self.mm_per_sec)
        self.right_stepper.step(steps, time_to_step)
        self.left_stepper.step(-steps, time_to_step)

    def move_y(
            self,
            mm: float
    ):
        steps = int(mm * self.steps_per_mm)
        time_to_step = timedelta(seconds=mm / self.mm_per_sec)
        self.right_stepper.step(steps, time_to_step)
        self.left_stepper.step(-steps, time_to_step)
