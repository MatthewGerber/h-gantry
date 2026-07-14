import io
import logging
import math
import os.path
import pickle
from collections import deque
from datetime import timedelta
from enum import IntEnum, Enum, auto
from threading import RLock, Lock
from time import time
from typing import Tuple, List, Optional, Union, cast

import numpy as np
from matplotlib import pyplot as plt
from smbus2 import SMBus
from spyrograph import Hypotrochoid
# noinspection PyProtectedMember
from spyrograph.core._trochoid import _Trochoid

from raspberry_py.gpio import CkPin, Component, Clock
from raspberry_py.gpio.adc import ADS7830
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.controls import Joystick
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoA4988, StepperMotorDriverAsynchronousReturn
from raspberry_py.rest.application import RpyFlask, CallImageBytes
from raspberry_py.utils import get_base_64_str


class UncalibratedError(ValueError):
    """
    Error raised when attempting to move an uncalibrated gantry.
    """


class Move:
    """
    Move record for gantry.
    """

    def __init__(
            self,
            to_x_mm: float,
            to_y_mm: float,
            move_x_mm: float,
            move_y_mm: float,
            left_stepper_steps: int,
            right_stepper_steps: int,
            start_time_epoch: float,
            time_to_step: timedelta,
            idx: int
    ):
        """
        Initialize the move.

        :param to_x_mm: To x position (mm).
        :param to_y_mm: To y position (mm).
        :param move_x_mm: X offset (mm).
        :param move_y_mm: Y offset (mm).
        :param left_stepper_steps: Left stepper steps.
        :param right_stepper_steps: Right stepper steps.
        :param start_time_epoch: Start timestamp (epoch).
        :param time_to_step: Duration of step.
        :param idx: Sequence index.
        """

        self.to_x_mm = to_x_mm
        self.to_y_mm = to_y_mm
        self.move_x_mm = move_x_mm
        self.move_y_mm = move_y_mm
        self.left_stepper_steps = left_stepper_steps
        self.right_stepper_steps = right_stepper_steps
        self.start_time_epoch = start_time_epoch
        self.time_to_step = time_to_step
        self.idx = idx

        self.end_time_epoch = self.start_time_epoch + self.time_to_step.total_seconds()

        # the driver return values are initially empty and are filled in when the move is sent to the driver
        self.get_left_driver_return_value: Optional[StepperMotorDriverAsynchronousReturn] = None
        self.get_right_driver_return_value: Optional[StepperMotorDriverAsynchronousReturn] = None


class HGantry(Component):
    """
    Control for a two-axis gantry using two fixed-position stepper motors.
    """

    class CalibrationStatus(Enum):
        """
        Calibration statuses.
        """

        # initialized but pre-calibration
        NONE = auto()

        # in the middle of the calibration process
        CALIBRATING = auto()

        # calibrated
        CALIBRATED = auto()

    class State(Component.State):
        """
        Gantry state.
        """

        def __init__(
                self,
                started: bool,
                enabled: bool,
                calibration_status: 'HGantry.CalibrationStatus',
                x: float,
                y: float,
                x_est: float,
                y_est: float
        ):
            """
            Initialize state.

            :param started: Whether the gantry has been started.
            :param enabled: Whether the gantry is enabled.
            :param calibration_status: Calibration status.
            :param x: X position based on moves known to be completed. This might be inaccurate due to buffering.
            :param y: Y position based on moves known to be completed. This might be inaccurate due to buffering.
            :param x_est: Estimated true X position based on move timing.
            :param y_est: Estimated true Y position based on move timing.
            """

            self.started = started
            self.enabled = enabled
            self.calibration_status = calibration_status
            self.x = x
            self.y = y
            self.x_est = x_est
            self.y_est = y_est

        def __eq__(
                self,
                other: object
        ) -> bool:
            """
            Check equality with another state.

            :param other: State.
            :return: True if equal and False otherwise.
            """

            if not isinstance(other, HGantry.State):
                raise ValueError(f'Expected a {HGantry.State}')

            return (
                self.started == other.started and
                self.enabled == other.enabled and
                self.calibration_status == other.calibration_status and
                self.x == other.x and
                self.y == other.y and
                self.x_est == other.x_est and
                self.y_est == other.y_est
            )

        def __str__(
                self
        ) -> str:
            """
            Get string.

            :return: String.
            """

            return (
                f'started={self.started}, enabled={self.enabled}, calibration={self.calibration_status}, '
                f'x={self.x:.3f}, y={self.y:.3f}, '
                f'x est.={self.x_est:.3f}, y est.={self.y_est:.3f}'
            )

        def set(
                self,
                **kwargs
        ) -> 'HGantry.State':
            """
            Set attributes on the current state, returning a copy with the new values.

            :param kwargs: New attributes and values.
            :return: New state.
            """

            new_attributes = self.__dict__.copy()
            new_attributes.update(kwargs)

            return HGantry.State(**new_attributes)

    class Command(IntEnum):
        """
        Commands.
        """

        INIT_LIMIT_SWITCHES = 1
        STEP = 2

    class ComponentId(IntEnum):
        """
        Components.
        """

        LIMIT_SWITCHES = 2

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
        assert isinstance(left_driver, StepperMotorDriverArduinoA4988)
        self.left_driver = left_driver
        assert self.left_driver.asynchronous

        right_driver = self.right_stepper.driver
        assert isinstance(right_driver, StepperMotorDriverArduinoA4988)
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
            with open(self.state_path, 'rb') as f:
                loaded_state = pickle.load(f)
            for attribute, value in loaded_state['attributes'].items():
                setattr(self, attribute, value)
            state: HGantry.State = loaded_state['object']
            logging.info(f'State loaded:  {state}')
        else:
            logging.info(f'No state file exists:  {self.state_path}')
            self.x = 0.0
            self.y = 0.0
            self.left_right_mm = 0.0
            self.bottom_top_mm = 0.0
            self.move_idx = 1  # the arduino driver uses idx 0 as a special "not driving" value
            state = HGantry.State(
                False, True, HGantry.CalibrationStatus.NONE, self.x, self.y, self.x, self.y
            )
            logging.info(f'State initialized:  {state}')

        super().__init__(state)

        # synchronize driver indices with gantry move index
        self.left_driver.idx = self.move_idx
        self.right_driver.idx = self.move_idx

        # move buffer management in python and arduino. these must be protected with a lock because they are accessed
        # from multiple threads (e.g., async calls from rest api).
        self.moves_pending_in_python: List[Move] = []
        self.moves_pending_in_arduino: deque[Move] = deque()
        self.min_moves_pending_in_arduino = 5  # keep moves in arduino to maintain momentum
        self.max_moves_pending_in_arduino = 50  # arduino has limited memory for its move buffer
        self.completed_move_points: List[Tuple[float, float]] = []
        self.actual_x = self.x  # self.x reflects pending moves, whereas self.actual_x reflects completed moves.
        self.actual_y = self.y  # self.y reflects pending moves, whereas self.actual_y reflects completed moves.
        self.move_lock = RLock()
        self.driver_read_lock = Lock()

        # create an a/d converter for the joystick and rescale the digital outputs to be in a range. report all state
        # updates so that we get regular joystick events even when the joystick isn't changing position. both the adc
        # and joystick report synchronous events that land at move_to_point, which uses a lock to limit step commands
        # to the arduino.
        joystick_y_ad_channel = 0
        joystick_x_ad_channel = 1
        self.adc_bus_path = '/dev/i2c-1'
        self.adc = ADS7830(
            input_voltage=3.3,
            bus=SMBus(),
            address=ADS7830.ADDRESS,
            command=ADS7830.COMMAND,
            channel_rescaled_range={
                joystick_y_ad_channel: (-1.0, 1.0),
                joystick_x_ad_channel: (-1.0, 1.0)
            }
        )
        self.adc.only_report_state_changes = False

        # create a joystick. invert the y-axis values so that pushing forward increases them. center the gantry on
        # joystick press and move otherwise. report all state updates so that we get regular joystick events even when
        # the joystick isn't changing position. both the adc and joystick report synchronous events that land at
        # move_to_point, which uses a lock to limit step commands to the arduino.
        self.joystick = Joystick(
            adc=self.adc,
            x_channel=joystick_x_ad_channel,
            y_channel=joystick_y_ad_channel,
            z_pin=self.joystick_z_pin,
            invert_y=True
        )
        self.joystick.only_report_state_changes = False
        self.joystick.event(lambda s: self.joystick_move(s))
        self.joystick_update_interval_seconds = 0.01
        self.joystick_move_mm = 10.0
        self.joystick_min_speed_mm_per_sec = 20.0
        self.joystick_max_speed_mm_per_sec = 150.0

        # plotting
        self.plot_lock = Lock()

        # blocking movements will wait for all moves to come back from the arduino, but non-blocking movements will not
        # wait. in the latter case, we need a separate signal that periodically sends/receives moves to/from the
        # arduino, thus ensuring that all moves are processed. also estimate the gantry's current location. do all of
        # this on a clock.
        self.pump_clock = Clock(0.1)
        self.pump_clock.event(self.pump)

    def pump(
            self,
            clock_state: Clock.State
    ):
        """
        Pump the buffers and position estimation.

        :param clock_state: State of clock driving the pump.
        """

        logging.debug(f'Pumping:  {clock_state.tick}')

        self.send_moves_to_arduino(True)
        self.read_completed_moves_from_arduino(False)
        self.set_state_with_estimated_location()

    def joystick_move(
            self,
            joystick_state: Joystick.State
    ):
        """
        Move according to a new joystick state.

        :param joystick_state: Joystick state.
        """

        # center on joystick press
        if joystick_state.z:
            self.center(100.0, True)

        else:

            # ignore negligible joystick movements and noise
            joystick_magnitude = math.sqrt(joystick_state.x ** 2 + joystick_state.y ** 2)
            if joystick_magnitude > 0.5:

                # move in the joystick direction as indicated by the vector norm
                move_vector = np.array([joystick_state.x, joystick_state.y])
                norm = np.linalg.norm(move_vector)
                if norm != 0.0:
                    move_x_mm, move_y_mm = (move_vector / norm) * self.joystick_move_mm
                    try:
                        self.move_to_offset(
                            move_x_mm,
                            move_y_mm,
                            self.joystick_min_speed_mm_per_sec +
                            (
                                self.joystick_max_speed_mm_per_sec -
                                self.joystick_min_speed_mm_per_sec
                            ) * (joystick_magnitude * 2.0 - 1.0),
                            True
                        )
                    except ValueError as e:
                        logging.error(f'Failed to move according to joystick:  {e}')

    def start(
            self
    ):
        """
        Start the gantry.
        """

        # switch to automatic buffering for initialization
        self.arduino_serial.manual_buffer = False
        self.left_stepper.start()
        self.right_stepper.start()
        limit_switches_inited = bool(self.arduino_serial.write_then_read(
            HGantry.Command.INIT_LIMIT_SWITCHES.to_bytes(1) +
            HGantry.ComponentId.LIMIT_SWITCHES.to_bytes(1) +
            self.left_limit_switch_arduino_pin.to_bytes(1) +
            self.right_limit_switch_arduino_pin.to_bytes(1) +
            self.bottom_limit_switch_arduino_pin.to_bytes(1) +
            self.top_limit_switch_arduino_pin.to_bytes(1),
            True,
            1,
            False
        ))
        if not limit_switches_inited:
            raise ValueError('Failed to initialize Arduino limit switches.')

        self.adc.open(self.adc_bus_path)
        self.joystick.start_updating_state(self.joystick_update_interval_seconds)

        # switch to manual buffering and start pumping
        self.arduino_serial.manual_buffer = True
        self.pump_clock.start()

        self.set_state(cast(HGantry.State, self.state).set(started=True))

    def stop(
            self
    ):
        """
        Stop the gantry.
        """

        if not cast(HGantry.State, self.state).started:
            logging.info('Gantry already stopped.')
            return

        # process the buffer to obtain current x/y position
        try:
            self.clear_move_buffer()
        except UncalibratedError:
            logging.error('Tried to clear move buffer when stopping, but gantry is uncalibrated.')

        # stop pumping and switch to automatic buffering to stop the steppers
        self.pump_clock.stop()
        self.arduino_serial.manual_buffer = False

        self.joystick.stop_updating_state()
        self.adc.close()
        self.left_stepper.stop()
        self.right_stepper.stop()

        state = cast(HGantry.State, self.state).set(started=False)
        self.set_state(state)

        # save the gantry state, which is a combination of attributes on the current python object, plus the
        # raspberry-py state object.
        logging.info(f'Saving state to file:  {self.state_path}')
        with open(self.state_path, 'wb') as f:
            with self.move_lock:
                pickle.dump({
                    'attributes': {
                        'x': self.x,
                        'y': self.y,
                        'left_right_mm': self.left_right_mm,
                        'bottom_top_mm': self.bottom_top_mm,
                        'move_idx': self.move_idx
                    },
                    'object': state
                }, f)  # type: ignore
                logging.info('Saved state.')

    def started(
            self
    ) -> bool:
        """
        Get whether the gantry has been started.

        :return: True if started and False otherwise.
        """

        return cast(HGantry.State, self.state).started

    def center(
            self,
            mm_per_sec: float,
            block: bool
    ) -> Optional[CallImageBytes]:
        """
        Center the gantry.

        :param mm_per_sec: Speed.
        :param block: Whether to block until the movement is complete.
        :return: Image of the completed drawing, which will be non-None only if `block` is True, which will wait for the
        drawing to complete.
        """

        logging.info('Centering gantry.')
        self.move_to_point(self.left_right_mm / 2.0, self.bottom_top_mm / 2.0, mm_per_sec, block)

        # we can only return an image of the drawing if we blocked and waited for it to complete
        if block:
            call_image_bytes = CallImageBytes(self.get_line_plot())
        else:
            call_image_bytes = None

        return call_image_bytes

    def move_to_left_limit(
            self,
            step_distance_mm: float,
            mm_per_sec: float
    ):
        """
        Move to the left limit.

        :param step_distance_mm: Distance (+mm) of each step toward the limit.
        :param mm_per_sec: speed.
        """

        self.move_to_offset_limit(-step_distance_mm, 0.0, mm_per_sec)

    def move_to_right_limit(
            self,
            step_distance_mm: float,
            mm_per_sec: float
    ):
        """
        Move to the right limit.

        :param step_distance_mm: Distance (+mm) of each step toward the limit.
        :param mm_per_sec: Speed.
        """

        self.move_to_offset_limit(step_distance_mm, 0.0, mm_per_sec)

    def move_to_bottom_limit(
            self,
            step_distance_mm: float,
            mm_per_sec: float
    ):
        """
        Move to the bottom limit.

        :param step_distance_mm: Distance (+mm) of each step toward the limit.
        :param mm_per_sec: Speed.
        """

        self.move_to_offset_limit(0.0, -step_distance_mm, mm_per_sec)

    def move_to_top_limit(
            self,
            step_distance_mm: float,
            mm_per_sec: float
    ):
        """
        Move to the top limit.

        :param step_distance_mm: Distance (+mm) of each step toward the limit.
        :param mm_per_sec: Speed.
        """

        self.move_to_offset_limit(0.0, step_distance_mm, mm_per_sec)

    def calibrate(
            self,
            mm_per_sec: float
    ):
        """
        Calibrate the gantry by measuring unknown positions and dimensions.

        :param mm_per_sec: Speed.
        """

        self.set_state(cast(HGantry.State, self.state).set(calibration_status=HGantry.CalibrationStatus.CALIBRATING))

        move_distance_mm = 500.0
        limit_switch_buffer_mm = 5.0

        # measure distance between horizontal limits, and set actual x position.
        self.move_to_left_limit(move_distance_mm, mm_per_sec)
        self.move_to_offset(limit_switch_buffer_mm, 0.0, mm_per_sec, True)
        left_x = self.x
        self.move_to_right_limit(move_distance_mm, mm_per_sec)
        self.move_to_offset(-limit_switch_buffer_mm, 0.0, mm_per_sec, True)
        right_x = self.x
        self.left_right_mm = self.actual_x = self.x = right_x - left_x

        # measure distance between vertical limits, and set actual y position.
        self.move_to_bottom_limit(move_distance_mm, mm_per_sec)
        self.move_to_offset(0.0, limit_switch_buffer_mm, mm_per_sec, True)
        bottom_y = self.y
        self.move_to_top_limit(move_distance_mm, mm_per_sec)
        self.move_to_offset(0.0, -limit_switch_buffer_mm, mm_per_sec, True)
        top_y = self.y
        self.bottom_top_mm = self.actual_y = self.y = top_y - bottom_y

        self.set_state(cast(HGantry.State, self.state).set(calibration_status=HGantry.CalibrationStatus.CALIBRATED))

        self.clear_point_history()
        self.completed_move_points = [(self.x, self.y)]

    def get_calibration_status(
            self
    ) -> 'HGantry.CalibrationStatus':
        """
        Get calibration status.

        :return: Status.
        """

        return cast(HGantry.State, self.state).calibration_status

    def get_move_to(
            self,
            x: float,
            y: float
    ) -> Tuple[float, float]:
        """
        Get the movement to a point in terms of x and y distances (mm).

        :param x: Point's x position.
        :param y: Point's y position.
        :return: Movement as a 2-tuple of x and y distances, each in mm.
        """

        with self.move_lock:
            return x - self.x, y - self.y

    def assert_point_in_bounds(
            self,
            x: float,
            y: float
    ):
        """
        Assert that a point is in bounds.

        :param x: X.
        :param y: Y.
        """

        if not (0.0 <= x <= self.left_right_mm and 0.0 <= y <= self.bottom_top_mm):
            raise ValueError(
                f'Point ({x}, {y}) is out of bounds [0.0, {self.left_right_mm}], [0.0, {self.bottom_top_mm}].'
            )

    @staticmethod
    def get_distance_to_offset(
            x_offset: float,
            y_offset: float
    ) -> float:
        """
        Get straight-line distance when moving given x and y distances.

        :param x_offset: x distance.
        :param y_offset: y distance.
        :return: Straight-line distance.
        """

        return math.sqrt(x_offset ** 2 + y_offset ** 2)

    @staticmethod
    def get_distance_between_points(
            p1: Tuple[float, float],
            p2: Tuple[float, float]
    ) -> float:
        """
        Get distance between points.

        :param p1: Point 1.
        :param p2: Point 2.
        :return: Distance.
        """

        return HGantry.get_distance_to_offset(p2[0] - p1[0], p2[1] - p1[1])

    def get_distance_to_point(
            self,
            point: Tuple[float, float]
    ) -> float:
        """
        Get distance from the gantry's effective current position to a given point.

        :return: Distance (mm).
        """

        return self.get_distance_to_offset(*self.get_move_to(*point))

    def move_to_point(
            self,
            x: float,
            y: float,
            mm_per_sec: float,
            block: bool
    ) -> Optional[bool]:
        """
        Move to a point.

        :param x: X coordinate to move to.
        :param y: Y coordinate to move to.
        :param mm_per_sec: Speed in mm per second.
        :param block: Whether to block until the movement is complete. If True, then the move (and all pending moves)
        will be completed, and the return value will be non-None. If False, then the return value will be None.
        :return: None if `block` is False and non-None if True. When non-None:  True if move was completed without
        hitting a limit switch; False if limit switch was hit before move was completed.
        """

        if not self.started():
            raise ValueError('Gantry has not been started. Cannot move.')

        if self.get_calibration_status() not in [
            HGantry.CalibrationStatus.CALIBRATING, HGantry.CalibrationStatus.CALIBRATED
        ]:
            raise UncalibratedError('Cannot move unless calibrating or calibrated.')

        # when we're calibrating, we intentionally move in ways that test the boundaries of the gantry. don't check
        # bounds when calibrating.
        if self.get_calibration_status() != HGantry.CalibrationStatus.CALIBRATING:
            self.assert_point_in_bounds(x, y)

        with self.move_lock:

            # if disabled, then add to point history but do nothing else.
            if not self.enabled():
                self.completed_move_points.append((x, y))
                return True

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

            # calculate time to step directly to the new point. the steppers will be moving concurrently, so each should
            # take the same time, though they will step at different speeds because their number of steps might differ.
            distance_mm = HGantry.get_distance_to_offset(move_x_mm, move_y_mm)
            time_to_step = timedelta(seconds=distance_mm / mm_per_sec)

            # estimate the starting time of the move. this will be after the newest move (most recent; last to finish)
            # if any moves exist. if no moves exist, then the current move will start at the current time.
            fifo_moves = self.get_fifo_moves()
            if len(fifo_moves) > 0:
                start_time_epoch = fifo_moves[-1].end_time_epoch
            else:
                start_time_epoch = time()

            move_idx = self.move_idx
            self.move_idx += 1
            self.send_moves_to_arduino(
                True,
                [
                    Move(
                        x,
                        y,
                        move_x_mm,
                        move_y_mm,
                        left_stepper_steps,
                        right_stepper_steps,
                        start_time_epoch,
                        time_to_step,
                        move_idx
                    )
                ]
            )

            # advance x and y positions as if the move was already completed, ignoring buffering. this is important
            # because subsequent calls to move need to be relative to this resulting location. we track the gantry's
            # actual x and y positions separately (actual_x and actual_y) based on move-complete responses returned by
            # the arduino. eventually these values will converge, so long as we don't hit limit switches.
            self.x += move_x_mm
            self.y += move_y_mm

        # block for the move if needed. do this outside the move lock so that we don't hold the lock while blocking. the
        # read operation below handles locking for itself.
        succeeded_without_limit: Optional[bool] = None
        if block:
            succeeded_without_limit = self.read_completed_moves_from_arduino(True)

        return succeeded_without_limit

    def set_state_with_estimated_location(
            self
    ):
        """
        Set the current state using the estimated location.
        """

        x_est, y_est = self.estimate_current_location()
        self.set_state(cast(HGantry.State, self.state).set(x_est=x_est, y_est=y_est))

    def estimate_current_location(
            self,
    ) -> Tuple[float, float]:
        """
        Estimate the current location.

        :return: Estimated location as x-y tuple.
        """

        current_time_epoch = time()

        fifo_moves = self.get_fifo_moves()

        # if there are no moves, then the x/y position will reflect the current position.
        if len(fifo_moves) == 0:
            with self.move_lock:
                curr_x, curr_y = self.x, self.y

        # if all moves are in the future, then the actual x/y positions that we track are the most accurate estimate of
        # the current location. this should not be generally possible, as time only moves forward from the creation of
        # each move.
        elif current_time_epoch < fifo_moves[0].start_time_epoch:
            with self.move_lock:
                curr_x, curr_y = self.actual_x, self.actual_y

        # if all moves are in the past, then they should all be complete, and the best estimate is the final move
        # position.
        elif current_time_epoch >= fifo_moves[-1].end_time_epoch:
            final_move = fifo_moves[-1]
            curr_x, curr_y = final_move.to_x_mm, final_move.to_y_mm

        # otherwise, we're in the middle of a move. find it and estimate our progress through it.
        else:
            curr_move = next(
                (
                    move
                    for move in fifo_moves
                    if move.start_time_epoch <= current_time_epoch < move.end_time_epoch
                )
            )
            fraction_through_curr_move = (
                (current_time_epoch - curr_move.start_time_epoch) /
                (curr_move.end_time_epoch - curr_move.start_time_epoch)
            )
            curr_move_start_x = curr_move.to_x_mm - curr_move.move_x_mm
            curr_x = curr_move_start_x + fraction_through_curr_move * curr_move.move_x_mm
            curr_move_start_y = curr_move.to_y_mm - curr_move.move_y_mm
            curr_y = curr_move_start_y + fraction_through_curr_move * curr_move.move_y_mm

        return curr_x, curr_y

    def send_move_to_arduino(
            self,
            move: Move
    ):
        """
        Send a move to the Arduino.

        :param move: Move.
        """

        with self.move_lock:

            # send step command for the joint action of the two steppers, plus a dummy component that will be ignored.
            # the steppers will send their step commands next, which the arduino will process jointly.
            self.arduino_serial.write_then_read(
                HGantry.Command.STEP.to_bytes(1) +
                (0).to_bytes(1),
                False,
                0,
                False
            )

            # step motors and retain the driver return value functions in the move. the stepper drivers are asserted to
            # operate asynchronously, so each return value will be a function that returns a tuple of (1) stepper
            # identifier, (2) the number of skipped steps due to hitting a limit switch, and (3) the move index. these
            # functions may be called later to await these tuples.
            (
                move.get_left_driver_return_value,
                move.get_right_driver_return_value
            ) = (
                self.left_stepper.step(move.left_stepper_steps, move.time_to_step),
                self.right_stepper.step(move.right_stepper_steps, move.time_to_step)
            )

            self.moves_pending_in_arduino.append(move)

    def send_moves_to_arduino(
            self,
            only_send_if_needed: bool,
            moves: Optional[List[Move]] = None
    ):
        """
        Send moves to the Arduino if any are available on the Python side.

        :param only_send_if_needed: Only send if the Arduino pending move buffer is low. If False, then the moves will
        be sent regardless of whether Arduino needs them.
        :param moves: Moves to add to the Python-side move buffer, or None to operate on moves that already exist.
        """

        if moves is None:
            moves = []

        with self.move_lock:

            self.moves_pending_in_python.extend(moves)
            num_python_moves_available = len(self.moves_pending_in_python)
            num_moves_pending_in_arduino = len(self.moves_pending_in_arduino)

            if only_send_if_needed:
                if num_moves_pending_in_arduino <= self.min_moves_pending_in_arduino:
                    max_num_moves_to_send = self.max_moves_pending_in_arduino - num_moves_pending_in_arduino
                else:
                    max_num_moves_to_send = 0
            else:
                max_num_moves_to_send = num_python_moves_available

            if num_python_moves_available > 0 and max_num_moves_to_send > 0:

                moves_to_send = self.moves_pending_in_python[:max_num_moves_to_send]
                for move_to_send in moves_to_send:
                    self.send_move_to_arduino(move_to_send)

                # push all bytes to arduino at once, in a single lump. this minimizes the number of serial read/writes.
                self.arduino_serial.flush_manually()

                self.moves_pending_in_python = self.moves_pending_in_python[max_num_moves_to_send:]

                logging.debug(
                    f'Sent {len(moves_to_send)} move(s) to Arduino. {len(self.moves_pending_in_python)} pending moves '
                    f'remain in Python.'
                )

    def get_move_to_read_from_arduino(
            self,
            clear_move_buffers: bool
    ) -> Optional[Move]:
        """
        Get a move to read from Arduino.

        :param clear_move_buffers: Whether move buffers should be cleared.
        :return: Move to read, or None if there is no move.
        """

        move = None

        with self.move_lock:

            # check whether we're ready to read a move
            ready = False
            if self.arduino_serial.connection.in_waiting > 0:
                ready = True
            elif clear_move_buffers:
                moves_pending = len(self.moves_pending_in_python) + len(self.moves_pending_in_arduino) > 0
                if moves_pending:
                    self.send_moves_to_arduino(True)
                    ready = True

            # get next move that will come back from arduino. we cannot be in a situation where the arduino buffer is
            # empty, since either (1) there are bytes waiting, or (2) we're clearing and there's a move somewhere in the
            # pipeline. in case (2), there will be a move in the arduino buffer because we just sent moves above, and we
            # will have placed at least one move into the arduino buffer.
            if ready:
                move = self.moves_pending_in_arduino[0]

        return move

    def read_move_from_arduino(
            self,
            move: Move
    ) -> Tuple[float, float, float]:
        """
        Read a move from Arduino, which will block until the Arduino sends the read-completed message back.

        :param move: Move to read.
        :return: 3-tuple of elapsed seconds for the move, left stepper skipped steps, and right stepper skipped steps.
        """

        assert move.get_left_driver_return_value is not None
        assert move.get_right_driver_return_value is not None

        # get result for each stepper. the drivers are asynchronous, and so the results will come back in an
        # unpredictable order. however, the result tuples have the stepper identifier as the first element,
        # so we can key on that to obtain the result for each stepper.
        logging.debug('Reading move response from Arduino driver.')
        stepper_id_skipped_steps_idx = {
            stepper_id: (skipped_steps, idx)
            for stepper_id, skipped_steps, idx in [
                move.get_left_driver_return_value(),
                move.get_right_driver_return_value()
            ]
        }
        assert len(stepper_id_skipped_steps_idx) == 2
        assert all(idx == move.idx for _, idx in stepper_id_skipped_steps_idx.values())
        with self.move_lock:
            assert move.idx == self.moves_pending_in_arduino[0].idx
            self.moves_pending_in_arduino.popleft()
            logging.debug(f'Moves pending in Arduino:  {len(self.moves_pending_in_arduino)}')
        elapsed_seconds = time() - move.start_time_epoch
        left_stepper_skipped_steps = stepper_id_skipped_steps_idx[self.left_driver.identifier][0]
        right_stepper_skipped_steps = stepper_id_skipped_steps_idx[self.right_driver.identifier][0]

        return elapsed_seconds, left_stepper_skipped_steps, right_stepper_skipped_steps

    def update_stepper_state(
            self,
            stepper: Stepper,
            steps: float,
            skipped: float,
            elapsed_seconds: float
    ):
        """
        Update stepper state.

        :param stepper: Stepper to update.
        :param steps: Intended steps.
        :param skipped: Steps skipped.
        :param elapsed_seconds: Elapsed seconds.
        """

        stepper_state: Stepper.State = stepper.state
        super(Stepper, stepper).set_state(
            Stepper.State(
                round(stepper_state.step + steps - skipped),
                timedelta(seconds=elapsed_seconds)
            )
        )

    def read_completed_moves_from_arduino(
            self,
            clear_move_buffers: bool
    ) -> Optional[bool]:
        """
        Read completed moves from the Arduino.

        :param clear_move_buffers: Whether to continue sending, waiting for, and reading moves until all move buffers
        are clear. If False, then this function will only read one or more moves if they are already present in the
        serial read buffer.
        :return: True if a move was read without hitting a limit switch, False if limit switch was hit by a read move,
        and None if no moves were read.
        """

        succeeded_without_limit: Optional[bool] = None

        # do not permit concurrent reads, as this can mix up the driver return sequence identifiers. block if the caller
        # is requesting to clear the move buffers, since whoever might already be holding the lock might not be clearing
        # them, and we must honor the caller's desire to clear.
        driver_read_lock_acquired = self.driver_read_lock.acquire(blocking=clear_move_buffers)
        if driver_read_lock_acquired:
            try:
                while (move := self.get_move_to_read_from_arduino(clear_move_buffers)) is not None:

                    (
                        elapsed_seconds,
                        left_stepper_skipped_steps,
                        right_stepper_skipped_steps
                    ) = self.read_move_from_arduino(move)

                    self.update_stepper_state(
                        self.left_stepper,
                        move.left_stepper_steps,
                        left_stepper_skipped_steps,
                        elapsed_seconds
                    )

                    self.update_stepper_state(
                        self.right_stepper,
                        move.right_stepper_steps,
                        right_stepper_skipped_steps,
                        elapsed_seconds
                    )

                    skipped_x_mm, skipped_y_mm = self.get_x_mm_y_mm_from_steps(
                        left_stepper_skipped_steps,
                        right_stepper_skipped_steps
                    )

                    # set to success only once. if we hit a limit, then always set to failure and do not reset to
                    # success. the caller needs to know if we hit a limit at any point.
                    if skipped_x_mm == 0.0 and skipped_y_mm == 0.0:
                        if succeeded_without_limit is None:
                            succeeded_without_limit = True
                    else:
                        logging.debug(f'Hit limit and skipped:  {skipped_x_mm} mm (x); {skipped_y_mm} mm (y)')
                        succeeded_without_limit = False

                    with self.move_lock:

                        # subtract any skipped movement due to limit switches. we previously added the move x/y offsets
                        # when creating the move, so all we need to do is subtract the skipped distances.
                        self.x -= skipped_x_mm
                        self.y -= skipped_y_mm

                        # advance actual x and y positions, minus any skipped movement due to limit switches.
                        self.actual_x += move.move_x_mm - skipped_x_mm
                        self.actual_y += move.move_y_mm - skipped_y_mm

                        # set the state with the actual values
                        self.set_state(cast(HGantry.State, self.state).set(x=self.actual_x, y=self.actual_y))

                        self.completed_move_points.append((self.actual_x, self.actual_y))

            finally:
                self.driver_read_lock.release()

        return succeeded_without_limit

    def clear_move_buffer(
            self
    ) -> bool:
        """
        Clear the move buffer. Does not move the gantry beyond the moves currently in the buffer. Blocks until all
        moves are complete.

        :return: True if the buffer was cleared without hitting a limit switch; False if limit switch was hit by the
        final move.
        """

        return self.move_to_offset(0.0, 0.0, 1.0, True)

    def clear_point_history(
            self
    ):
        """
        Clear the point history.
        """

        with self.move_lock:
            self.completed_move_points.clear()
            self.completed_move_points = [(self.x, self.y)]

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
        #
        return (
            (left_stepper_steps + right_stepper_steps) / (2.0 * self.steps_per_mm),
            (left_stepper_steps - right_stepper_steps) / (2.0 * self.steps_per_mm)
        )

    def move_to_points(
            self,
            points: List[Tuple[float, float]],
            mm_per_sec: float,
            return_to_current_position: bool,
            block: bool,
            ignore_moves_shorter_than_mm: float
    ):
        """
        Move through a list of points.

        :param points: Points.
        :param mm_per_sec: Speed in mm per second.
        :param block: Whether to block until the movement is complete.
        :param ignore_moves_shorter_than_mm: Ignore point-to-point moves shorter than this many mm.
        :param return_to_current_position: Whether to return to the current position after moving to the points.
        """

        with self.move_lock:
            original_x, original_y = self.x, self.y

        num_points = len(points)
        for i, (x, y) in enumerate(points):
            distance_mm = self.get_distance_to_point((x, y))
            diff_from_threshold = distance_mm - ignore_moves_shorter_than_mm
            if np.isclose(diff_from_threshold, 0.0) or diff_from_threshold < 0.0:
                logging.debug(f'Skipping non-move point {i + 1} of {num_points}:  {x:.3f},{y:.3f}')
            else:
                logging.debug(f'Moving to point {i + 1} of {num_points}:  {x:.3f},{y:.3f}')
                self.move_to_point(x, y, mm_per_sec, False)

        if return_to_current_position:
            self.move_to_point(original_x, original_y, mm_per_sec, False)

        if block:
            self.clear_move_buffer()

    def move_to_offset(
            self,
            x_offset_mm: float,
            y_offset_mm: float,
            mm_per_sec: float,
            block: bool
    ) -> Optional[bool]:
        """
        Move to an offset from the current position.

        :param x_offset_mm: X offset.
        :param y_offset_mm: Y offset.
        :param mm_per_sec: Speed in mm per second.
        :param block: Whether to block until the movement is complete.
        :return: True if move was achieved without hitting a limit switch; False if limit switch was hit before move was
        achieved. Will be None if the move was buffered and not completed.
        """

        with self.move_lock:
            x, y = self.x, self.y

        return self.move_to_point(x + x_offset_mm, y + y_offset_mm, mm_per_sec, block)

    def move_to_offset_limit(
            self,
            x_offset_mm: float,
            y_offset_mm: float,
            mm_per_sec: float
    ):
        """
        Move to a limit in a given offset direction.

        :param x_offset_mm: X offset.
        :param y_offset_mm: Y offset.
        :param mm_per_sec: Speed in mm per second.
        """

        # disable the timer so that it doesn't read moves and cause a return of None in the move checks below
        self.pump_clock.stop()

        # move by offset until we hit a limit switch
        while self.move_to_offset(x_offset_mm, y_offset_mm, mm_per_sec, True):
            pass

        # back away from the switch in 2mm steps until we can move the offset direction by 1mm
        else:
            while not self.move_to_offset(np.sign(x_offset_mm), np.sign(y_offset_mm), mm_per_sec, True):
                self.move_to_offset(2.0 * -np.sign(x_offset_mm), 2.0 * -np.sign(y_offset_mm), mm_per_sec, True)

            self.clear_move_buffer()

        self.pump_clock.start()

    def draw_spirograph_from_params(
            self,
            R: int,
            r: int,
            d: int,
            theta_step: float,
            scale: float,
            mm_per_sec: float,
            return_to_current_position: bool,
            block: bool,
            ignore_moves_shorter_than_mm: float
    ) -> Optional[CallImageBytes]:
        """
        Draw a spirograph from parameters centered at the current location.

        :param R: Radius of the fixed circle.
        :param r: Radius of the rolling circle.
        :param d: Distance of the trace point from the rolling circle.
        :param theta_step: Step size (radians).
        :param scale: Scale.
        :param mm_per_sec: Speed.
        :param return_to_current_position: Whether to return to current position.
        :param block: Whether to block until the movement is complete.
        :param ignore_moves_shorter_than_mm: Ignore point-to-point moves shorter than this many mm.
        :return: Image of the completed drawing, which will be non-None only if `block` is True, which will wait for the
        drawing to complete.
        """

        R = int(R * scale)
        r = int(r * scale)
        d = int(d * scale)

        g = Hypotrochoid(
            R=R,
            r=r,
            d=d,
            theta_start=0.0,
            theta_stop=2.0 * np.pi * (r / math.gcd(R, r)),  # complete draw without repetition
            theta_step=theta_step
        )

        with self.move_lock:
            center = self.x, self.y

        self.draw_spirograph(
            g,
            center,
            mm_per_sec,
            return_to_current_position,
            block,
            ignore_moves_shorter_than_mm
        )

        # we can only return an image of the drawing if we blocked and waited for it to complete
        if block:
            call_image_bytes = CallImageBytes(self.get_line_plot())
        else:
            call_image_bytes = None

        return call_image_bytes

    def draw_spirograph(
            self,
            g: _Trochoid,
            center: Tuple[float, float],
            mm_per_sec: float,
            return_to_current_position: bool,
            block: bool,
            ignore_moves_shorter_than_mm: float
    ) -> _Trochoid:
        """
        Draw a spirograph object.

        :param g: Spirograph.
        :param center: Location of center.
        :param mm_per_sec: Speed.
        :param return_to_current_position: Whether to return to current position.
        :param block: Whether to block until the movement is complete.
        :param ignore_moves_shorter_than_mm: Ignore point-to-point moves shorter than this many mm.
        :return: Resulting spirograph, which might be scaled and translated.
        """

        if g.max_x - g.min_x > 0.99 * self.left_right_mm:
            g = g.scale(0.99 * self.left_right_mm / (g.max_x - g.min_x))
            logging.warning('Spirograph x out of bounds. Rescaled.')

        if g.max_y - g.min_y > 0.99 * self.bottom_top_mm:
            g = g.scale(0.99 * self.bottom_top_mm / (g.max_y - g.min_y))
            logging.warning('Spirograph y out of bounds. Rescaled.')

        center_x, center_y = center
        half_width = (g.max_x - g.min_x) / 2.0
        left_border = center_x - half_width
        half_height = (g.max_y - g.min_y) / 2.0
        bottom_border = center_y - half_height
        g = g.translate(left_border - g.min_x, bottom_border - g.min_y)
        logging.info(
            f'Tracing spirograph within bounds:  ({g.min_x:.3f},{g.min_y:.3f}) (LL) ({g.max_x:.3f},{g.max_y:.3f}) (UR)'
        )

        self.move_to_points(
            list(zip(g.x, g.y)),
            mm_per_sec,
            return_to_current_position,
            block,
            ignore_moves_shorter_than_mm
        )

        return g

    def draw_spiral(
            self,
            outer_diameter_mm: float,
            loop_spacing_mm: float,
            step_degrees: float,
            mm_per_sec: float,
            return_to_current_position: bool,
            block: bool,
            ignore_moves_shorter_than_mm: float
    ) -> Optional[CallImageBytes]:
        """
        Draw a spiral centered at the current location.

        :param outer_diameter_mm: Outer diameter (mm) of the spiral.
        :param loop_spacing_mm: Loop spacing (mm).
        :param step_degrees: Step degrees.
        :param mm_per_sec: Speed.
        :param return_to_current_position: Whether to return to current position.
        :param block: Whether to block until the movement is complete.
        :param ignore_moves_shorter_than_mm: Ignore point-to-point moves shorter than this many mm.
        :return: Image of the completed drawing, which will be non-None only if `block` is True, which will wait for the
        drawing to complete.
        """

        radius_mm = outer_diameter_mm / 2.0
        turn_count = radius_mm / loop_spacing_mm
        turn_radians = turn_count * 2.0 * math.pi
        step_radians = math.radians(step_degrees)
        loop_spacing_mm_per_radian = loop_spacing_mm / (2.0 * math.pi)

        logging.info(f'Drawing spiral:  r={radius_mm} mm, turns={turn_count}, loop spacing={loop_spacing_mm} mm')

        with self.move_lock:
            center_x, center_y = self.x, self.y

        spiral_points = [(center_x, center_y)]
        for theta_radians in np.arange(0.0, turn_radians + step_radians, step_radians):
            polar_radius = loop_spacing_mm_per_radian * theta_radians
            spiral_x = polar_radius * math.cos(theta_radians) + center_x
            spiral_y = polar_radius * math.sin(theta_radians) + center_y
            spiral_points.append((spiral_x, spiral_y))

        self.move_to_points(
            spiral_points,
            mm_per_sec,
            return_to_current_position,
            block,
            ignore_moves_shorter_than_mm
        )

        # we can only return an image of the drawing if we blocked and waited for it to complete
        if block:
            call_image_bytes = CallImageBytes(self.get_line_plot())
        else:
            call_image_bytes = None

        return call_image_bytes

    def wipe(
            self,
            y_spacing_mm: float,
            mm_per_sec: float,
            block: bool
    ) -> Optional[CallImageBytes]:
        """
        Wipe the board by drawing a line-by-line pattern.

        :param y_spacing_mm: Vertical spacing of the lines.
        :param mm_per_sec: Speed.
        :param block: Whether to block until the movement is complete.
        :return: Image of the completed drawing, which will be non-None only if `block` is True, which will wait for the
        drawing to complete.
        """

        logging.info(f'Wiping:  y spacing={y_spacing_mm} mm')

        with self.move_lock:
            curr_x_mm, curr_y_mm = self.x, self.y

        def move(
                x_mm: float,
                y_mm: float
        ):
            """
            Move to an x/y location and update current location.

            :param x_mm: x location (mm).
            :param y_mm: y location (mm).
            """

            nonlocal curr_x_mm
            nonlocal curr_y_mm

            self.move_to_point(x_mm, y_mm, mm_per_sec, True)

            with self.move_lock:
                curr_x_mm, curr_y_mm = self.x, self.y

        half_of_bottom_top_mm = self.bottom_top_mm / 2.0

        move(0.0, curr_y_mm)
        wipe_left_to_right = True

        # wipe bottom-up halfway
        for wipe_y_mm in np.arange(0.0, half_of_bottom_top_mm + y_spacing_mm, y_spacing_mm):
            move(curr_x_mm, wipe_y_mm)
            if wipe_left_to_right:
                move(self.left_right_mm, wipe_y_mm)
            else:
                move(0.0, wipe_y_mm)

            wipe_left_to_right = not wipe_left_to_right

        # wipe top down
        for wipe_y_mm in np.arange(self.bottom_top_mm, half_of_bottom_top_mm - y_spacing_mm, -y_spacing_mm):
            move(curr_x_mm, wipe_y_mm)
            if wipe_left_to_right:
                move(self.left_right_mm, wipe_y_mm)
            else:
                move(0.0, wipe_y_mm)

            wipe_left_to_right = not wipe_left_to_right

        return self.center(mm_per_sec, block)

    def enable(
            self
    ):
        """
        Enable the gantry. Moves will be executed and state will be updated.
        """

        self.set_state(cast(HGantry.State, self.state).set(enabled=True))

    def disable(
            self
    ):
        """
        Disable the gantry. Move points will be tracked and drawn in the line plot; however, moves will not actually be
        executed, and the state will not be updated.
        """

        self.set_state(cast(HGantry.State, self.state).set(enabled=False))

    def enabled(
            self
    ) -> bool:
        """
        Get whether the gantry is enabled.

        :return: True if enabled.
        """

        return cast(HGantry.State, self.state).enabled

    def get_fifo_moves(
            self
    ) -> List[Move]:
        """
        Get moves in FIFO order with the one finishing first being first in the returned list.

        :return: List of Move objects.
        """

        with self.move_lock:
            return list(self.moves_pending_in_arduino) + self.moves_pending_in_python

    def get_line_plot(
            self
    ) -> str:
        """
        Get line plot for the gantry. Will block until exclusive plotting access is obtained.

        :return: Base-64 encoded image of line plot.
        """

        with self.move_lock:
            completed_move_points = self.completed_move_points.copy()
            pending_moves = self.get_fifo_moves()

        # need exclusive access to plotting to prevent concurrent calls from interfering with each other
        with self.plot_lock:
            plt.plot(
                *zip(*completed_move_points),
                linestyle='-',
                marker='.',
                markersize=0.05,
                label='Completed'
            )
            plt.plot(
                *zip(
                    *[
                        (m.to_x_mm, m.to_y_mm)
                        for m in pending_moves
                    ]
                ),
                linestyle='-',
                marker='o',
                fillstyle='none',
                alpha=0.5,
                label='Future'
            )
            plotted = len(completed_move_points) + len(pending_moves) > 0
            plt.gcf().set_size_inches(8.0, 8.0)
            plt.gca().set_aspect('equal')
            plt.grid()
            if plotted:
                plt.legend()
            plt.xlim(-5.0, self.left_right_mm + 5.0)
            plt.ylim(-5.0, self.bottom_top_mm + 5.0)
            plt.xlabel('mm')
            plt.ylabel('mm')
            plt.tight_layout()

            buffer = io.BytesIO()
            plt.savefig(buffer, format='jpeg', bbox_inches='tight')
            plt.close()
            buffer.seek(0)
            line_plot_base64_str = get_base_64_str(buffer.getvalue())

        return line_plot_base64_str

    def get_ui_elements(
            self
    ) -> List[Tuple[Union[str, Tuple[str, str]], str]]:
        """
        Get UI elements for the current component.

        :return: List of 2-tuples of (1) element key and (2) element content.
        """

        # wipe
        wipe_y_spacing_mm_textbox_id, wipe_y_spacing_mm_textbox_ui_element = RpyFlask.get_textbox(
            'wipe-y_spacing_mm',
            'y spacing (mm)',
            '10.0',
            RpyFlask.TextboxType.NUMBER
        )

        wipe_mm_per_sec_textbox_id, wipe_mm_per_sec_textbox_ui_element = RpyFlask.get_textbox(
            'wipe-mm_per_sec',
            'Speed (mm/sec)',
            '150.0',
            RpyFlask.TextboxType.NUMBER
        )

        wipe_block_switch_id, wipe_block_switch_ui_element = RpyFlask.get_switch(
            'wipe-block',
            None,
            None,
            'Block until complete',
            True
        )

        wipe_dyn_args = [
            ('y_spacing_mm', float, wipe_y_spacing_mm_textbox_id),
            ('mm_per_sec', float, wipe_mm_per_sec_textbox_id),
            ('block', bool, wipe_block_switch_id)
        ]

        # spirograph
        spiro_R_textbox_id, spiro_R_textbox_ui_element = RpyFlask.get_textbox(
            'spiro-upper-r',
            'Radius (R; mm) of the fixed circle along which the moving circle rolls',
            '350',
            RpyFlask.TextboxType.NUMBER
        )

        spiro_r_textbox_id, spiro_r_textbox_ui_element = RpyFlask.get_textbox(
            'spiro-lower-r',
            'Radius (r; mm) of the rolling circle',
            '200',
            RpyFlask.TextboxType.NUMBER
        )

        spiro_d_textbox_id, spiro_d_textbox_ui_element = RpyFlask.get_textbox(
            'spiro-d',
            'Distance (d; mm) of the trace point from the rolling circle',
            '100',
            RpyFlask.TextboxType.NUMBER
        )

        spiro_theta_step_textbox_id, spiro_theta_step_textbox_ui_element = RpyFlask.get_textbox(
            'spiro-theta_step',
            'Rolling step size (theta_step; radians)',
            '0.01',
            RpyFlask.TextboxType.NUMBER
        )

        spiro_scale_textbox_id, spiro_scale_textbox_ui_element = RpyFlask.get_textbox(
            'spiro-scale',
            'Scaling (greater than 0.0)',
            '0.5',
            RpyFlask.TextboxType.NUMBER
        )

        spiro_mm_per_sec_textbox_id, spiro_mm_per_sec_textbox_ui_element = RpyFlask.get_textbox(
            'spiro-mm_per_sec',
            'Speed (mm/sec)',
            '100.0',
            RpyFlask.TextboxType.NUMBER
        )

        spiro_return_switch_id, spiro_return_switch_ui_element = RpyFlask.get_switch(
            'spiro-return_to_current_position',
            None,
            None,
            'Return to current position',
            True
        )

        spiro_block_switch_id, spiro_block_switch_ui_element = RpyFlask.get_switch(
            'spiro-block',
            None,
            None,
            'Block until complete',
            True
        )

        spiro_ignore_moves_textbox_id, spiro_ignore_moves_textbox_ui_element = RpyFlask.get_textbox(
            'spiro-ignore_moves_shorter_than_mm',
            'Ignore moves shorter than (mm)',
            '10.0',
            RpyFlask.TextboxType.NUMBER
        )

        draw_spirograph_dyn_args = [
            ('R', int, spiro_R_textbox_id),
            ('r', int, spiro_r_textbox_id),
            ('d', int, spiro_d_textbox_id),
            ('theta_step', float, spiro_theta_step_textbox_id),
            ('scale', float, spiro_scale_textbox_id),
            ('mm_per_sec', float, spiro_mm_per_sec_textbox_id),
            ('return_to_current_position', bool, spiro_return_switch_id),
            ('block', bool, spiro_block_switch_id),
            ('ignore_moves_shorter_than_mm', float, spiro_ignore_moves_textbox_id)
        ]

        # spiral
        spiral_outer_diameter_mm_textbox_id, spiral_outer_diameter_mm_textbox_ui_element = RpyFlask.get_textbox(
            'spiral-outer_diameter_mm',
            'Outer diameter (mm)',
            '100.0',
            RpyFlask.TextboxType.NUMBER
        )

        spiral_loop_spacing_mm_textbox_id, spiral_loop_spacing_mm_textbox_ui_element = RpyFlask.get_textbox(
            'spiral-loop_spacing_mm',
            'Loop spacing (mm)',
            '10.0',
            RpyFlask.TextboxType.NUMBER
        )

        spiral_step_degrees_textbox_id, spiral_step_degrees_textbox_ui_element = RpyFlask.get_textbox(
            'spiral-step_degrees',
            'Step size (degrees)',
            '5.0',
            RpyFlask.TextboxType.NUMBER
        )

        spiral_mm_per_sec_textbox_id, spiral_mm_per_sec_textbox_ui_element = RpyFlask.get_textbox(
            'spiral-mm_per_sec',
            'Speed (mm/sec)',
            '100.0',
            RpyFlask.TextboxType.NUMBER
        )

        spiral_return_switch_id, spiral_return_switch_ui_element = RpyFlask.get_switch(
            'spiral-return_to_current_position',
            None,
            None,
            'Return to current position',
            True
        )

        spiral_block_switch_id, spiral_block_switch_ui_element = RpyFlask.get_switch(
            'spiral-block',
            None,
            None,
            'Block until complete',
            True
        )

        spiral_ignore_moves_textbox_id, spiral_ignore_moves_textbox_ui_element = RpyFlask.get_textbox(
            'spiral-ignore_moves_shorter_than_mm',
            'Ignore moves shorter than (mm)',
            '5.0',
            RpyFlask.TextboxType.NUMBER
        )

        draw_spiral_dyn_args = [
            ('outer_diameter_mm', float, spiral_outer_diameter_mm_textbox_id),
            ('loop_spacing_mm', float, spiral_loop_spacing_mm_textbox_id),
            ('step_degrees', float, spiral_step_degrees_textbox_id),
            ('mm_per_sec', float, spiral_mm_per_sec_textbox_id),
            ('return_to_current_position', bool, spiral_return_switch_id),
            ('block', bool, spiral_block_switch_id),
            ('ignore_moves_shorter_than_mm', float, spiral_ignore_moves_textbox_id)
        ]

        # common argument for adding to the history
        add_to_history = {'add_to_history': True}

        return [
            RpyFlask.get_button(self.id, self.calibrate, {'mm_per_sec': 100.0}, None, None, None, None, 'Calibrate'),
            RpyFlask.get_button(self.id, self.center, {**add_to_history, 'mm_per_sec': 100.0, 'block': True}, None, None, None, None, 'Center'),
            RpyFlask.get_button(self.id, self.move_to_offset, {**add_to_history, 'x_offset_mm': -10.0, 'y_offset_mm': 0.0, 'mm_per_sec': 100.0, 'block': False}, None, None, None, None, '<', 'left'),
            RpyFlask.get_button(self.id, self.move_to_offset, {**add_to_history, 'x_offset_mm': 10.0, 'y_offset_mm': 0.0, 'mm_per_sec': 100.0, 'block': False}, None, None, None, None, '>', 'right'),
            RpyFlask.get_button(self.id, self.move_to_offset, {**add_to_history, 'x_offset_mm': 0.0, 'y_offset_mm': 10.0, 'mm_per_sec': 100.0, 'block': False}, None, None, None, None, '^', 'up'),
            RpyFlask.get_button(self.id, self.move_to_offset, {**add_to_history, 'x_offset_mm': 0.0, 'y_offset_mm': -10.0, 'mm_per_sec': 100.0, 'block': False}, None, None, None, None, 'v', 'down'),
            RpyFlask.get_image(self.id, 600, self.get_line_plot, timedelta(seconds=1.0), None, 1.0),
            RpyFlask.get_button(self.id, self.clear_point_history, None, None, None, None, None, 'Clear Plot'),
            RpyFlask.get_button(self.id, self.clear_move_buffer, None, None, None, None, None, 'Clear Move Buffer'),

            RpyFlask.get_button(self.id, self.wipe, add_to_history, wipe_dyn_args, None, None, None, 'Wipe'),
            (wipe_y_spacing_mm_textbox_id, wipe_y_spacing_mm_textbox_ui_element),
            (wipe_mm_per_sec_textbox_id, wipe_mm_per_sec_textbox_ui_element),
            (wipe_block_switch_id, wipe_block_switch_ui_element),

            RpyFlask.get_button(self.id, self.draw_spirograph_from_params, add_to_history, draw_spirograph_dyn_args, None, None, None, 'Draw'),
            (spiro_R_textbox_id, spiro_R_textbox_ui_element),
            (spiro_r_textbox_id, spiro_r_textbox_ui_element),
            (spiro_d_textbox_id, spiro_d_textbox_ui_element),
            (spiro_theta_step_textbox_id, spiro_theta_step_textbox_ui_element),
            (spiro_scale_textbox_id, spiro_scale_textbox_ui_element),
            (spiro_mm_per_sec_textbox_id, spiro_mm_per_sec_textbox_ui_element),
            (spiro_ignore_moves_textbox_id, spiro_ignore_moves_textbox_ui_element),
            (spiro_return_switch_id, spiro_return_switch_ui_element),
            (spiro_block_switch_id, spiro_block_switch_ui_element),

            RpyFlask.get_button(self.id, self.draw_spiral, add_to_history, draw_spiral_dyn_args, None, None, None, 'Draw'),
            (spiral_outer_diameter_mm_textbox_id, spiral_outer_diameter_mm_textbox_ui_element),
            (spiral_loop_spacing_mm_textbox_id, spiral_loop_spacing_mm_textbox_ui_element),
            (spiral_step_degrees_textbox_id, spiral_step_degrees_textbox_ui_element),
            (spiral_mm_per_sec_textbox_id, spiral_mm_per_sec_textbox_ui_element),
            (spiral_return_switch_id, spiral_return_switch_ui_element),
            (spiral_block_switch_id, spiral_block_switch_ui_element),
            (spiral_ignore_moves_textbox_id, spiral_ignore_moves_textbox_ui_element),

            RpyFlask.get_switch(self.id, self.start, self.stop, 'Started', False),
            RpyFlask.get_switch(self.id, self.enable, self.disable, 'Enabled', True)
        ]


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
