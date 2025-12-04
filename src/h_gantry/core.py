import json
import logging
import math
import os.path
from collections import deque
from datetime import timedelta
from enum import IntEnum
from threading import Lock
from time import time, sleep
from typing import Tuple, List, Optional, Callable, NamedTuple, Union

import numpy as np
from raspberry_py.gpio import CkPin, Component
from raspberry_py.gpio.adc import ADS7830
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.controls import Joystick
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoUln2003
from raspberry_py.rest.application import RpyFlask
from smbus2 import SMBus
# noinspection PyProtectedMember
from spyrograph.core._trochoid import _Trochoid


class Move(NamedTuple):
    """
    Move record for gantry.
    """

    move_x_mm: float
    move_y_mm: float
    left_stepper_steps: int
    get_left_driver_return_value: Callable
    right_stepper_steps: int
    get_right_driver_return_value: Callable
    timestamp: float


class HGantry(Component):
    """
    Control for a two-axis gantry using two fixed-position stepper motors.
    """

    class State(Component.State):
        """
        Gantry state.
        """

        def __init__(
                self,
                x: float,
                y: float
        ):
            """
            Initialize state.

            :param x: X position.
            :param y: Y position.
            """

            self.x = x
            self.y = y

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

            return self.x == other.x and self.y == other.y

        def __str__(
                self
        ) -> str:
            """
            Get string.

            :return: String.
            """

            return f'x={self.x:.3f}, y={self.y:.3f}'

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

        self.move_to_point_lock = Lock()

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
            logging.info(f'State loaded. Position=({self.x:.3f},{self.y:.3f})')
        else:
            logging.info(f'No state file exists:  {self.state_path}')
            self.x = 0.0
            self.y = 0.0
            self.left_right_mm = 0.0
            self.bottom_top_mm = 0.0

        self.actual_x = self.x
        self.actual_y = self.y

        super().__init__(HGantry.State(self.x, self.y))

        # create an a/d converter for the joystick and rescale the digital outputs to be in a range. report all state
        # updates so that we get regular joystick events even when the joystick isn't changing position. both the adc
        # and joystick report synchronous events that land at move_to_point, which uses a lock to limit step commands
        # to the arduino.
        joystick_y_ad_channel = 0
        joystick_x_ad_channel = 1
        self.adc = ADS7830(
            input_voltage=3.3,
            bus=SMBus('/dev/i2c-1'),
            address=ADS7830.ADDRESS,
            command=ADS7830.COMMAND,
            channel_rescaled_range={
                joystick_y_ad_channel: (-9.0, 9.0),
                joystick_x_ad_channel: (-9.0, 9.0)
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

        self.move_buffer: deque[Move] = deque()
        self.move_buffer_max_len = 500
        self.move_buffer_min_len = 10

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
            self.center(100.0, True, True)

        # ignore negligible joystick movements and noise
        elif math.sqrt(joystick_state.x ** 2 + joystick_state.y ** 2) > 2.0:

            # move 1 mm in the joystick direction as indicated by the vector norm
            move_vector = np.array([joystick_state.x, joystick_state.y])
            norm = np.linalg.norm(move_vector)
            if norm != 0.0:
                move_x_mm, move_y_mm = move_vector / norm
                try:
                    self.move_to_offset(
                        move_x_mm,
                        move_y_mm,
                        HGantry.get_speed_from_joystick_state(joystick_state),
                        False,
                        True
                    )
                except ValueError as e:
                    logging.error(f'Failed to move according to joystick:  {e}')

    @staticmethod
    def get_speed_from_joystick_state(
            joystick_state: Joystick.State
    ) -> float:
        """
        Get speed from a joystick state.

        :param joystick_state: State.
        :return: Speed.
        """

        return 10.0  # math.sqrt(joystick_state.x ** 2 + joystick_state.y ** 2)

    def start(
            self
    ):
        """
        Start the gantry.
        """

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

        self.joystick.start_updating_state(self.joystick_update_interval_seconds)

    def stop(
            self,
            save_state: bool
    ):
        """
        Stop the gantry.

        :param save_state: Whether to save the gantry's state after stopping.
        """

        # process the buffer to obtain current x/y position
        self.clear_async_results_buffer()

        self.joystick.stop_updating_state()
        self.adc.close()
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
            mm_per_sec: float,
            block: bool,
            check_bounds: bool
    ):
        """
        Center the gantry.

        :param mm_per_sec: Speed.
        :param block: Whether to block until the movement is complete.
        :param check_bounds: Whether to check bounds of the point. Raises an exception if check fails.
        """

        logging.info('Centering gantry.')
        self.move_to_point(self.left_right_mm / 2.0, self.bottom_top_mm / 2.0, mm_per_sec, block, check_bounds)

    def move_to_left_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the left limit.

        :param mm_per_sec: speed.
        """

        self.move_to_offset_limit(-10.0, 0.0, mm_per_sec)

    def move_to_right_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the right limit.

        :param mm_per_sec: Speed.
        """

        self.move_to_offset_limit(10.0, 0.0, mm_per_sec)

    def move_to_bottom_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the bottom limit.

        :param mm_per_sec: Speed.
        """

        self.move_to_offset_limit(0.0, -10.0, mm_per_sec)

    def move_to_top_limit(
            self,
            mm_per_sec: float
    ):
        """
        Move to the top limit.

        :param mm_per_sec: Speed.
        """

        self.move_to_offset_limit(0.0, 10.0, mm_per_sec)

    def calibrate(
            self,
            mm_per_sec: float
    ):
        """
        Calibrate the gantry by measuring unknown positions and dimensions.

        :param mm_per_sec: Speed.
        """

        # measure distance between horizontal limits, and set actual x position.
        self.move_to_left_limit(mm_per_sec)
        left_x = self.x
        self.move_to_right_limit(mm_per_sec)
        right_x = self.x
        self.left_right_mm = self.x = right_x - left_x

        # measure distance between vertical limits, and set actual y position.
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
        Get the movement to a point in terms of x and y distances (mm).

        :param x: Point's x position.
        :param y: Point's y position.
        :return: Movement as a 2-tuple of x and y distances, each in mm.
        """

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

    def move_to_point(
            self,
            x: float,
            y: float,
            mm_per_sec: float,
            block: bool,
            check_bounds: bool
    ) -> Optional[bool]:
        """
        Move to a point.

        :param x: X coordinate to move to.
        :param y: Y coordinate to move to.
        :param mm_per_sec: Speed in mm per second.
        :param block: Whether to block until the movement is complete.
        :param check_bounds: Whether to check bounds of the point. Raises an exception if check fails.
        :return: True if move was achieved without hitting a limit switch; False if limit switch was hit before move was
        achieved. Will be None if the move was buffered and no other moves were processed. If a previously buffered
        move was processed by the current call, then the return value will be for that move.
        """

        try:

            self.move_to_point_lock.acquire()

            if check_bounds:
                self.assert_point_in_bounds(x, y)

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

            # send step command for the joint action of the two steppers, plus a dummy component that will be ignored.
            # the steppers will send their step commands next, which the arduino will process jointly. no need to flush
            # here, since the stepper drivers will flush.
            self.arduino_serial.write_then_read(
                HGantry.Command.STEP.to_bytes(1) +
                (0).to_bytes(1),
                False,
                0,
                False
            )

            # step motors and record in the buffer. the stepper drivers are asserted to operate asynchronously, so each
            # return value will be a function that returns a stepper identifier and the number of skipped steps due to
            # hitting a limit switch.
            self.move_buffer.append(Move(
                move_x_mm,
                move_y_mm,
                left_stepper_steps,
                self.left_stepper.step(left_stepper_steps, time_to_step),
                right_stepper_steps,
                self.right_stepper.step(right_stepper_steps, time_to_step),
                time()
            ))

            # if the caller wants to block and wait for the current move to complete, then set the min/max buffer
            # lengths to zero, which will force all buffered steps (including the current) to complete before returning.
            if block:
                min_buffer_len = max_buffer_len = 0

            # otherwise, allow movements to buffer up to the given length, at which point we'll process moves until we
            # get to the minimum length.
            else:
                min_buffer_len = self.move_buffer_min_len
                max_buffer_len = self.move_buffer_max_len

            succeeded_without_limit = None

            if len(self.move_buffer) > max_buffer_len:

                logging.info(
                    f'Move buffer length ({len(self.move_buffer)}) greater than maximum ({max_buffer_len}). '
                    'Draining...'
                )

                while len(self.move_buffer) > min_buffer_len:

                    move = self.move_buffer.popleft()
                    elapsed_seconds = time() - move.timestamp

                    # get result (skipped steps) for each stepper. the drivers are asynchronous and so the results will
                    # come back in an unpredictable order. however, the result tuples have the stepper identifier as the
                    # first element, so we can key on that to obtain the result for each stepper.
                    stepper_id_skipped_steps = {
                        stepper_id: skipped_steps
                        for stepper_id, skipped_steps in [
                            move.get_left_driver_return_value(),
                            move.get_right_driver_return_value()
                        ]
                    }
                    assert len(stepper_id_skipped_steps) == 2
                    left_stepper_skipped_steps = stepper_id_skipped_steps[self.left_driver.identifier]
                    right_stepper_skipped_steps = stepper_id_skipped_steps[self.right_driver.identifier]

                    # update stepper states now that we have results
                    left_stepper_state: Stepper.State = self.left_stepper.state
                    super(Stepper, self.left_stepper).set_state(
                        Stepper.State(
                            round(
                                left_stepper_state.step +
                                move.left_stepper_steps -
                                left_stepper_skipped_steps
                            ),
                            timedelta(seconds=elapsed_seconds)
                        )
                    )
                    right_stepper_state: Stepper.State = self.right_stepper.state
                    super(Stepper, self.right_stepper).set_state(
                        Stepper.State(
                            round(
                                right_stepper_state.step +
                                move.right_stepper_steps -
                                right_stepper_skipped_steps
                            ),
                            timedelta(seconds=elapsed_seconds)
                        )
                    )

                    # calculate skipped distances from skipped steps
                    skipped_x_mm, skipped_y_mm = self.get_x_mm_y_mm_from_steps(
                        left_stepper_skipped_steps,
                        right_stepper_skipped_steps
                    )

                    # subtract any skipped movement due to limit switches
                    self.x -= skipped_x_mm
                    self.y -= skipped_y_mm

                    # advance actual x and y positions, minus any skipped movement due to limit switches.
                    self.actual_x += move.move_x_mm - skipped_x_mm
                    self.actual_y += move.move_y_mm - skipped_y_mm
                    self.set_state(HGantry.State(self.actual_x, self.actual_y))

                    if skipped_x_mm != 0.0 or skipped_y_mm != 0.0:
                        logging.debug(f'Hit limit and skipped:  {skipped_x_mm} mm (x); {skipped_y_mm} mm (y)')
                        succeeded_without_limit = False

                    logging.debug(f'Move buffer length:  {len(self.move_buffer)}')

                logging.info('Drained.')

            # advance x and y positions as if the moves are already completed, ignoring buffering. this is important
            # because subsequent calls to move need to be relative to this resulting location.
            self.x += move_x_mm
            self.y += move_y_mm

            return succeeded_without_limit

        # ensure lock is released
        finally:
            self.move_to_point_lock.release()

    def clear_async_results_buffer(
            self
    ) -> bool:
        """
        Clear the results buffer. Does not move the gantry beyond the moves currently in the buffer. Blocks until all
        moves are complete.

        :return: True if the buffer was cleared without hitting a limit switch; False if limit switch was hit by the
        final move.
        """

        return self.move_to_offset(0.0, 0.0, 1.0, True, False)

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
            check_bounds: bool
    ):
        """
        Trace a list of points.

        :param points: Points.
        :param mm_per_sec: Speed in mm per second.
        :param block: Whether to block until the movement is complete.
        :param check_bounds: Whether to check bounds of the point. Raises an exception if check fails.
        :param return_to_current_position: Whether to return to the current position after moving to the points.
        """

        if check_bounds:
            all(self.assert_point_in_bounds(x, y) for x, y in points)

        if return_to_current_position:
            points = points.copy()
            points.append((self.x, self.y))

        num_points = len(points)
        for i, (x, y) in enumerate(points):
            logging.debug(f'Moving to point {i + 1} of {num_points}:  {x:.3f},{y:.3f}')
            self.move_to_point(x, y, mm_per_sec, block, check_bounds)

    def move_to_offset(
            self,
            x_offset_mm: float,
            y_offset_mm: float,
            mm_per_sec: float,
            block: bool,
            check_bounds: bool
    ) -> Optional[bool]:
        """
        Move to an offset from the current position.

        :param x_offset_mm: X offset.
        :param y_offset_mm: Y offset.
        :param mm_per_sec: Speed in mm per second.
        :param block: Whether to block until the movement is complete.
        :param check_bounds: Whether to check bounds of the point. Raises an exception if check fails.
        :return: True if move was achieved without hitting a limit switch; False if limit switch was hit before move was
        achieved. Will be None if the move was buffered and not completed.
        """

        return self.move_to_point(self.x + x_offset_mm, self.y + y_offset_mm, mm_per_sec, block, check_bounds)

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

        # pump moves into the buffer until we get one back that indicates a limit switch was hit
        move_distance_mm = math.sqrt(x_offset_mm ** 2 + y_offset_mm ** 2)
        move_time_seconds = move_distance_mm / mm_per_sec
        sleep_time_seconds = move_time_seconds / 2.0
        while self.move_to_offset(x_offset_mm, y_offset_mm, mm_per_sec, False, False) != False:
            sleep(sleep_time_seconds)
        else:
            self.clear_async_results_buffer()

    def trace_spyrograph(
            self,
            g: _Trochoid,
            center: Tuple[float, float],
            mm_per_sec: float,
            return_to_current_position: bool,
            block: bool,
            check_bounds: bool
    ) -> _Trochoid:
        """
        Trace a spyrograph object.

        :param g: Spyrograph.
        :param center: Location of center.
        :param mm_per_sec: Speed.
        :param return_to_current_position: Whether to return to current position.
        :param block: Whether to block until the movement is complete.
        :param check_bounds: Whether to check bounds of the point. Raises an exception if check fails.
        :return: Resulting spyrograph, which might be scaled and translated.
        """

        center_x, center_y = center
        half_width = (g.max_x - g.min_x) / 2.0
        left_border = center_x - half_width
        half_height = (g.max_y - g.min_y) / 2.0
        bottom_border = center_y - half_height
        g = g.translate(left_border - g.min_x, bottom_border - g.min_y)
        logging.info(
            f'Tracing spyrograph within bounds:  ({g.min_x:.3f},{g.min_y:.3f}) (LL) ({g.max_x:.3f},{g.max_y:.3f}) (UR)'
        )

        if g.max_x > self.left_right_mm:
            g = g.scale(1.0 - g.max_x / self.left_right_mm)
            logging.warning('Spyrograph x out of bounds. Rescaled.')

        if g.max_y > self.bottom_top_mm:
            g = g.scale(1.0 - g.max_y / self.bottom_top_mm)
            logging.warning('Spyrograph y out of bounds. Rescaled.')

        self.move_to_points(
            list(zip(g.x, g.y)),
            mm_per_sec,
            return_to_current_position,
            False,
            check_bounds
        )

        if block:
            self.clear_async_results_buffer()

        return g

    def get_ui_elements(
            self
    ) -> List[Tuple[Union[str, Tuple[str, str]], str]]:
        """
        Get UI elements for the current component.

        :return: List of 2-tuples of (1) element key and (2) element content.
        """

        return [
            RpyFlask.get_button(self.id, self.calibrate, {'mm_per_sec': 10.0}, None, None, None, 'Calibrate')
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
