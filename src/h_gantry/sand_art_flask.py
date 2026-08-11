import logging
import os
from threading import Lock

import RPi.GPIO as gpio
import microcontroller
import neopixel
import serial
from serial import Serial

from h_gantry.core import HGantry
from raspberry_py.gpio import CkPin, setup
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.lights import FrameLedStrip, LedStrip
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoA4988
from raspberry_py.rest.application import app

setup(gpio.BCM)

STEPPER_FULL_STEPS_PER_REVOLUTION = 200
STEPPER_OUTPUT_ROTOR_RATIO = 1.0 / 1.0

locking_serial = LockingSerial(
    connection=Serial(
        port='/dev/serial0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    ),
    throughput_step_size=0.05,
    manual_buffer=False
)
locking_serial.synchronize_epoch_time( 4, 5, 6)
locking_serial.manual_buffer = True

left_stepper = Stepper(
    full_steps_per_revolution=STEPPER_FULL_STEPS_PER_REVOLUTION,
    output_rotor_ratio=STEPPER_OUTPUT_ROTOR_RATIO,
    driver=StepperMotorDriverArduinoA4988(
        driver_pin=5,
        disable_pin=10,
        direction_pin=6,
        identifier=0,
        serial=locking_serial,
        asynchronous=True
    ),
    reverse=False
)

right_stepper = Stepper(
    full_steps_per_revolution=STEPPER_FULL_STEPS_PER_REVOLUTION,
    output_rotor_ratio=STEPPER_OUTPUT_ROTOR_RATIO,
    driver=StepperMotorDriverArduinoA4988(
        driver_pin=7,
        disable_pin=11,
        direction_pin=8,
        identifier=1,
        serial=locking_serial,
        asynchronous=True
    ),
    reverse=False
)

gantry = HGantry(
    joystick_z_pin=CkPin.GPIO17,
    left_stepper=left_stepper,
    right_stepper=right_stepper,
    left_limit_switch_arduino_pin=3,
    right_limit_switch_arduino_pin=4,
    bottom_limit_switch_arduino_pin=2,
    top_limit_switch_arduino_pin=12,
    arduino_serial=locking_serial,
    timing_pulley_dia_mm=12.97,
    state_path=os.path.expanduser('~/Desktop/h-gantry-state.pickle')
)
gantry.id = 'gantry-1'
app.add_component(gantry)

gantry.event(lambda s: logging.debug(f'Gantry state:  {s}'))

# configure lighting as an event on the gantry state change
pixels = neopixel.NeoPixel(microcontroller.Pin(int(CkPin.MOSI)), 288 - 15, brightness=0.1, auto_write=False)
led_strip = FrameLedStrip(
    pixels,
    7.0,
    482.6,
    482.6,
    illuminated_width_height_x_off_y_off_mm=(
        7.9375,
        7.9375,
        36.0,
        36.0
    )
)
led_lock = Lock()
def update_led_strip_on_gantry_update(
        gantry_state: HGantry.State
):
    """
    Update the LED strip when the gantry state changes.

    :param gantry_state: Gantry state.
    """

    if led_lock.acquire(blocking=False):
        try:
            if (
                gantry_state.started and
                gantry_state.enabled and
                gantry_state.calibration_status == HGantry.CalibrationStatus.CALIBRATED
            ):
                led_strip.cross_point(gantry_state.x_est, gantry_state.y_est, FrameLedStrip.GREEN)
            elif not gantry_state.started:
                led_strip.cross_point(gantry_state.x_est, gantry_state.y_est, FrameLedStrip.RED)
            elif not gantry_state.enabled:
                led_strip.cross_point(gantry_state.x_est, gantry_state.y_est, FrameLedStrip.YELLOW)
            else:
                led_strip.turn_off()
        except LedStrip.InvalidPixelError as e:
            logging.error(f'Error while setting LED strip:  {e}')
        finally:
            led_lock.release()

gantry.event(update_led_strip_on_gantry_update, synchronous=False)

def on_exit():
    """
    Clean up, save state, etc.
    """

    gantry.stop()

app.register_on_exit_callback(on_exit)
app.start(__name__)
