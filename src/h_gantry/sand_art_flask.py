import logging
import os

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

logging.basicConfig(level=logging.INFO)

locking_serial = LockingSerial(
    connection=Serial(
        port='/dev/serial0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    ),
    throughput_step_size=0.05,
    manual_buffer=True
)

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
    state_path=os.path.expanduser('~/Desktop/h-gantry-state.json')
)
gantry.event(lambda s: logging.debug(f'Gantry state:  {s}'))

pixels = neopixel.NeoPixel(microcontroller.Pin(int(CkPin.MOSI)), 144, brightness=0.1, auto_write=False)
led_strip = FrameLedStrip(pixels, 7.0, 100.0, 100.0)

def update_led_strip_on_gantry_update(
        gantry_state: HGantry.State
):
    """
    Update the LED strip when the gantry state changes.

    :param gantry_state: Gantry state.
    """

    try:
        led_strip.turn_off()
        led_strip.cross_point(100.0 * gantry_state.x / gantry.left_right_mm, 100.0 * gantry_state.y / gantry.bottom_top_mm, FrameLedStrip.GREEN)
        led_strip.show()
    except LedStrip.InvalidPixelError as e:
        logging.error(f'Error while setting LED strip:  {e}')

gantry.event(update_led_strip_on_gantry_update)

gantry.id = 'gantry-1'
gantry.start()

app.add_component(gantry)

def on_exit():
    """
    Clean up, save state, etc.
    """

    gantry.stop(True)

app.register_on_exit_callback(on_exit)
