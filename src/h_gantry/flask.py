import logging
import os

import serial
from raspberry_py.gpio import CkPin
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoUln2003
from raspberry_py.rest.application import app
from serial import Serial

from src.h_gantry.core import HGantry

locking_serial = LockingSerial(
    connection=Serial(
        port='/dev/serial0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    ),
    throughput_step_size=0.05
)

poles = 32
output_rotor_ratio = 1.0 / 64.0

left_stepper = Stepper(
    poles=poles,
    output_rotor_ratio=output_rotor_ratio,
    driver=StepperMotorDriverArduinoUln2003(
        driver_pin_1=5,
        driver_pin_2=6,
        driver_pin_3=7,
        driver_pin_4=8,
        identifier=0,
        serial=locking_serial,
        asynchronous=True
    ),
    reverse=False
)

right_stepper = Stepper(
    poles=poles,
    output_rotor_ratio=output_rotor_ratio,
    driver=StepperMotorDriverArduinoUln2003(
        driver_pin_1=9,
        driver_pin_2=10,
        driver_pin_3=11,
        driver_pin_4=13,
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
gantry.event(lambda s: logging.debug(f'Gantry position:  {s}'))
gantry.id = 'gantry-1'

app.add_component(gantry)
