import logging
import os.path

import serial
from serial import Serial

from h_gantry.core import HGantry, generate_circle_points
from raspberry_py.gpio import setup, cleanup
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoUln2003


def main():
    """
    Gantry demonstration.
    """

    logging.basicConfig(level=logging.INFO)

    setup()

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
    output_rotor_ratio = 1 / 64.0

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
        left_stepper=left_stepper,
        right_stepper=right_stepper,
        left_limit_switch_arduino_pin=3,
        right_limit_switch_arduino_pin=4,
        bottom_limit_switch_arduino_pin=2,
        top_limit_switch_arduino_pin=12,
        arduino_serial=locking_serial,
        timing_pulley_dia_mm=12.27,
        state_path=os.path.expanduser('~/Desktop/h-gantry-state.json')
    )

    gantry.start()
    gantry.calibrate(100.0)
    gantry.center(100.0)
    # gantry.move_to_point(-50.0, 0.0, 100.0)
    # gantry.move_to_point(50.0, 0.0, 100.0)
    # gantry.move_to_point(0.0, 0.0, 100.0)
    # gantry.move_to_point(0.0, 50.0, 100.0)
    # gantry.move_to_point(0.0, -50.0, 100.0)
    # gantry.move_to_point(0.0, 0.0, 100.0)
    # circle_points = generate_circle_points(gantry.x, gantry.y, 50.0, 1.0)
    # gantry.move_to_points(circle_points, 100.0, True)

    gantry.stop(True)

    cleanup()


def test_cross_pattern(
        gantry: HGantry,
        speed_mm_per_sec: float
):
    gantry.move_to_point(10, 0, speed_mm_per_sec)
    gantry.move_to_point(-10, 0, speed_mm_per_sec)
    gantry.move_to_point(0, 0, speed_mm_per_sec)
    gantry.move_to_point(0, 10, speed_mm_per_sec)
    gantry.move_to_point(0, -10, speed_mm_per_sec)
    gantry.move_to_point(0, 0, speed_mm_per_sec)


if __name__ == '__main__':
    main()
