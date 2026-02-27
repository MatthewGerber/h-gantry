import logging
import os.path
import time

import serial
from serial import Serial

from h_gantry.core import HGantry
from raspberry_py.gpio import setup, cleanup, CkPin
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoA4988


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
        throughput_step_size=0.05,
        manual_buffer=True
    )

    stepper_full_steps_per_revolution = 200
    stepper_output_rotor_ratio = 1.0 / 1.0

    left_stepper = Stepper(
        full_steps_per_revolution=stepper_full_steps_per_revolution,
        output_rotor_ratio=stepper_output_rotor_ratio,
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
        full_steps_per_revolution=stepper_full_steps_per_revolution,
        output_rotor_ratio=stepper_output_rotor_ratio,
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
    gantry.event(lambda s: logging.debug(f'Gantry position:  {s}'))

    gantry.start()

    # gantry.move_to_offset(10.0, 0.0, 100.0, True, False)
    # gantry.move_to_offset_limit(1.0, 1.0, 10.0)
    # gantry.move_to_offset(0.0, 10.0, 100.0, False, True)
    # gantry.move_to_offset(0.0, 10.0, 100.0, False, True)
    # gantry.clear_move_buffer()
    # gantry.move_to_offset(0.0, -10.0, 10.0, True, True)
    # gantry.move_to_offset(0.0, 0.0, 1.0, True)
    # gantry.calibrate(150.0)
    # gantry.center(150.0, True, True)
    # gantry.move_to_top_limit(10.0)
    # circle_points = generate_circle_points(gantry.x, gantry.y, 50.0, 0.5)
    # gantry.move_to_points(circle_points, 10.0, True)
    # for i in range(30):
    #     gantry.joystick.update_state()
    #     time.sleep(0.5)
    #     print(f'Joystick update {i}')
    # try:
    #     time.sleep(1000.0)
    # except KeyboardInterrupt:
    #     pass

    # try:
    #     time.sleep(1000.0)
    # except KeyboardInterrupt:
    #     pass

    # gantry.draw_spirograph_from_params(
    #     350,
    #     200,
    #     100,
    #     0.01,
    #     0.5,
    #     150.0,
    #     10.0,
    #     True,
    #     True,
    #     True
    # )

    gantry.stop(True)

    cleanup()


if __name__ == '__main__':
    main()
