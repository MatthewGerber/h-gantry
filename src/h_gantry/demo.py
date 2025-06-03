import serial
from serial import Serial

from h_gantry.core import HGantry
from raspberry_py.gpio import setup, cleanup
from raspberry_py.gpio.communication import LockingSerial
from raspberry_py.gpio.motors import Stepper, StepperMotorDriverArduinoUln2003


def main():
    """
    Gantry demonstration.
    """

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
            driver_pin_4=12,
            identifier=1,
            serial=locking_serial,
            asynchronous=True
        ),
        reverse=False
    )

    gantry = HGantry(
        left_stepper=left_stepper,
        right_stepper=right_stepper,
        timing_pulley_tooth_count=32,
        timing_pulley_tooth_pitch_mm=2.0
    )

    gantry.start()
    gantry.calibrate()
    # gantry.home()
    gantry.move_x(20.0)
    gantry.move_x(-20.0)
    gantry.move_y(20.0)
    gantry.move_y(-20.0)
    gantry.stop()

    cleanup()


if __name__ == '__main__':
    main()
