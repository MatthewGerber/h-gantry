from h_gantry.core import HGantry
from raspberry_py.gpio import setup, cleanup, CkPin
from raspberry_py.gpio.motors import Stepper


def main():
    """
    Gantry demonstration.
    """

    setup()

    poles = 32
    output_rotor_ratio = 1 / 64.0

    left_stepper = Stepper(
        poles=poles,
        output_rotor_ratio=output_rotor_ratio,
        driver_pin_1=CkPin.GPIO18,
        driver_pin_2=CkPin.GPIO23,
        driver_pin_3=CkPin.GPIO24,
        driver_pin_4=CkPin.GPIO25
    )

    right_stepper = Stepper(
        poles=poles,
        output_rotor_ratio=output_rotor_ratio,
        driver_pin_1=CkPin.GPIO18,
        driver_pin_2=CkPin.GPIO23,
        driver_pin_3=CkPin.GPIO24,
        driver_pin_4=CkPin.GPIO25
    )

    gantry = HGantry(
        left_stepper=left_stepper,
        right_stepper=right_stepper
    )

    gantry.start()
    gantry.calibrate()
    gantry.home()
    gantry.move_x(20.0)
    gantry.move_x(-20.0)
    gantry.move_y(20.0)
    gantry.move_y(-20.0)
    gantry.stop()

    cleanup()


if __name__ == '__main__':
    main()
