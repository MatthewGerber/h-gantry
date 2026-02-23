from typing import List, Tuple

import matplotlib.pyplot as plt

MIN_US_PER_DRIVE_FROM_STOPPED = 1000
MIN_US_PER_DRIVE = 100
FULL_ACCEL_INTERVAL_SEC = 0.5
US_PER_SEC = 1e6
MAX_DRIVE_ACC_US_PER_DRIVE_PER_US = (MIN_US_PER_DRIVE_FROM_STOPPED - MIN_US_PER_DRIVE) / (FULL_ACCEL_INTERVAL_SEC * float(US_PER_SEC))


def get_drive_delays_us(
        prev_drive_delay_us: int,
        num_drives: int,
        total_desired_delay_us: int,
        min_us_per_drive: int,
        min_us_per_drive_from_stopped: int,
        max_drive_acc_us_per_drive_per_us: float
) -> List[int]:

    drive_delay_us = [0] * num_drives

    if prev_drive_delay_us == -1:
        num_delays = num_drives - 1
        i = 1
        delay_to_curr_drive_us = min_us_per_drive_from_stopped
    else:
        num_delays = num_drives
        i = 0
        delay_to_curr_drive_us = prev_drive_delay_us

    target_delay_per_drive_us = max(float(min_us_per_drive), total_desired_delay_us / num_delays)
    plt.axhline(target_delay_per_drive_us, linestyle='dashed', color='orange', label=f'Constant delay:  {target_delay_per_drive_us:.2f} us')
    trunc_loss = 0.0
    while i < len(drive_delay_us):

        truncated_delay_us = int(delay_to_curr_drive_us)

        trunc_loss += delay_to_curr_drive_us - truncated_delay_us
        if trunc_loss > 1.0:
            loss_correction = int(trunc_loss)
            truncated_delay_us += loss_correction
            trunc_loss -= loss_correction

        drive_delay_us[i] = truncated_delay_us

        if delay_to_curr_drive_us > target_delay_per_drive_us:
            max_accel_this_drive = delay_to_curr_drive_us * max_drive_acc_us_per_drive_per_us
            adjusted_delay_us = max(target_delay_per_drive_us, delay_to_curr_drive_us - max_accel_this_drive)
            if adjusted_delay_us <= target_delay_per_drive_us:
                num_remaining_delays = len(drive_delay_us) - i - 1
                remaining_delay_us = total_desired_delay_us - sum(drive_delay_us)
                target_delay_per_drive_us = max(float(min_us_per_drive), remaining_delay_us / num_remaining_delays)
        else:
            adjusted_delay_us = target_delay_per_drive_us

        delay_to_curr_drive_us = adjusted_delay_us

        i += 1

    plt.plot(
        drive_delay_us,
        label='Drive delay (us)'
    )
    plt.scatter(0, prev_drive_delay_us, label=f'Prev delay:  {prev_drive_delay_us} us')
    total_actual_delay_us = sum(drive_delay_us)
    plt.xlabel(
        f'Delay (us);\n'
        f'desired total {total_desired_delay_us / 1000.0:.1f} ms;\n'
        f'actual total {total_actual_delay_us / 1000.0:.1f} ms;\n'
        f'diff (actual - desired) {(total_actual_delay_us - total_desired_delay_us) / 1000.0:.1f} ms'
    )
    plt.ylabel('Delay (us)')
    plt.axhline(min_us_per_drive, linestyle='dashed', color='red', label=f'Min feasible delay:  {min_us_per_drive} us')
    plt.legend()
    plt.tight_layout()
    plt.show()
    plt.close()

    print(str(drive_delay_us))

    return drive_delay_us


def get_dual_delays(
        prev_a_drive_delay_us: int,
        num_a_drives: int,
        prev_b_drive_delay_us: int,
        num_b_drives: int,
        total_desired_delay_us: int,
        min_us_per_drive: int,
        min_us_per_drive_from_stopped: int,
        max_drive_acc_us_per_drive_per_us: float
) -> Tuple[List[int], List[int]]:
    a_drive_delays = get_drive_delays_us(prev_a_drive_delay_us, num_a_drives, total_desired_delay_us, min_us_per_drive, min_us_per_drive_from_stopped, max_drive_acc_us_per_drive_per_us)
    total_a_delay_us = sum(a_drive_delays)
    b_drive_delays = get_drive_delays_us(prev_b_drive_delay_us, num_b_drives, total_desired_delay_us, min_us_per_drive, min_us_per_drive_from_stopped, max_drive_acc_us_per_drive_per_us)
    total_b_delay_us = sum(b_drive_delays)

    print(
        f'Before realignment:\n'
        f'Total a delay:  {total_a_delay_us}\n'
        f'Total b delay:  {total_b_delay_us}'
    )

    if total_a_delay_us == total_b_delay_us:
        pass
    elif total_a_delay_us > total_b_delay_us:
        b_drive_delays = get_drive_delays_us(prev_b_drive_delay_us, num_b_drives, total_a_delay_us, min_us_per_drive, min_us_per_drive_from_stopped, max_drive_acc_us_per_drive_per_us)
    else:
        a_drive_delays = get_drive_delays_us(prev_a_drive_delay_us, num_a_drives, total_b_delay_us, min_us_per_drive, min_us_per_drive_from_stopped, max_drive_acc_us_per_drive_per_us)

    print(
        f'After realignment:\n'
        f'Total a delay:  {sum(a_drive_delays)}\n'
        f'Total b delay:  {sum(b_drive_delays)}'
    )

    return a_drive_delays, b_drive_delays

def main():

    stepper_full_steps_per_revolution = 200
    drives_per_step = 4
    revolutions = 4
    drives = revolutions * stepper_full_steps_per_revolution * drives_per_step

    # get_drive_delays_us(-1, drives, 500 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)
    # get_drive_delays_us(600, drives, 500 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)
    # get_drive_delays_us(100, drives, 500 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)
    # get_drive_delays_us(-1, drives, 4000 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)
    # delays = get_drive_delays_us(-1, drives, 3000 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)
    # delays.extend(get_drive_delays_us(937, drives, 500 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US))
    # plt.plot(delays)
    # plt.tight_layout()
    # plt.show()
    # plt.close()
    # get_drive_delays_us(MIN_US_PER_DRIVE, drives, 4000 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)
    # get_drive_delays_us(MIN_US_PER_DRIVE, drives, 1000 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)

    a_delays, b_delays = get_dual_delays(-1, drives, -1, int(drives / 2), 3000 * 1000, MIN_US_PER_DRIVE, MIN_US_PER_DRIVE_FROM_STOPPED, MAX_DRIVE_ACC_US_PER_DRIVE_PER_US)


if __name__ == '__main__':
    main()
