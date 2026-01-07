const size_t FLOAT_BYTES_LEN = 4;
const size_t LONG_BYTES_LEN = 4;
const unsigned long US_PER_SEC = 1e6;  // microseconds per second

// SerialUSB writes to the arduino IDE serial monitor; _UART1_ writes to the serial tx/rx gpio pins
#define SerialUART _UART1_

// structure that gives simultaneous access to floating-point numbers and their underlying bytes.
typedef union {
  float number;
  byte bytes[FLOAT_BYTES_LEN];
} floatbytes;

// stepper driver configuration
const byte STEPPER_DRIVER_NUM_IN_PINS = 1;  // number of pins providing input to the stepper motor driver.
const byte NUM_MICROSTEPS = 2;  // 1:  full steps, 2:  half steps, 4:  quarter steps, etc.
const byte DRIVE_SEQUENCE_LEN = 2;  // always 2, since microstepping is specified with separate outputs
const byte DRIVE_SEQUENCE[DRIVE_SEQUENCE_LEN][STEPPER_DRIVER_NUM_IN_PINS] = {
  { LOW },
  { HIGH }
};
const unsigned long MIN_US_PER_DRIVE = 500;  // fastest driving with acceleration
const unsigned long MIN_US_PER_DRIVE_FROM_STOPPED = 2000;  // fastest driving directly from a dead stop
const float FULL_ACCEL_INTERVAL_SEC = 0.1;  // fastest acceleration from dead stop to fastest
const byte MICROSTEP_MS1_OUTPUT_PIN = 9;

// config for the ULN2003, which is driven by four output pins (4 driver pins @ 2 drives per step):
// const byte STEPPER_DRIVER_NUM_IN_PINS = 4;  // number of pins providing input to the stepper motor driver.
// const byte NUM_MICROSTEPS = 2;  // 1:  full steps, 2:  half steps, 4:  quarter steps, etc.
// const byte DRIVE_SEQUENCE_LEN = 8;  // 4 pins @ half steps
// const byte DRIVE_SEQUENCE[DRIVE_SEQUENCE_LEN][STEPPER_DRIVER_NUM_IN_PINS] = {
//   { HIGH, LOW, LOW, LOW },
//   { HIGH, HIGH, LOW, LOW },
//   { LOW, HIGH, LOW, LOW },
//   { LOW, HIGH, HIGH, LOW },
//   { LOW, LOW, HIGH, LOW },
//   { LOW, LOW, HIGH, HIGH },
//   { LOW, LOW, LOW, HIGH },
//   { HIGH, LOW, LOW, HIGH }
// };
// const unsigned long MIN_US_PER_DRIVE = 1000;  // fastest driving with acceleration
// const unsigned long MIN_US_PER_DRIVE_FROM_STOPPED = 1e6;  // fastest driving directly from a dead stop
// const float FULL_ACCEL_INTERVAL_SEC = 0.25;  // fastest acceleration from dead stop to fastest

// config for the A4988 driver (e.g., for nema steppers):
// const byte STEPPER_DRIVER_NUM_IN_PINS = 1;  // number of pins providing input to the stepper motor driver.
// const byte NUM_MICROSTEPS = 2;  // 1:  full steps, 2:  half steps, 4:  quarter steps, etc.
// const byte DRIVE_SEQUENCE_LEN = 2;  // always 2, since microstepping is specified with separate outputs
// const byte DRIVE_SEQUENCE[DRIVE_SEQUENCE_LEN][STEPPER_DRIVER_NUM_IN_PINS] = {
//   { LOW },
//   { HIGH }
// };
// const unsigned long MIN_US_PER_DRIVE = 500;  // fastest driving with acceleration
// const unsigned long MIN_US_PER_DRIVE_FROM_STOPPED = 1e6;  // fastest driving directly from a dead stop
// const float FULL_ACCEL_INTERVAL_SEC = 0.1;  // fastest acceleration from dead stop to fastest
// const byte MICROSTEP_MS1_OUTPUT_PIN = 9;

// Microstep configuration for the A4988
// MS1	MS2	  MS3	Microstep Resolution -- these are pull-down, so to get half stepping we only need to output HIGH to MS1
// Low	Low	  Low	Full step
// High	Low   Low	Half step
// Low	High  Low	Quarter step
// High	High	Low	Eighth step
// High	High	High	Sixteenth step

const float MAX_DRIVE_ACC_US_PER_DRIVE_PER_US = (MIN_US_PER_DRIVE_FROM_STOPPED - MIN_US_PER_DRIVE) / (FULL_ACCEL_INTERVAL_SEC * float(US_PER_SEC));  // maximum acceleration:  slowest to fastest within given interval

// left stepper
const byte LEFT_STEPPER_ID = 0;
byte left_driver_pins[STEPPER_DRIVER_NUM_IN_PINS];
int left_driver_dir_pin;  // -1 for no direction pin
long left_stepper_drive_idx;
long left_stepper_drive_target;
int left_stepper_drive_increment;
long left_stepper_limit_skipped_drives;
unsigned long left_stepper_us_per_drive;
unsigned long left_stepper_previous_drive_us;
unsigned long left_stepper_us_per_drive_target;
unsigned long left_stepper_previous_acceleration_us;
bool left_stepper_inited = false;

// right stepper
const byte RIGHT_STEPPER_ID = 1;
byte right_driver_pins[STEPPER_DRIVER_NUM_IN_PINS];
int right_driver_dir_pin;  // -1 for no direction pin
long right_stepper_drive_idx;
long right_stepper_drive_target;
int right_stepper_drive_increment;
long right_stepper_limit_skipped_drives;
unsigned long right_stepper_us_per_drive;
unsigned long right_stepper_previous_drive_us;
unsigned long right_stepper_us_per_drive_target;
unsigned long right_stepper_previous_acceleration_us;
bool right_stepper_inited = false;

// linked list of steps to take, acting as a read buffer. each step determines how the
// left and right steppers should move in tandem.
struct step {
  int left_stepper_num_drives;
  unsigned long left_stepper_us_per_drive;
  int right_stepper_num_drives;
  unsigned long right_stepper_us_per_drive;
  step* next;
};
step* steps_head = nullptr;
step* steps_tail = nullptr;
unsigned int steps_len = 0;

/**
 * Add a step to the move buffer.
 *
 * @param left_stepper_num_drives Number of drives to apply to the left stepper.
 * @param left_stepper_us_per_drive Microseconds per drive.
 * @param right_stepper_num_drives Number of drives to apply to the right stepper.
 * @param right_stepper_us_per_drive Microseconds per drive.
 */
void add_step(
  int left_stepper_num_drives, 
  unsigned long left_stepper_us_per_drive,
  int right_stepper_num_drives, 
  unsigned long right_stepper_us_per_drive
) {

  step* new_step = new step();
  new_step->left_stepper_num_drives = left_stepper_num_drives;
  new_step->left_stepper_us_per_drive = left_stepper_us_per_drive;
  new_step->right_stepper_num_drives = right_stepper_num_drives;
  new_step->right_stepper_us_per_drive = right_stepper_us_per_drive;
  new_step->next = nullptr;

  if (steps_head == nullptr) {
    steps_head = steps_tail = new_step;
  }
  else {
    steps_tail->next = new_step;
    steps_tail = new_step;
  }
  steps_len += 1;

}

/**
 * Remove and return the next step from the buffer.
 * 
 * @return The next step, or null if the buffer is empty.
 */
step* get_next_step() {
  step* next_step = steps_head;
  if (next_step != nullptr) {
    steps_head = next_step->next;
    if (steps_head == nullptr) {
      steps_tail = nullptr;
    }
    steps_len -=1;
  }
  return next_step;
}

/**
 * Start a movement step.
 *
 * @param to_start Step to start.
 * @param drive_left_immediately Whether to drive the left stepper immediately (true) or wait for the specified time to elapse (false).
 * @param drive_right_immediately Whether to drive the right stepper immediately (true) or wait for the specified time to elapse (false).
 * @param curr_time_us Current time in microseconds, as returned by micros().
 */
void start_step(step* to_start, bool drive_left_immediately, bool drive_right_immediately, unsigned long curr_time_us) {

  if (to_start->left_stepper_num_drives == 0) {
    left_stepper_drive_increment = 0;
    left_stepper_us_per_drive = 0;
    left_stepper_us_per_drive_target = 0;
    write_stepper_done(LEFT_STEPPER_ID, 0);
  }
  else {
    left_stepper_drive_idx = mod(left_stepper_drive_idx, DRIVE_SEQUENCE_LEN);  // mod initial drive idx to avoid overflow
    left_stepper_drive_target = left_stepper_drive_idx + to_start->left_stepper_num_drives;
    left_stepper_drive_increment = to_start->left_stepper_num_drives > 0 ? 1 : -1;

    if (left_driver_dir_pin >= 0) {
      digitalWrite(left_driver_dir_pin, left_stepper_drive_increment < 0 ? LOW : HIGH);
    }

    left_stepper_limit_skipped_drives = 0;

    left_stepper_us_per_drive_target = to_start->left_stepper_us_per_drive;

    // if we're accelerating from a dead stop, limit initial speed to the fastest direct acceleration. if we're presently moving, then we can accelerate from the current speed.
    if (left_stepper_us_per_drive == 0) {
      left_stepper_us_per_drive = left_stepper_us_per_drive_target > MIN_US_PER_DRIVE_FROM_STOPPED ? left_stepper_us_per_drive_target : MIN_US_PER_DRIVE_FROM_STOPPED;
    }

    if (drive_left_immediately) {
      left_stepper_previous_drive_us = curr_time_us - left_stepper_us_per_drive;
    }

    left_stepper_previous_acceleration_us = curr_time_us;
  }

  if (to_start->right_stepper_num_drives == 0) {
    right_stepper_drive_increment = 0;
    right_stepper_us_per_drive = 0;
    right_stepper_us_per_drive_target = 0;
    write_stepper_done(RIGHT_STEPPER_ID, 0);
  }
  else {
    right_stepper_drive_idx = mod(right_stepper_drive_idx, DRIVE_SEQUENCE_LEN);  // mod initial drive idx to avoid overflow
    right_stepper_drive_target = right_stepper_drive_idx + to_start->right_stepper_num_drives;
    right_stepper_drive_increment = to_start->right_stepper_num_drives > 0 ? 1 : -1;

    if (right_driver_dir_pin >= 0) {
      digitalWrite(right_driver_dir_pin, right_stepper_drive_increment < 0 ? LOW : HIGH);
    }

    right_stepper_limit_skipped_drives = 0;

    right_stepper_us_per_drive_target = to_start->right_stepper_us_per_drive;

    // if we're accelerating from a dead stop, limit initial speed to the fastest direct acceleration. if we're presently moving, then we can accelerate from the current speed.
    if (right_stepper_us_per_drive == 0) {
      right_stepper_us_per_drive = right_stepper_us_per_drive_target > MIN_US_PER_DRIVE_FROM_STOPPED ? right_stepper_us_per_drive_target : MIN_US_PER_DRIVE_FROM_STOPPED;
    }

    if (drive_right_immediately) {
      right_stepper_previous_drive_us = curr_time_us - right_stepper_us_per_drive;
    }

    right_stepper_previous_acceleration_us = curr_time_us;
  }

}

// limit switches
const byte LIMIT_SWITCHES_ID = 2;
byte left_limit_switch_pin;
byte right_limit_switch_pin;
byte bottom_limit_switch_pin;
byte top_limit_switch_pin;
bool limit_switches_inited = false;

// command id and component id
const size_t CMD_BYTES_LEN = 2;

// command:  init component
const byte CMD_INIT = 1;
const size_t CMD_INIT_STEPPER_ARGS_LEN = STEPPER_DRIVER_NUM_IN_PINS + 2;  // 1 byte per pin plus 2 bytes for optional direction pin, which is -1 for no direction pin.
const size_t CMD_INIT_LIMIT_SWITCHES_ARGS_LEN = 4;

// command:  step
const byte CMD_STEP = 2;
const byte CMD_STEP_ARGS_LEN = 12;

// command:  stop
const byte CMD_STOP = 3;

void long_to_bytes(long value, byte bytes[]) {
  bytes[0] = (byte)(value >> 24);
  bytes[1] = (byte)(value >> 16);
  bytes[2] = (byte)(value >> 8);
  bytes[3] = (byte)value;
}

long bytes_to_long(byte bytes[], size_t start_idx) {
  uint32_t value = ((uint32_t)bytes[start_idx]) << 24;
  value |= bytes[start_idx + 1] << 16;
  value |= bytes[start_idx + 2] << 8;
  value |= bytes[start_idx + 3];
  return (int32_t)value;
}

void write_long(long value) {
  byte bytes[LONG_BYTES_LEN];
  long_to_bytes(value, bytes);
  SerialUART.write(bytes, LONG_BYTES_LEN);
}

void unsigned_long_to_bytes(unsigned long value, byte bytes[]) {
  bytes[0] = (byte)(value >> 24);
  bytes[1] = (byte)(value >> 16);
  bytes[2] = (byte)(value >> 8);
  bytes[3] = (byte)value;
}

unsigned long bytes_to_unsigned_long(byte bytes[], size_t start_idx) {
  uint32_t value = ((uint32_t)bytes[start_idx]) << 24;
  value |= bytes[start_idx + 1] << 16;
  value |= bytes[start_idx + 2] << 8;
  value |= bytes[start_idx + 3];
  return value;
}

void write_unsigned_long(unsigned long value) {
  byte bytes[LONG_BYTES_LEN];
  unsigned_long_to_bytes(value, bytes);
  SerialUART.write(bytes, LONG_BYTES_LEN);
}

void int_to_bytes(int value, byte bytes[]) {
  bytes[0] = (byte)(value >> 8);
  bytes[1] = (byte)value;
}

int bytes_to_int(byte bytes[], size_t start_idx) {
  uint16_t value = ((uint16_t)bytes[start_idx]) << 8;
  value |= bytes[start_idx + 1];
  return (int16_t)value;
}

void unsigned_int_to_bytes(unsigned int value, byte bytes[]) {
  bytes[0] = (byte)(value >> 8);
  bytes[1] = (byte)value;
}

unsigned int bytes_to_unsigned_int(byte bytes[], size_t start_idx) {
  uint16_t value = ((uint16_t)bytes[start_idx]) << 8;
  value |= bytes[start_idx + 1];
  return value;
}

void set_float_bytes(byte dest[], byte src[], size_t src_start_idx) {
  dest[0] = src[src_start_idx];
  dest[1] = src[src_start_idx + 1];
  dest[2] = src[src_start_idx + 2];
  dest[3] = src[src_start_idx + 3];
}

void write_float(floatbytes f) {
  SerialUART.write(f.bytes, FLOAT_BYTES_LEN);
}

void write_bool(bool value) {
  SerialUART.write(value);
}

void write_byte(byte value) {
  SerialUART.write(value);
}

byte mod(long x, byte y) {
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

/**
 * Drive the left stepper at its current drive index.
 *
 * @param curr_time_us Current time in microseconds.
 */
void drive_left_stepper(unsigned long curr_time_us) {
  byte drive_sequence_idx = mod(left_stepper_drive_idx, DRIVE_SEQUENCE_LEN);
  for (byte pin_idx = 0; pin_idx < STEPPER_DRIVER_NUM_IN_PINS; ++pin_idx) {
    digitalWrite(left_driver_pins[pin_idx], DRIVE_SEQUENCE[drive_sequence_idx][pin_idx]);
  }
  left_stepper_previous_drive_us = curr_time_us;
}

void stop_left_stepper() {
  for (byte pin_idx = 0; pin_idx < STEPPER_DRIVER_NUM_IN_PINS; ++pin_idx) {
    digitalWrite(left_driver_pins[pin_idx], LOW);
  }
}

void drive_right_stepper(unsigned long curr_time_us) {
  byte drive_sequence_idx = mod(right_stepper_drive_idx, DRIVE_SEQUENCE_LEN);
  for (byte pin_idx = 0; pin_idx < STEPPER_DRIVER_NUM_IN_PINS; ++pin_idx) {
    digitalWrite(right_driver_pins[pin_idx], DRIVE_SEQUENCE[drive_sequence_idx][pin_idx]);
  }
  right_stepper_previous_drive_us = curr_time_us;
}

void stop_right_stepper() {
  for (byte pin_idx = 0; pin_idx < STEPPER_DRIVER_NUM_IN_PINS; ++pin_idx) {
    digitalWrite(right_driver_pins[pin_idx], LOW);
  }
}

void write_stepper_done(byte stepper_id, long limit_skipped_drives) {
    write_byte(stepper_id);
    floatbytes limit_skipped_steps;
    limit_skipped_steps.number = limit_skipped_drives / float(NUM_MICROSTEPS);
    write_float(limit_skipped_steps);
    SerialUART.flush();
  }

void setup() {
  // SerialUSB.begin(9600);
  SerialUART.begin(115200, SERIAL_8N1);

  // configure microstepping, which is 1,0,0 (half steps)
  pinMode(MICROSTEP_MS1_OUTPUT_PIN, OUTPUT);
  digitalWrite(MICROSTEP_MS1_OUTPUT_PIN, HIGH);
}

void loop() {

  unsigned long curr_time_us = micros();

  bool completed_left_stepper = false;
  bool completed_right_stepper = false;

  // process the current step command if everything is initialized
  if (left_stepper_inited && right_stepper_inited && limit_switches_inited) {

    /* check whether the cart is moving left, right, up, and down. this calculation is 
     * based on the following equations:
     * 
     * 1) calculate x and y steps from mm travels and steps/mm:
     * --------------------------------------------------------
     * x_steps = move_x_mm * steps_per_mm
     * y_steps = move_y_mm * steps_per_mm
     * 
     * 2) using (1), translate x and y steps to stepper motor steps:
     * -------------------------------------------------------------
     * left_stepper_steps (lss) = x_steps + y_steps
     * right_stepper_steps (rss) = x_steps - y_steps
     * 
     * 3) using (2), obtain x and y steps in terms of lss and rss:
     * -----------------------------------------------------------
     * a) x_steps = lss - y_steps
     * b) x_steps = rss + y_steps
     * c) y_steps = lss - x_steps
     * d) y_steps = x_steps - rss
     * 
     * 4) use (3a) and (3d) for x_steps:
     * ---------------------------------
     * x_steps = lss - x_steps + rss
     * 2 * x_steps = lss + rss
     * x_steps = (lss + rss) / 2
     * 
     * a) the gantry will move left when x_steps = (lss + rss) / 2 < 0, or when lss + rss < 0.
     * b) the gantry will move right when lss + rss > 0.
     * c) the gantry will remain at its x position when lss + rss = 0. in this case, either
     * lss and rss are both 0 (steppers do not move), or lss = -rss, in which case the
     * steppers move equally in opposite directions, which is up/down movement.
     * 
     * 5) use (3c) and (3b) for y_steps:
     * ---------------------------------
     * y_steps = lss - rss - y_steps
     * 2 * y_steps = lss - rss
     * y_steps = (lss - rss) / 2
     * 
     * a) the gantry will move down when y_steps = (lss - rss) / 2 < 0, or when lss - rss < 0.
     * b) the gantry will move up when lss - rss > 0.
     * c) the gantry will remain at its y position when lss - rss = 0. in this case, either
     * lss and rss are both 0 (steppers do not move), or lss = rss, in which case the
     * steppers move equally in the same direction, which is left/right movement.
     *
     * 6) note that we take half steps, which means we drive a motor twice per step. for
     * the purpose of checking left/right and down/up movement, it is equivalent to check
     * the number of left and right drives that remain.
    */
    bool moving_left = false;
    bool moving_right = false;
    bool moving_down = false;
    bool moving_up = false;

    // calculate left and right drives remaining
    long left_stepper_drives_remaining = left_stepper_drive_target - left_stepper_drive_idx;
    long right_stepper_drives_remaining = right_stepper_drive_target - right_stepper_drive_idx;

    // check condition (4a) for movement left
    long left_plus_right = left_stepper_drives_remaining + right_stepper_drives_remaining;
    if (left_plus_right < 0) {
      moving_left = true;
    }
    // check condition (4b) for movement right
    else if (left_plus_right > 0) {
      moving_right = true;
    }

    // check condition (5a) for movement down
    long left_minus_right = left_stepper_drives_remaining - right_stepper_drives_remaining;
    if (left_minus_right < 0) {
      moving_down = true;
    }
    // check condition (5b) for movement up
    else if (left_minus_right > 0) {
      moving_up = true;
    }

    /* check whether the gantry has hit a limit and must stop. this is indicated by pressing 
     * a limit switch in the direction of travel.
     */
    bool limited_travel = (
      (moving_left && !digitalRead(left_limit_switch_pin)) ||
      (moving_right && !digitalRead(right_limit_switch_pin)) ||
      (moving_down && !digitalRead(bottom_limit_switch_pin)) ||
      (moving_up && !digitalRead(top_limit_switch_pin))
    );

    // accelerate left stepper to target speed limited by maximum acceleration
    if (left_stepper_us_per_drive > left_stepper_us_per_drive_target) {
      unsigned long left_stepper_accelerate_us_per_drive = left_stepper_us_per_drive - left_stepper_us_per_drive_target;
      unsigned long us_since_acceleration = curr_time_us - left_stepper_previous_acceleration_us;
      unsigned long permissible_acceleration_us = (unsigned long)(us_since_acceleration * MAX_DRIVE_ACC_US_PER_DRIVE_PER_US);
      if (left_stepper_accelerate_us_per_drive > permissible_acceleration_us) {
        left_stepper_accelerate_us_per_drive = permissible_acceleration_us;
      }
      if (left_stepper_accelerate_us_per_drive > 0) {
        left_stepper_us_per_drive -= left_stepper_accelerate_us_per_drive;
        left_stepper_previous_acceleration_us = curr_time_us;
      }
    }
    // allow instantaneous deceleration to the target
    else {
      left_stepper_us_per_drive = left_stepper_us_per_drive_target;
    }

    // accelerate right stepper to target speed limited by maximum acceleration
    if (right_stepper_us_per_drive > right_stepper_us_per_drive_target) {
      unsigned long right_stepper_accelerate_us_per_drive = right_stepper_us_per_drive - right_stepper_us_per_drive_target;
      unsigned long us_since_acceleration = curr_time_us - right_stepper_previous_acceleration_us;
      unsigned long permissible_acceleration_us = (unsigned long)(us_since_acceleration * MAX_DRIVE_ACC_US_PER_DRIVE_PER_US);
      if (right_stepper_accelerate_us_per_drive > permissible_acceleration_us) {
        right_stepper_accelerate_us_per_drive = permissible_acceleration_us;
      }
      if (right_stepper_accelerate_us_per_drive > 0) {
        right_stepper_us_per_drive -= right_stepper_accelerate_us_per_drive;
        right_stepper_previous_acceleration_us = curr_time_us;
      }
    }
    // allow instantaneous deceleration to the target
    else {
      right_stepper_us_per_drive = right_stepper_us_per_drive_target;
    }

    /* drive the left stepper if needed to reach target and enough time has elapsed. modular arithmetic handles micros() overflow naturally.
     * if travel is limited, do not drive the stepper but record the skipped increment for reporting back to the caller. report back to the
     * caller when the drive index reaches the target.
    */
    if ((left_stepper_drive_idx != left_stepper_drive_target) && ((curr_time_us - left_stepper_previous_drive_us) >= left_stepper_us_per_drive)) {
      left_stepper_drive_idx += left_stepper_drive_increment;
      if (limited_travel) {
        left_stepper_limit_skipped_drives += left_stepper_drive_increment;
      }
      else {
        drive_left_stepper(curr_time_us);
      }
      if (left_stepper_drive_idx == left_stepper_drive_target) {
        write_stepper_done(LEFT_STEPPER_ID, left_stepper_limit_skipped_drives);
        completed_left_stepper = true;
      }
    }

    /* drive the right stepper if needed to reach target and enough time has elapsed. modular arithmetic handles micros() overflow naturally.
     * if travel is limited, do not drive the stepper but record the skipped increment for reporting back to the caller. report back to the
     * caller when the drive index reaches the target.
    */
    if ((right_stepper_drive_idx != right_stepper_drive_target) && ((curr_time_us - right_stepper_previous_drive_us) >= right_stepper_us_per_drive)) {
      right_stepper_drive_idx += right_stepper_drive_increment;
      if (limited_travel) {
        right_stepper_limit_skipped_drives += right_stepper_drive_increment;
      }
      else {
        drive_right_stepper(curr_time_us);
      }
      if (right_stepper_drive_idx == right_stepper_drive_target) {
        write_stepper_done(RIGHT_STEPPER_ID, right_stepper_limit_skipped_drives);
        completed_right_stepper = true;
      }
    }
  }

  // process a commands sent over the serial connection
  while (SerialUART.available()) {

    byte command_bytes[CMD_BYTES_LEN];
    SerialUART.readBytes(command_bytes, CMD_BYTES_LEN);
    byte command = command_bytes[0];
    byte component_id = command_bytes[1];

    // initialize a component
    if (command == CMD_INIT) {
      if (component_id == LEFT_STEPPER_ID) {

        byte args[CMD_INIT_STEPPER_ARGS_LEN];
        SerialUART.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);

        for (byte pin_idx = 0; pin_idx < STEPPER_DRIVER_NUM_IN_PINS; ++pin_idx) {
          byte pin = args[pin_idx];
          left_driver_pins[pin_idx] = pin;
          pinMode(pin, OUTPUT);
        }

        left_driver_dir_pin = bytes_to_int(args, STEPPER_DRIVER_NUM_IN_PINS);
        if (left_driver_dir_pin >= 0) {
          pinMode(left_driver_dir_pin, OUTPUT);
        }

        left_stepper_drive_idx = 0;
        left_stepper_drive_target = left_stepper_drive_idx;
        left_stepper_drive_increment = 0;
        left_stepper_us_per_drive = 0;
        left_stepper_us_per_drive_target = 0;
        drive_left_stepper(curr_time_us);
        left_stepper_inited = true;
        write_bool(true);
      }
      else if (component_id == RIGHT_STEPPER_ID) {

        byte args[CMD_INIT_STEPPER_ARGS_LEN];
        SerialUART.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);

        for (byte pin_idx = 0; pin_idx < STEPPER_DRIVER_NUM_IN_PINS; ++pin_idx) {
          byte pin = args[pin_idx];
          right_driver_pins[pin_idx] = pin;
          pinMode(pin, OUTPUT);
        }

        right_driver_dir_pin = bytes_to_int(args, STEPPER_DRIVER_NUM_IN_PINS);
        if (right_driver_dir_pin >= 0) {
          pinMode(right_driver_dir_pin, OUTPUT);
        }

        right_stepper_drive_idx = 0;
        right_stepper_drive_target = right_stepper_drive_idx;
        right_stepper_drive_increment = 0;
        right_stepper_us_per_drive = 0;
        right_stepper_us_per_drive_target = 0;
        drive_right_stepper(curr_time_us);
        right_stepper_inited = true;
        write_bool(true);
      }
      else if (component_id == LIMIT_SWITCHES_ID) {
        byte args[CMD_INIT_LIMIT_SWITCHES_ARGS_LEN];
        SerialUART.readBytes(args, CMD_INIT_LIMIT_SWITCHES_ARGS_LEN);
        left_limit_switch_pin = args[0];
        pinMode(left_limit_switch_pin, INPUT);
        right_limit_switch_pin = args[1];
        pinMode(right_limit_switch_pin, INPUT);
        bottom_limit_switch_pin = args[2];
        pinMode(bottom_limit_switch_pin, INPUT);
        top_limit_switch_pin = args[3];
        pinMode(top_limit_switch_pin, INPUT);
        limit_switches_inited = true;
        write_bool(true);
      }
    }
    else if (command == CMD_STEP) {

      byte args[CMD_STEP_ARGS_LEN];
      SerialUART.readBytes(args, CMD_STEP_ARGS_LEN);

      // left stepper:  calculate number of drives and microseconds per drive based on total ms to step. impose maximum drive rate.
      // skip the first two bytes sent by the stepper, which are the step command and stepper identifier.
      int left_stepper_num_drives = bytes_to_int(args, 2) * NUM_MICROSTEPS;
      unsigned long left_stepper_us_per_drive = 0;
      if (left_stepper_num_drives != 0) {
        unsigned int left_stepper_ms_to_step = bytes_to_unsigned_int(args, 4);
        left_stepper_us_per_drive = (unsigned long)((left_stepper_ms_to_step / float(abs(left_stepper_num_drives))) * 1000.0);
        if (left_stepper_us_per_drive < MIN_US_PER_DRIVE) {
          left_stepper_us_per_drive = MIN_US_PER_DRIVE;
        }
      }

      // right stepper:  calculate number of drives and microseconds per drive based on total ms to step. impose maximum drive rate.
      // skip the first two bytes sent by the stepper, which are the step command and stepper identifier.
      int right_stepper_num_drives = bytes_to_int(args, 8) * NUM_MICROSTEPS;
      unsigned long right_stepper_us_per_drive = 0;
      if (right_stepper_num_drives != 0) {
        unsigned int right_stepper_ms_to_step = bytes_to_unsigned_int(args, 10);
        right_stepper_us_per_drive = (unsigned long)((right_stepper_ms_to_step / float(abs(right_stepper_num_drives))) * 1000.0);
        if (right_stepper_us_per_drive < MIN_US_PER_DRIVE) {
          right_stepper_us_per_drive = MIN_US_PER_DRIVE;
        }
      }

      add_step(
        left_stepper_num_drives, 
        left_stepper_us_per_drive,
        right_stepper_num_drives, 
        right_stepper_us_per_drive
      );

    }
    else if (command == CMD_STOP) {
      if (component_id == LEFT_STEPPER_ID && left_stepper_inited) {
        stop_left_stepper();
        write_bool(true);
      }
      else if (component_id == RIGHT_STEPPER_ID && right_stepper_inited) {
        stop_right_stepper();
        write_bool(true);
      }
    }
  }

  // if both steppers are at their targets, then attempt to start the next step.
  if (left_stepper_inited && left_stepper_drive_idx == left_stepper_drive_target && right_stepper_inited && right_stepper_drive_idx == right_stepper_drive_target) {

    step* next_step = get_next_step();

    /* if the buffer has run out of steps, then the steppers won't drive further. we're going to lose any momentum 
     * that we have. set us/drive to zero, which will require the steppers to accelerate from their slowest speed
     * when they resume stepping.
    */
    if (next_step == nullptr) {
      left_stepper_us_per_drive = 0;
      left_stepper_us_per_drive_target = 0;
      right_stepper_us_per_drive = 0;
      right_stepper_us_per_drive_target = 0;
    }

    /* if the stepper just completed, then we'll set the drive values but wait the given drive delay to ensure proper 
     * stepper timing in relation to the step that just completed. if the stepper did not just complete, then begin 
     * driving the steppers immediately since we must have just received a new step command.
    */
    else {
      start_step(next_step, !completed_left_stepper, !completed_right_stepper, curr_time_us);
      delete next_step;
    }
  }
}