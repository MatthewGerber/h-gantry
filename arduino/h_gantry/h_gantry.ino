const size_t FLOAT_BYTES_LEN = 4;

// structure that gives simultaneous access to floating-point numbers and their underlying bytes.
typedef union {
  float number;
  byte bytes[FLOAT_BYTES_LEN];
} floatbytes;

// stepper driver configuration
const byte STEPPER_NUM_PHASES = 4;
const byte STEPPER_DRIVES_PER_STEP = 2;
const byte DRIVE_SEQUENCE_LEN = STEPPER_NUM_PHASES * STEPPER_DRIVES_PER_STEP;
const byte DRIVE_SEQUENCE[][DRIVE_SEQUENCE_LEN] = {
  { HIGH, LOW, LOW, LOW },
  { HIGH, HIGH, LOW, LOW },
  { LOW, HIGH, LOW, LOW },
  { LOW, HIGH, HIGH, LOW },
  { LOW, LOW, HIGH, LOW },
  { LOW, LOW, HIGH, HIGH },
  { LOW, LOW, LOW, HIGH },
  { HIGH, LOW, LOW, HIGH }
};
const unsigned int MIN_US_PER_DRIVE = 1000;

// left stepper
const byte LEFT_STEPPER_ID = 0;
byte left_driver_pin_1;
byte left_driver_pin_2;
byte left_driver_pin_3;
byte left_driver_pin_4;
long left_stepper_drive_idx;
long left_stepper_drive_target;
int left_stepper_drive_increment;
bool left_stepper_inited = false;
unsigned long left_stepper_us_per_drive;
unsigned long left_stepper_previous_drive_us;
long left_stepper_limit_skipped_drives;

// right stepper
const byte RIGHT_STEPPER_ID = 1;
byte right_driver_pin_1;
byte right_driver_pin_2;
byte right_driver_pin_3;
byte right_driver_pin_4;
long right_stepper_drive_idx;
long right_stepper_drive_target;
int right_stepper_drive_increment;
bool right_stepper_inited = false;
unsigned long right_stepper_us_per_drive;
unsigned long right_stepper_previous_drive_us;
long right_stepper_limit_skipped_drives;

// limit switches
const byte LIMIT_SWITCHES_ID = 2;
byte left_limit_switch_pin;
byte right_limit_switch_pin;
byte bottom_limit_switch_pin;
byte top_limit_switch_pin;
bool limit_switches_inited = false;

// top-level command:  command id and component id
const size_t NUM_COMMANDS_BYTES_LEN = 1;
const size_t CMD_BYTES_LEN = 2;

// top-level command:  init component
const byte CMD_INIT = 1;
const size_t CMD_INIT_STEPPER_ARGS_LEN = 4;
const size_t CMD_INIT_LIMIT_SWITCHES_ARGS_LEN = 4;

// top-level command:  step
const byte CMD_STEP = 2;
const byte CMD_STEP_ARGS_LEN = 5;

// top-level command:  stop
const byte CMD_STOP = 3;

// switch between usb serial (SerialUSB; to write to arduino IDE serial monitor) and tx/rx serial (_UART1_; to write to raspberry pi)
#define SerialUART _UART1_

void setup() {
  // SerialUSB.begin(9600);
  SerialUART.begin(115200, SERIAL_8N1);
}

long bytes_to_long(byte bytes[]) {
  long value = 0;
  value += ((long)bytes[0]) << 24;
  value += ((long)bytes[1]) << 16;
  value += ((long)bytes[2]) << 8;
  value += ((long)bytes[3]);
  return value;
}

unsigned int bytes_to_unsigned_int(byte bytes[], size_t start_idx) {
  unsigned int value = 0;
  value += ((unsigned int)bytes[start_idx]) << 8;
  value += ((unsigned int)bytes[start_idx + 1]);
  return value;
}

void long_to_bytes(long value, byte bytes[]) {
  bytes[3] = (byte)value;
  bytes[2] = (byte)(value >> 8);
  bytes[1] = (byte)(value >> 16);
  bytes[0] = (byte)(value >> 24);
}

void write_long(long value) {
  byte bytes[FLOAT_BYTES_LEN];
  long_to_bytes(value, bytes);
  SerialUART.write(bytes, FLOAT_BYTES_LEN);
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

void set_float_bytes(byte dest[], byte src[], size_t src_start_idx) {
  dest[0] = src[src_start_idx];
  dest[1] = src[src_start_idx + 1];
  dest[2] = src[src_start_idx + 2];
  dest[3] = src[src_start_idx + 3];
}

byte mod(long x, byte y){
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

void drive_left_stepper() {
  byte drive_sequence_idx = mod(left_stepper_drive_idx, DRIVE_SEQUENCE_LEN);
  digitalWrite(left_driver_pin_1, DRIVE_SEQUENCE[drive_sequence_idx][0]);
  digitalWrite(left_driver_pin_2, DRIVE_SEQUENCE[drive_sequence_idx][1]);
  digitalWrite(left_driver_pin_3, DRIVE_SEQUENCE[drive_sequence_idx][2]);
  digitalWrite(left_driver_pin_4, DRIVE_SEQUENCE[drive_sequence_idx][3]);
  left_stepper_previous_drive_us = micros();
}

void stop_left_stepper() {
  digitalWrite(left_driver_pin_1, LOW);
  digitalWrite(left_driver_pin_2, LOW);
  digitalWrite(left_driver_pin_3, LOW);
  digitalWrite(left_driver_pin_4, LOW);
}

void drive_right_stepper() {
  byte drive_sequence_idx = mod(right_stepper_drive_idx, DRIVE_SEQUENCE_LEN);
  digitalWrite(right_driver_pin_1, DRIVE_SEQUENCE[drive_sequence_idx][0]);
  digitalWrite(right_driver_pin_2, DRIVE_SEQUENCE[drive_sequence_idx][1]);
  digitalWrite(right_driver_pin_3, DRIVE_SEQUENCE[drive_sequence_idx][2]);
  digitalWrite(right_driver_pin_4, DRIVE_SEQUENCE[drive_sequence_idx][3]);
  right_stepper_previous_drive_us = micros();
}

void stop_right_stepper() {
  digitalWrite(right_driver_pin_1, LOW);
  digitalWrite(right_driver_pin_2, LOW);
  digitalWrite(right_driver_pin_3, LOW);
  digitalWrite(right_driver_pin_4, LOW);
}

void loop() {

  /* check whether the cart is moving left, right, up, and down. this calculation is based on the 
   * following equations:
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
  if (left_stepper_inited && right_stepper_inited) {

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
  }

  // check whether the gantry has hit a limit and must stop. this is indicated by pressing a limit switch in the direction of travel.
  bool limited_travel = false;
  if (limit_switches_inited) {
    bool left_limit_switch_pressed = !digitalRead(left_limit_switch_pin);
    bool right_limit_switch_pressed = !digitalRead(right_limit_switch_pin);
    bool bottom_limit_switch_pressed = !digitalRead(bottom_limit_switch_pin);
    bool top_limit_switch_pressed = !digitalRead(top_limit_switch_pin);
    if (
      (moving_left && left_limit_switch_pressed) || 
      (moving_right && right_limit_switch_pressed) ||
      (moving_down && bottom_limit_switch_pressed) ||
      (moving_up && top_limit_switch_pressed)
    ) {
      limited_travel = true;
    }
  }

  /* drive the left stepper if needed to reach target and enough time has elapsed. modular arithmetic handles micros() overflow naturally.
   * if travel is limited, do not drive the stepper but record the skipped increment for reporting back to the caller. report back to the
   * caller when the drive index reaches the target.
   */
  if (left_stepper_inited && (left_stepper_drive_idx != left_stepper_drive_target) && ((micros() - left_stepper_previous_drive_us) >= left_stepper_us_per_drive)) {
    left_stepper_drive_idx += left_stepper_drive_increment;
    if (limited_travel) {
      left_stepper_limit_skipped_drives += left_stepper_drive_increment;
    }
    else {
      drive_left_stepper();
    }
    if (left_stepper_drive_idx == left_stepper_drive_target) {
      write_byte(LEFT_STEPPER_ID);
      floatbytes left_stepper_limit_skipped_steps;
      left_stepper_limit_skipped_steps.number = left_stepper_limit_skipped_drives / float(STEPPER_DRIVES_PER_STEP);
      write_float(left_stepper_limit_skipped_steps);
      SerialUART.flush();
    }
  }

  /* drive the right stepper if needed to reach target and enough time has elapsed. modular arithmetic handles micros() overflow naturally.
   * if travel is limited, do not drive the stepper but record the skipped increment for reporting back to the caller. report back to the
   * caller when the drive index reaches the target.
   */
  if (right_stepper_inited && (right_stepper_drive_idx != right_stepper_drive_target) && ((micros() - right_stepper_previous_drive_us) >= right_stepper_us_per_drive)) {
    right_stepper_drive_idx += right_stepper_drive_increment;
    if (limited_travel) {
      right_stepper_limit_skipped_drives += right_stepper_drive_increment;
    }
    else {
      drive_right_stepper();
    }
    if (right_stepper_drive_idx == right_stepper_drive_target) {
      write_byte(RIGHT_STEPPER_ID);
      floatbytes right_stepper_limit_skipped_steps;
      right_stepper_limit_skipped_steps.number = right_stepper_limit_skipped_drives / float(STEPPER_DRIVES_PER_STEP);
      write_float(right_stepper_limit_skipped_steps);
      SerialUART.flush();
    }
  }

  // process a commands sent over the serial connection
  if (SerialUART.available()) {

    // read number of commands to process. we process multiple commands per loop because the steppers and their
    // drive indices need to remain synchronized when evaluating left/right and down/up movement above. reading
    // a single step command per loop results in one stepper advancing before the other, throwing off equality 
    // checks.
    byte num_commands_bytes[NUM_COMMANDS_BYTES_LEN];
    SerialUART.readBytes(num_commands_bytes, NUM_COMMANDS_BYTES_LEN);
    byte num_commands = num_commands_bytes[0];

    // process each command
    for (int i = 0; i < num_commands; ++i) {

      byte command_bytes[CMD_BYTES_LEN];
      SerialUART.readBytes(command_bytes, CMD_BYTES_LEN);
      byte command = command_bytes[0];
      byte component_id = command_bytes[1];

      // initialize a component
      if (command == CMD_INIT) {
        if (component_id == LEFT_STEPPER_ID) {
          byte args[CMD_INIT_STEPPER_ARGS_LEN];
          SerialUART.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);
          left_driver_pin_1 = args[0];
          pinMode(left_driver_pin_1, OUTPUT);
          left_driver_pin_2 = args[1];
          pinMode(left_driver_pin_2, OUTPUT);
          left_driver_pin_3 = args[2];
          pinMode(left_driver_pin_3, OUTPUT);
          left_driver_pin_4 = args[3];
          pinMode(left_driver_pin_4, OUTPUT);
          left_stepper_drive_idx = 0;
          left_stepper_drive_target = left_stepper_drive_idx;
          left_stepper_drive_increment = 0;
          left_stepper_us_per_drive = 0;
          left_stepper_previous_drive_us = 0;
          drive_left_stepper();
          left_stepper_inited = true;
          write_bool(true);
        }
        else if (component_id == RIGHT_STEPPER_ID) {
          byte args[CMD_INIT_STEPPER_ARGS_LEN];
          SerialUART.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);
          right_driver_pin_1 = args[0];
          pinMode(right_driver_pin_1, OUTPUT);
          right_driver_pin_2 = args[1];
          pinMode(right_driver_pin_2, OUTPUT);
          right_driver_pin_3 = args[2];
          pinMode(right_driver_pin_3, OUTPUT);
          right_driver_pin_4 = args[3];
          pinMode(right_driver_pin_4, OUTPUT);
          right_stepper_drive_idx = 0;
          right_stepper_drive_target = right_stepper_drive_idx;
          right_stepper_drive_increment = 0;
          right_stepper_us_per_drive = 0;
          right_stepper_previous_drive_us = 0;
          drive_right_stepper();
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
        if (component_id == LEFT_STEPPER_ID && left_stepper_inited) {
          
          byte args[CMD_STEP_ARGS_LEN];
          SerialUART.readBytes(args, CMD_STEP_ARGS_LEN);

          // calculate number of drives from steps and increment direction. increment comes in as 0 (decrement) or 1 (increment).
          unsigned int left_stepper_num_steps = bytes_to_unsigned_int(args, 0);
          unsigned int left_stepper_num_drives = left_stepper_num_steps * STEPPER_DRIVES_PER_STEP;
          left_stepper_drive_increment = args[2];
          if (left_stepper_drive_increment == 0) {
            left_stepper_drive_increment = -1;
          }
          left_stepper_drive_idx = mod(left_stepper_drive_idx, DRIVE_SEQUENCE_LEN);  // mod initial drive idx to avoid overflow
          left_stepper_drive_target = left_stepper_drive_idx + (left_stepper_num_drives * left_stepper_drive_increment);
          left_stepper_limit_skipped_drives = 0;

          // set microseconds per drive based on total ms to step. impose maximum drive rate.
          unsigned int left_stepper_ms_to_step = bytes_to_unsigned_int(args, 3);
          left_stepper_us_per_drive = (unsigned long)((left_stepper_ms_to_step / float(left_stepper_num_drives)) * 1000.0);
          if (left_stepper_us_per_drive < MIN_US_PER_DRIVE) {
            left_stepper_us_per_drive = MIN_US_PER_DRIVE;
          }
          left_stepper_previous_drive_us = micros() - left_stepper_us_per_drive;  // drive immediately on next loop
        }
        else if (component_id == RIGHT_STEPPER_ID && right_stepper_inited) {

          byte args[CMD_STEP_ARGS_LEN];
          SerialUART.readBytes(args, CMD_STEP_ARGS_LEN);

          // calculate number of drives from steps and increment direction. increment comes in as 0 (decrement) or 1 (increment).
          unsigned int right_stepper_num_steps = bytes_to_unsigned_int(args, 0);
          unsigned int right_stepper_num_drives = right_stepper_num_steps * STEPPER_DRIVES_PER_STEP;
          right_stepper_drive_increment = args[2];
          if (right_stepper_drive_increment == 0) {
            right_stepper_drive_increment = -1;
          }
          right_stepper_drive_idx = mod(right_stepper_drive_idx, DRIVE_SEQUENCE_LEN);  // mod initial drive idx to avoid overflow
          right_stepper_drive_target = right_stepper_drive_idx + (right_stepper_num_drives * right_stepper_drive_increment);
          right_stepper_limit_skipped_drives = 0;

          // set microseconds per drive based on total ms to step. impose maximum drive rate.
          unsigned int right_stepper_ms_to_step = bytes_to_unsigned_int(args, 3);
          right_stepper_us_per_drive = (unsigned long)((right_stepper_ms_to_step / float(right_stepper_num_drives)) * 1000.0);
          if (right_stepper_us_per_drive < MIN_US_PER_DRIVE) {
            right_stepper_us_per_drive = MIN_US_PER_DRIVE;
          }
          right_stepper_previous_drive_us = micros() - right_stepper_us_per_drive;  // drive immediately on next loop
        }
      }
      if (command == CMD_STOP) {
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
  }
}