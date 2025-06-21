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
unsigned long left_stepper_limit_skipped_increments;

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
unsigned long right_stepper_limit_skipped_increments;

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

// switch between usb serial (SerialUSB; to write to arduino IDE serial monitor) and tx/rx serial (_UART1_; to write to raspberry pi)
#define SerialUART _UART1_

void setup() {
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

void set_float_bytes(byte dest[], byte src[], size_t src_start_idx) {
  dest[0] = src[src_start_idx];
  dest[1] = src[src_start_idx + 1];
  dest[2] = src[src_start_idx + 2];
  dest[3] = src[src_start_idx + 3];
}

void test_step() {

  if (!left_stepper_inited) {
    left_driver_pin_1 = 5;
    pinMode(left_driver_pin_1, OUTPUT);
    left_driver_pin_2 = 6;
    pinMode(left_driver_pin_2, OUTPUT);
    left_driver_pin_3 = 7;
    pinMode(left_driver_pin_3, OUTPUT);
    left_driver_pin_4 = 8;
    pinMode(left_driver_pin_4, OUTPUT);
    left_stepper_drive_idx = 0;
    left_stepper_drive_target = left_stepper_drive_idx;
    left_stepper_drive_increment = 0;
    left_stepper_inited = true;

    unsigned int left_stepper_num_steps = 1000;
    unsigned int left_stepper_num_drives = left_stepper_num_steps * STEPPER_DRIVES_PER_STEP;
    left_stepper_drive_target = left_stepper_drive_idx + left_stepper_num_drives;
    left_stepper_drive_increment = 1;

    // set microseconds per drive
    unsigned int left_stepper_ms_to_step = 2000;
    left_stepper_us_per_drive = (unsigned long)((left_stepper_ms_to_step / float(left_stepper_num_drives)) * 1000.0);
    if (left_stepper_us_per_drive < MIN_US_PER_DRIVE) {
      left_stepper_us_per_drive = MIN_US_PER_DRIVE;
    }
    left_stepper_previous_drive_us = micros() - left_stepper_us_per_drive;  // drive immediately on next loop
  }
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

void drive_right_stepper() {
  byte drive_sequence_idx = mod(right_stepper_drive_idx, DRIVE_SEQUENCE_LEN);
  digitalWrite(right_driver_pin_1, DRIVE_SEQUENCE[drive_sequence_idx][0]);
  digitalWrite(right_driver_pin_2, DRIVE_SEQUENCE[drive_sequence_idx][1]);
  digitalWrite(right_driver_pin_3, DRIVE_SEQUENCE[drive_sequence_idx][2]);
  digitalWrite(right_driver_pin_4, DRIVE_SEQUENCE[drive_sequence_idx][3]);
  right_stepper_previous_drive_us = micros();
}

void loop() {

  /* check whether the cart is moving left, right, up, and down. this calculation is based on the 
   * following equations.
   * 
   * ...
   * 
   */ 
  bool moving_left = false;
  bool moving_right = false;
  bool moving_up = false;
  bool moving_down = false;
  if (left_stepper_inited && right_stepper_inited) {
    long left_stepper_drives_remaining = left_stepper_drive_target - left_stepper_drive_idx;
    long right_stepper_drives_remaining = right_stepper_drive_target - right_stepper_drive_idx;

    long left_plus_right = left_stepper_drives_remaining + right_stepper_drives_remaining;
    if (left_plus_right < 0) {
      moving_left = true;
    } else if (left_plus_right > 0) {
      moving_right = true;
    }

    long left_minus_right = left_stepper_drives_remaining - right_stepper_drives_remaining;
    if (left_minus_right < 0) {
      moving_down = true;
    } else if (left_minus_right > 0) {
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
    if (limited_travel) {
      left_stepper_limit_skipped_increments += left_stepper_drive_increment;
    }
    else {
      drive_left_stepper();
    }
    left_stepper_drive_idx += left_stepper_drive_increment;
    if (left_stepper_drive_idx == left_stepper_drive_target) {
      unsigned long left_stepper_limit_skipped_steps = left_stepper_limit_skipped_increments / STEPPER_DRIVES_PER_STEP;
      SerialUART.println(String(LEFT_STEPPER_ID) + "," + String(left_stepper_limit_skipped_steps));
    }
  }

  /* drive the right stepper if needed to reach target and enough time has elapsed. modular arithmetic handles micros() overflow naturally.
   * if travel is limited, do not drive the stepper but record the skipped increment for reporting back to the caller. report back to the
   * caller when the drive index reaches the target.
   */
  if (right_stepper_inited && (right_stepper_drive_idx != right_stepper_drive_target) && ((micros() - right_stepper_previous_drive_us) >= right_stepper_us_per_drive)) {
    if (limited_travel) {
      right_stepper_limit_skipped_increments += right_stepper_drive_increment;
    }
    else {
      drive_right_stepper();
    }
    right_stepper_drive_idx += right_stepper_drive_increment;
    if (right_stepper_drive_idx == right_stepper_drive_target) {
      unsigned long right_stepper_limit_skipped_steps = right_stepper_limit_skipped_increments / STEPPER_DRIVES_PER_STEP;
      SerialUART.println(String(RIGHT_STEPPER_ID) + "," + String(right_stepper_limit_skipped_steps));
    }
  }

  // process a commands sent over the serial connection
  if (SerialUART.available()) {

    // read number of commands to process
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
        if (component_id == LEFT_STEPPER_ID) {
          
          byte args[CMD_STEP_ARGS_LEN];
          SerialUART.readBytes(args, CMD_STEP_ARGS_LEN);

          // calculate numbers of drives from steps and increment direction. increment comes in as 0 (decrement) or 1 (increment).
          unsigned int left_stepper_num_steps = bytes_to_unsigned_int(args, 0);
          unsigned int left_stepper_num_drives = left_stepper_num_steps * STEPPER_DRIVES_PER_STEP;
          left_stepper_drive_increment = args[2];
          if (left_stepper_drive_increment == 0) {
            left_stepper_drive_increment = -1;
          }
          left_stepper_drive_idx = mod(left_stepper_drive_idx + left_stepper_drive_increment, DRIVE_SEQUENCE_LEN);  // mod initial drive idx to avoid overflow
          left_stepper_drive_target = left_stepper_drive_idx + ((left_stepper_num_drives - 1) * left_stepper_drive_increment);
          left_stepper_limit_skipped_increments = 0;

          // set microseconds per drive based on ms per drive
          unsigned int left_stepper_ms_to_step = bytes_to_unsigned_int(args, 3);
          left_stepper_us_per_drive = (unsigned long)((left_stepper_ms_to_step / float(left_stepper_num_drives)) * 1000.0);
          if (left_stepper_us_per_drive < MIN_US_PER_DRIVE) {
            left_stepper_us_per_drive = MIN_US_PER_DRIVE;
          }
          left_stepper_previous_drive_us = micros() - left_stepper_us_per_drive;  // drive immediately on next loop
        }
        else if (component_id == RIGHT_STEPPER_ID) {

          byte args[CMD_STEP_ARGS_LEN];
          SerialUART.readBytes(args, CMD_STEP_ARGS_LEN);

          // calculate numbers of drives from steps and increment direction. increment comes in as 0 (decrement) or 1 (increment).
          unsigned int right_stepper_num_steps = bytes_to_unsigned_int(args, 0);
          unsigned int right_stepper_num_drives = right_stepper_num_steps * STEPPER_DRIVES_PER_STEP;
          right_stepper_drive_increment = args[2];
          if (right_stepper_drive_increment == 0) {
            right_stepper_drive_increment = -1;
          }
          right_stepper_drive_idx = mod(right_stepper_drive_idx + right_stepper_drive_increment, DRIVE_SEQUENCE_LEN);  // mod initial drive idx to avoid overflow
          right_stepper_drive_target = right_stepper_drive_idx + ((right_stepper_num_drives - 1) * right_stepper_drive_increment);
          right_stepper_limit_skipped_increments = 0;

          // set microseconds per drive based on ms per drive
          unsigned int right_stepper_ms_to_step = bytes_to_unsigned_int(args, 3);
          right_stepper_us_per_drive = (unsigned long)((right_stepper_ms_to_step / float(right_stepper_num_drives)) * 1000.0);
          if (right_stepper_us_per_drive < MIN_US_PER_DRIVE) {
            right_stepper_us_per_drive = MIN_US_PER_DRIVE;
          }
          right_stepper_previous_drive_us = micros() - right_stepper_us_per_drive;  // drive immediately on next loop
        }
      }
    }
  }
}