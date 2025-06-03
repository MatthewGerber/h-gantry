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
const unsigned int MIN_US_PER_DRIVE = 800;

// left stepper
const byte LEFT_STEPPER_ID = 0;
byte left_driver_pin_1;
byte left_driver_pin_2;
byte left_driver_pin_3;
byte left_driver_pin_4;
unsigned int left_stepper_drive_idx;
unsigned int left_stepper_drive_target;
int left_stepper_drive_increment;
bool left_stepper_inited = false;
unsigned long left_stepper_us_per_drive;
unsigned long left_stepper_previous_drive_us;

// right stepper
const byte RIGHT_STEPPER_ID = 1;
byte right_driver_pin_1;
byte right_driver_pin_2;
byte right_driver_pin_3;
byte right_driver_pin_4;
unsigned int right_stepper_drive_idx;
unsigned int right_stepper_drive_target;
int right_stepper_drive_increment;
bool right_stepper_inited = false;
unsigned long right_stepper_us_per_drive;
unsigned long right_stepper_previous_drive_us;

// top-level command:  command id and component id
const size_t CMD_BYTES_LEN = 2;

// top-level command:  init component
const byte CMD_INIT = 1;
const size_t CMD_INIT_STEPPER_ARGS_LEN = 4;

// top-level command:  step
const byte CMD_STEP = 2;
const byte CMD_STEP_ARGS_LEN = 5;

// switch between usb serial (SerialUSB; to write to arduino IDE serial monitor) and tx/rx serial (_UART1_; to write to raspberry pi)
#define SerialX _UART1_

void setup() {

  SerialX.begin(115200, SERIAL_8N1);

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
  SerialX.write(bytes, FLOAT_BYTES_LEN);
}

void write_float(floatbytes f) {
  SerialX.write(f.bytes, FLOAT_BYTES_LEN);
}

void write_bool(bool value) {
  SerialX.write(value);
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

void loop() {

  // drive the left stepper if needed to reach target
  if (left_stepper_inited && left_stepper_drive_idx != left_stepper_drive_target) {

    // check if enough time has elapsed. modular arithmetic handles overflow naturally.
    unsigned long curr_micros = micros();
    unsigned long elapsed_micros = curr_micros - left_stepper_previous_drive_us;
    if (elapsed_micros >= left_stepper_us_per_drive) {

      // output current drive sequence
      byte drive_sequence_idx = left_stepper_drive_idx % DRIVE_SEQUENCE_LEN;
      digitalWrite(left_driver_pin_1, DRIVE_SEQUENCE[drive_sequence_idx][0]);
      digitalWrite(left_driver_pin_2, DRIVE_SEQUENCE[drive_sequence_idx][1]);
      digitalWrite(left_driver_pin_3, DRIVE_SEQUENCE[drive_sequence_idx][2]);
      digitalWrite(left_driver_pin_4, DRIVE_SEQUENCE[drive_sequence_idx][3]);
      left_stepper_previous_drive_us = curr_micros;

      // increment the drive index. turn driver off if we've reached the target.
      left_stepper_drive_idx = (left_stepper_drive_idx + left_stepper_drive_increment);
      if (left_stepper_drive_idx == left_stepper_drive_target) {
        digitalWrite(left_driver_pin_1, 0);
        digitalWrite(left_driver_pin_2, 0);
        digitalWrite(left_driver_pin_3, 0);
        digitalWrite(left_driver_pin_4, 0);
        SerialX.println(String(LEFT_STEPPER_ID) + "0");
      }
    }
  }

  // drive the right stepper if needed to reach target
  if (right_stepper_inited && right_stepper_drive_idx != right_stepper_drive_target) {

    // check if enough time has elapsed. modular arithmetic handles overflow naturally.
    unsigned long curr_micros = micros();
    unsigned long elapsed_micros = curr_micros - right_stepper_previous_drive_us;
    if (elapsed_micros >= right_stepper_us_per_drive) {

      // output current drive sequence
      byte drive_sequence_idx = right_stepper_drive_idx % DRIVE_SEQUENCE_LEN;
      digitalWrite(right_driver_pin_1, DRIVE_SEQUENCE[drive_sequence_idx][0]);
      digitalWrite(right_driver_pin_2, DRIVE_SEQUENCE[drive_sequence_idx][1]);
      digitalWrite(right_driver_pin_3, DRIVE_SEQUENCE[drive_sequence_idx][2]);
      digitalWrite(right_driver_pin_4, DRIVE_SEQUENCE[drive_sequence_idx][3]);
      right_stepper_previous_drive_us = curr_micros;

      // increment the drive index. turn driver off if we've reached the target.
      right_stepper_drive_idx = (right_stepper_drive_idx + right_stepper_drive_increment);
      if (right_stepper_drive_idx == right_stepper_drive_target) {
        digitalWrite(right_driver_pin_1, 0);
        digitalWrite(right_driver_pin_2, 0);
        digitalWrite(right_driver_pin_3, 0);
        digitalWrite(right_driver_pin_4, 0);
        SerialX.println(String(RIGHT_STEPPER_ID) + "0");
      }
    }
  }

  // process a command sent over the serial connection
  if (SerialX.available()) {

    byte command_bytes[CMD_BYTES_LEN];
    SerialX.readBytes(command_bytes, CMD_BYTES_LEN);
    byte command = command_bytes[0];
    byte component_id = command_bytes[1];

    // initialize a component
    if (command == CMD_INIT) {
      if (component_id == LEFT_STEPPER_ID) {
        Serial.print("Initializing left stepper...");
        byte args[CMD_INIT_STEPPER_ARGS_LEN];
        SerialX.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);
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
        left_stepper_inited = true;
        left_stepper_us_per_drive = 0;
        left_stepper_previous_drive_us = 0;
        write_bool(true);
        Serial.println("done.");
      }
      else if (component_id == RIGHT_STEPPER_ID) {
        Serial.print("Initializing right stepper...");
        byte args[CMD_INIT_STEPPER_ARGS_LEN];
        SerialX.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);
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
        right_stepper_inited = true;
        right_stepper_us_per_drive = 0;
        right_stepper_previous_drive_us = 0;
        write_bool(true);
        Serial.println("done.");
      }
    }
    else if (command == CMD_STEP) {
      if (component_id == LEFT_STEPPER_ID) {
        
        byte args[CMD_STEP_ARGS_LEN];
        SerialX.readBytes(args, CMD_STEP_ARGS_LEN);

        // calculate numbers of drives from steps and increment direction. increment comes in as 0 (decrement) or 1 (increment).
        unsigned int left_stepper_num_steps = bytes_to_unsigned_int(args, 0);
        unsigned int left_stepper_num_drives = left_stepper_num_steps * STEPPER_DRIVES_PER_STEP;
        left_stepper_drive_increment = args[2];
        if (left_stepper_drive_increment == 0) {
          left_stepper_drive_increment = -1;
        }
        left_stepper_drive_target = left_stepper_drive_idx + (left_stepper_num_drives * left_stepper_drive_increment);

        // set microseconds per drive based on ms per drive
        unsigned int left_stepper_ms_to_step = bytes_to_unsigned_int(args, 3);
        left_stepper_us_per_drive = (unsigned long)((left_stepper_ms_to_step / float(left_stepper_num_drives)) * 1000.0);
        if (left_stepper_us_per_drive < MIN_US_PER_DRIVE) {
          left_stepper_us_per_drive = MIN_US_PER_DRIVE;
        }
        left_stepper_previous_drive_us = micros() - left_stepper_us_per_drive;  // drive immediately on next loop

        Serial.println("Stepping left stepper " + String(left_stepper_num_steps) + " steps in " + String(left_stepper_num_drives) + " drives @ " + String(left_stepper_us_per_drive) + " us/drive.");
      }
      else if (component_id == RIGHT_STEPPER_ID) {
        byte args[CMD_STEP_ARGS_LEN];
        SerialX.readBytes(args, CMD_STEP_ARGS_LEN);

        // calculate numbers of drives from steps and increment direction. increment comes in as 0 (decrement) or 1 (increment).
        unsigned int right_stepper_num_steps = bytes_to_unsigned_int(args, 0);
        unsigned int right_stepper_num_drives = right_stepper_num_steps * STEPPER_DRIVES_PER_STEP;
        right_stepper_drive_increment = args[2];
        if (right_stepper_drive_increment == 0) {
          right_stepper_drive_increment = -1;
        }
        right_stepper_drive_target = right_stepper_drive_idx + (right_stepper_num_drives * right_stepper_drive_increment);

        // set microseconds per drive based on ms per drive
        unsigned int right_stepper_ms_to_step = bytes_to_unsigned_int(args, 3);
        right_stepper_us_per_drive = (unsigned long)((right_stepper_ms_to_step / float(right_stepper_num_drives)) * 1000.0);
        if (right_stepper_us_per_drive < MIN_US_PER_DRIVE) {
          right_stepper_us_per_drive = MIN_US_PER_DRIVE;
        }
        right_stepper_previous_drive_us = micros() - right_stepper_us_per_drive;  // drive immediately on next loop

        Serial.println("Stepping right stepper " + String(right_stepper_num_steps) + " steps in " + String(right_stepper_num_drives) + " drives @ " + String(right_stepper_us_per_drive) + " us/drive.");
      }
    }
  }
}