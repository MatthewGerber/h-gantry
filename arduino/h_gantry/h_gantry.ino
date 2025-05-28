const size_t FLOAT_BYTES_LEN = 4;

// structure that gives simultaneous access to floating-point numbers and their underlying bytes.
typedef union {
  float number;
  byte bytes[FLOAT_BYTES_LEN];
} floatbytes;

const byte DRIVE_SEQUENCE_LEN = 8;
/*byte drive_sequence[][DRIVE_SEQUENCE_LEN] = {
  { HIGH, LOW, LOW, LOW },
  { HIGH, HIGH, LOW, LOW },
  { LOW, HIGH, LOW, LOW },
  { LOW, HIGH, HIGH, LOW },
  { LOW, LOW, HIGH, LOW },
  { LOW, LOW, HIGH, HIGH },
  { LOW, LOW, LOW, HIGH },
  { HIGH, LOW, LOW, HIGH }
};*/

// left stepper
const byte LEFT_STEPPER_ID = 0;
byte left_driver_pin_1;
byte left_driver_pin_2;
byte left_driver_pin_3;
byte left_driver_pin_4;
byte left_stepper_drive_idx;
byte left_stepper_drive_target;
byte left_stepper_drive_increment;
unsigned int left_stepper_ms_per_drive;
unsigned long left_stepper_next_drive_ms;
bool left_stepper_inited = false;

// right stepper
const byte RIGHT_STEPPER_ID = 1;

// top-level command:  command id and component id
const size_t CMD_BYTES_LEN = 2;

// init components
const byte CMD_INIT = 1;
const size_t CMD_INIT_STEPPER_ARGS_LEN = 4;

// step
const byte CMD_STEP = 2;
const byte CMD_STEP_ARGS_LEN = 5;

void setup() {

  //Serial.begin(115200, SERIAL_8N1);

  Serial.begin(9600);

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
  Serial.write(bytes, FLOAT_BYTES_LEN);
}

void write_float(floatbytes f) {
  Serial.write(f.bytes, FLOAT_BYTES_LEN);
}

void write_bool(bool value) {
  Serial.write(value);
}

void set_float_bytes(byte dest[], byte src[], size_t src_start_idx) {
  dest[0] = src[src_start_idx];
  dest[1] = src[src_start_idx + 1];
  dest[2] = src[src_start_idx + 2];
  dest[3] = src[src_start_idx + 3];
}

void loop() {

  Serial.println("hello");

  if (left_stepper_inited) {
    Serial.println("Inited");
  }
  else {
    Serial.println("Not inited");
  }

  // drive the left stepper if needed and sufficient time has elapsed
  /*if (left_stepper_inited) {
    Serial.println("Driving");
    if (left_stepper_drive_idx != left_stepper_drive_target && millis() >= left_stepper_next_drive_ms) {
      digitalWrite(left_driver_pin_1, drive_sequence[left_stepper_drive_idx][0]);
      digitalWrite(left_driver_pin_2, drive_sequence[left_stepper_drive_idx][1]);
      digitalWrite(left_driver_pin_3, drive_sequence[left_stepper_drive_idx][2]);
      digitalWrite(left_driver_pin_4, drive_sequence[left_stepper_drive_idx][3]);
      left_stepper_drive_idx = (left_stepper_drive_idx + left_stepper_drive_increment) % DRIVE_SEQUENCE_LEN;
      left_stepper_next_drive_ms = millis() + left_stepper_ms_per_drive;

      if (left_stepper_drive_idx == left_stepper_drive_target) {
        write_bool(false);
      }
    }
  }*/

  Serial.println("also here");

  // process a command sent over the serial connection
  /*if (false) {  //Serial.available()) {

    Serial.println("serial available");

    byte command_bytes[CMD_BYTES_LEN];
    Serial.readBytes(command_bytes, CMD_BYTES_LEN);
    byte command = command_bytes[0];
    byte component_id = command_bytes[1];

    // initialize a component
    if (command == CMD_INIT) {

      if (component_id == LEFT_STEPPER_ID) {
        byte args[CMD_INIT_STEPPER_ARGS_LEN];
        Serial.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);
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
      } else if (component_id == RIGHT_STEPPER_ID) {
      }
    } else if (command == CMD_STEP) {
      if (component_id == LEFT_STEPPER_ID) {
        byte args[CMD_STEP_ARGS_LEN];
        Serial.readBytes(args, CMD_STEP_ARGS_LEN);
        unsigned int left_stepper_num_steps = bytes_to_unsigned_int(args, 0);
        unsigned int left_stepper_num_drives = left_stepper_num_steps * 2;
        left_stepper_drive_target = left_stepper_drive_idx + left_stepper_num_drives;
        left_stepper_drive_increment = args[2];
        unsigned int left_stepper_ms_to_step = bytes_to_unsigned_int(args, 3);
        left_stepper_ms_per_drive = int(left_stepper_ms_to_step / float(left_stepper_num_drives));
        left_stepper_next_drive_ms = millis() + left_stepper_ms_per_drive;
      } else if (component_id == RIGHT_STEPPER_ID) {
      }
    }
  } 
  else */
  
  if (left_stepper_inited) {
    Serial.println("already initialized");
  } 
  else {
    Serial.println("initializing");
    //left_driver_pin_1 = 5;
    //pinMode(left_driver_pin_1, OUTPUT);
    /*left_driver_pin_2 = 6;
    pinMode(left_driver_pin_2, OUTPUT);
    left_driver_pin_3 = 7;
    pinMode(left_driver_pin_3, OUTPUT);
    left_driver_pin_4 = 8;
    pinMode(left_driver_pin_4, OUTPUT);
    left_stepper_drive_idx = 0;
    left_stepper_drive_target = left_stepper_drive_idx;
    left_stepper_drive_increment = 0;
    float steps_per_degree = (32.0 / (1.0 / 64.0)) / 360.0;
    unsigned int left_stepper_num_steps = unsigned int(180 * steps_per_degree);
    unsigned int left_stepper_num_drives = left_stepper_num_steps * 2;
    left_stepper_drive_target = left_stepper_drive_idx + left_stepper_num_drives;
    left_stepper_drive_increment = 1;
    unsigned int left_stepper_ms_to_step = 5000;
    left_stepper_ms_per_drive = int(left_stepper_ms_to_step / float(left_stepper_num_drives));
    left_stepper_next_drive_ms = millis() + left_stepper_ms_per_drive;*/
    left_stepper_inited = true;
    Serial.println("initialized!");
  }

  Serial.println("delaying");
  delay(1000);
}
