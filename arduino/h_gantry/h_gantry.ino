// SerialUSB writes to the arduino IDE serial monitor. _UART1_ writes to the serial tx/rx gpio pins.
// define a nicer variable to refer to serial tx/rx.
#define SerialUART _UART1_

bool DEBUG = true;

const size_t FLOAT_BYTES_LEN = 4;
const size_t LONG_BYTES_LEN = 4;
const size_t UNSIGNED_INT_BYTES_LEN = 2;
const unsigned long US_PER_SEC = 1e6;  // microseconds per second

/* stepper done response.
 * 
 * stepper identifier:  byte (1 byte)
 * number of skipped steps:  fixed-point long (4 bytes)
 * move sequence index:  unsigned int (2 bytes)
 * completion timestamp epoch:  unsigned long (4 bytes)
*/
const size_t STEPPER_DONE_RESPONSE_LEN = 11;

/* maximum number of responses before force-flushing the response buffer. ensures the buffer 
 * does not exhaust memory. larger values decrease chatter back to the caller but also make
 * the responses have higher latency.
*/
const byte MAX_NUM_STEPPER_DONE_RESPONSES_TO_BUFFER = 50;

/* minimum step buffer length before force-flushing the response buffer. ensures responsiveness to the 
 * caller, who might be waiting for responses before sending more steps.
*/ 
const byte MIN_STEP_BUFFER_LEN_BEFORE_FLUSHING_STEPPER_DONE_RESPONSE_BUFFER = 10;

const size_t STEPPER_DONE_RESPONSE_BUFFER_LEN = MAX_NUM_STEPPER_DONE_RESPONSES_TO_BUFFER * STEPPER_DONE_RESPONSE_LEN;
byte STEPPER_DONE_RESPONSE_BUFFER[STEPPER_DONE_RESPONSE_BUFFER_LEN];
byte NUM_STEPPER_DONE_RESPONSES_BUFFERED = 0;

// structure that gives simultaneous access to single-precision floating-point numbers and their underlying bytes
typedef union {
  float number;
  byte bytes[FLOAT_BYTES_LEN];
} floatbytes;

// stepper driver configuration
const byte STEPPER_DRIVER_NUM_IN_PINS = 1;  // number of pins providing input to the A4988 driver
const byte DRIVES_PER_STEP = 4;  // 2 drives/microstep (HIGH then LOW) * 2 microsteps/step (HALF step) = 4 drives
const byte DRIVE_SEQUENCE_LEN = 2;  // always 2, since microstepping configuration is set by separate output pins to the A4988 driver
const byte DRIVE_SEQUENCE[DRIVE_SEQUENCE_LEN][STEPPER_DRIVER_NUM_IN_PINS] = {
  { LOW },
  { HIGH }
};
const unsigned long MIN_US_PER_DRIVE = 100;  // fastest driving with acceleration from a slower speed
const unsigned long MIN_US_PER_DRIVE_FROM_STOPPED = 500;  // fastest driving directly from a dead stop
const double FULL_ACCEL_INTERVAL_SEC = 0.25;  // fastest acceleration from dead stop to fastest
const byte A4988_MS1_OUTPUT_PIN = 9;  // sets half-step output in the A4988 driver

// Microstep configuration for the A4988
// MS1	MS2	  MS3	Microstep Resolution -- these are pull-down, so to get half stepping we only need to output HIGH to MS1.
// Low	Low	  Low	Full step
// High	Low   Low	Half step
// Low	High  Low	Quarter step
// High	High	Low	Eighth step
// High	High	High	Sixteenth step

// config for the ULN2003, which is driven by four output pins (4 driver pins @ 2 drives per step):
// const byte STEPPER_DRIVER_NUM_IN_PINS = 4;  // number of pins providing input to the stepper motor driver.
// const byte DRIVES_PER_STEP = 2;  // 1:  full steps, 2:  half steps
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
// const double FULL_ACCEL_INTERVAL_SEC = 0.25;  // fastest acceleration from dead stop to fastest

// maximum acceleration (us/drive/us):  slowest to fastest within the full-acceleration interval
const double MAX_DRIVE_ACC_US_PER_DRIVE_PER_US = (MIN_US_PER_DRIVE_FROM_STOPPED - MIN_US_PER_DRIVE) / (FULL_ACCEL_INTERVAL_SEC * double(US_PER_SEC));

// command id and component id
const size_t CMD_BYTES_LEN = 2;

// command:  init component
const byte CMD_INIT = 1;
const size_t CMD_INIT_STEPPER_ARGS_LEN = STEPPER_DRIVER_NUM_IN_PINS + 8;  // 1 byte per pin plus 2 bytes for optional disable pin, 2 bytes for optional direction pin, and 4 bytes for float scale
const size_t CMD_INIT_LIMIT_SWITCHES_ARGS_LEN = 4;  // one read pin per switch * 4 switches

// command:  step
const byte CMD_STEP = 2;
const byte CMD_STEP_ARGS_LEN = 16;

// command:  stop
const byte CMD_STOP = 3;

// command:  get current time (us)
const byte CMD_GET_CURRENT_TIME_US = 4;

// reusable structure for stepper configuration and drive state
struct stepper {
  byte identifier;
  byte driver_num_in_pins = STEPPER_DRIVER_NUM_IN_PINS;
  byte driver_pins[STEPPER_DRIVER_NUM_IN_PINS];
  float float_scale;
  int driver_disable_pin = -1;  // -1 for no disable pin
  bool driver_is_disabled = false;
  int driver_dir_pin = -1;  // -1 for no direction pin
  long drive_idx = 0;  // signed for direction
  long drive_target = 0;  // signed for direction
  long drives_remaining = 0;  // signed for direction
  int drive_increment = 0;  // -1 or +1
  long limit_skipped_drives = 0;  // number of drives skipped due to limit switches
  unsigned long us_per_drive = 0;  // current drive rate
  unsigned long previous_drive_us = 0;  // time of previous drive
  unsigned long us_per_drive_target = 0;  // target drive rate to be obtained via acceleration/deceleration
  unsigned long previous_acceleration_us = 0;  // time of previous acceleration
  unsigned long us_remaining = 0;  // time remaining to complete drives to target

  bool is_inited = false;  // whether driver is initialized
};

stepper left_stepper;
stepper right_stepper;

/* fifo linked list of steps to take, acting as a read buffer. each step determines
 * how the left and right steppers should move in tandem.
*/
struct step {
  long left_stepper_num_drives;
  long right_stepper_num_drives;
  unsigned long us_to_drive;
  unsigned int idx;
  step* next;
};
step* steps_head = nullptr;  // next step to process
step* steps_tail = nullptr;  // final step to process
unsigned int steps_len = 0;
unsigned int curr_step_idx = 0;  // current step index being processed; will be 0 when not processing a step.

void long_to_bytes(long value, byte bytes[]) {
  bytes[0] = (byte)(value >> 24);
  bytes[1] = (byte)(value >> 16);
  bytes[2] = (byte)(value >> 8);
  bytes[3] = (byte)value;
}

long bytes_to_long(byte bytes[], size_t start_idx) {
  uint32_t value = ((uint32_t)bytes[start_idx]) << 24;
  value |= ((uint32_t)bytes[start_idx + 1]) << 16;
  value |= ((uint32_t)bytes[start_idx + 2]) << 8;
  value |= ((uint32_t)bytes[start_idx + 3]);
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
  value |= ((uint32_t)bytes[start_idx + 1]) << 16;
  value |= ((uint32_t)bytes[start_idx + 2]) << 8;
  value |= ((uint32_t)bytes[start_idx + 3]);
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
  value |= ((uint16_t)bytes[start_idx + 1]);
  return (int16_t)value;
}

void unsigned_int_to_bytes(unsigned int value, byte bytes[]) {
  bytes[0] = (byte)(value >> 8);
  bytes[1] = (byte)value;
}

unsigned int bytes_to_unsigned_int(byte bytes[], size_t start_idx) {
  uint16_t value = ((uint16_t)bytes[start_idx]) << 8;
  value |= ((uint16_t)bytes[start_idx + 1]);
  return value;
}

void write_unsigned_int(unsigned int value) {
  byte bytes[UNSIGNED_INT_BYTES_LEN];
  unsigned_int_to_bytes(value, bytes);
  SerialUART.write(bytes, UNSIGNED_INT_BYTES_LEN);
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

/**
 * True modulo operator that wraps to the maximum value for negative numbers. The standard
 * C/Arduino behavior of % is the remainder operator, which takes the sign of the dividend.
 * When the dividend is negative, the remainder is also negative, which does not work when
 * we want to use the result for looping indices over an array. Instead, we need the true
 * mathematical modulo operator. The C/Arduino % operator behaves like math.fmod in Python.
 * The % operator in Python behaves like the true modulo operator and also the same as the 
 * function below.
 *
 * @param x Dividend.
 * @param y Divisor.
 * @return x modulo y.
*/
byte mod(long x, byte y) {
  return ((x % y) + y) % y;  // alternatively:  x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

/**
 * Add a step to the move buffer.
 *
 * @param left_stepper_num_drives Number of drives to apply to the left stepper.
 * @param right_stepper_num_drives Number of drives to apply to the right stepper.
 * @param us_to_drive Microseconds in which to achieve the drives.
 * @param idx Step index.
*/
void add_step(
  long left_stepper_num_drives, 
  long right_stepper_num_drives,
  unsigned long us_to_drive,
  unsigned int idx
) {

  step* new_step = new step();
  new_step->left_stepper_num_drives = left_stepper_num_drives;
  new_step->right_stepper_num_drives = right_stepper_num_drives;
  new_step->us_to_drive = us_to_drive;
  new_step->idx = idx;
  new_step->next = nullptr;

  // if list is empty, head and tail both point to new step.
  if (steps_head == nullptr) {
    steps_head = steps_tail = new_step;
  }
  // otherwise, there is a tail, and its next points to the new step.
  else {
    steps_tail->next = new_step;
    steps_tail = new_step;
  }

  steps_len += 1;

}

/**
 * Remove and return the next step from the move buffer.
 * 
 * @return The next step, or null if the move buffer is empty.
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
 * Initialize a stepper.
 * 
 * @param stepper_to_init Stepper to initialize.
*/
void init_stepper(stepper* stepper_to_init) {

  byte args[CMD_INIT_STEPPER_ARGS_LEN];
  SerialUART.readBytes(args, CMD_INIT_STEPPER_ARGS_LEN);

  for (byte pin_idx = 0; pin_idx < stepper_to_init->driver_num_in_pins; ++pin_idx) {
    byte pin = args[pin_idx];
    stepper_to_init->driver_pins[pin_idx] = pin;
    pinMode(pin, OUTPUT);
  }

  stepper_to_init->driver_disable_pin = bytes_to_int(args, stepper_to_init->driver_num_in_pins);
  if (stepper_to_init->driver_disable_pin >= 0) {
    pinMode(stepper_to_init->driver_disable_pin, OUTPUT);
    digitalWrite(stepper_to_init->driver_disable_pin, HIGH);
    stepper_to_init->driver_is_disabled = true;
  }

  stepper_to_init->driver_dir_pin = bytes_to_int(args, stepper_to_init->driver_num_in_pins + 2);
  if (stepper_to_init->driver_dir_pin >= 0) {
    pinMode(stepper_to_init->driver_dir_pin, OUTPUT);
  }

  stepper_to_init->float_scale = float(bytes_to_unsigned_long(args, stepper_to_init->driver_num_in_pins + 4));

  stepper_to_init->is_inited = true;
  write_bool(true);
  
}

/**
 * Get the drive-delay target that achieves a number of drives in a given time interval.
 *
 * @param stepper_to_delay Stepper for which to get delay.
 * @return The drive-delay target (us).
*/
unsigned long get_drive_delay_target(stepper* stepper_to_delay) {

    if (DEBUG) {
      SerialUSB.println("Getting drive delay for stepper " + String(stepper_to_delay->identifier) + ":  " + String(stepper_to_delay->drives_remaining) + " drives in " + String(stepper_to_delay->us_remaining) + " us.");
    }
    
    unsigned long delay_target_us = 0;

    unsigned long num_delays = abs(stepper_to_delay->drives_remaining);
    if (num_delays > 0) {
      delay_target_us = (unsigned long)(stepper_to_delay->us_remaining / double(num_delays));
    }
    
    // impose maximum drive rate (minimum delay)
    if (delay_target_us < MIN_US_PER_DRIVE) {
      delay_target_us = MIN_US_PER_DRIVE;
    }

    if (DEBUG) {
      SerialUSB.println("Target drive delay:  " + String(delay_target_us) + " us.");
    }

    return delay_target_us;
}

/**
 * Start a stepper.
 *
 * @param stepper_to_start Stepper to start.
 * @param num_drives Number of drives (signed per direction).
 * @param us_to_drive Total us to drive.
 * @param curr_time_us Current time in us.
*/
void start_stepper(
  stepper* stepper_to_start,
  long num_drives,
  unsigned long us_to_drive,
  unsigned long curr_time_us
) {

  if (DEBUG) {
    SerialUSB.println("Starting stepper " + String(stepper_to_start->identifier) + " with " + String(num_drives) + " drives.");
  }

  // if the step does not involve any drives, then reset state.
  if (num_drives == 0) {
    stepper_to_start->drive_increment = 0;
    stepper_to_start->limit_skipped_drives = 0;
    stepper_to_start->us_per_drive = 0;
    stepper_to_start->us_per_drive_target = 0;
    write_stepper_done(stepper_to_start, curr_step_idx, curr_time_us);
  }

  // otherwise, configure the stepper to run.
  else {

    // enable the stepper driver, since we're about to use the stepper.
    if (stepper_to_start->driver_disable_pin >= 0 && stepper_to_start->driver_is_disabled) {
      digitalWrite(stepper_to_start->driver_disable_pin, LOW);
      stepper_to_start->driver_is_disabled = false;
    }

    stepper_to_start->drive_idx = mod(stepper_to_start->drive_idx, DRIVE_SEQUENCE_LEN);  // mod initial drive idx to avoid overflow
    stepper_to_start->drive_target = stepper_to_start->drive_idx + num_drives;
    stepper_to_start->drives_remaining = stepper_to_start->drive_target - stepper_to_start->drive_idx;
    int previous_drive_increment = stepper_to_start->drive_increment;
    stepper_to_start->drive_increment = num_drives > 0 ? 1 : -1;
    bool changing_direction = stepper_to_start->drive_increment != previous_drive_increment;

    // set the direction
    if (stepper_to_start->driver_dir_pin >= 0) {
      digitalWrite(stepper_to_start->driver_dir_pin, stepper_to_start->drive_increment < 0 ? LOW : HIGH);
    }
    stepper_to_start->limit_skipped_drives = 0;
    stepper_to_start->us_remaining = us_to_drive;
    stepper_to_start->us_per_drive_target = get_drive_delay_target(stepper_to_start);

    /* if we're changing direction, then set the drive speed to the fastest permissible from a stopped position. we'll accelerate
     * (per limit) or decelerate (instantaneously) from this speed upon the next acceleration.
    */
    if (changing_direction) {
      stepper_to_start->us_per_drive = MIN_US_PER_DRIVE_FROM_STOPPED;
    }
    /* otherwise, maintain the current drive rate and accelerate/decelerate from this rate. we might be starting
     * from a dead stop or a very low rate, so use the min between this value and the minimum delay from a dead
     * stop. we'll accelerate/decelerate from this value.
    */
    else {
      stepper_to_start->us_per_drive = min(MIN_US_PER_DRIVE_FROM_STOPPED, stepper_to_start->us_per_drive);
    }

    stepper_to_start->previous_drive_us = curr_time_us;
    stepper_to_start->previous_acceleration_us = curr_time_us;

    if (DEBUG) {
      SerialUSB.println("Done starting stepper " + String(stepper_to_start->identifier) + " at " + String(stepper_to_start->us_per_drive) + " us/drive.");
    }
  }
}

/**
 * Acclerate a stepper.
 *
 * @param stepper_to_acc Stepper to accelerate.
 * @param curr_time_us Current time in us.
*/
void accelerate_stepper(
  stepper* stepper_to_acc,
  unsigned long curr_time_us
) {

  // reduce stepper delay to target delay limited by maximum acceleration
  if (stepper_to_acc->us_per_drive > stepper_to_acc->us_per_drive_target) {

    unsigned long reduce_delay_us = stepper_to_acc->us_per_drive - stepper_to_acc->us_per_drive_target;

    // limit reduction by passed time and max acceleration
    unsigned long us_since_acceleration = curr_time_us - stepper_to_acc->previous_acceleration_us;
    unsigned long max_reduction_us = (unsigned long)(us_since_acceleration * MAX_DRIVE_ACC_US_PER_DRIVE_PER_US);
    if (reduce_delay_us > max_reduction_us) {
      reduce_delay_us = max_reduction_us;
    }

    // insufficient time may have passed
    if (reduce_delay_us > 0) {

      stepper_to_acc->us_per_drive -= reduce_delay_us;
      stepper_to_acc->previous_acceleration_us = curr_time_us;

      // if we've achieved the target delay, then recalculate the target delay to make up for time lost 
      // due to the acceleration interval, when the stepper was not operating at the ideal delay.
      if (stepper_to_acc->us_per_drive == stepper_to_acc->us_per_drive_target) {
        stepper_to_acc->us_per_drive_target = get_drive_delay_target(stepper_to_acc);
      }
    }
  }
  // if the stepper is at the target delay, mark the current time as the one from which to perform subsequent accelerations.
  else if (stepper_to_acc->us_per_drive == stepper_to_acc->us_per_drive_target) {
    stepper_to_acc->previous_acceleration_us = curr_time_us;
  }
  // allow instantaneous deceleration to the target
  else {
    stepper_to_acc->us_per_drive = stepper_to_acc->us_per_drive_target;
    stepper_to_acc->previous_acceleration_us = curr_time_us;
  }
}

/**
 * Drive a stepper to its target index.
 *
 * @param stepper_to_drive Stepper to drive.
 * @param limited_travel Whether travel is limited.
 * @param curr_time_us Current time in microseconds.
 * @return Whether stepper has completed.
 */
void drive_stepper(
  stepper* stepper_to_drive,
  bool limited_travel,
  unsigned long curr_time_us
) {

  /* drive the stepper if needed to reach target and enough time has elapsed. modular arithmetic handles micros() overflow 
   * naturally, when curr_time_us wraps back around to zero creating a negative elapsed time. if travel is being limited 
   * by a limit switch, then drive immediately without waiting for elapsed time, to skip the rest of the move.
  */
  unsigned long us_elapsed_since_previous_drive = curr_time_us - stepper_to_drive->previous_drive_us;
  if (limited_travel || (stepper_to_drive->drives_remaining != 0 && us_elapsed_since_previous_drive >= stepper_to_drive->us_per_drive)) {

    stepper_to_drive->drive_idx += stepper_to_drive->drive_increment;
    stepper_to_drive->drives_remaining -= stepper_to_drive->drive_increment;

    if (stepper_to_drive->us_remaining > us_elapsed_since_previous_drive) {
      stepper_to_drive->us_remaining -= us_elapsed_since_previous_drive;
    }
    else {
      stepper_to_drive->us_remaining = 0;
    }

    // if travel is limited, then do not drive the stepper but record the skipped increment for reporting back to the caller.
    if (limited_travel) {
      stepper_to_drive->limit_skipped_drives += stepper_to_drive->drive_increment;
    }

    // otherwise, drive the next index in the sequence.
    else {
      byte drive_sequence_idx = mod(stepper_to_drive->drive_idx, DRIVE_SEQUENCE_LEN);
      for (byte pin_idx = 0; pin_idx < stepper_to_drive->driver_num_in_pins; ++pin_idx) {
        digitalWrite(stepper_to_drive->driver_pins[pin_idx], DRIVE_SEQUENCE[drive_sequence_idx][pin_idx]);
      }
    }

    stepper_to_drive->previous_drive_us = curr_time_us;

    if (stepper_to_drive->drives_remaining == 0) {
      write_stepper_done(stepper_to_drive, curr_step_idx, curr_time_us);
    }
  }
}

/**
 * Disable a stepper, which cuts current and lets its coils cool.
 *
 * @param stepper_to_disable Stepper to disable.
*/
void disable_stepper(
  stepper* stepper_to_disable
) {
  if (stepper_to_disable->driver_disable_pin >= 0 && !stepper_to_disable->driver_is_disabled) {
    digitalWrite(stepper_to_disable->driver_disable_pin, HIGH);
    stepper_to_disable->driver_is_disabled = true;
  }
}

/**
 * Wrapper of memcpy that returns the next starting index to write.
 *
 * @param dest Destination array.
 * @param start Start index within destination array to write.
 * @param data Data to write.
 * @param data_len Length of data to write.
 * @return Next starting index within the destination array.
*/
size_t memcpy_wrap(byte dest[], size_t start, byte data[], size_t data_len) {
  memcpy(dest + start, data, data_len);
  return start + data_len;
}

/**
 * Write back to client that stepper is done.
 *
 * @param stepper_done Stepper that is done.
 * @param idx Move sequence index that is done.
 * @param done_time_us Timestamp (us) when the stepper/move completed.
*/
void write_stepper_done(stepper* stepper_done, unsigned int idx, unsigned long done_time_us) {

    if (DEBUG) {
      SerialUSB.println("Stepper " + String(stepper_done->identifier) + " done with move " + String(idx) + " at time " + String(done_time_us));
    }

    byte two_bytes[2];
    byte four_bytes[4];

    // form response:  stepper identifier, number of skipped steps, move sequence index, and completion timestamp
    byte data[STEPPER_DONE_RESPONSE_LEN];
    size_t data_idx = 0;

    data[data_idx] = stepper_done->identifier;
    data_idx += 1;
    
    unsigned_long_to_bytes((long)stepper_done->limit_skipped_drives * stepper_done->float_scale, four_bytes);
    data_idx = memcpy_wrap(data, data_idx, four_bytes, 4);
    
    unsigned_int_to_bytes(idx, two_bytes);
    data_idx = memcpy_wrap(data, data_idx, two_bytes, 2);

    unsigned_long_to_bytes(done_time_us, four_bytes);
    data_idx = memcpy_wrap(data, data_idx, four_bytes, 4);

    if (data_idx == STEPPER_DONE_RESPONSE_LEN) {
      size_t copy_start_idx = NUM_STEPPER_DONE_RESPONSES_BUFFERED * STEPPER_DONE_RESPONSE_LEN;
      memcpy(STEPPER_DONE_RESPONSE_BUFFER + copy_start_idx, data, STEPPER_DONE_RESPONSE_LEN);
      NUM_STEPPER_DONE_RESPONSES_BUFFERED += 1;
      check_stepper_done_buffer(false);
    }
    else if (DEBUG) {
      SerialUSB.println("Rotary state data index/length mismatch.");
    }
}

/** 
 * Check the stepper-done buffer, writing messages back to the client if the buffer is full.
 *
 * @param force_flush Whether to force writing all stepper-done messages back to the client.
*/
void check_stepper_done_buffer(bool force_flush) {

  if (NUM_STEPPER_DONE_RESPONSES_BUFFERED == 0) {
    return;
  }

  if (force_flush || NUM_STEPPER_DONE_RESPONSES_BUFFERED >= MAX_NUM_STEPPER_DONE_RESPONSES_TO_BUFFER) {
      SerialUART.write(STEPPER_DONE_RESPONSE_BUFFER, NUM_STEPPER_DONE_RESPONSES_BUFFERED * STEPPER_DONE_RESPONSE_LEN);
      SerialUART.flush();
      NUM_STEPPER_DONE_RESPONSES_BUFFERED = 0;
    }
}

/**
 * Stop a stepper.
 *
 * @param stepper_to_stop Stepper to stop.
*/
void stop_stepper(stepper* stepper_to_stop) {

  disable_stepper(stepper_to_stop);

  for (byte pin_idx = 0; pin_idx < stepper_to_stop->driver_num_in_pins; ++pin_idx) {
    digitalWrite(stepper_to_stop->driver_pins[pin_idx], LOW);
  }
  write_bool(true);
}

// limit switches
const byte LIMIT_SWITCHES_ID = 2;
byte left_limit_switch_pin;
byte right_limit_switch_pin;
byte bottom_limit_switch_pin;
byte top_limit_switch_pin;
bool limit_switches_inited = false;

void setup() {

  left_stepper.identifier = 0;
  right_stepper.identifier = 1;

  if (DEBUG) {
    SerialUSB.begin(9600);
  }

  SerialUART.begin(115200, SERIAL_8N1);

  // configure microstepping on the a4988 driver, which is 1,0,0 (half steps)
  pinMode(A4988_MS1_OUTPUT_PIN, OUTPUT);
  digitalWrite(A4988_MS1_OUTPUT_PIN, HIGH);
}

void loop() {

  unsigned long curr_time_us = micros();

  // process the current step command if everything is initialized
  if (left_stepper.is_inited && right_stepper.is_inited && limit_switches_inited) {

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
     *    lss and rss are both 0 (steppers do not move), or lss = -rss, in which case the
     *    steppers move equally in opposite directions, which is up/down movement.
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
     *    lss and rss are both 0 (steppers do not move), or lss = rss, in which case the
     *    steppers move equally in the same direction, which is left/right movement.
     *
     * 6) note that we take half steps, which means we drive a motor twice per step. for
     *    the purpose of checking left/right and down/up movement, it is equivalent to check
     *    the number of left and right drives that remain.
    */
    bool moving_left = false;
    bool moving_right = false;
    bool moving_down = false;
    bool moving_up = false;

    // check condition (4a) for movement left
    long left_plus_right = left_stepper.drives_remaining + right_stepper.drives_remaining;
    if (left_plus_right < 0) {
      moving_left = true;
    }
    // check condition (4b) for movement right
    else if (left_plus_right > 0) {
      moving_right = true;
    }

    // check condition (5a) for movement down
    long left_minus_right = left_stepper.drives_remaining - right_stepper.drives_remaining;
    if (left_minus_right < 0) {
      moving_down = true;
    }
    // check condition (5b) for movement up
    else if (left_minus_right > 0) {
      moving_up = true;
    }

    /* check whether the gantry has hit a limit and must stop. this is indicated by pressing 
     * a limit switch in the direction of travel. it is fine to continue traveling in a direction
     * that is not toward a pressed limit switch.
     */
    bool limited_travel = (
      (moving_left && !digitalRead(left_limit_switch_pin)) ||
      (moving_right && !digitalRead(right_limit_switch_pin)) ||
      (moving_down && !digitalRead(bottom_limit_switch_pin)) ||
      (moving_up && !digitalRead(top_limit_switch_pin))
    );

    accelerate_stepper(&left_stepper, curr_time_us);
    drive_stepper(&left_stepper, limited_travel, curr_time_us);

    accelerate_stepper(&right_stepper, curr_time_us);
    drive_stepper(&right_stepper, limited_travel, curr_time_us);
  }

  // process a commands sent over the serial connection
  while (SerialUART.available()) {

    byte command_bytes[CMD_BYTES_LEN];
    SerialUART.readBytes(command_bytes, CMD_BYTES_LEN);
    byte command = command_bytes[0];
    byte component_id = command_bytes[1];

    // initialize a component
    if (command == CMD_INIT) {
      if (component_id == left_stepper.identifier) {        
        init_stepper(&left_stepper);
      }
      else if (component_id == right_stepper.identifier) {
        init_stepper(&right_stepper);
      }
      else if (component_id == LIMIT_SWITCHES_ID) {
        byte args[CMD_INIT_LIMIT_SWITCHES_ARGS_LEN];
        SerialUART.readBytes(args, CMD_INIT_LIMIT_SWITCHES_ARGS_LEN);
        left_limit_switch_pin = args[0];
        pinMode(left_limit_switch_pin, INPUT_PULLUP);
        right_limit_switch_pin = args[1];
        pinMode(right_limit_switch_pin, INPUT_PULLUP);
        bottom_limit_switch_pin = args[2];
        pinMode(bottom_limit_switch_pin, INPUT_PULLUP);
        top_limit_switch_pin = args[3];
        pinMode(top_limit_switch_pin, INPUT_PULLUP);
        limit_switches_inited = true;
        write_bool(true);
      }
    }
    else if (command == CMD_STEP) {

      byte args[CMD_STEP_ARGS_LEN];
      SerialUART.readBytes(args, CMD_STEP_ARGS_LEN);

      // left stepper:  calculate number of drives. skip the first two bytes sent by the stepper, which are the step command and stepper identifier.
      long left_stepper_num_drives = bytes_to_int(args, 2) * DRIVES_PER_STEP;
      unsigned long left_stepper_us_to_drive = ((unsigned long)bytes_to_unsigned_int(args, 4)) * 1000;
      unsigned int left_stepper_step_idx = bytes_to_unsigned_int(args, 6);

      // right stepper:  calculate number of drives. skip the first two bytes sent by the stepper, which are the step command and stepper identifier.
      long right_stepper_num_drives = bytes_to_int(args, 10) * DRIVES_PER_STEP;
      unsigned long right_stepper_us_to_drive = ((unsigned long)bytes_to_unsigned_int(args, 12)) * 1000;
      unsigned int right_stepper_step_idx = bytes_to_unsigned_int(args, 14);

      // indices must be the same; otherwise, we're out of sync with the caller. us to drive must also be the same, since the steppers always move in tandem.
      if (left_stepper_step_idx == right_stepper_step_idx && left_stepper_us_to_drive == right_stepper_us_to_drive) {
        add_step(
          left_stepper_num_drives, 
          right_stepper_num_drives,
          left_stepper_us_to_drive,  // same as right
          left_stepper_step_idx  // same as right
        );
      }
      else {
        SerialUSB.println("Invalid step received.");
      }
    }
    else if (command == CMD_STOP) {
      if (component_id == left_stepper.identifier && left_stepper.is_inited) {
        stop_stepper(&left_stepper);
      }
      else if (component_id == right_stepper.identifier && right_stepper.is_inited) {
        stop_stepper(&right_stepper);
      }
    }
    else if (command == CMD_GET_CURRENT_TIME_US) {
      write_unsigned_long(curr_time_us);
    }
  }

  // if both steppers are initialized and are at their targets, then attempt to start the next step.
  if (left_stepper.is_inited && left_stepper.drives_remaining == 0 && right_stepper.is_inited && right_stepper.drives_remaining == 0) {

    step* next_step = get_next_step();

    // if the buffer is low/empty, then force-flush the stepper done buffer. the caller might 
    // be waiting on this signal that the buffer is low before sending more moves.
    if (steps_len <= MIN_STEP_BUFFER_LEN_BEFORE_FLUSHING_STEPPER_DONE_RESPONSE_BUFFER) {
      check_stepper_done_buffer(true);
    }

    // if the buffer is empty, then we have nothing further to do at this time.
    if (next_step == nullptr) {

      /* if the buffer has run out of steps, then the steppers won't drive further. we're going to lose any momentum 
       * that we have. set us/drive to zero, which will require the steppers to accelerate from their slowest speed
       * when they resume stepping.
      */
      left_stepper.us_per_drive = 0;
      left_stepper.us_per_drive_target = 0;
      left_stepper.drive_increment = 0;
      disable_stepper(&left_stepper);

      right_stepper.us_per_drive = 0;
      right_stepper.us_per_drive_target = 0;
      right_stepper.drive_increment = 0;
      disable_stepper(&right_stepper);

      curr_step_idx = 0;

    }
    // immediately configure the next step
    else {

      curr_step_idx = next_step->idx;

      if (DEBUG) {
        SerialUSB.println("Starting next step:  " + String(curr_step_idx));
      }

      start_stepper(&left_stepper, next_step->left_stepper_num_drives, next_step->us_to_drive, curr_time_us);
      start_stepper(&right_stepper, next_step->right_stepper_num_drives, next_step->us_to_drive, curr_time_us);

      delete next_step;

      if (DEBUG) {
        SerialUSB.println("Done starting step. Deleted step reference.");
      }
    }
  }
}