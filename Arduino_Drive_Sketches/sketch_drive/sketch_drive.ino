// Define constants for the encoder pins and PWM pins
const int ENCODER_RIGHT_A = 4;
const int ENCODER_RIGHT_B = 5;
const int ENCODER_LEFT_A = 6;
const int ENCODER_LEFT_B = 7;
const int PWM_LEFT = 3;
const int PWM_RIGHT = 2;

// Define variables to hold encoder counts and previous counts
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
// Overall current rotation 0.0-1.0
float current_rotation_right = 0.0;
float current_rotation_left = 0.0;
volatile float overall_rotation_right = 0.0;
volatile float overall_rotation_left = 0.0;

// Define constants for PID loop tuning
const float KP = 1.0;
const float KI = 0.0;
const float KD = 0.0;

// wheel rotation storage /////////////////
float total_rotations_left = 0;
float total_rotations_right = 0;
volatile float last_overall_rotation_right;
volatile float last_overall_rotation_left;
///////////////////////////////////////////

// static wheel varibles //////////////////
const int set_point_max = 230;
const float maxrps = 100.0;
const float ticks_per_rotation = 12 * 74.83; // 48 TICKS per rotation of the motor and 75 rotations of the motor per rotation of the wheel as the gearbox is 75:1 we are using 12 because we only have 1 encoder per motor as left_a is dead
///////////////////////////////////////////

// Desired wheel velocity (1800 ticks per second, 2 sec per rotation)
const float left_set_point = 1.0; // 0.0 - 1.0
const float right_set_point = 1.0;

float process_var_left = 0.0;
float process_var_right = 0.0;

float left_proportion = 0.0;
float right_proportion = 0.0;
float left_integral = 0.0;
float right_integral = 0.0;
float left_derivative = 0.0;
float right_derivative = 0.0;
float left_error = 0.0;
float right_error = 0.0;
float left_prev_error = 0.0;
float right_prev_error = 0.0;

// loop varibles //////

volatile float last_millis_loop = millis();
volatile float millis_passed_loop = 0.0;

///////////////////////

// Interrupt service routine for the left encoder
void encoder_left_isr_b()
{
  // when a is rising and b is low we are going forwards
  encoder_left_count += 1;
  
  if (encoder_left_count >= ticks_per_rotation) {
    encoder_left_count -= ticks_per_rotation;
    total_rotations_left += 1;
  }
}

// Interrupt service routine for the right encoder
void encoder_right_isr_b()
{
  encoder_right_count += 1;

  if (encoder_right_count >= ticks_per_rotation) {
    encoder_right_count -= ticks_per_rotation;
    total_rotations_right += 1;
  }
}

void setup()
{
  // Set up encoder pins as inputs
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Attach interrupt service routines to encoder pins

  // we arent using the A encoders cause Left A is dead...
  // attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoder_left_isr_a, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoder_right_isr_a, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), encoder_left_isr_b, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), encoder_right_isr_b, RISING);


  // Set up PWM pins as outputs
  // pinMode(PWM_LEFT, OUTPUT);
  // pinMode(PWM_RIGHT, OUTPUT);
  // analogWrite(PWM_LEFT, 90);
  // analogWrite(PWM_RIGHT, 90);

  Serial.begin(9600);
  Serial.println("output started");
}

float errorPrev = 0.0;
float Iprev = 0.0;
float calculate_pid(float seconds_passed, float rps, float sp) {
  float pv = rps / maxrps;
  float error = sp - pv;
  float P = KP * error;
  float I = Iprev + KI * error * seconds_passed;
  float D = KD * (error - errorPrev) / seconds_passed;
  float CV = P + I + D;
  CV = set_point_max * CV;
  errorPrev = error;
  Iprev = I;
  return CV;
}

float get_rps_right(long seconds_passed) {
  // Calculate right and right velocities based on encoder counts
  current_rotation_right = encoder_right_count / ticks_per_rotation;
  overall_rotation_right = total_rotations_right + current_rotation_right;

  float rps = (overall_rotation_right - last_overall_rotation_right) / seconds_passed;
  last_overall_rotation_right = overall_rotation_right;

  return rps;
}

float get_rps_left(long seconds_passed) {
  // Calculate left and right velocities based on encoder counts
  current_rotation_left = encoder_left_count / ticks_per_rotation;
  overall_rotation_left = total_rotations_left + current_rotation_left;

  float rps = (overall_rotation_left - last_overall_rotation_left) / seconds_passed;
  last_overall_rotation_left = overall_rotation_left;

  return rps;
}

float left_last_pid_millis = millis();
float left_pid_millis_passed = (millis() - left_last_pid_millis);
float right_last_pid_millis = millis();
float right_pid_millis_passed = (millis() - right_last_pid_millis);

void pid_loop_left(float millis_passed) {
  long seconds_passed = millis_passed / 1000;
  long rps = get_rps_left(seconds_passed);
  long pid_calc = calculate_pid(seconds_passed, rps, left_set_point);
  left_last_pid_millis = millis();  
  analogWrite(PWM_LEFT, pid_calc);  
  Serial.println(rps);
}

void pid_loop_right(float millis_passed) {
  long seconds_passed = millis_passed / 1000;
  long rps = get_rps_right(seconds_passed);
  long pid_calc = calculate_pid(seconds_passed, rps, right_set_point);
  left_last_pid_millis = millis();
  analogWrite(PWM_RIGHT, pid_calc);
  Serial.println(rps);
}

float pid_loop_delay = 100.0;
void loop()
{
  millis_passed_loop = (millis() - last_millis_loop);
  last_millis_loop = millis();

  left_pid_millis_passed = (millis() - left_last_pid_millis);
  right_pid_millis_passed = (millis() - right_last_pid_millis);
  
  if (left_pid_millis_passed >= pid_loop_delay) {
    pid_loop_left(left_pid_millis_passed);
  }

  if (right_pid_millis_passed >= pid_loop_delay) {
    pid_loop_right(right_pid_millis_passed);
  }
}
