// Define constants for the encoder pins and PWM pins
// #define ENCODER_RIGHT_A 4
#define ENCODER_RIGHT_B 6
// #define ENCODER_LEFT_A 6
#define ENCODER_LEFT_B 7
// atremis
// B on motor driver
// currently right motor
#define leftMotor1 2
#define leftMotor2 3
// A on motor driver
// currently left motor
#define rightMotor1 4
#define rightMotor2 5

// Define variables to hold encoder counts and previous counts
volatile float encoder_left_count = 0;
volatile float encoder_right_count = 0;
// Overall current rotation 0.0-1.0
volatile float current_rotation_right = 0.0;
volatile float current_rotation_left = 0.0;
volatile float overall_rotation_right = 0.0;
volatile float overall_rotation_left = 0.0;

// Define constants for PID loop tuning
const float KP = 1.0;
const float KI = 0.5;
const float KD = 0.5;

float left_error_prev = 0.0;
float left_i_prev = 0.0;
float right_error_prev = 0.0;
float right_i_prev = 0.0;

// wheel rotation storage /////////////////
float total_rotations_left = 0;
float total_rotations_right = 0;
volatile float last_overall_rotation_right;
volatile float last_overall_rotation_left;
///////////////////////////////////////////

// static wheel varibles //////////////////
const float maxrps = 3.0;
const float ticks_per_rotation = 12 * 74.83; // 48 TICKS per rotation of the motor and 75 rotations of the motor per rotation of the wheel as the gearbox is 75:1 we are using 12 because we only have 1 encoder per motor as left_a is dead
///////////////////////////////////////////

// Desired wheel velocity (1800 ticks per second, 2 sec per rotation)

//////////////////// move vars ////
float left_set_point = 1.0; // 0.0 - 1.0
float right_set_point = 1.0;
float left_dir = true;
float right_dir = true;
//////////////////////////////////////////////

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


// Possible limitation for the motor driver and PWM signal setup
// 254 seems to usually work, 253 has never failed
int maxPossSpeed = 254;
int minPossSpeed = 0;

void clear_pid_hist() {
  left_i_prev = 0.0;
  left_error_prev = 0.0;
  right_i_prev = 0.0;
  right_error_prev = 0.0;
}

void right_motor_speed(bool dir, int speed){
  if(speed > maxPossSpeed) speed = maxPossSpeed;
  if(speed < minPossSpeed) speed = minPossSpeed;

  //if true, go forwards
  if(dir == true){
    analogWrite(rightMotor1, speed);
    analogWrite(rightMotor2, 0);
  }
  //else, go backwards
  else{
    analogWrite(rightMotor1, 0);
    analogWrite(rightMotor2, speed);
    }
}

void left_motor_speed( bool dir, int speed){
  if(speed > maxPossSpeed) speed = maxPossSpeed;
  if(speed < minPossSpeed) speed = minPossSpeed;

  //if true, go forwards
  if(dir == true){
    analogWrite(leftMotor1, speed);
    analogWrite(leftMotor2, 0);
  }
  //else, go backwards
  else{
    analogWrite(leftMotor1, 0);
    analogWrite(leftMotor2, speed);
    }
}

// void rotate_robot(bool dir, int speed){
//   if(speed > maxPossSpeed) speed = maxPossSpeed;
//   if(speed < minPossSpeed) speed = minPossSpeed;

//   // we might want to do something tricky with the speeds being passed to each motor

//   //if true, roatate clockwise
//   if(dir == true){
//     leftMotorSpeed(true, speed);
//     rightMotorSpeed(false, speed);
//   }

//   //else, rotate counterclockwise
//   if(dir == false){
//     leftMotorSpeed(false, speed);
//     rightMotorSpeed(true, speed);
//   }
// }

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

void ros_rotate(bool clockwise, float speed) {
  clear_pid_hist();
  float fixed_speed = speed / 255.0;
  left_dir = true;
  right_dir = true;
  if (clockwise) {
    left_set_point = fixed_speed;
    right_set_point = 0.0;
  } else {
    right_set_point = fixed_speed;
    left_set_point = 0.0;
  }
}

void ros_move(bool fwd, float speed) {
  clear_pid_hist();
  float fixed_speed = speed / 255.0;
  left_set_point = fixed_speed;
  left_dir = fwd;
  right_set_point = fixed_speed;
  right_dir = fwd;
}

void setup()
{
  // Set up encoder pins as inputs
  // pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  // pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Attach interrupt service routines to encoder pins

  // we arent using the A encoders cause Left A is dead...
  // attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoder_left_isr_a, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoder_right_isr_a, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), encoder_left_isr_b, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), encoder_right_isr_b, RISING);


  // Set up PWM pins as outputs
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  // analogWrite(PWM_LEFT, 90);
  // analogWrite(PWM_RIGHT, 90);

  // ros_move(true, 255);

  Serial.begin(9600);
  Serial.println("output started");
}


float calculate_pid_left(float seconds_passed, float rps, float sp) {
  float pv = rps / maxrps;
  float error = sp - pv;
  float P = KP * error;
  float I = left_i_prev + KI * error * seconds_passed;
  float D = KD * (error - left_error_prev) / seconds_passed;
  float CV = P + I + D;
  CV = maxPossSpeed * CV;
  left_error_prev = error;
  left_i_prev = I;
  return CV;
}

float calculate_pid_right(float seconds_passed, float rps, float sp) {
  float pv = rps / maxrps;
  float error = sp - pv;
  float P = KP * error;
  float I = left_i_prev + KI * error * seconds_passed;
  float D = KD * (error - right_error_prev) / seconds_passed;
  float CV = P + I + D;
  CV = maxPossSpeed * CV;
  right_error_prev = error;
  left_i_prev = I;
  return CV;
}

float get_rps_right(float seconds_passed) {
  // Calculate right and right velocities based on encoder counts
  current_rotation_right = encoder_right_count / ticks_per_rotation;
  overall_rotation_right = total_rotations_right + current_rotation_right;

  float rps = (overall_rotation_right - last_overall_rotation_right) / seconds_passed;
  last_overall_rotation_right = overall_rotation_right;

  return rps;
}

float get_rps_left(float seconds_passed) {
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
  float seconds_passed = millis_passed / 1000.0;
  float rps = get_rps_left(seconds_passed);
  float pid_calc = calculate_pid_left(seconds_passed, rps, left_set_point);
  left_last_pid_millis = millis();  
  left_motor_speed(right_dir, pid_calc);  // Amongi
  Serial.println(pid_calc);
}

void pid_loop_right(float millis_passed) {
  float seconds_passed = millis_passed / 1000.0;
  float rps = get_rps_right(seconds_passed);
  float pid_calc = calculate_pid_right(seconds_passed, rps, right_set_point);
  right_last_pid_millis = millis();
  right_motor_speed(right_dir, pid_calc); // Amongi
}

float pid_loop_delay = 1000.0; // loop delay in millis
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
