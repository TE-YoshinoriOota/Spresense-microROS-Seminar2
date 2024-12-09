#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

/* Use CMSIS library */
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <arm_math.h>

#include <MP.h>

#define MOTOR_SW 12
#define M_CTRL   8
#define ROT_ENABLE 4
#define R_EN 11
#define L_EN 10
#define UPDATE_DURATION_IN_MICRO (30000) // 30 msec

// #define INTERACTIVE
#define MIN_PWM 40
#define MAX_PWM 75
#define DLT_PWM  1

const bool debug_led = false;

//  rotary encoder
volatile uint32_t R = 0;
volatile uint32_t L = 0;
volatile uint32_t curR = 0;
volatile uint32_t curL = 0;
void EncoderR() {  ++R; }
void EncoderL() {  ++L; }

// rover constants
const float radius = 0.018; // m
const float theta =  9.0; // degree
const float d = 0.09; // distance from the center to the wheel

void calc_speed(float target_vel_R, float target_vel_L, float* curr_R_vel, float* curr_L_vel, float* duration) 
{
  static uint32_t last_time_in_millis = 0;

  noInterrupts();
  curR = R; R = 0;
  curL = L; L = 0;
  interrupts();

  uint32_t curr_time_in_millis = millis();
  *duration = (curr_time_in_millis - last_time_in_millis)/1000.; // millis to sec

  float milage_R = PI*radius*theta*curR/180.0;
  float milage_L = PI*radius*theta*curL/180.0;
  if (target_vel_R < 0.)  milage_R = -milage_R;
  if (target_vel_L < 0.)  milage_L = -milage_L;

  *curr_R_vel = milage_R / (*duration); // m
  *curr_L_vel = milage_L / (*duration); // m
}

static float target_R_vel = 0.0;
static float target_L_vel = 0.0;
static bool target_vel_update = false;

// odometry pose
static float odm_lin_x = 0.0;
static float odm_ang_z = 0.0;
static float odm_pos_x = 0.0;
static float odm_pos_y = 0.0;
// odometry quaternion
static float odm_qt_qz = 0.0;
static float odm_qt_qw = 0.0;

static const float test_x = 0.00;  // m/sec
static const float test_z = 1.57;
//static const float test_z = 0.00;
static uint32_t running_time = 3000; // ms



void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_SW, OUTPUT);
  digitalWrite(MOTOR_SW, LOW);
  attachInterrupt(R_EN, EncoderR, CHANGE);
  attachInterrupt(L_EN, EncoderL, CHANGE);

  // setup pwm pins
  setup_pwm(0);
  setup_pwm(1);
  setup_pwm(2);
  setup_pwm(3);

  // motor power on
  digitalWrite(MOTOR_SW, HIGH);  // motor power on
  digitalWrite(M_CTRL, HIGH);    // lidar on
  digitalWrite(ROT_ENABLE, LOW); // Rotary encoder on
  sleep(1);

  while (Serial.available()) { Serial.read(); }
  delay(100);

  Serial.println("Waiting for start...");
  while (!Serial.available());

  String t = Serial.readStringUntil('\n');
  Serial.println(t);
  running_time = t.toInt();
  Serial.println("running time is " + String(running_time) + "ms");
}

//#define USE_PID_CONTROL

void motor_control(float linear_x, float angular_z, float* curr_R_vel_out, float* curr_L_vel_out, float* duration_out) 
{
  static int32_t duty_R = 0;
  static int32_t duty_L = 0;

  // calculate target speeds for wheels
  float target_R_vel = linear_x + angular_z*d;
  float target_L_vel = linear_x - angular_z*d;

  // PID coefficients
  const float rKp = 500;     const float lKp = 500;
  const float rKi =  10;     const float lKi =  10;
  const float rKd = 100;     const float lKd = 100;

  float R_err      = 0.;         float L_err      = 0.;
  float diff_R_err = 0.;         float diff_L_err = 0.;
  static float sigma_R_err = 0;  static float sigma_L_err = 0;
  static float last_R_err  = 0;  static float last_L_err  = 0;

  const float limit_speed = 0.01;
  if (abs(target_R_vel) > limit_speed || abs(target_L_vel) > limit_speed) {
    if (abs(target_R_vel) >= abs(target_L_vel)) {
      target_R_vel = target_R_vel/abs(target_R_vel)*limit_speed;
      target_L_vel = target_L_vel/abs(target_R_vel)*limit_speed;
    } else {
      target_L_vel = target_L_vel/abs(target_L_vel)*limit_speed;
      target_R_vel = target_R_vel/abs(target_L_vel)*limit_speed;
    }
  } 
  
  if (target_R_vel == 0.00) { 
    duty_R = 0; last_R_err = 0; sigma_R_err = 0;
  }

  if (target_L_vel == 0.00) {
    duty_L = 0; last_L_err = 0; sigma_L_err = 0;
  }

  float curr_R_vel = 0.;  
  float curr_L_vel = 0.;
  float duration;

  calc_speed(target_R_vel, target_L_vel, &curr_R_vel, &curr_L_vel, &duration);

  // pid calculation
  R_err = target_R_vel - curr_R_vel;
  L_err = target_L_vel - curr_L_vel;
  sigma_R_err += R_err * duration;
  sigma_L_err += L_err * duration;
  diff_R_err   = (R_err - last_R_err) / duration;
  diff_L_err   = (R_err - last_R_err) / duration;
  duty_R += (int32_t)(rKp*R_err + rKi*sigma_R_err + rKi*diff_R_err);
  duty_L += (int32_t)(rKp*L_err + rKi*sigma_L_err + rKi*diff_L_err);
  if (abs(duty_R) > 255) duty_R = duty_R/abs(duty_R)*255;
  if (abs(duty_L) > 255) duty_L = duty_L/abs(duty_L)*255;

  last_R_err = R_err;
  last_L_err = L_err;

  *curr_R_vel_out = curr_R_vel;
  *curr_L_vel_out = curr_L_vel; 
  *duration_out = duration;

  // motor contorl by duty ratio
  uint8_t l = abs(duty_L);
  if (target_L_vel > 0.0) {
    // backward         // forward
    pwm_control(0, 0);  pwm_control(1, l); 
  } else if (target_L_vel < 0.0) {
    pwm_control(0, l);  pwm_control(1, 0);
  } else if (target_L_vel == 0.0) {
    pwm_control(0, 0);  pwm_control(1, 0);  
  }

  uint8_t r = abs(duty_R);
  if (target_R_vel > 0.0) {
    // backward         // forward
    pwm_control(2, 0);  pwm_control(3, r);
  } else if (target_R_vel < 0.0) {
    pwm_control(2, r);  pwm_control(3, 0);    
  } else if (target_R_vel == 0.0) {
    pwm_control(2, 0);  pwm_control(3, 0);     
  }

  //MPLog("target_R_vel: %01.3f cur_R_vel: %01.3f\n", target_R_vel, curr_R_vel);
  //MPLog("target_L_vel: %01.3f cur_L_vel: %01.3f\n", target_L_vel, curr_L_vel);
  //MPLog("duty_R      : %03d   duty_L   : %03d\n"  , duty_R      , duty_L    );

}

struct qua {
  float qx;
  float qy;
  float qz;
  float qw;
};

void quaternion_from_euler(float ai, float aj, float ak, struct qua* q) {
    ai /= 2.0;
    aj /= 2.0;
    ak /= 2.0;
    float ci = arm_cos_f32(ai);
    float si = arm_sin_f32(ai);
    float cj = arm_cos_f32(aj);
    float sj = arm_sin_f32(aj);
    float ck = arm_cos_f32(ak);
    float sk = arm_sin_f32(ak);
    float cc = ci*ck;
    float cs = ci*sk;
    float sc = si*ck;
    float ss = si*sk;

    q->qx = cj*sc - sj*cs;
    q->qy = cj*ss + sj*cc;
    q->qz = cj*cs - sj*sc;
    q->qw = cj*cc + sj*ss;
}

static float ang_z = 0.0;
void update_odometry(float curr_R_vel, float curr_L_vel, float duration) {
  // clac pose
  static float _odm_pos_x = 0.0;
  static float _odm_pos_y = 0.0;
  float last_ang_z = ang_z;
  ang_z += (curr_R_vel - curr_L_vel) * duration / (2.*d) * 0.98;
  if (ang_z > 2.*PI) ang_z -= 2.*PI;
  _odm_pos_x += (curr_R_vel + curr_L_vel)*duration/2.*arm_cos_f32(last_ang_z+ang_z/2);
  _odm_pos_y += (curr_R_vel + curr_L_vel)*duration/2.*arm_sin_f32(last_ang_z+ang_z/2);
  // calc quaternion
  struct qua q;
  quaternion_from_euler(0., 0., ang_z, &q);
  float _odm_qt_qz = q.qz;
  float _odm_qt_qw = q.qw;

  odm_lin_x = (curr_R_vel + curr_L_vel)/2;
  odm_ang_z = (curr_R_vel - curr_L_vel)/(2*d);
  odm_pos_x = _odm_pos_x;
  odm_pos_y = _odm_pos_y;
  odm_qt_qz = _odm_qt_qz;
  odm_qt_qw = _odm_qt_qw;
}

#define UPDATE_CMD_VEL (101)
#define REQ_ODOMETRY   (102)


void loop() {
  static uint32_t start_time = millis();
  float curr_R_vel;
  float curr_L_vel;
  static float linear_x  = 0.0;
  static float angular_z = 0.0;

  // conduct the body control every UPDATE_DURATION
  linear_x = test_x;
  angular_z = test_z;
  float duration = 0.0;
  motor_control(linear_x, angular_z, &curr_R_vel, &curr_L_vel, &duration);
  update_odometry(curr_R_vel, curr_L_vel, duration);
  uint32_t running_duration = millis() - start_time;
  if (running_duration > running_time) {
    linear_x = 0;
    angular_z = 0;
    float duration = 0.0;
    motor_control(linear_x, angular_z, &curr_R_vel, &curr_L_vel, &duration);
    update_odometry(curr_R_vel, curr_L_vel, duration);
    running_duration = millis() - start_time;
    Serial.println("Runnging Time: " + String(running_duration));
    Serial.println("linear   x: " + String(odm_lin_x,4));
    Serial.println("angluar  z: " + String(odm_ang_z,4));
    Serial.println("position x: " + String(odm_pos_x,4));
    Serial.println("position y: " + String(odm_pos_y,4));
    Serial.println("angular  z: " + String(ang_z,4));
    while(1);
  }
  usleep(UPDATE_DURATION_IN_MICRO);
}
