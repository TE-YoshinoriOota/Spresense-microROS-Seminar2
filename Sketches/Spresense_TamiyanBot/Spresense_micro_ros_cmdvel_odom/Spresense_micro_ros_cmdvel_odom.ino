#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

/* Use CMSIS library */
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <arm_math.h>
#include <math.h>

#include <MP.h>

#define MOTOR_SW 12
#define M_CTRL   8
#define ROT_ENABLE 4
#define R_EN 11
#define L_EN 10
#define UPDATE_DURATION_IN_MICRO (5000) // 5 msec

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


static float target_R_vel = 0.0;
static float target_L_vel = 0.0;
static bool target_vel_update = false;

// odometry pose
static float odm_lin_x = 0.0;
static float odm_ang_z = 0.0;
static float odm_pos_x = 0.0;
static float odm_pos_y = 0.0;
static float odm_pos_z = 0.036;
// odometry quaternion
static float odm_qt_qx = 0.0;
static float odm_qt_qy = 0.0;
static float odm_qt_qz = 0.0;
static float odm_qt_qw = 0.0;

// quaternion struct
struct qua {
  float qx;
  float qy;
  float qz;
  float qw;
};


void setup() {
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

  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);
}

// rover wheel constants
const float radius = 0.016; // m
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

  float milage_R = M_PI*radius*theta*curR/180.0;
  float milage_L = M_PI*radius*theta*curL/180.0;
  if (target_vel_R < 0.)  milage_R = -milage_R;
  if (target_vel_L < 0.)  milage_L = -milage_L;

  *curr_R_vel = milage_R / (*duration); // m
  *curr_L_vel = milage_L / (*duration); // m
}


//#define USE_PID_CONTROL
void motor_control(float linear_x, float angular_z, float* curr_R_vel_out, float* curr_L_vel_out, float* duration_out) 
{
  // calculate target speeds for wheels
  float target_R_vel = linear_x + angular_z*d;
  float target_L_vel = linear_x - angular_z*d;

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

  float curr_R_vel = 0.;  
  float curr_L_vel = 0.;
  float duration;

  calc_speed(target_R_vel, target_L_vel, &curr_R_vel, &curr_L_vel, &duration);

  *curr_R_vel_out = curr_R_vel;
  *curr_L_vel_out = curr_L_vel; 
  *duration_out = duration;

  // motor contorl by fix duty
  uint8_t l = 240;
  uint8_t r = 180;
  if (target_L_vel > 0.0 && target_R_vel > 0.0) { // forward
    pwm_control(0, 0);  pwm_control(1, 255);
    pwm_control(2, 0);  pwm_control(3, 255);
    delayMicroseconds(100); 
    pwm_control(0, 0);  pwm_control(1, l); 
    pwm_control(2, 0);  pwm_control(3, r);

  } else if (target_L_vel > 0.0 && target_R_vel < 0.0) { // cw
    pwm_control(0, 0);  pwm_control(1, 255);
    pwm_control(2, 255);  pwm_control(3, 0);
    delayMicroseconds(100); 
    pwm_control(0, 0);  pwm_control(1, l); 
    pwm_control(2, r);  pwm_control(3, 0);    
  } else if (target_L_vel < 0.0 && target_R_vel > 0.0) { // ccw
    pwm_control(0, 255);  pwm_control(1, 0);
    pwm_control(2, 0);  pwm_control(3, 255);
    delayMicroseconds(100); 
    pwm_control(0, l);  pwm_control(1, 0);
    pwm_control(2, 0);  pwm_control(3, r);
  } else if (target_L_vel < 0.0 && target_R_vel < 0.0) { // backwards
    pwm_control(0, 255);  pwm_control(1, 0);
    pwm_control(2, 255);  pwm_control(3, 0);
    delayMicroseconds(100); 
    pwm_control(0, l);  pwm_control(1, 0);
    pwm_control(2, r);  pwm_control(3, 0);    
  } else if (target_L_vel == 0.0 && target_R_vel == 0.0) { // stop
    pwm_control(0, 0);  pwm_control(1, 0);  
    pwm_control(2, 0);  pwm_control(3, 0);     
    delayMicroseconds(100); 
  }
  //MPLog("target_R_vel: %01.3f cur_R_vel: %01.3f\n", target_R_vel, curr_R_vel);
  //MPLog("target_L_vel: %01.3f cur_L_vel: %01.3f\n", target_L_vel, curr_L_vel);
  //MPLog("duty_R      : %03d   duty_L   : %03d\n"  , duty_R      , duty_L    );

}

void quaternion_from_euler(float roll, float pitch, float yaw, struct qua* q) {
    float cr = arm_cos_f32(roll * 0.5f);
    float sr = arm_sin_f32(roll * 0.5f);
    float cp = arm_cos_f32(pitch * 0.5f);
    float sp = arm_sin_f32(pitch * 0.5f);
    float cy = arm_cos_f32(yaw * 0.5f);
    float sy = arm_sin_f32(yaw * 0.5f);

    q->qw = cr * cp * cy + sr * sp * sy;
    q->qx = sr * cp * cy - cr * sp * sy;
    q->qy = cr * sp * cy + sr * cp * sy;
    q->qz = cr * cp * sy - sr * sp * cy;
}

void update_odometry(float curr_R_vel, float curr_L_vel, float duration) {
  // calc pose
  static float pos_x = 0.0;
  static float pos_y = 0.0;
  static float ang_z = 0.0;

  float v = (curr_R_vel + curr_L_vel) / 2.;
  float delta_x = v*duration*arm_cos_f32(ang_z);
  float delta_y = v*duration*arm_sin_f32(ang_z);

  //float last_ang_z = ang_z;
  float omega = (curr_R_vel - curr_L_vel) / (2.*d - 0.01);  // 0.01 is adjustment valueIM
  float delta_angle = omega * duration;

  pos_x += delta_x;
  pos_y += delta_y;
  ang_z += delta_angle;
  if (ang_z >=  2.*M_PI) ang_z -= 2.*M_PI;
  if (ang_z <= -2.*M_PI) ang_z += 2.*M_PI;

  // calc quaternion
  struct qua q;
  quaternion_from_euler(0., 0., ang_z, &q);

  odm_lin_x = v;
  odm_ang_z = omega;
  odm_pos_x = pos_x;
  odm_pos_y = pos_y;
  odm_qt_qx = q.qx;
  odm_qt_qy = q.qy;
  odm_qt_qz = q.qz;
  odm_qt_qw = q.qw;
}

#define UPDATE_CMD_VEL (101)
#define REQ_ODOMETRY   (102)

void loop() {
  
  float curr_R_vel;
  float curr_L_vel;
  static float linear_x  = 0.0;
  static float angular_z = 0.0;

  // conduct the body control every UPDATE_DURATION
  float duration = 0.0;
  motor_control(linear_x, angular_z, &curr_R_vel, &curr_L_vel, &duration);
  update_odometry(curr_R_vel, curr_L_vel, duration);

  int8_t recvid;
  void* msgin;
  int ret;
  ret = MP.Recv(&recvid, &msgin);
  if (ret > 0) {
    switch(recvid) {
    case UPDATE_CMD_VEL:
      {
        if (debug_led) {
          static bool output = false;
          digitalWrite(LED0, output = output ? false : true);
        }
        geometry_msgs__msg__Twist* cmd_vel = (geometry_msgs__msg__Twist*)msgin;
        linear_x  = cmd_vel->linear.x;
        angular_z = cmd_vel->angular.z;
      }
      break;
    case REQ_ODOMETRY:
      {
        /*
        pose.covariance = [  
          pos_x,   pos_xy,  pos_xz,  pos_xrx,  pos_xry,  pos_xrz,  
          pos_yx,  pos_y,   pos_yz,  pos_yrx,  pos_yry,  pos_yrz,  
          pos_zx,  pos_zy,  pos_z,   pos_zrx,  pos_zry,  pos_zrz,  
          pos_rx,  pos_ryx, pos_rz,  pos_rxx,  pos_rxy,  pos_rxz,  
          pos_ry,  pos_ryy, pos_ryz, pos_ryyx, pos_ryy,  pos_ryz,  
          pos_rz,  pos_rzy, pos_rzz, pos_rzx,  pos_rzyx, pos_rzz
        ]
        */
        static double pose_covariance[36] = {
          0.05000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000,
          0.00000, 0.05000, 0.00000, 0.00000, 0.00000, 0.00000,
          0.00000, 0.00000, 0.10000, 0.00000, 0.00000, 0.00000,
          0.00000, 0.00000, 0.00000, 0.10000, 0.00000, 0.00000,
          0.00000, 0.00000, 0.00000, 0.00000, 0.10000, 0.00000,
          0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.20000,
        }; 

        /*
        twist.covariance = [  
          vel_x,  vel_xy,  vel_xz,  vel_xrx, vel_xry,  vel_xrz,  
          vel_yx, vel_y,   vel_yz,  vel_yrx, vel_yry,  vel_yrz,  
          vel_zx, vel_zy,  vel_z,   vel_zrx, vel_zry,  vel_zrz,  
          vel_rx, vel_ryx, vel_rz,  vel_rxx, vel_rxy,  vel_rxz,  
          vel_ry, vel_ryy, vel_ryz, vel_ryyx,vel_ryy,  vel_ryz,  
          vel_rz, vel_rzy, vel_rzz, vel_rzx, vel_rzyx, vel_rzz
        ]
        */
        static double twist_covariance[36] = {
          0.10000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000,
          0.00000, 0.10000, 0.00000, 0.00000, 0.00000, 0.00000,
          0.00000, 0.00000, 9999.00, 0.00000, 0.00000, 0.00000,
          0.00000, 0.00000, 0.00000, 0.20000, 0.00000, 0.00000,
          0.00000, 0.00000, 0.00000, 0.00000, 0.20000, 0.00000,
          0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.30000,
        }; 

        static nav_msgs__msg__Odometry odm;
        odm.twist.twist.linear.x = odm_lin_x;
        odm.twist.twist.angular.z = odm_ang_z;
        memcpy(odm.twist.covariance, twist_covariance, sizeof(double)*36);
        odm.pose.pose.position.x = odm_pos_x;
        odm.pose.pose.position.y = odm_pos_y;
        odm.pose.pose.position.z = odm_pos_z;
        odm.pose.pose.orientation.x = odm_qt_qx;
        odm.pose.pose.orientation.y = odm_qt_qy;
        odm.pose.pose.orientation.z = odm_qt_qz;
        odm.pose.pose.orientation.w= odm_qt_qw;
        memcpy(odm.pose.covariance, pose_covariance, sizeof(double)*36);

        MP.Send(recvid, &odm);
      }
      break;
    }
  }
  usleep(UPDATE_DURATION_IN_MICRO);
}
