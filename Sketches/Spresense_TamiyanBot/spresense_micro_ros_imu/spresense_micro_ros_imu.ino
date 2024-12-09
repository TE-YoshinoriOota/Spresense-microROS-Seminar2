#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>
sensor_msgs__msg__Imu msg;
sensor_msgs__msg__Imu msg_snd;

#include <MP.h>
#include "BMI270_Arduino.h"
extern BMI270Class BMI270;
extern int8_t configure_sensor(struct bmi2_dev *dev);

typedef struct {
    float q0, q1, q2, q3;  // elements of quaternion
    float beta;            // filter gain
} MadgwickAHRS;

static MadgwickAHRS ahrs;

extern void MadgwickAHRSinit(MadgwickAHRS *ahrs, float beta);
extern void MadgwickAHRSupdateIMU(MadgwickAHRS *ahrs, float gx, float gy, float gz, float ax, float ay, float az);

void imu_error() {
  while (true) {
    static bool output = false;
    digitalWrite(LED2, output = output ? false : true);
    usleep(100000);
  }
}

void setup() {
  int ret;
  ret = BMI270.begin(BMI270_I2C,BMI2_I2C_SEC_ADDR);
  if (ret != BMI2_OK) { imu_error(); }

  ret = configure_sensor();
  if (ret != BMI2_OK) { imu_error(); }

  MadgwickAHRSinit(&ahrs, 0.1); // filter gain is 0.1f

  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);
}

void loop() {
  int8_t recvid;
  void* msgin;
  int ret;

  ret = MP.Recv(&recvid, &msgin);
  if (ret > 0) {
    memcpy(&msg_snd, &msg, sizeof(sensor_msgs__msg__Imu));
    MP.Send(recvid, &msg_snd);
  }

  struct bmi2_sens_float sensor_data;
  ret = BMI270.bmi2_get_sensor_float(&sensor_data);
  if (ret != BMI2_OK) { imu_error(); }

  float ax =  -sensor_data.acc.y;
  float ay =   sensor_data.acc.x;
  float az =   sensor_data.acc.z;
  float gx =  -sensor_data.gyr.y*M_PI/180.; // degree to radian
  float gy =   sensor_data.gyr.x*M_PI/180.;  
  float gz =   sensor_data.gyr.z*M_PI/180.;
  MadgwickAHRSupdateIMU(&ahrs, gx, gy, gz, ax, ay, az);

  msg.orientation.x = ahrs.q0;
  msg.orientation.y = ahrs.q1;
  msg.orientation.z = ahrs.q2;
  msg.orientation.w = ahrs.q3;


  msg.angular_velocity.x = gx;
  msg.angular_velocity.y = gy;
  msg.angular_velocity.z = gz;

  msg.linear_acceleration.x = ax;
  msg.linear_acceleration.y = ay;
  msg.linear_acceleration.z = ax;

  static double orientation_covariance[9] = {
    0.020, 0.000, 0.000,
    0.000, 0.020, 0.000,
    0.000, 0.000, 0.100
  };
  memcpy(msg.orientation_covariance, orientation_covariance, sizeof(double)*9);

  static double angular_velocity_covariance[9] = {
    0.003, 0.000, 0.000,
    0.000, 0.003, 0.000,
    0.000, 0.000, 0.003
  };
  memcpy(msg.angular_velocity_covariance, angular_velocity_covariance, sizeof(double)*9);

  static double linear_acceleration_covariance[9] = {
    0.010, 0.000, 0.000,
    0.000, 0.010, 0.000,
    0.000, 0.000, 0.010
  };
  memcpy(msg.linear_acceleration_covariance, linear_acceleration_covariance, sizeof(double)*9);

  delay(100);
}
