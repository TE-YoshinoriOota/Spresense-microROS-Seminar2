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

/*
  msg.linear_acceleration.x = sensor_data.acc.x;
  msg.linear_acceleration.y = sensor_data.acc.y;
  msg.linear_acceleration.z = sensor_data.acc.z;
  msg.angular_velocity.x = sensor_data.gyr.x;
  msg.angular_velocity.y = sensor_data.gyr.y;
  msg.angular_velocity.z = sensor_data.gyr.z;
*/
  msg.linear_acceleration.x = -sensor_data.acc.y;
  msg.linear_acceleration.y = sensor_data.acc.x;
  msg.linear_acceleration.z = sensor_data.acc.z;
  msg.angular_velocity.x = -sensor_data.gyr.y;
  msg.angular_velocity.y = sensor_data.gyr.x;
  msg.angular_velocity.z = sensor_data.gyr.z;

  delay(100);
}
