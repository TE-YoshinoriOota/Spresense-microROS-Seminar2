#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>

void setup_odm_msgs(nav_msgs__msg__Odometry* odm) {
  // setup frame id
  static const int name_size = 5;
  static char odom_name[name_size] = {0};
  static const int child_name_size = 16;
  static char child_name[child_name_size] = {0};
  odm->header.frame_id.data = odom_name;
  sprintf(odm->header.frame_id.data, "odom");
  odm->header.frame_id.size = strlen("odom");
  odm->header.frame_id.capacity = name_size;
  odm->child_frame_id.data = child_name;
  sprintf(odm->child_frame_id.data, "base_footprint");
  odm->child_frame_id.size = strlen("base_footprint");
  odm->child_frame_id.capacity = child_name_size;

  // set header time
  uint32_t current_time_in_micros = micros();
  odm->header.stamp.sec = current_time_in_micros/1000000;
  odm->header.stamp.nanosec = (current_time_in_micros - (odm->header.stamp.sec)*1000000)*1000; 
}

void setup_tf_msgs(geometry_msgs__msg__TransformStamped* tf) {
  // setup frame id
  static const int name_size = 5;
  static char odom_name[name_size] = {0};
  static const int child_name_size = 16;
  static char child_name[child_name_size] = {0};
  tf->header.frame_id.data = odom_name;
  sprintf(tf->header.frame_id.data, "odom");
  tf->header.frame_id.size = strlen("odom");
  tf->header.frame_id.capacity = name_size;
  tf->child_frame_id.data = child_name;
  sprintf(tf->child_frame_id.data, "base_footprint");
  tf->child_frame_id.size = strlen("base_footprint");
  tf->child_frame_id.capacity = child_name_size;

  // set header time
  uint32_t current_time_in_micros = micros();
  tf->header.stamp.sec = current_time_in_micros/1000000;
  tf->header.stamp.nanosec = (current_time_in_micros - (tf->header.stamp.sec)*1000000)*1000; 
}

void setup_imu_msgs(sensor_msgs__msg__Imu* imu) {
  // setup frame id
  static const int name_size = 16;
  static char imu_name[name_size];
  imu->header.frame_id.data = imu_name;
  sprintf(imu->header.frame_id.data, "imu_link");
  imu->header.frame_id.size = strlen("imu_link");
  imu->header.frame_id.capacity = name_size;

  // setup header time
  uint32_t current_time_in_micros = micros();
  imu->header.stamp.sec = current_time_in_micros/1000000;
  imu->header.stamp.nanosec = (current_time_in_micros - (imu->header.stamp.sec)*1000000)*1000; 
}

void setup_scan_msgs(sensor_msgs__msg__LaserScan* scan) {
  static uint32_t last_time_in_micros = 0;
  static const int scan_buf_size = 360;

  // setup frame id
  static const int name_size = 16;
  static char scan_name[name_size];
  scan->header.frame_id.data = scan_name;
  sprintf(scan->header.frame_id.data, "scan_link");
  scan->header.frame_id.size = strlen("scan_link");
  scan->header.frame_id.capacity = name_size;
  scan->range_max = 5.00;
  scan->range_min = 0.02;
  scan->angle_min = 0.0;    // first_angle*PI/180.0;
  scan->angle_max = 2.*PI;  // last_angle*PI/180.0; 
  scan->angle_increment = 2.*PI/scan_buf_size;
  scan->ranges.size = scan_buf_size*sizeof(float); // lsn*sizeof(float);
  scan->ranges.capacity = scan_buf_size*sizeof(float); // rx_buf_size*sizeof(float);
  scan->time_increment = 0.0;

  // set header time
  uint32_t current_time_in_micros = micros();
  scan->header.stamp.sec = current_time_in_micros/1000000;
  scan->header.stamp.nanosec = (current_time_in_micros - (scan->header.stamp.sec)*1000000)*1000; 
  scan->scan_time = float(current_time_in_micros - last_time_in_micros)/1000000;
}