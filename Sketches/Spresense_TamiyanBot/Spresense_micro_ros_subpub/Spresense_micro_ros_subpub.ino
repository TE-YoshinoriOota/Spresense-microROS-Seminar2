#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

static rcl_allocator_t rcl_allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t cmd_subscriber;
static rcl_publisher_t odm_publisher;
static rcl_publisher_t imu_publisher;
static rcl_publisher_t scan_publisher;
static rcl_publisher_t tf_publisher;
static rclc_executor_t executor;
static rcl_timer_t odm_timer;
static rcl_timer_t imu_timer;
static rcl_timer_t scan_timer;

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
geometry_msgs__msg__Twist cmd_vel;
geometry_msgs__msg__TransformStamped tf;
nav_msgs__msg__Odometry odm;
sensor_msgs__msg__Imu imu;
sensor_msgs__msg__LaserScan scan;

//#include <uxr/client/transport.h>
//#include <rmw_microros/rmw_microros.h>

// error check for micro_ros functions 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
void error_loop() { while(1){ digitalWrite(LED0, !digitalRead(LED0)); delay(1000); }}


#include <MP.h>
#include <MPMutex.h>
const uint8_t odm_core  = 1;
const uint8_t imu_core  = 2;
const uint8_t scan_core = 3;
uint8_t dummy = 100;
MPMutex mutex0(MP_MUTEX_ID0);
MPMutex mutex1(MP_MUTEX_ID1);
MPMutex mutex2(MP_MUTEX_ID2);

extern void setup_odm_msgs(nav_msgs__msg__Odometry* odm);
extern void setup_imu_msgs(sensor_msgs__msg__Imu* imu);
extern void setup_scan_msgs(sensor_msgs__msg__LaserScan* scan);

const bool debug_led = true;

#define UPDATE_CMD_VEL (101)
#define REQ_ODOMETRY   (102)

void cmd_vel_callback(const void * msgin) {
  if (debug_led) {
    static bool output = false;
    digitalWrite(LED0, output = output ? false : true);
  }
  geometry_msgs__msg__Twist* cmd_ = (geometry_msgs__msg__Twist*)msgin;
  static geometry_msgs__msg__Twist cmdout;
  memcpy(&cmdout, cmd_, sizeof(geometry_msgs__msg__Twist));
  int8_t sndid = UPDATE_CMD_VEL;
  MP.Send(sndid, &cmdout, odm_core);
}

void odm_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  if (debug_led) {
    static bool output = false;
    digitalWrite(LED1, output = output ? false : true);   
  }

  int8_t sndid = REQ_ODOMETRY;
  static nav_msgs__msg__Odometry* odm_rcv;
  MP.Send(sndid, dummy, odm_core);
  MP.Recv(&sndid, &odm_rcv, odm_core);
  memcpy(&odm, odm_rcv, sizeof(nav_msgs__msg__Odometry));

  tf.transform.translation.x = odm.pose.pose.position.x;
  tf.transform.translation.y = odm.pose.pose.position.y;
  tf.transform.rotation.w = odm.pose.pose.orientation.w;
  tf.transform.rotation.z = odm.pose.pose.orientation.z;

  setup_tf_msgs(&tf);
  setup_odm_msgs(&odm);

  //RCSOFTCHECK(rcl_publish(&tf_publisher, &tf, NULL));
  RCSOFTCHECK(rcl_publish(&odm_publisher, &odm, NULL));
}


void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;
  int ret;

  if (debug_led) {
    static bool output = false;
    digitalWrite(LED2, output = output ? false : true);
  }

  // get imu data from subcore
  int8_t sndid = 100;
  static sensor_msgs__msg__Imu* imu_rcv;
  MP.Send(sndid, dummy, imu_core);
  MP.Recv(&sndid, &imu_rcv, imu_core);
  memcpy(&imu, imu_rcv, sizeof(sensor_msgs__msg__Imu));

  setup_imu_msgs(&imu);
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu, NULL));
}

void scan_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  if (debug_led) {
    static bool output = false;
    digitalWrite(LED3, output = output ? false : true);
  }

  // get scan data from subcore
  int8_t sndid = 100;
  static const int scan_buf_size = 360;
  static float *data;
  static float scan_buf[scan_buf_size] = {0};
  static float intensities[scan_buf_size] = {0};
  MP.Send(sndid, dummy, scan_core);
  MP.Recv(&sndid, &data, scan_core);
  memcpy(&scan_buf[0], &data[0], sizeof(float)*360);
  setup_scan_msgs(&scan);
  scan.ranges.size = scan_buf_size;
  scan.ranges.capacity = scan_buf_size;
  scan.ranges.data = &scan_buf[0];
  scan.intensities.size = scan_buf_size;
  scan.intensities.capacity = scan_buf_size;
  scan.intensities.data = &intensities[0];


  RCSOFTCHECK(rcl_publish(&scan_publisher, &scan, NULL));
}

void setup() {
  set_microros_transports();  
  delay(2000);
  
  rcl_allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &rcl_allocator));   
  RCCHECK(rclc_node_init_default(&node, "my_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&cmd_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(&odm_publisher,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  RCCHECK(rclc_publisher_init_default(&imu_publisher,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));

  RCCHECK(rclc_publisher_init_default(&scan_publisher,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));

  RCCHECK(rclc_publisher_init_default(&tf_publisher,  &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped), "tf"));

  const uint32_t imu_timer_timeout = 30;
  RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(imu_timer_timeout), imu_timer_callback));
  const uint32_t scan_timer_timeout = 150;
  RCCHECK(rclc_timer_init_default(&scan_timer, &support, RCL_MS_TO_NS(scan_timer_timeout), scan_timer_callback));
  const uint32_t odm_timer_timeout = 30;
  RCCHECK(rclc_timer_init_default(&odm_timer, &support, RCL_MS_TO_NS(odm_timer_timeout), odm_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &rcl_allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odm_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &scan_timer));

  MP.begin(odm_core);
  MP.begin(imu_core);
  MP.begin(scan_core);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
  //usleep(100000);
}
