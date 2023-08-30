#if (SUBCORE != 3)
#error "Core selection is wrong!!"
#endif

#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/laser_scan.h>
sensor_msgs__msg__LaserScan msg;

#include <MP.h>

#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>

#define PH_LSB   (0)
#define PH_MSB   (1)
#define PH_SIZE  (2)
#define CT       (2)
#define CT_SIZE  (1)
#define LSN      (3)
#define LSN_SIZE (1)
#define FSA_LSB  (4)
#define FSA_MSB  (5)
#define FSA_SIZE (2)
#define LSA_LSB  (6)
#define LSA_MSB  (7)
#define LSA_SIZE (2)
#define CS_LSB   (8)
#define CS_MSB   (9)
#define CS_SIZE  (2)
#define HEADER_SIZE (CS_MSB+1)

#define PI (3.14159265359)

// #define X2_DEBUG

#define SW_SERIAL
#ifdef SW_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial mySerial(7,8); // RX, TX
#else
HardwareSerial &mySerial = Serial2;
#endif

const int rx_buf_size = 1024;
uint8_t rx_buf[rx_buf_size];
uint16_t rx_ptr = 0;

const int range_size = 360;
uint8_t last_value = 0;
float ranges[range_size];
float outbuf[range_size];

const bool x2_debug = false;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);

  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);
}


void loop() {

  if (!mySerial.available()) return;
  while (mySerial.available()) {
    uint8_t value = mySerial.read();

    rx_buf[rx_ptr++] = value;
    if (rx_ptr >= rx_buf_size) rx_ptr = 0;

    if (rx_buf[rx_ptr-1] == 0x55 && rx_buf[rx_ptr-2] == 0xaa) {
      if (rx_ptr > HEADER_SIZE && rx_buf[PH_MSB] == 0x55 && rx_buf[PH_LSB] == 0xaa) {
        /*** rx_buf[0]=0xaa, rx_buf[1]=0x55 .... rx_buf[rx_ptr-2]=0xaa, rx_buf[rx_ptr-1]=0x55 ***/
        uint16_t length = rx_ptr-PH_SIZE-HEADER_SIZE; 
        float first_angle = float(((rx_buf[FSA_MSB] << 8) | rx_buf[FSA_LSB]) >> 1)/64.;
        float last_angle  = float(((rx_buf[LSA_MSB] << 8) | rx_buf[LSA_LSB]) >> 1)/64.;
        float diff_angle = last_angle - first_angle;
        if (diff_angle < 360.0) diff_angle = 360.0 + diff_angle;
        float tick_angle = diff_angle / ((length/2)-1);

        uint16_t lsn = rx_buf[LSN];
        if (x2_debug) if (lsn != length/2) MPLog("Warining: data length and lsn are not match! [%d:%d]\n", length/2, lsn);
        
        uint16_t csum = (rx_buf[CS_MSB] << 8) | rx_buf[CS_LSB];
        uint16_t bcc = calc_crc(&rx_buf[0], length);
        // if (bcc == csum && lsn == length/2) {
        // Test: even if the lsn and the length are no match, the process is forcibly going
        if (bcc == csum) {
          float diff_angle = last_angle - first_angle;
          if (diff_angle < 0) diff_angle = 360.0 + diff_angle;
          float angle_increment = diff_angle/(lsn-1);

          int i = 0;
          for (int n = 0; n < length; n += 2) {
            float cur_angle = angle_increment*(i++) + first_angle;
            float distance = float(rx_buf[HEADER_SIZE+n] | (rx_buf[HEADER_SIZE+n+1] << 8)) / 4000;  // meter
            if (distance == 0.0) continue;
            float y = 21.8*(155.3-distance);
            float x = 155.3*distance;
            float correct_angle = atan2(y, x);
            correct_angle = correct_angle*180.0/PI;  // rad to degree
            cur_angle = cur_angle + correct_angle;
            if (cur_angle > 360) cur_angle = cur_angle-360.0;
            int int_angle = floorf(cur_angle);
            ranges[int_angle] = distance;
            if (x2_debug) MPLog("%d, %f, %f\n", cur_angle, int_angle, ranges[int_angle]);
          }
          
          int8_t recvid;
          void* msgin;
          int ret;
          ret = MP.Recv(&recvid, &msgin);
          if (ret > 0) {
            //memcpy(&outbuf[0], &ranges[0], 360*sizeof(float));
            int n = 359;
            for (int i = 0; i < 360; ++i) {
              outbuf[n--] = ranges[i]; // ranges is cw, but scan topic is ccw
            }
            MP.Send(recvid, &outbuf);
            if (x2_debug)  for (int n = 0; n < 360; ++n) MPLog("[%d] %f\n", n, ranges[n]); 
            memset(&ranges[0], 0, sizeof(float)*360);
          }
        } else {
          if (x2_debug) MPLog("Warning: CRC check error: %d:%d\n", bcc, csum);
        }
      }
      rx_ptr = 0;
      memset(&rx_buf[0], 0, rx_buf_size);
      //memset(&ranges[0], 0, range_size*sizeof(float));
      rx_buf[rx_ptr++] = last_value;  // 0xaa
      rx_buf[rx_ptr++] = value;       // 0x55
    }
    last_value = value;
  }
}

uint16_t calc_crc(uint8_t* msg, uint16_t data_size) {
  uint16_t crc = 0x0000;
  uint16_t data_length = data_size / 2;
  uint16_t *msg_ptr = (uint16_t*)msg;
  uint16_t *data_ptr = (uint16_t*)(msg+HEADER_SIZE);
  static const int msg_length = (HEADER_SIZE - CS_SIZE) / 2; // 4
  for (uint16_t i = 0; i < msg_length; i++) crc ^= *(msg_ptr++);
  for (uint16_t i = 0; i < data_length; i++) crc ^= *(data_ptr++);
  return crc;
}
