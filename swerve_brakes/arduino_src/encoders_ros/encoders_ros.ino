#include <AS5600.h>
#include <Wire.h>
#include <multi_channel_relay.h>
#include "TCA9548A.h"

#include <ros.h>

#include <swerve_msgs/TriggerBrakes.h>
#include <std_msgs/Float32MultiArray.h>

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#define SYS_VOL 3.3
#else
#define SERIAL Serial
#define SYS_VOL 5
#endif

TCA9548A<TwoWire> TCA;
AMS_5600 sensor_0, sensor_1, sensor_2, sensor_3;

bool magnet_0_detected, magnet_1_detected, magnet_2_detected, magnet_3_detected = false;

Multi_Channel_Relay relay;

ros::NodeHandle nh;

std_msgs::Float32MultiArray jointAngles;
ros::Publisher p("/brake_joint_angles_raw", &jointAngles);

using swerve_msgs::TriggerBrakes;
void callback(const TriggerBrakes::Request& req, TriggerBrakes::Response& res) {
  if (req.front_right) {
    relay.turn_on_channel(1);
  } else {
    relay.turn_off_channel(1);
  }
  if (req.back_left) {
    relay.turn_on_channel(2);
  } else {
    relay.turn_off_channel(2);
  }
  if (req.back_right) {
    relay.turn_on_channel(3);
  } else {
    relay.turn_off_channel(3);
  }
  if (req.front_left) {
    relay.turn_on_channel(4);
  } else {
    relay.turn_off_channel(4);
  }
  res.success = true;
}
ros::ServiceServer<TriggerBrakes::Request, TriggerBrakes::Response> server("brakes", &callback);

void setup() {
  SERIAL.begin(57600);
  TCA.begin(Wire);
  relay.begin(0x11);
  jointAngles.data_length = 4;
  jointAngles.data = (float*)malloc(sizeof(float) * jointAngles.data_length);
  jointAngles.data[0] = -1;
  jointAngles.data[1] = -1;
  jointAngles.data[2] = -1;
  jointAngles.data[3] = -1;

  nh.initNode();
  nh.logwarn("init node...");
  nh.advertise(p);
  nh.advertiseService(server);
  while (1) {
    nh.logwarn("wait for magnets...");
    TCA.openChannel(TCA_CHANNEL_4);
    magnet_0_detected = (sensor_0.detectMagnet() == 1);
    TCA.closeAll();

    TCA.openChannel(TCA_CHANNEL_5);
    magnet_1_detected = (sensor_1.detectMagnet() == 1);
    TCA.closeAll();

    TCA.openChannel(TCA_CHANNEL_6);
    magnet_2_detected = (sensor_2.detectMagnet() == 1);
    TCA.closeAll();

    TCA.openChannel(TCA_CHANNEL_7);
    magnet_3_detected = (sensor_3.detectMagnet() == 1);
    TCA.closeAll();

    if (magnet_0_detected && magnet_1_detected && magnet_2_detected && magnet_3_detected) {
      break;
    }
    p.publish(&jointAngles);
    nh.spinOnce();
  }
}
/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  return newAngle * 0.087;
}
void loop() {
  TCA.openChannel(TCA_CHANNEL_4);
  jointAngles.data[0] = convertRawAngleToDegrees(sensor_0.getRawAngle());
  TCA.closeAll();

  TCA.openChannel(TCA_CHANNEL_5);
  jointAngles.data[1] = convertRawAngleToDegrees(sensor_1.getRawAngle());
  TCA.closeAll();

  TCA.openChannel(TCA_CHANNEL_6);
  jointAngles.data[2] = convertRawAngleToDegrees(sensor_2.getRawAngle());
  TCA.closeAll();

  TCA.openChannel(TCA_CHANNEL_7);
  jointAngles.data[3] = convertRawAngleToDegrees(sensor_3.getRawAngle());
  TCA.closeAll();

  p.publish(&jointAngles);
  nh.spinOnce();
}
