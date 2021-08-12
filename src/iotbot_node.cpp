#include <ros/ros.h>
#include <ros/console.h>

#include "IOTBot.h"

/**
 * @author Stefan May
 * @date 08.05.2021
 * @brief ROS node for the IOT2050 robot shield, Version 2
 **/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "iotbot_node");
  iotbot::ChassisParams chassisParams;
  iotbot::MotorParams motorParams;

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  nh.param("track",               chassisParams.track,                 0.3f);
  nh.param("wheelBase",           chassisParams.wheelBase,             0.3f);
  nh.param("wheelDiameter",       chassisParams.wheelDiameter,         0.15f);
  nh.param("chFrontLeft",         chassisParams.chFrontLeft,           2);
  nh.param("chFrontRight",        chassisParams.chFrontRight,          1);
  nh.param("chRearLeft",          chassisParams.chRearLeft,            3);
  nh.param("chRearRight",         chassisParams.chRearRight,           0);
  nh.param("direction",           chassisParams.direction,            -1);
  nh.param("enableYMotion",       chassisParams.enableYMotion,         0);
  nh.param("gearRatio",           motorParams.gearRatio,              70.f);
  nh.param("encoderRatio",        motorParams.encoderRatio,           64.f);
  nh.param("rpmMax",              motorParams.rpmMax,                140.f);
  nh.param("controlFrequency",    motorParams.controlFrequency,    16000);
  nh.param("kp",                  motorParams.kp,                      1.f);
  nh.param("ki",                  motorParams.ki,                      0.f);
  nh.param("kd",                  motorParams.kd,                      0.f);
  nh.param("lowPassInputFilter",  motorParams.lowPassInputFilter,      1.f);
  nh.param("lowPassEncoderTicks", motorParams.lowPassEncoderTicks,     1.f);

  iotbot::IOTBot robot(chassisParams, motorParams);
  robot.run();
}
