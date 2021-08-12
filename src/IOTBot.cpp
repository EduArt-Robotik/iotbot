#include <iostream>
#include "IOTBot.h"

using namespace std;

namespace iotbot
{

IOTBot::IOTBot(ChassisParams &chassisParams, MotorParams &motorParams)
{
  _motorParams  = new MotorParams(motorParams);
  _chassisParams = chassisParams;
  // ensure that the direction parameter is set properly (either 1 or -1)
  if(_chassisParams.direction>0) _chassisParams.direction = 1;
  else _chassisParams.direction = -1;

  _shield = new IOTShield();
  _shield->setGearRatio(_motorParams->gearRatio);
  _shield->setTicksPerRev(_motorParams->encoderRatio);
  _shield->setKp(_motorParams->kp);
  _shield->setKi(_motorParams->ki);
  _shield->setKd(_motorParams->kd);
  _shield->setControlFrequency(_motorParams->controlFrequency);
  _shield->setLowPassSetPoint(_motorParams->lowPassInputFilter);
  _shield->setLowPassEncoder(_motorParams->lowPassEncoderTicks);
  _shield->setLighting(iotbot::dimLight, _rgb);

  _rad2rpm    = (chassisParams.wheelBase+chassisParams.track)/chassisParams.wheelDiameter; // (lx+ly)/2 * 1/r
  _rpm2rad    = 1.0 / _rad2rpm;
  _ms2rpm     = 60.0/(chassisParams.wheelDiameter*M_PI);
  _rpm2ms     = 1.0 / _ms2rpm;
  _vMax       = motorParams.rpmMax * _rpm2ms;
  _omegaMax   = motorParams.rpmMax * _rpm2rad;

  _subJoy     = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &IOTBot::joyCallback, this);
  _subVel     = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 1, &IOTBot::velocityCallback, this);
  _srvEnable  = _nh.advertiseService("enable", &IOTBot::enableCallback, this);
  _pubToF     = _nh.advertise<std_msgs::Float32MultiArray>("tof", 1);
  _pubRPM     = _nh.advertise<std_msgs::Float32MultiArray>("rpm", 1);
  _pubVoltage = _nh.advertise<std_msgs::Float32>("voltage", 1);
  _pubIMU     = _nh.advertise<sensor_msgs::Imu>("imu", 1);

  _rpm[0] = 0.0;
  _rpm[1] = 0.0;
  _rpm[2] = 0.0;
  _rpm[3] = 0.0;

  _rgb[0] = 0;
  _rgb[1] = 0;
  _rgb[2] = 0;

  ROS_INFO_STREAM("Initialized IOTBot with vMax: " << _vMax << " m/s");
}

IOTBot::~IOTBot()
{
  _shield->setLighting(iotbot::pulsation, _rgb);
  delete _shield;
  delete _motorParams;
}

void IOTBot::run()
{
  ros::Rate rate(100);
  _lastCmd = ros::Time::now();

  bool run = true;

  float r[4];
  float voltage;
  int state = 0;
  unsigned int cnt = 0;
  std_msgs::Float32MultiArray msgToF;
  std_msgs::Float32MultiArray msgRPM;
  std_msgs::Float32           msgVoltage;
  sensor_msgs::Imu            msgIMU;
  while(run)
  {
    ros::spinOnce();

    ros::Time tNow = ros::Time::now();
    ros::Duration dt = tNow - _lastCmd;
    bool lag = (dt.toSec()>0.5);
    if(lag)
    {
      ROS_WARN_STREAM("Lag detected ... deactivate motor control");
      _rpm[0] = 0.0;
      _rpm[1] = 0.0;
      _rpm[2] = 0.0;
      _rpm[3] = 0.0;
      _shield->setRPM(_rpm);
      _shield->disable();
    }
    else
    {
      _shield->setRPM(_rpm);
    }
    
    const std::vector<float> vToF = _shield->getRangeMeasurements();
    msgToF.data = vToF;
    _pubToF.publish(msgToF);

    const std::vector<float> vRPM = _shield->getRPM();
    msgRPM.data = vRPM;
    _pubRPM.publish(msgRPM);

    float voltage = _shield->getSystemVoltage();
    msgVoltage.data = voltage;
    _pubVoltage.publish(msgVoltage);

    const std::vector<float> vAcceleration = _shield->getAcceleration();
    const std::vector<float> vAngularRate  = _shield->getAngularRate();
    msgIMU.header.frame_id = "odom";
    msgIMU.header.stamp    = tNow;
    msgIMU.orientation_covariance      = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
    msgIMU.linear_acceleration.x       = vAcceleration[0];
    msgIMU.linear_acceleration.y       = vAcceleration[1];
    msgIMU.linear_acceleration.z       = -vAcceleration[2];
    msgIMU.angular_velocity.x          = -vAngularRate[0];
    msgIMU.angular_velocity.y          = -vAngularRate[1];
    msgIMU.angular_velocity.z          = vAngularRate[2];
    msgIMU.angular_velocity_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
    _pubIMU.publish(msgIMU);

    rate.sleep();

    run = ros::ok();
  }
}

bool IOTBot::enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
   if(request.data==true)
   {
     ROS_INFO("Enabling robot");
     _shield->enable();
   }
   else
   {
     ROS_INFO("Disabling robot");
     _shield->disable();
   }
   response.success = true;
   return true;
}

void IOTBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Assignment of joystick axes to motor commands
  float fwd      = joy->axes[1];            // Range of values [-1:1]
  float left     = joy->axes[0];            // Range of values [-1:1]
  float turn     = joy->axes[2];            // Range of values [-1:1]
  float throttle = (joy->axes[3]+1.0)/2.0;  // Range of values [0:1]

  // Enable movement in the direction of the y-axis only when the button 12 is pressed
  if(!joy->buttons[11])
    left = 0;

  static int32_t btn0Prev   = joy->buttons[0];
  static int32_t btn1Prev   = joy->buttons[1];
  static int32_t btn2Prev   = joy->buttons[2];
  static int32_t btn3Prev   = joy->buttons[3];
  static int32_t btn4Prev   = joy->buttons[4];
  static int32_t btn5Prev   = joy->buttons[5];
  static int32_t btn10Prev  = joy->buttons[10];

  if(joy->buttons[10] && !btn10Prev)
  {
     ROS_INFO("Enabling robot");
     _shield->enable();
  }
  if(joy->buttons[0] && !btn0Prev)
  {
     ROS_INFO("Setting beam light");
     _shield->setLighting(iotbot::beamLight, _rgb);
  }
  if(joy->buttons[1] && !btn1Prev)
  {
     ROS_INFO("Setting warning light");
     _rgb[0] = 0xFF;
     _rgb[1] = 0x88;
     _rgb[2] = 0x00;
     _shield->setLighting(iotbot::warningLight, _rgb);
  }
  if(joy->buttons[2] && !btn2Prev)
  {
     ROS_INFO("Setting flash light on left side");
     _rgb[0] = 0xFF;
     _rgb[1] = 0x88;
     _rgb[2] = 0x00;
     _shield->setLighting(iotbot::flashLeft, _rgb);
  }
  if(joy->buttons[3] && !btn3Prev)
  {
     ROS_INFO("Setting flash light on right side");
     _rgb[0] = 0xFF;
     _rgb[1] = 0x88;
     _rgb[2] = 0x00;
     _shield->setLighting(iotbot::flashRight, _rgb);
  }
  if(joy->buttons[4] && !btn4Prev)
  {
     ROS_INFO("Setting rotational light");
     _rgb[0] = 0x00;
     _rgb[1] = 0x00;
     _rgb[2] = 0xFF;
     _shield->setLighting(iotbot::rotation, _rgb);
  }
  if(joy->buttons[5] && !btn5Prev)
  {
     ROS_INFO("Setting running light");
     _rgb[0] = 0xFF;
     _rgb[1] = 0x00;
     _rgb[2] = 0x00;
     _shield->setLighting(iotbot::running, _rgb);
  }
  btn0Prev   = joy->buttons[0];
  btn1Prev   = joy->buttons[1];
  btn2Prev   = joy->buttons[2];
  btn3Prev   = joy->buttons[3];
  btn4Prev   = joy->buttons[4];
  btn5Prev   = joy->buttons[5];
  btn10Prev  = joy->buttons[10];

  if(!btn0Prev && !btn1Prev && !btn2Prev && !btn3Prev && !btn4Prev && !btn5Prev)
     _shield->setLighting(iotbot::dimLight, _rgb);
  
  float vFwd  = throttle * fwd  * _vMax;
  float vLeft = throttle * left * _vMax;
  float omega = throttle * turn * _omegaMax;

  controlMotors(vFwd, vLeft, omega);
}

void IOTBot::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

void IOTBot::controlMotors(float vFwd, float vLeft, float omega)
{
  float rpmFwd   = vFwd  * _ms2rpm;
  float rpmLeft  = vLeft * _ms2rpm;
  float rpmOmega = omega * _rad2rpm;
  
  // deactivated movement in y-direction for certain kinematic concepts, e.g., skid steering.
  if(!_chassisParams.enableYMotion)
    rpmLeft = 0;

  // leading signs -> see derivation: Stefan May, Skriptum Mobile Robotik
  _rpm[_chassisParams.chFrontLeft]  =  rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.chFrontRight] = -rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.chRearLeft]   =  rpmFwd + rpmLeft - rpmOmega;
  _rpm[_chassisParams.chRearRight]  = -rpmFwd + rpmLeft - rpmOmega;

  // possibility to flip directions
  _rpm[0] *= _chassisParams.direction;
  _rpm[1] *= _chassisParams.direction;
  _rpm[2] *= _chassisParams.direction;
  _rpm[3] *= _chassisParams.direction;

  // Normalize values, if any value exceeds the maximum
  float rpmMax = std::abs(_rpm[0]);
  for(int i=1; i<4; i++)
  {
    if(std::abs(_rpm[i]) > _motorParams->rpmMax)
      rpmMax = std::abs(_rpm[i]);
  }

  if(rpmMax > _motorParams->rpmMax)
  {
    float factor = _motorParams->rpmMax / rpmMax;
    for(int i=0; i<4; i++)
      _rpm[i] *= factor;
  }

  _lastCmd = ros::Time::now();
}

} // end namespace
