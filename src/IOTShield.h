#ifndef __IOTSHIELD_H
#define __IOTSHIELD_H

#include <vector>
#if _WITH_MRAA
#include "mraa/common.hpp"
#include "mraa/uart.hpp"
#else
#include <stdint.h>
#include <string.h>
#endif

namespace iotbot
{

// Common parameters
#define CMD_ENABLE          0x01
#define CMD_DISABLE         0x02
#define CMD_SETTIMEOUT      0x03
#define CMD_SETPWMMAX       0x04
#define CMD_SENDRPM         0x05
#define CMD_SENDPOS         0x06
#define CMD_INVERTENC       0x07
#define CMD_LOWVOLTAGECHECK 0x08
#define CMD_STALLCHECK      0x09
#define CMD_IMURAWDATA      0x0A
#define CMD_FUSEIMUWEIGHT   0x0B
#define CMD_UARTTIMEOUT     0x0C
#define CMD_IMUCALIBRATE    0x0D
#define CMD_IMUFIXZAXIS     0x0E

// Operating commands
#define CMD_SETPWM          0x10
#define CMD_SETRPM          0x11
#define CMD_FREQ            0x12
#define CMD_SYNC            0x13

// Closed/Open loop controller parameters
#define CMD_CTL_KP          0x20
#define CMD_CTL_KI          0x21
#define CMD_CTL_KD          0x22
#define CMD_CTL_ANTIWINDUP  0x23
#define CMD_CTL_INPUTFILTER 0x24
#define CMD_CTL_ENCLOWPASS  0x25

// Platform parameters
#define CMD_GEARRATIO       0x30 // Ratio of motor gears
#define CMD_TICKSPERREV     0x31 // Ticks per motor revolution ((raising + falling edges) x 2 channels)

#define CMD_AUX1            0x40 // Enable/Disable auxilary power output 1
#define CMD_AUX2            0x41 // Enable/Disable auxilary power output 2
#define CMD_LIGHTS_OFF      0x42 // Switch lights off
#define CMD_DIM_LIGHT       0x43 // Dimmed headlight
#define CMD_HIGH_BEAM       0x44 // High beam headlight
#define CMD_FLASH_ALL       0x45 // Flash lights
#define CMD_FLASH_LEFT      0x46 // Flash lights
#define CMD_FLASH_RIGHT     0x47 // Flash lights
#define CMD_PULSATION       0x48 // Pulsation
#define CMD_ROTATION        0x49 // Rotation light, e.g. police light in blue
#define CMD_RUNNING         0x4A // Running light

enum eLighting {lightsOff    = CMD_LIGHTS_OFF,
                dimLight     = CMD_DIM_LIGHT,
                beamLight    = CMD_HIGH_BEAM,
                warningLight = CMD_FLASH_ALL,
                flashLeft    = CMD_FLASH_LEFT,
                flashRight   = CMD_FLASH_RIGHT,
                pulsation    = CMD_PULSATION,
                rotation     = CMD_ROTATION,
                running      = CMD_RUNNING};

/**
 * @class IOTShield
 * @brief Interface class to IOTShield via UART
 * @author Stefan May
 * @date 04.07.2021
 */
class IOTShield
{
public:
   /**
    * Default Constructor
    */
   IOTShield();

   /**
    * Destructor
    */  
   ~IOTShield();

   /**
    * Enable shield (must be done before steering)
    * @return success
    */
   bool enable();
   
   /**
    * Disable shield (no motion can be performed after disabling)
    * @return success
    */
   bool disable();

   /**
    * Enable/Disable IMU rawdata transmission.
    * @param[in] rawdata true: raw data is transmitted (default), false: fused data as quaternion is transmitted.
    * @return success
    */
   bool setIMURawFormat(bool rawdata);

   /**
    * Fix IMU z-axis
    * @param[in] fix true: fix z-axis, false: enable z-axis (yaw angle estimation)
    * @return success
    */
   bool fixIMUZAxis(bool fix);

   /**
    * Set time without UART communication until an enable state will be removed.
    * @param 
    */
   bool setTimeout(float timeout);

   /**
    * Set weight of drift parameter. Gyro data is used for fast orientation changes,
    * while the accelerometer comensates drift for 2 of the degrees of freedom.
    * @param[in] weight a larger weight increases the belief in the accelerometer values [0; 1], default: 0.03f.
    * return success
    */
   bool setDriftWeight(float weight);

   /**
    * Trigger IMU calibration (takes about 1 second). The platform needs to stand still during this procedure.
    * @return success
    */
   bool calibrateIMU();

   /**
    * Set ration of motor gears
    * @param[in] gearRatio gear ratio
    * @return success
    */
   bool setGearRatio(float gearRatio);
   
   /**
    * Set ticks per revolution of encoders
    * @param[in] ticksPerRev ticks per revoluation (raising and falling edges)
    * @return success
    */
   bool setTicksPerRev(float ticksPerRev);

   /**
    * Set proportional coefficient of closed loop controller
    * @param[in] kp proportional weight
    * @return success
    */
   bool setKp(float kp);

   /**
    * Set integration coefficient of closed loop controller
    * @param[in] ki integration coefficient
    * @return success
    */   
   bool setKi(float ki);

   /**
    * Set differentiating coefficient of closed loop controller
    * @param[in] kd differentiating coefficient
    * @return success
    */   
   bool setKd(float kd);

   /**
    * Set frequency of bridge driver for motor control.
    * param[in] freq frequency in HZ in range [1000, 1000000]
    * return success
    */
   bool setControlFrequency(uint32_t freq);

   /**
    * Set low pass coefficient of set point. New values are weighted with this value.
    * @param[in] weight low pass coefficient of set point
    * @return success
    */  
   bool setLowPassSetPoint(float weight);

   /**
    * Set low pass coefficient of encoder measurements. New values are weighted with this value.
    * @param[in] weight low pass coefficient of encoder measurements
    * @return success
    */ 
   bool setLowPassEncoder(float weight);

   /**
    * Set PWM of motors
    * @param[in] pulse width modulation of motors
    * @return success
    */
   bool setPWM(int8_t pwm[4]);

   /**
    * Set command variable of closed loop controllers for motor control
    * @param[in] rpm revolutions per minute
    * @return success
    */
   bool setRPM(float rpm[4]);

   /**
    * Get actual revolutions per minute
    * @return revolutions per minutes
    */
   const std::vector<float> getRPM();

   /**
    * Set lighting effects
    * @param[in] light lighting effect
    * @param[in] rgb color triple for lighting effect
    * return success
    */
   bool setLighting(eLighting light, unsigned char rgb[3]);
   
   /**
    * Switch on/off auxiliary output (channel 1)
    * @param[in] on on/off state
    * @return success
    */
   bool setAUX1(bool on);
   
   /**
    * Switch on/off auxiliary output (channel 2)
    * @param[in] on on/off state
    * @return success
    */
   bool setAUX2(bool on);
   
   /**
    * Get system voltage
    * @return system voltage [V]
    */
   float getSystemVoltage();
   
   /**
    * Get load current (consumed by IOT2050 and drives)
    * @return load current [A]
    */
   float getLoadCurrent();
   
   /**
    * Get time-of-flight measurements
    * @return ToF measurements
    */
   const std::vector<float> getRangeMeasurements();

   /**
    * Get linear acceleration in g (multiple of 9,81 m/s^2)
    * return 3-dimensional measurement vector
    */
   const std::vector<float> getAcceleration();

   /**
    * Get angular rate in degrees per second
    * return 3-dimensional measurement vector
    */
   const std::vector<float> getAngularRate();
   
   /**
    * Get fused IMU data (accelerometer and gyroscope)
    * return orientation as quaternion (w x y z)
    */
   const std::vector<float> getOrientation();
   
   /**
    * Get onboard temperature measurement.
    * return temperature
    */
   const float getTemperature();
   
private:

   void sendReceive();

#if _WITH_MRAA
   mraa::Uart* _uart;
#endif

   char _txBuf[11];
   
   char _rxBuf[32];
   
   float _systemVoltage;
   
   float _loadCurrent;
   
   std::vector<float> _rpm;
      
   std::vector<float> _ranges;
   
   std::vector<float> _acceleration;

   std::vector<float> _angularRate;
   
   std::vector<float> _q;

   double _timeCom;
   
   bool _rawdata;
   
   float _temperature;
   
};

} // namespace

#endif //__IOTSHIELD_H
