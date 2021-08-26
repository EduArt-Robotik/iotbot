#include "IOTShield.h"
#include <iostream>
#include <sys/time.h>
#include <unistd.h>

namespace iotbot
{

void byteArrayToFloat(int8_t* byteArray, float* floatVar)
{
	uint32_t* var = (uint32_t*)floatVar;
	*var          = byteArray[0] << 24;
	*var         |= byteArray[1] << 16;
	*var         |= byteArray[2] << 8;
	*var         |= byteArray[3];
}

void floatToByteArray(float* floatVar, int8_t* byteArray)
{
   uint8_t* var = (uint8_t*)(floatVar);
   memcpy(byteArray, var, sizeof(floatVar));
}

void intToByteArray(uint32_t* iVar, int8_t* byteArray)
{
   uint8_t* var = (uint8_t*)(iVar);
   memcpy(byteArray, var, sizeof(iVar));
}

const char* devPath = "/dev/ttyS1";

IOTShield::IOTShield()
{
   _timeCom = 0.0;

   _rpm.resize(4);
   _ranges.resize(4);
   _acceleration.resize(3);
   _angularRate.resize(3);

#if _WITH_MRAA
   _uart = new mraa::Uart(devPath);

   if (_uart->setBaudRate(115200) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setFlowcontrol(false, false) != mraa::SUCCESS) {
      std::cerr << "Error setting flow control UART" << std::endl;
   }
   
   _uart->flush();
#else
   std::cerr << "UART interface not available. MRAA is missing!" << std::endl;
#endif
}
   
IOTShield::~IOTShield()
{
#if _WITH_MRAA
   delete _uart;
#endif
}

bool IOTShield::enable()
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_ENABLE;
   _txBuf[10] = 0xEE;
   sendReceive();
   return 1;
}

bool IOTShield::disable()
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_DISABLE;
   _txBuf[10] = 0xEE;
   sendReceive();
   return 1;
}

bool IOTShield::setGearRatio(float gearRatio)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_GEARRATIO;
   floatToByteArray(&gearRatio, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}
   
bool IOTShield::setTicksPerRev(float ticksPerRev)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_TICKSPERREV;
   floatToByteArray(&ticksPerRev, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}

bool IOTShield::setKp(float kp)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_CTL_KP;
   floatToByteArray(&kp, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}
   
bool IOTShield::setKi(float ki)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_CTL_KI;
   floatToByteArray(&ki, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}
      
bool IOTShield::setKd(float kd)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_CTL_KD;
   floatToByteArray(&kd, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}

bool IOTShield::setControlFrequency(uint32_t freq)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_FREQ;
   intToByteArray(&freq, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}

bool IOTShield::setLowPassSetPoint(float weight)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_CTL_INPUTFILTER;
   floatToByteArray(&weight, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}

bool IOTShield::setLowPassEncoder(float weight)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_CTL_ENCLOWPASS;
   floatToByteArray(&weight, (int8_t*)&(_txBuf[2]));
   sendReceive();
   return 1;
}

bool IOTShield::setPWM(int8_t pwm[4])
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_SETPWM;
   memcpy(&(_txBuf[2]), pwm, 4);
   sendReceive();
   return 1;
}
   
bool IOTShield::setRPM(float rpm[4])
{
   int16_t irpm[4];
   for(int i=0; i<4; i++)
   	irpm[i] = ((int16_t)(rpm[i]*100.f+0.5f));
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_SETRPM;
   memcpy(&(_txBuf[2]), irpm, 4*sizeof(int16_t));
   sendReceive();
   return 1;
}

const std::vector<float> IOTShield::getRPM()
{
   return _rpm;
}

bool IOTShield::setLighting(eLighting light, unsigned char rgb[3])
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = light;
   _txBuf[2] = rgb[0];
   _txBuf[3] = rgb[1];
   _txBuf[4] = rgb[2];
   _txBuf[10] = 0xEE;
   sendReceive();
   return 0;
}

bool IOTShield::setAUX1(bool on)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_AUX1;
   _txBuf[2] = (on ? 0x01 : 0x00);
   for(int i=3; i<10; i++)
      _txBuf[i] = 0;
   _txBuf[10] = 0xEE;
   sendReceive();
   return 0;
}
   
bool IOTShield::setAUX2(bool on)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_AUX2;
   _txBuf[2] = on;
   for(int i=3; i<10; i++)
      _txBuf[i] = 0;
   _txBuf[10] = 0xEE;
   sendReceive();
   return 0;
}

float IOTShield::getSystemVoltage()
{
   return _systemVoltage;
}
   
const std::vector<float> IOTShield::getRangeMeasurements()
{
   return _ranges;
}

const std::vector<float> IOTShield::getAcceleration()
{
   return _acceleration;
}

const std::vector<float> IOTShield::getAngularRate()
{
   return _angularRate;
}

void IOTShield::sendReceive()
{
#if _WITH_MRAA
    timeval clock;
    double now = 0.0;
    do
    {
       ::gettimeofday(&clock, 0);
       now = static_cast<double>(clock.tv_sec) + static_cast<double>(clock.tv_usec) * 1.0e-6;
    }while((now - _timeCom) < 0.008);
    _timeCom = now;
   _uart->write((char*)_txBuf, 11);
   _uart->read(_rxBuf, 32);

   for(int i=0; i<3; i++)
   {
      int16_t  val     = _rxBuf[10+2*i];
      val              = val << 8;
      val             |= _rxBuf[9+2*i];
      // convert from mg*10 to g
      _acceleration[i] = ((float)val)/10000.f;

      val              = _rxBuf[16+2*i];
      val              = val << 8;
      val             |= _rxBuf[15+2*i];
      // convert from mdps/10 to dps
      _angularRate[i]  = ((float)val)/100.f;
   }
   for(int i=0; i<4; i++)
   {
      int16_t val  = _rxBuf[2*i+2] << 8;
      val         |= _rxBuf[2*i+1];
      _rpm[i]      = ((float)val)/100.f;
   
      unsigned short distanceInMM = _rxBuf[22+2*i];
      distanceInMM                = distanceInMM << 8;
      distanceInMM               |= _rxBuf[21+2*i];
      _ranges[i]                  = (float)distanceInMM;
      _ranges[i]                 /= 1000.f;
   }
   unsigned int voltage = _rxBuf[30];
   voltage              = voltage << 8;
   voltage             |= _rxBuf[29];
   _systemVoltage       = (float) voltage / 100.f;
#else
   std::cerr << "Ignoring UART communication demand." << std::endl;
#endif
}

} // namespace
