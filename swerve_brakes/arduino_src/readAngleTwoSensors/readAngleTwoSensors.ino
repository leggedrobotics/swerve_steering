#include <Wire.h>
#include <AS5600.h>
#include "TCA9548A.h"

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

TCA9548A<TwoWire> TCA;
AMS_5600 sensor_0;
AMS_5600 sensor_1;

int ang, lang = 0;
bool magnet_0_detected, magnet_1_detected = false;

void setup()
{
  SERIAL.begin(115200);
  Wire.begin();
  TCA.begin(Wire);
  SERIAL.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  while(1){
      TCA.openChannel(TCA_CHANNEL_4);
      if(sensor_0.detectMagnet() == 1 ){
          SERIAL.print("Current Magnitude 0: ");
          SERIAL.println(sensor_0.getMagnitude());
          magnet_0_detected = true; 
      }
      else{
          SERIAL.println("Can not detect magnet 0");
          magnet_0_detected = false;
      }
      TCA.closeAll();
      TCA.openChannel(TCA_CHANNEL_5);
      if(sensor_1.detectMagnet() == 1 ){
          SERIAL.print("Current Magnitude 1: ");
          SERIAL.println(sensor_1.getMagnitude());
          magnet_1_detected = true; 
      }
      else{
          SERIAL.println("Can not detect magnet 1");
          magnet_1_detected = false;
      }
      TCA.closeAll();
      
      if (magnet_0_detected && magnet_1_detected){
        break;
      }
      delay(1000);
  }
}
/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  ang = retVal;
  return ang;
}
void loop()
{
    TCA.openChannel(TCA_CHANNEL_4);
    SERIAL.println(String(convertRawAngleToDegrees(sensor_0.getRawAngle()),DEC));
    TCA.closeAll();
    TCA.openChannel(TCA_CHANNEL_5);
    SERIAL.println(String(convertRawAngleToDegrees(sensor_1.getRawAngle()),DEC));
    TCA.closeAll();
    delay(1000);
}
