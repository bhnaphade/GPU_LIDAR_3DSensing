/* =============================================================================
  LIDAR-Lite v2: Change the I2C address of multiple sensors with PWR_EN line

  This example demonstrates how to chage the i2c address of multiple sensors.

  The library is in BETA, so subscribe to the github repo to recieve updates, or
  just check in periodically:
  https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library

  To learn more read over lidarlite.cpp as each function is commented
=========================================================================== */

#include <Wire.h>
#include <LIDARLite.h>

int sensorPins[] = {4,5,6,7}; // Array of pins connected to the sensor Power Enable lines
unsigned char addresses[] = {0x66,0x68,0x64,0x70};
byte data[]={0};
byte i;
uint16_t sen0,sen1,sen2,sen3 = 0;               // 0001 0110 0100 0111
uint16_t mask   = B11111111;          // 0000 0000 1111 1111
uint8_t first_half0,first_half1,first_half2,first_half3 ;
uint8_t sencond_half0,sencond_half1, sencond_half2, sencond_half3;
LIDARLite myLidarLite;

void setup() {
  Serial.begin(9600);
   pinMode(13, OUTPUT);
     myLidarLite.begin();
  myLidarLite.changeAddressMultiPwrEn(4,sensorPins,addresses,false);
}

void loop() 
{

 //Serial.print('A');
    while(1)
    {           while(Serial.read()!='S')
    {   
     digitalWrite(13, LOW); ;
    }
    digitalWrite(13, HIGH);
            sen0 =(uint16_t) myLidarLite.distance(true,true,addresses[0]);
            first_half0   = (sen0& 0xFF00) >> 8;   // >>>> >>>> 0001 0110
            sencond_half0 = sen0 & mask; // ____ ____ 0100 0111
            sen1 =(uint16_t) myLidarLite.distance(true,true,addresses[1]);
            first_half1   = (sen1& 0xFF00) >> 8;   // >>>> >>>> 0001 0110
            sencond_half1 = sen1 & mask; // ____ ____ 0100 0111
            sen2 =(uint16_t) myLidarLite.distance(true,true,addresses[2]);
            first_half2   = (sen2& 0xFF00) >> 8;   // >>>> >>>> 0001 0110
            sencond_half2 = sen2 & mask; // ____ ____ 0100 0111
            sen3 =(uint16_t) myLidarLite.distance(true,true,addresses[3]);
            first_half3   = (sen3& 0xFF00) >> 8;   // >>>> >>>> 0001 0110
            sencond_half3 = sen3 & mask; // ____ ____ 0100 0111
            Serial.write(first_half0);
            Serial.write(sencond_half0);
            Serial.write(first_half1);
            Serial.write(sencond_half1);
            Serial.write(first_half2);
            Serial.write(sencond_half2);
             Serial.write(first_half3);
            Serial.write(sencond_half3);         
    }
}
