/*
 * Copyright (C) 2017 P.Bernal-Polo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// TO CHANGE THE OPTIMIZATION LEVEL, edit:
// /opt/arduino-1.8.5/hardware/arduino/avr/platform.txt
// and substitute:
// compiler.c.flags=-c -g -Os -w -ffunction-sections -fdata-sections -MMD
// compiler.cpp.flags=-c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD
// by:
// compiler.c.flags=-c -g -O3 -w -ffunction-sections -fdata-sections -MMD
// compiler.cpp.flags=-c -g -O3 -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD


//#define DEBUG_MODE
#define DEVICE_ID 14
#define LED_PIN 13


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
//#include "I2Cdev.h"
//#include "MPU6050.h"
#include "IPM_nano33iot.h"
#include "MessageManager.h"
#include <Arduino_LSM6DS3.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 //   #include "Wire.h"
//#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high


IPM_nano33iot mAG = IPM_nano33iot( DEVICE_ID );
MessageManager MM( 2 );

// variables to store the measurements
float ax, ay, az;
float wx, wy, wz;
float T;
int16_t ax2, ay2, az2;
int16_t wx2, wy2, wz2;
int16_t T2;

bool blinkState = false;


void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's & Gyroscope in degrees/second");
  //Serial.println("X\tY\tZ\tA\tB\tC");
}


void loop() {
  // read raw accel/gyro measurements from device
  if (IMU.accelerationAvailable()& IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax , ay , az);
    ax2=(int16_t)ax;
    ay2=(int16_t)ay;
    az2=(int16_t)az;
    IMU.readGyroscope(wx , wy , wz);
    wx2=(int16_t)wx;
    wy2=(int16_t)wy;
    wz2=(int16_t)wz;
    IMU.readTemperature(T);
    T2=(int16_t)T;
  }
  //IMU.( &ax , &ay , &az , &wx , &wy , &wz );
  //T = accelgyro.getTemperature();
  // we set the information packet
  mAG.set_a( ax2 , ay2 , az2 );
  mAG.set_w( wx2 , wy2 , wz2 );
  mAG.set_T( T2 );
  
#if defined DEBUG_MODE
  Serial.print( ax2 );   Serial.print("\t");   Serial.print( ay2 );   Serial.print("\t");   Serial.print( az2 );   Serial.print("\t\t");
  Serial.print( wx2 );   Serial.print("\t");   Serial.print( wy2 );   Serial.print("\t");   Serial.print( wz2 );
  Serial.println();
#else
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( mAG.get_length() , mAG.get_bytes() );
  // and we send it
  Serial.write( (byte*)toWrite , MM.get_messageOutLength() );
#endif
}


void test_measurementFrequency() {
  int N = 1000;
  float dt0 = 0.0;
  float dt1 = 0.0;
  float dt2 = 0.0;
  float dt3 = 0.0;
  for(int i=0; i<N; i++){
    unsigned long t0 = micros();
    IMU.readAcceleration( ax , ay , az);
    unsigned long t1 = micros();
    IMU.readGyroscope(wx , wy , wz);
    //unsigned long t2 = micros();
    //accelgyro.getRotation(wx , wy , wz );
    unsigned long t3 = micros();
    IMU.readTemperature(T);
    unsigned long t4 = micros();
    dt0 += t1-t0;
    dt1 += t3-t1;
    dt2 += t4-t3;
    //dt3 += t4-t3;
  }
  Serial.print( 1.0e6/(dt0/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt1/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt2/N) , 6 );
  //Serial.print( " " );
  //Serial.print( 1.0e6/(dt3/N) , 6 );
  Serial.println();
}
