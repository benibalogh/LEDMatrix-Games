/*
 Name:		MPU6050Libized.h
 Created:	10/24/2019 11:18:53 AM
 Author:	Benjamin Balogh
 Editor:	http://www.visualmicro.com
*/

#ifndef _MPU6050Libized_h
#define _MPU6050Libized_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


struct YawPitchRoll {
    float yaw;
    float pitch;
    float roll;
};

class MPU6050Libized {
public:
    static volatile bool mpuInterrupt;      // indicates whether MPU interrupt pin has gone high

protected:
    uint8_t _interrupPin;

    MPU6050 mpu;

    uint8_t mpuIntStatus;                   // holds actual interrupt status byte from MPU
    uint16_t packetSize;                    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;                     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];                 // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;                           // [w, x, y, z]         quaternion container
    VectorInt16 aa;                         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;                     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;                    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;                    // [x, y, z]            gravity vector
    float ypr[3];                           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

public:
    // Constructor
    MPU6050Libized(uint8_t pin);

    bool init(int16_t xa, int16_t ya, int16_t za, int16_t xg, int16_t yg, int16_t zg);  // call this function after object instantiation
    bool checkMPUDataAvailable();
    YawPitchRoll getYawPitchRoll();
    // TODO: void calibrate();
};

#endif

