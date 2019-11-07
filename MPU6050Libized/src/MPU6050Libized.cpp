/*
 Name:		MPU6050Libized.cpp
 Created:	10/24/2019 11:18:53 AM
 Author:	Benjamin Balogh
 Editor:	http://www.visualmicro.com
*/

#include "MPU6050Libized.h"

volatile bool MPU6050Libized::mpuInterrupt;      // indicates whether MPU interrupt pin has gone high

MPU6050Libized::MPU6050Libized(uint8_t interruptPin) {
    pinMode(interruptPin, INPUT);
    _interrupPin = interruptPin;
    ypr[0] = 0.0f; ypr[1] = 0.0f; ypr[2] = 0.0f;
}

static void dmpDataReady() {
    MPU6050Libized::mpuInterrupt = true;
}

bool MPU6050Libized::init(int16_t xa, int16_t ya, int16_t za, int16_t xg, int16_t yg, int16_t zg) {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    #ifdef DEBUG
        Serial.begin(115200);
    #endif

    // initialize device
    mpu.initialize();

    // verify connection
    uint8_t connectFailedCounter = 0;
    while (!mpu.testConnection()) {
        if (connectFailedCounter++ >= 5)
            return false;  // return after 5 unsuccessful connections
        delay(500);
    }

    // load and configure the DMP
    auto devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(xa); mpu.setYAccelOffset(ya); mpu.setZAccelOffset(za); mpu.setXGyroOffset(xg); mpu.setYGyroOffset(yg); mpu.setZGyroOffset(zg);

    bool dmpReady = false;
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        //mpu.CalibrateAccel();
        //mpu.CalibrateGyro();
        //mpu.PrintActiveOffsets();

        // turn on the DMP as it is ready
        mpu.setDMPEnabled(true);

        // enable interrupt detection on the Arduino
        attachInterrupt(digitalPinToInterrupt(_interrupPin), dmpDataReady, FALLING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        dmpReady = false;
        
        #ifdef DEBUG
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        #endif
    }
    return dmpReady;
}

// should be called in loop()
bool MPU6050Libized::checkMPUDataAvailable() {
    return MPU6050Libized::mpuInterrupt;
}

// should be called when checkMPUDataAvailable() returns true
YawPitchRoll MPU6050Libized::getYawPitchRoll() {
    mpuInterrupt = false;
    fifoCount = mpu.getFIFOCount();
    uint16_t maxPackets = 20;  // 20*42=840 leaving us with  2 Packets (out of a total of 24 packets) left before we overflow.
    // If we overflow the entire FIFO buffer will be corrupt and we must discard it!

    if (fifoCount % packetSize || fifoCount > packetSize * maxPackets || fifoCount < packetSize) {
        mpuIntStatus = mpu.getIntStatus();
        mpu.resetFIFO();     // clear the buffer and start over
        mpu.getIntStatus();  // make sure status is cleared as we will read it again.
        return { ypr[0], ypr[1], ypr[2] };
    } else {
        while (fifoCount >= packetSize) {              // Get the packets until we have the latest!
            if (fifoCount < packetSize)
                break;                                 // Something is left over and we don't want it!!!
            mpu.getFIFOBytes(fifoBuffer, packetSize);  // lets do the magic and get the data
            fifoCount -= packetSize;
        }
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if (fifoCount > 0)
            mpu.resetFIFO();

        return { ypr[0], ypr[1], ypr[2] };
    }
}
