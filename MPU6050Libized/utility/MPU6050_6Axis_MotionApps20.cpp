#include "MPU6050_6Axis_MotionApps20.h"

uint8_t MPU6050::dmpInitialize() {
	
	// reset device
	DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
	reset();
	delay(30); // wait after reset
	

	// enable sleep mode and wake cycle
	/*Serial.println(F("Enabling sleep mode..."));
	setSleepEnabled(true);
	Serial.println(F("Enabling wake cycle..."));
	setWakeCycleEnabled(true);*/

	
	// disable sleep mode
	setSleepEnabled(false);

	// get MPU hardware revision
	setMemoryBank(0x10, true, true);
	setMemoryStartAddress(0x06);
	Serial.println(F("Checking hardware revision..."));
	Serial.print(F("Revision @ user[16][6] = "));
	Serial.println(readMemoryByte(), HEX);
	Serial.println(F("Resetting memory bank selection to 0..."));
	setMemoryBank(0, false, false);

	// check OTP bank valid
	DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
	DEBUG_PRINT(F("OTP bank is "));
	DEBUG_PRINTLN(getOTPBankValid() ? F("valid!") : F("invalid!"));

	// setup weird slave stuff (?)
	DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
	setSlaveAddress(0, 0x7F);
	DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
	setI2CMasterModeEnabled(false);
	DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
	setSlaveAddress(0, 0x68);
	DEBUG_PRINTLN(F("Resetting I2C Master control..."));
	resetI2CMaster();
	delay(20);
	DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
	setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

	DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
	setIntEnabled(1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_INTERRUPT_DMP_INT_BIT);

	DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
	setRate(4); // 1khz / (1 + 4) = 200 Hz

	DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
	setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

	DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
	setDLPFMode(MPU6050_DLPF_BW_42);

	DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
	setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	// load DMP code into memory banks
	DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
	DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
	DEBUG_PRINTLN(F(" bytes)"));
	if (!writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) return 1; // Failed
	DEBUG_PRINTLN(F("Success! DMP code written and verified."));

	// Set the FIFO Rate Divisor int the DMP Firmware Memory
	unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
	writeMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

	//write start address MSB into register
	setDMPConfig1(0x03);
	//write start address LSB into register
	setDMPConfig2(0x00);

	DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
	setOTPBankValid(false);

	DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
	setMotionDetectionThreshold(2);

	DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
	setZeroMotionDetectionThreshold(156);

	DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
	setMotionDetectionDuration(80);

	DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
	setZeroMotionDetectionDuration(0);
	DEBUG_PRINTLN(F("Enabling FIFO..."));
	setFIFOEnabled(true);

	DEBUG_PRINTLN(F("Resetting DMP..."));
	resetDMP();

	DEBUG_PRINTLN(F("DMP is good to go! Finally."));

	DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
	setDMPEnabled(false);

	DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
	dmpPacketSize = 42;

	DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
	resetFIFO();
	getIntStatus();

	return 0; // success
}

bool MPU6050::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

uint8_t MPU6050::dmpGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[28] << 24) | ((uint32_t)packet[29] << 16) | ((uint32_t)packet[30] << 8) | packet[31]);
    data[1] = (((uint32_t)packet[32] << 24) | ((uint32_t)packet[33] << 16) | ((uint32_t)packet[34] << 8) | packet[35]);
    data[2] = (((uint32_t)packet[36] << 24) | ((uint32_t)packet[37] << 16) | ((uint32_t)packet[38] << 8) | packet[39]);
    return 0;
}

uint8_t MPU6050::dmpGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) | packet[29];
    data[1] = (packet[32] << 8) | packet[33];
    data[2] = (packet[36] << 8) | packet[37];
    return 0;
}

uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[28] << 8) | packet[29];
    v -> y = (packet[32] << 8) | packet[33];
    v -> z = (packet[36] << 8) | packet[37];
    return 0;
}

uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}

uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}

uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}

uint8_t MPU6050::dmpGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[20] << 8) | packet[21];
    data[2] = (packet[24] << 8) | packet[25];
    return 0;
}

uint8_t MPU6050::dmpGetGyro(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[16] << 8) | packet[17];
    v -> y = (packet[20] << 8) | packet[21];
    v -> z = (packet[24] << 8) | packet[25];
    return 0;
}

uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    v -> x = vRaw -> x - gravity -> x*8192;
    v -> y = vRaw -> y - gravity -> y*8192;
    v -> z = vRaw -> z - gravity -> z*8192;
    return 0;
}

uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}

uint8_t MPU6050::dmpGetGravity(int16_t *data, const uint8_t* packet) {
    /* +1g corresponds to +8192, sensitivity is 2g. */
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
    data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
    data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
	       - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (2 * 16384);
    return status;
}

uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}

#ifdef USE_OLD_DMPGETYAWPITCHROLL
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}
#else 
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);
    if (gravity -> z < 0) {
        if(data[1] > 0) {
            data[1] = PI - data[1]; 
        } else { 
            data[1] = -PI - data[1];
        }
    }
    return 0;
}
#endif

uint8_t MPU6050::dmpProcessFIFOPacket(const unsigned char *dmpData) {
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;
        
        // increment external process count variable, if supplied
        if (processed != 0) (*processed)++;
    }
    return 0;
}

uint16_t MPU6050::dmpGetFIFOPacketSize() {
    return dmpPacketSize;
}