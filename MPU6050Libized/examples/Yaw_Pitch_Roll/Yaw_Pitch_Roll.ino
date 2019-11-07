#include "MPU6050Libized.h"

constexpr auto MPU_READ_INTERVAL = 16;
constexpr auto MPU_INTERRUPT_PIN = 2;

MPU6050Libized motion(MPU_INTERRUPT_PIN);
YawPitchRoll ypr;

uint32_t currentTime, lastMPUReadTime;

void setup() {
    Serial.begin(115200);
    motion.init(-3091, -4415, 737, -25, 20, 0);

    lastMPUReadTime = millis();
    Serial.print("Start time: ");
    Serial.print(lastMPUReadTime);
    Serial.println(" ms");
}

void loop() {
    currentTime = millis();

    if (currentTime - lastMPUReadTime >= MPU_READ_INTERVAL && motion.checkMPUDataAvailable()) {
        ypr = motion.getYawPitchRoll();

        Serial.print("ypr\t");
        Serial.print(ypr.yaw * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr.pitch * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr.roll * 180 / M_PI);

        Serial.print("Time: ");
        Serial.print(currentTime - lastMPUReadTime);
        Serial.println(" ms");

        lastMPUReadTime = currentTime;
    }
}
