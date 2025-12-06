#include <Wire.h>
#include "mpu.h"

void setupMPU() {
    Wire.begin();

    // Wake up MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Set gyro range ±250
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Set accelerometer range ±2g
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();
}

void readMPU(float &ax, float &ay, float &az) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);   // starting register for accel
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    while (Wire.available() < 6);

    int16_t rawX = Wire.read()<<8 | Wire.read();
    int16_t rawY = Wire.read()<<8 | Wire.read();
    int16_t rawZ = Wire.read()<<8 | Wire.read();

    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;
}
