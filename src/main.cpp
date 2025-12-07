#include <Arduino.h>
#include <SoftwareSerial.h>

#include "pins.h"
#include "motors.h"
#include "ultrasonic.h"
#include "joystick.h"
#include "deadzone.h"
#include "linefollow.h"
#include "mpu.h"
#include "eeprom_store.h"

float maxAx = 0.0;
float maxAy = 0.0;
int mode = 0;   // 0 = Manual, 1 = Autonomous

SoftwareSerial bluetooth(BT_TX, BT_RX);

void setup() {
    Serial.begin(115200);
    bluetooth.begin(9600);

    pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
    pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

    setupMPU();
    Serial.println("Autonomous Car + Crash Avoidance Ready");
}

void loop() {

    // Read Bluetooth packet
    while (bluetooth.available()) {
        char c = bluetooth.read();
        processJoystickPacket(c);
    }

    // Mode print (only once per switch)
    static int lastMode = -1;
    if (lastMode != mode) {
        if (mode == 0) Serial.println("Mode: Manual");
        else Serial.println("Mode: Autonomous");
        lastMode = mode;
    }

    // AUTONOMOUS MODE
    if (mode == 1) {

        readLineSensors();

        bool L = (lineL < 600);
        bool M = (lineM < 600);
        bool R = (lineR < 600);

        int pattern = 0;
        if (L) pattern |= 1;
        if (M) pattern |= 2;
        if (R) pattern |= 4;

        if (pattern == 0) {
            stopMotors();
            return;
        }

        long dist = readDistance();
        if (dist > 0 && dist < 15) {
            stopMotors();
            return;
        }

        if (pattern == 2)      driveMotors(150, 150, true, true);   // Straight
        else if (pattern == 1) driveMotors(120, 0,   true, false);  // Left
        else if (pattern == 4) driveMotors(0,   120, false, true);  // Right
        else                   stopMotors();

        return;
    }

// MANUAL MODE
int x = joyX;
int y = joyY;

int speedValue = 0;
String direction = "--";

// DEADZONE
if (abs(x - 512) < 40 && abs(y - 512) < 40) {
    stopMotors();
    Serial.print("X: "); Serial.print(x);
    Serial.print("   Y: "); Serial.print(y);
    Serial.print("   Direction: "); Serial.print(direction);
    Serial.print("   Speed: "); Serial.println(speedValue);
    return;
}

// FORWARD (Y HIGH)   - backward
if (y > 552) {
    direction = "FORWARD";
    speedValue = map(y, 552, 1023, 0, 255);
    driveMotors(speedValue, speedValue, false, false);
}

// BACKWARD (Y LOW) --
else if (y < 472) {
    direction = "BACKWARD";
    speedValue = map(512 - y, 0, 511, 0, 255);
    driveMotors(speedValue, speedValue, true, true);
}

// RIGHT (X HIGH)
else if (x > 552) {
    direction = "RIGHT";
    speedValue = map(x, 552, 1023, 0, 255);
    driveMotors(speedValue, speedValue, false, true);
}

// LEFT (X LOW)
else if (x < 472) {
    direction = "LEFT";
    speedValue = map(512 - x, 0, 511, 0, 255);
    driveMotors(speedValue, speedValue, true, false);
}

// Only print when joystick is OUTSIDE deadzone OR direction is not neutral
if (abs(x - 512) > 40 || abs(y - 512) > 40 || direction != "--") {

    static int lastX = -1;
    static int lastY = -1;
    static String lastDir = "";
    static int lastSpeed = -1;

    // Only print when ACTUALLY changed
    if (x != lastX || y != lastY || direction != lastDir || speedValue != lastSpeed) {
        Serial.print("X: "); Serial.print(x);
        Serial.print("   Y: "); Serial.print(y);
        Serial.print("   Direction: "); Serial.print(direction);
        Serial.print("   Speed: "); Serial.println(speedValue);

        lastX = x;
        lastY = y;
        lastDir = direction;
        lastSpeed = speedValue;
    }
}

}