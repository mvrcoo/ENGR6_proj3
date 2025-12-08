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

    pinMode(enA, OUTPUT); 
    pinMode(in1, OUTPUT); 
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT); 
    pinMode(in3, OUTPUT); 
    pinMode(in4, OUTPUT);
    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT);

    setupMPU();
    Serial.println("Autonomous Car + Crash Avoidance Ready");
}

void loop() {
    // Read Bluetooth packet
    while (bluetooth.available()) {
        char c = bluetooth.read();
        processJoystickPacket(c);
    }

    // Mode print (only once per mode switch)
    static int lastMode = -1;
    if (lastMode != mode) {
        if (mode == 0) Serial.println("Mode: Manual");
        else Serial.println("Mode: Autonomous");
        // SAVE EEPROM WHEN EXITING AUTONOMOUS MODE
        if (lastMode == 1 && mode == 0) {
        saveMaxAccel(maxAx, maxAy);
        Serial.print("Saved MaxAx: "); Serial.print(maxAx);
        Serial.print("   MaxAy: "); Serial.println(maxAy);
}

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

        long dist = readDistance();

        // Movement decisions
        if (pattern == 0) {stopMotors();}
        else if (dist > 0 && dist < 15) {stopMotors();}
        else if (pattern == 2) {driveMotors(150, 150, true, true); }   // Forward
        else if (pattern == 1) {driveMotors(120, 0, true, false); }  // Left
        else if (pattern == 4) {driveMotors(0, 120, false, true); }   // Right

        //AUTONOMOUS TIMED DEBUG PRINt
        static unsigned long lastAutoPrint = 0;

        if (millis() - lastAutoPrint >= 150) {

            // Line sensor readings
            Serial.print("L: "); Serial.print(lineL);
            Serial.print("   M: "); Serial.print(lineM);
            Serial.print("   R: "); Serial.print(lineR);
            Serial.println();

            // Course status
            Serial.print("Course: ");
            if (pattern == 2) Serial.print("Straight");
            else if (pattern == 1) Serial.print("Adjust Left");
            else if (pattern == 4) Serial.print("Adjust Right");
            else Serial.print("No Line -> Stopping");
            Serial.println();

            // Distance reading
            Serial.print("Dist: "); 
            Serial.print(dist); 
            Serial.println(" cm");

            // Obstacle detected
            if (dist > 0 && dist < 15) {
                Serial.println("Course: Obstacle -> Stopping");
            }

            // ACCELEROMETER
            float ax, ay, az;
            readMPU(ax, ay, az);

            Serial.print("Ax: "); Serial.print(ax);
            Serial.print("   Ay: "); Serial.print(ay);

            // Update maximum acceleration recorded
            if (abs(ax) > abs(maxAx)) maxAx = ax;
            if (abs(ay) > abs(maxAy)) maxAy = ay;

            Serial.print("   MaxAx: "); Serial.print(maxAx);
            Serial.println();

            lastAutoPrint = millis();
        }

        return; // END AUTONOMOUS MODE
    }
    // MANUAL MODE
    int x = joyX;
    int y = joyY;

    int speedValue = 0;
    String direction = "--";

    // DEADZONE PRINT (slowed)
    if (abs(x - 512) < 40 && abs(y - 512) < 40) {
        stopMotors();
        direction = "--";
        speedValue = 0;

        static unsigned long lastNeutralPrint = 0;

        if (millis() - lastNeutralPrint >= 300) {
            Serial.print("X: "); Serial.print(x);
            Serial.print("   Y: "); Serial.print(y);
            Serial.print("   Direction: "); Serial.print(direction);
            Serial.print("   Speed: "); Serial.println(speedValue);
            lastNeutralPrint = millis();
        }
        return;
    }

    // FORWARD
    if (y > 552) {
        direction = "FORWARD";
        speedValue = map(y, 552, 1023, 0, 255);
        driveMotors(speedValue, speedValue, false, false);
    }
    // BACKWARD
    else if (y < 472) {
        direction = "BACKWARD";
        speedValue = map(512 - y, 0, 511, 0, 255);
        driveMotors(speedValue, speedValue, true, true);
    }
    // RIGHT
    else if (x > 552) {
        direction = "RIGHT";
        speedValue = map(x, 552, 1023, 0, 255);
        driveMotors(speedValue, speedValue, false, true);
    }
    // LEFT
    else if (x < 472) {
        direction = "LEFT";
        speedValue = map(512 - x, 0, 511, 0, 255);
        driveMotors(speedValue, speedValue, true, false);
    }

    // TIMED PRINT FOR MOVEMENT
    static unsigned long lastPrint = 0;
    bool moving = !(abs(x - 512) < 40 && abs(y - 512) < 40);

    if (moving && millis() - lastPrint >= 150) {
        Serial.print("X: "); Serial.print(x);
        Serial.print("   Y: "); Serial.print(y);
        Serial.print("   Direction: "); Serial.print(direction);
        Serial.print("   Speed: "); Serial.println(speedValue);

        lastPrint = millis();
    }
}