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

// Mode: 0 = Manual, 1 = Autonomous
int mode = 0;
int lastMode = -1;

// Bluetooth (HM-10)
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

    Serial.println("Autonomous Car + Crash Avoidance");
}

void loop() {

    // Read all Bluetooth joystick packets
    while (bluetooth.available()) {
        char c = bluetooth.read();
        processJoystickPacket(c);
    }

    // Joystick centered → toggle mode
    bool centered =
        (joyX > 350 && joyX < 650) &&
        (joyY > 350 && joyY < 650);

    static unsigned long lastToggle = 0;
    if (centered) {
        if (millis() - lastToggle > 500) {

            // Toggle between manual and autonomous
            if (mode == 0) {
                mode = 1;
                Serial.println("Mode: Autonomous");
            } else {
                mode = 0;
                Serial.println("Mode: Manual");
            }

            lastToggle = millis();
        }
    }

    //AUTONOMOUS MODE (mode = 1)
    if (mode == 1) {

        // Print mode only once (no spam)
        if (lastMode != mode) {
            Serial.println("Mode: Autonomous");
            lastMode = mode;
        }
        Serial.println("autonomous mode running..,");
        // Read line sensors
        readLineSensors();
        int threshold = 600;
        bool L = (lineL < threshold);
        bool M = (lineM < threshold);
        bool R = (lineR < threshold);

        // None of the sensors detect the line
        if (!L && !M && !R) {
            Serial.println("Line not detected — holding still");
            stopMotors();
            return;
        }
        // Crash avoidance
        long dist = readDistance();
        if (dist > 0 && dist < 15) {
            Serial.println("Stopping - Object Too Close");
            stopMotors();
            return;
        }

        // Line following behavior
        if (M && !L && !R) {
            Serial.println("Going Straight");
            driveMotors(150, 150, true, true);
        }
        else if (L && !M) {
            Serial.println("Turning Left");
            driveMotors(120, 0, true, false);
        }
        else if (R && !M) {
            Serial.println("Turning Right");
            driveMotors(0, 120, false, true);
        }
        else if (L && M && R) {
            Serial.println("Stopping - All Sensors on Line");
            stopMotors();
        }
        else {
            Serial.println("Stopping - Unknown Line Pattern");
            stopMotors();
        }

        return;
    }

    //  MANUAL MODE (mode = 0)
    if (lastMode != mode) {
        Serial.println("Mode: Manual");
        lastMode = mode;
    }

    // Local joystick values
    int lx = joyX;
    int ly = joyY;
    if (abs(lx - 512) < 40) lx = 512;
    if (abs(ly - 512) < 40) ly = 512;
    int speedX = lx - 512;
    int speedY = 512 - ly;
    if (speedX < 0) speedX = -speedX;
    if (speedY < 0) speedY = -speedY;

    String direction = "STOP";

    if (ly < 400) {
        direction = "FORWARD";
        driveMotors(180, 180, true, true);
    }
    else if (ly > 600) {
        direction = "BACKWARD";
        driveMotors(180, 180, false, false);
    }
    else if (lx < 400) {
        direction = "LEFT";
        driveMotors(150, 150, false, true);
    }
    else if (lx > 600) {
        direction = "RIGHT";
        driveMotors(150, 150, true, false);
    }
    else {
        stopMotors();
    }

    // Speed display depends on direction
    int displaySpeed = 0;
    if (direction == "FORWARD") displaySpeed = speedY;
    else if (direction == "BACKWARD") displaySpeed = speedY;
    else if (direction == "LEFT") displaySpeed = speedX;
    else if (direction == "RIGHT") displaySpeed = speedX;

    // Output joystick + motor info
    Serial.print("X: ");
    Serial.print(lx);
    Serial.print("   Y: ");
    Serial.print(ly);
    Serial.print("   Direction: ");
    Serial.print(direction);
    Serial.print("   Speed: ");
    Serial.println(displaySpeed);
}
