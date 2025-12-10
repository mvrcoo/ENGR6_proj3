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
    // Line sensor Digital Inputs
    pinMode(LINE_L, INPUT_PULLUP);
    pinMode(LINE_M, INPUT_PULLUP);
    pinMode(LINE_R, INPUT_PULLUP);

    setupMPU();
    Serial.println("Autonomous Car + Crash Avoidance Ready");
}

void loop() {

    // Read Bluetooth packet
    while (bluetooth.available()) {
        char c = bluetooth.read();
        processJoystickPacket(c);
    }
    // MODE CHANGE HANDLING + AUTO-SAVE ON EXIT FROM AUTONOMOUS
    static int lastMode = -1;

    if (lastMode != mode) {

        // If LEAVING autonomous mode → save max acceleration
        if (lastMode == 1 && mode == 0) {
            Serial.println("Saving Max Acceleration to EEPROM...");
            saveMaxAccel(maxAx, maxAy);
            Serial.print("Saved MaxAx: "); Serial.print(maxAx);
            Serial.print("   MaxAy: "); Serial.println(maxAy);
        }

        // Now print new mode
        if (mode == 0) Serial.println("Mode: Manual");
        else Serial.println("Mode: Autonomous");

        lastMode = mode;
    }

// =============================================
// AUTONOMOUS MODE — LEFT SENSOR FIX + MOVEMENT RESTORED
// =============================================
if (mode == 1) {

    int Lraw = digitalRead(LINE_L);
    int Rraw = digitalRead(LINE_R);

    long dist = readDistance();

    // ---------------------------------------------------------
    // FIX LEFT SENSOR POLARITY ONLY
    // ---------------------------------------------------------
    // Left sensor is reversed (HIGH = black)
    bool Lblack = (Lraw == HIGH);
    bool Lwhite = !Lblack;

    // Right sensor normal (LOW = black)
    bool Rblack = (Rraw == LOW);
    bool Rwhite = !Rblack;

    // ---------------------------------------------------------
    // OBSTACLE STOP ONLY
    // ---------------------------------------------------------
    if (dist > 0 && dist < 12) {
        stopMotors();
        return;
    }

    // ---------------------------------------------------------
    // MOVE FORWARD SLOWER (so it can follow the tape)
    // ---------------------------------------------------------
    int base = 70;      // forward speed
    int adjustLow = 50; // correction slow wheel
    int adjustHigh = 85;// correction fast wheel

    // ---------------------------------------------------------
    // LINE FOLLOWING — SAME LOGIC AS BEFORE BUT FIXED
    // ---------------------------------------------------------

    // BOTH BLACK → centered → go forward
    if (Lblack && Rblack) {
        driveMotors(base, base, false, false);
    }
    // LEFT BLACK → turn left
    else if (Lblack && Rwhite) {
        driveMotors(adjustLow, adjustHigh, false, false);
    }
    // RIGHT BLACK → turn right
    else if (Rblack && Lwhite) {
        driveMotors(adjustHigh, adjustLow, false, false);
    }
    else {
        // fallback forward (never stop)
        driveMotors(base, base, false, false);
    }

    // ---------------------------------------------------------
    // DEBUG
    // ---------------------------------------------------------
    static unsigned long last = 0;
    if (millis() - last >= 150) {
        Serial.print("Lraw: "); Serial.print(Lraw);
        Serial.print("  Rraw: "); Serial.print(Rraw);
        Serial.print("  Lblack: "); Serial.print(Lblack);
        Serial.print("  Rblack: "); Serial.println(Rblack);
        last = millis();
    }

    return;
}


    // MANUAL MODE
    int x = joyX;
    int y = joyY;

    int speedValue = 0;
    String direction = "--";

    // DEADZONE PRINT
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
    // MOVEMENT LOGIC
    if (y > 552) {
        direction = "FORWARD";
        speedValue = map(y, 552, 1023, 0, 255);
        driveMotors(speedValue, speedValue, false, false);
    }
    else if (y < 472) {
        direction = "BACKWARD";
        speedValue = map(512 - y, 0, 511, 0, 255);
        driveMotors(speedValue, speedValue, true, true);
    }
    else if (x > 552) {
        direction = "RIGHT";
        speedValue = map(x, 552, 1023, 0, 255);
        driveMotors(speedValue, speedValue, false, true);
    }
    else if (x < 472) {
        direction = "LEFT";
        speedValue = map(512 - x, 0, 511, 0, 255);
        driveMotors(speedValue, speedValue, true, false);
    }
    // TIMED MANUAL PRINT
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