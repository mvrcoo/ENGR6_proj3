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

    // Mode toggle when joystick is centered 
    bool centered = false;
    if (joyX > 350 && joyX < 650) {
        if (joyY > 350 && joyY < 650) centered = true;
    }

    static unsigned long lastToggle = 0;
    if (centered) {
        if (millis() - lastToggle > 500) {
            if (mode == 0) mode = 1;
            else mode = 0;
            lastToggle = millis();
        }
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
    int dx = x - 512;
    int dy = y - 512;

    if (dx < 40 && dx > -40) { dx = 0; x = 512; }
    if (dy < 40 && dy > -40) { dy = 0; y = 512; }

    String direction = "STOP";
    if (dy < 0) direction = "FORWARD";
    else if (dy > 0) direction = "BACKWARD";
    else if (dx < 0) direction = "LEFT";
    else if (dx > 0) direction = "RIGHT";

    int speedValue = 0;
    if (direction == "FORWARD")  speedValue = -dy;
    if (direction == "BACKWARD") speedValue = dy;
    if (direction == "LEFT")     speedValue = -dx;
    if (direction == "RIGHT")    speedValue = dx;

    if (direction == "FORWARD")      driveMotors(speedValue, speedValue, true, true);
    else if (direction == "BACKWARD")driveMotors(speedValue, speedValue, false, false);
    else if (direction == "LEFT")    driveMotors(speedValue, speedValue, false, true);
    else if (direction == "RIGHT")   driveMotors(speedValue, speedValue, true, false);
    else stopMotors();

    Serial.print("X: "); Serial.print(x);
    Serial.print("   Y: "); Serial.print(y);
    Serial.print("   Direction: "); Serial.print(direction);
    Serial.print("   Speed: "); Serial.println(speedValue);

    delay(80); 
}