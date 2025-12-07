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

// Global variables
float maxAx = 0.0;
float maxAy = 0.0;

int mode = 0; // 0: Manual, 1: Autonomous

SoftwareSerial bluetooth(BT_TX, BT_RX);
//setup function
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
//LOOP function
void loop() {

    float ax, ay, az;
    readMPU(ax, ay, az);

    if (abs(ax) > abs(maxAx)) maxAx = ax;
    if (abs(ay) > abs(maxAy)) maxAy = ay;

    saveMaxAccel(maxAx, maxAy);

    Serial.print("Accel X: "); Serial.print(ax);
    Serial.print(" Y: "); Serial.print(ay);
    Serial.print(" Z: "); Serial.println(az);

    // read distance from ultrasonic sensor
    long dist = readDistance();
    if (dist > 0 && dist < 20) {
        stopMotors();
        Serial.print("Crash Avoidance Distance: ");
        Serial.println(dist);
        delay(50);
        return;
    }
    // Read line sensors
    readLineSensors();
    Serial.print("Line Sensors: L="); // Print left sensor value
    Serial.print(lineL); 
    Serial.print(" M="); // Print middle sensor value
    Serial.print(lineM);
    Serial.print(" R="); // Print right sensor value
    Serial.println(lineR);

    // joystick packet processing
    while (bluetooth.available()) { // Process all available bytes
        char c = bluetooth.read(); // Read a byte from Bluetooth
        processJoystickPacket(c); // Process the joystick packet
    }
    // Mode Switch (UPDATED)
    static unsigned long firstTapTime = 0;
    static bool waitingForSecondTap = false;

    bool centered = (joyX > 480 && joyX < 540 && joyY > 480 && joyY < 540);
    if (centered) {
        if (!waitingForSecondTap) {
            waitingForSecondTap = true;
            firstTapTime = millis();
        }
        else {
            if (millis() - firstTapTime < 400) {

                if (mode == 0) {
                    mode = 1;
                    Serial.println("Mode: Autonomous");
                }
                else {
                    mode = 0;
                    Serial.println("Mode: Manual");
                }
            }
            waitingForSecondTap = false;
        }
    }

    if (waitingForSecondTap && (millis() - firstTapTime > 400)) {
        waitingForSecondTap = false;
    }

    // Mode 0 - Manual Control
    if (mode == 0) {
    bool forward   = isForward(joyX);
    bool backward  = isBackward(joyX);
    bool leftTurn  = isLeft(joyY);
    bool rightTurn = isRight(joyY);

    if (!forward && !backward && !leftTurn && !rightTurn) {
        stopMotors();
        delay(50);
        return;
    }

    if (forward) { //forward movement
        driveMotors(150, 150, true, true);
    }
    else if (backward) {
        int speed = map(joyX - 512, 0, 511, 0, 255);
        driveMotors(speed, speed, false, false);
    }
    else if (rightTurn) {
        int t = map(joyY - 512, 0, 511, 0, 255);
        driveMotors(t, t, true, false);
    }
    else if (leftTurn) {
        int t = map(512 - joyY, 0, 511, 0, 255);
        driveMotors(t, t, false, true);
    }
    
    return;

    }
    /*   Mode 1 - Autonomous Line Following.   */

    if (mode == 1) {
        Serial.println("autonomous mode running..,");
    
        int lineThreshold = 600; // Adjust based on calibration
        
        bool leftonLine = (lineL < lineThreshold);
        bool midonLine  = (lineM < lineThreshold);
        bool rightonLine= (lineR < lineThreshold);

        //straight
        if (midonLine && !leftonLine && !rightonLine) {
            driveMotors(150, 150, true, true);
            Serial.println("Going Straight");
        }
        // Turn Left
        else if (leftonLine && !midonLine) {
            driveMotors(120, 0, true, false);
            Serial.println("Turning Left");
        }
        // Turn Right
        else if (rightonLine && !midonLine) {
            driveMotors(0, 120, false, true);
            Serial.println("Turning Right");
        }
        // all black - stop
        else if (leftonLine && midonLine && rightonLine) {
            stopMotors();
            Serial.println("Stopping - All Sensors on Line");
        }
        //Lost line - stop
        else {
            stopMotors();
            Serial.println("Stopping - Line Lost");
        }

        return;
    }

    delay(50);
}
