#include <Arduino.h>
#include <SoftwareSerial.h>

#include "pins.h"
#include "motors.h"
#include "ultrasonic.h"
#include "joystick.h"
#include "deadzone.h"

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

    Serial.println("Autonomous Car + Crash Avoidance");
}

void loop() {

    long dist = readDistance();
    if (dist > 0 && dist < 20) {
        stopMotors();
        Serial.print("Crash Avoidance Distance: ");
        Serial.println(dist);
        delay(50);
        return;
    }

    while (bluetooth.available()) {
        char c = bluetooth.read();
        processJoystickPacket(c);
    }

    bool forward   = isForward(joyX);
    bool backward  = isBackward(joyX);
    bool leftTurn  = isLeft(joyY);
    bool rightTurn = isRight(joyY);

    if (!forward && !backward && !leftTurn && !rightTurn) {
        stopMotors();
        delay(50);
        return;
    }

    if (forward) {
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

    delay(50);
}
