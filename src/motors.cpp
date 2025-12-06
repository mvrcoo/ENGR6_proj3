#include <Arduino.h>
#include "pins.h"

void driveMotors(int Ls, int Rs, bool Lf, bool Rf) {
    if (Lf) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    analogWrite(enA, Ls);

    if (Rf) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    } else {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
    analogWrite(enB, Rs);
}

void stopMotors() {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}
