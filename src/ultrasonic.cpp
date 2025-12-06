#include <Arduino.h>
#include "pins.h"

long readDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000);
    long distance = duration * 0.034 / 2;
    return distance;
}
