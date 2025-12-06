#include <Arduino.h>
#include "pins.h"
#include "linefollow.h"

int lineL = 0;
int lineM = 0;
int lineR = 0;

void readLineSensors() {
    lineL = analogRead(LINE_L);
    lineM = analogRead(LINE_M);
    lineR = analogRead(LINE_R);
}
