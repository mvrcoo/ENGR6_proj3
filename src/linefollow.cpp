#include "linefollow.h"
#include "pins.h"
#include <Arduino.h>

int lineL, lineM, lineR;

void readLineSensors() {
    lineL = digitalRead(LINE_L);
    lineM = digitalRead(LINE_M);
    lineR = digitalRead(LINE_R);
}
