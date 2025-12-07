#include <Arduino.h>
#include "joystick.h"

String raw = "";
int joyX = 512;
int joyY = 512;

void processJoystickPacket(char c) {

    if (c == 3) {  // ETX = end of packet
        int commaIndex = raw.indexOf(',');

        if (commaIndex > 0) {
            String xs = raw.substring(1, commaIndex);      // skip leading 'd'
            String ys = raw.substring(commaIndex + 1);     // everything after comma

            joyX = xs.toInt();
            joyY = ys.toInt();
        }

        raw = "";   // reset buffer
    } 
    else {
        raw += c;   // accumulate characters
        Serial.println(raw);  // DEBUG PRINT
    }
}