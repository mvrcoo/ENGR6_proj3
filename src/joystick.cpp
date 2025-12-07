#include <Arduino.h>
#include "joystick.h"

String raw = "";
int joyX = 512;
int joyY = 512;

void processJoystickPacket(char c) {

    if (c == 3) {  // ETX (end of packet)

        // 1. Find control character 0x02 "␂"
        int start = raw.indexOf(char(2));
        if (start == -1) {
            raw = "";
            return;
        }

        start++; // Move to first digit after the control char

        // 2. Find comma separating X and Y
        int commaIndex = raw.indexOf(',', start);
        if (commaIndex == -1) {
            raw = "";
            return;
        }

        // 3. Extract X and Y
        String xs = raw.substring(start, commaIndex);
        String ys = raw.substring(commaIndex + 1);

        joyX = xs.toInt();
        joyY = ys.toInt();

        Serial.print("Parsed X=");
        Serial.print(joyX);
        Serial.print(" Y=");
        Serial.println(joyY);

        raw = "";
    }
    else {
        raw += c;
        //Serial.println(raw);
    }
}