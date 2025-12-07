#include <Arduino.h>
#include "joystick.h"

String raw = "";
int joyX = 512;
int joyY = 512;

void processJoystickPacket(char c) {

    if (c == 3) {  // ETX (end of packet)

        int start = raw.indexOf(char(2));
        if (start == -1) { raw = ""; return; }
        start++;

        int commaIndex = raw.indexOf(',', start);
        if (commaIndex == -1) { raw = ""; return; }

        String xs = raw.substring(start, commaIndex);
        String ys = raw.substring(commaIndex + 1);

        joyX = xs.toInt();
        joyY = ys.toInt();

        // Button D0 = Manual mode
        if (raw.indexOf("D0") != -1) {
            Serial.println("BUTTON D0 PRESSED");
            mode = 0;
        }

        // Button D1 = Autonomous mode
        if (raw.indexOf("D1") != -1) {
            Serial.println("BUTTON D1 PRESSED");
            mode = 1;
        }

        raw = "";
    }
    else {
        raw += c;
    }
}
