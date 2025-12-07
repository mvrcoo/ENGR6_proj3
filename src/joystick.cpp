#include <Arduino.h>
#include "joystick.h"

String raw = "";
int joyX = 512;
int joyY = 512;
int joyButton = 0;

void processJoystickPacket(char c) {

    if (c == 3) {  
        int commaIndex = raw.indexOf(',');
        int secondCommaIndex = raw.indexOf(',', commaIndex + 1);
        if (commaIndex > 0 && secondCommaIndex > commaIndex) {
            String xs = raw.substring(raw.lastIndexOf(2) + 1, commaIndex);
            String ys = raw.substring(commaIndex + 1, secondCommaIndex);
            String sw = raw.substring(secondCommaIndex + 1);

            joyX = xs.toInt();
            joyY = ys.toInt();
            joyButton = sw.toInt();
        }
        raw = "";
    } 
    else {
        raw += c;
    }
}
