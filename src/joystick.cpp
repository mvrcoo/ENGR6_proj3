#include <Arduino.h>

String raw = "";
int joyX = 512;
int joyY = 512;

void processJoystickPacket(char c) {

    if (c == 3) {  
        int commaIndex = raw.indexOf(',');
        if (commaIndex > 0) {
            String xs = raw.substring(raw.lastIndexOf(2) + 1, commaIndex);
            String ys = raw.substring(commaIndex + 1);

            joyX = xs.toInt();
            joyY = ys.toInt();
        }
        raw = "";
    } 
    else {
        raw += c;
    }
}
