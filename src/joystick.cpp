#include <Arduino.h>
#include "joystick.h"
#include "eeprom_store.h"

String raw = "";
int joyX = 512;
int joyY = 512;

void processJoystickPacket(char c) {

    // BUTTON HANDLING FOR D0 / D1 WHEN SENT AS SINGLE-BYTE PACKETS
    if (c == 'D') {
        raw = "D";
        return;
    }

    if (raw == "D" && (c == '0' || c == '1' || c == '2')) {
        raw += c;

        if (raw == "D0") {
            Serial.println("BUTTON D0 PRESSED");
            mode = 0;
        }
        else if (raw == "D1") {
            Serial.println("BUTTON D1 PRESSED");
            mode = 1;
        }
        else if (raw == "D2") {
            Serial.println("BUTTON D2 PRESSED (EEPROM Read)");

            float savedAx, savedAy;
            loadMaxAccel(savedAx, savedAy);

            Serial.println("EEPROM Stored Acceleration:");
            Serial.print("MaxAx: "); Serial.println(savedAx);
            Serial.print("MaxAy: "); Serial.println(savedAy);
        }

        raw = "";
        return;
    }

    // EXISTING JOYSTICK PACKET PARSER
    if (c == 3) {  // ETX (end of packet)

        int start = raw.indexOf(char(2)); // STX
        if (start == -1) { raw = ""; return; }
        start++;

        int commaIndex = raw.indexOf(',', start);
        if (commaIndex == -1) { raw = ""; return; }

        String xs = raw.substring(start, commaIndex);
        String ys = raw.substring(commaIndex + 1);

        joyX = xs.toInt();
        joyY = ys.toInt();

        // Embedded button support inside full packets
        if (raw.indexOf("D0") != -1) {
            Serial.println("BUTTON D0 PRESSED");
            mode = 0;
        }

        if (raw.indexOf("D1") != -1) {
            Serial.println("BUTTON D1 PRESSED");
            mode = 1;
        }

        if (raw.indexOf("D2") != -1) {
            Serial.println("BUTTON D2 PRESSED (EEPROM Read)");
            
            float savedAx, savedAy;
            loadMaxAccel(savedAx, savedAy);

            Serial.println("EEPROM Stored Acceleration:");
            Serial.print("MaxAx: "); Serial.println(savedAx);
            Serial.print("MaxAy: "); Serial.println(savedAy);
        }

        raw = "";
    }
    else {
        raw += c;
    }
}
