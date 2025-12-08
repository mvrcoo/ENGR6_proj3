#include <Arduino.h>
#include <EEPROM.h>
#include "eeprom_store.h"

//eeprom addresses
int addrAx = 0;  //4 bytes
int addrAy = sizeof(float);  //4 bytes

void saveMaxAccel(float ax, float ay) {
    EEPROM.put(0, ax);  
    EEPROM.put(4, ay);  
}

void loadMaxAccel(float &ax, float &ay) {
    EEPROM.get(0, ax);
    EEPROM.get(4, ay);
}
