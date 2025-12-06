#include <EEPROM.h>
#include "eeprom_store.h"

void saveMaxAccel(float ax, float ay) {
    EEPROM.put(0, ax);  
    EEPROM.put(4, ay);  
}

void loadMaxAccel(float &ax, float &ay) {
    EEPROM.get(0, ax);
    EEPROM.get(4, ay);
}
