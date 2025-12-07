#ifndef JOYSTICK_H
#define JOYSTICK_H

extern int joyX;
extern int joyY;

extern int mode;

void processJoystickPacket(char c);

#endif
