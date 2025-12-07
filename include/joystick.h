#ifndef JOYSTICK_H
#define JOYSTICK_H

extern int joyX;
extern int joyY;
extern int joyButton;

void processJoystickPacket(char c);

#endif
