#ifndef DEADZONE_H
#define DEADZONE_H

// Global deadzone threshold
extern int idle;

// Deadzone evaluation functions
bool isForward(int joyX);
bool isBackward(int joyX);
bool isLeft(int joyY);
bool isRight(int joyY);

#endif
