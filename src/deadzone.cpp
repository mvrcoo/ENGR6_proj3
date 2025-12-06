#include "deadzone.h"

int idle = 40;

bool isForward(int joyX) {
    return joyX < (512 - idle);
}

bool isBackward(int joyX) {
    return joyX > (512 + idle);
}

bool isLeft(int joyY) {
    return joyY < (512 - idle);
}

bool isRight(int joyY) {
    return joyY > (512 + idle);
}
