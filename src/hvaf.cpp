#include "hvaf.h"

// Lệnh đi tới
void hvafWalkForward() {
    extern moveVector vector;
    vector.move.x = 1.0;
    vector.move.y = 0.0;
    vector.move.z = 0.0;
    vector.rotate.yaw = 0.0;
}

// Lệnh đứng yên
void hvafStandStill() {
    extern moveVector vector;
    vector.move.x = 0.0;
    vector.move.y = 0.0;
    vector.move.z = 0.0;
    vector.rotate.yaw = 0.0;
}
