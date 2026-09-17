#ifndef KINEMATICS_ENGINE_H
#define KINEMATICS_ENGINE_H

#include "main.h"
#include "Kalman.h"

struct PositionData {
    float x, y, z;
};

class KinematicsEngine {
public:
    KinematicsEngine();
    void reset();

    PositionData getVelocity();
    PositionData update(float ax, float ay, float az,
                        float q0, float q1, float q2, float q3,
                        float dt);

private:
    Kalman1D kfX, kfY, kfZ;
    uint8_t zvu_counter[3] = {0, 0, 0};

    const float ZVU_THRESHOLD_XY = 0.03f;
    const float ZVU_THRESHOLD_Z  = 0.05f;
    const uint8_t ZVU_LIMIT = 5;

    void removeGravityAndToWorldFrame(float ax, float ay, float az,
                                      float q0, float q1, float q2, float q3,
                                      float &wX, float &wY, float &wZ);
};

#endif // KINEMATICS_ENGINE_H
