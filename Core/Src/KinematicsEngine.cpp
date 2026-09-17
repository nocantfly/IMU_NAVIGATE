#include "KinematicsEngine.h"

KinematicsEngine::KinematicsEngine() {
    reset();
}

void KinematicsEngine::reset() {
    kfX.reset();
    kfY.reset();
    kfZ.reset();
    zvu_counter[0] = 0;
    zvu_counter[1] = 0;
    zvu_counter[2] = 0;
}

void KinematicsEngine::removeGravityAndToWorldFrame(float ax, float ay, float az,
                                                    float q0, float q1, float q2, float q3,
                                                    float &wX, float &wY, float &wZ) {
    float gravX = 2.0f * (q1 * q3 - q0 * q2);
    float gravY = 2.0f * (q0 * q1 + q2 * q3);
    float gravZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float linX = ax - gravX;
    float linY = ay - gravY;
    float linZ = az - gravZ;

    float q0q0 = q0*q0; float q1q1 = q1*q1; float q2q2 = q2*q2; float q3q3 = q3*q3;

    wX = (q0q0+q1q1-q2q2-q3q3)*linX + 2.0f*(q1*q2-q0*q3)*linY + 2.0f*(q1*q3+q0*q2)*linZ;
    wY = 2.0f*(q1*q2+q0*q3)*linX + (q0q0-q1q1+q2q2-q3q3)*linY + 2.0f*(q2*q3-q0*q1)*linZ;
    wZ = 2.0f*(q1*q3-q0*q2)*linX + 2.0f*(q2*q3+q0*q1)*linY + (q0q0-q1q1-q2q2+q3q3)*linZ;
}

PositionData KinematicsEngine::update(float ax, float ay, float az,
                                      float q0, float q1, float q2, float q3,
                                      float dt) {
    float wX, wY, wZ;
    removeGravityAndToWorldFrame(ax, ay, az, q0, q1, q2, q3, wX, wY, wZ);

    float accInput[3] = {wX, wY, wZ};
    float thresholds[3] = {ZVU_THRESHOLD_XY, ZVU_THRESHOLD_XY, ZVU_THRESHOLD_Z};
    Kalman1D* kf[3] = {&kfX, &kfY, &kfZ};

    const float gScale = 981.0f; // Chuyển g sang cm/s^2

    for (int i = 0; i < 3; i++) {
        float a_cmss = accInput[i] * gScale;

        kf[i]->predict(a_cmss);

        if (fabsf(accInput[i]) < thresholds[i]) {
            zvu_counter[i]++;

            if (zvu_counter[i] >= ZVU_LIMIT) {
                kf[i]->update(0.0f);
            }
        } else {
            zvu_counter[i] = 0;
        }
    }

    return {kfX.getPosition(), kfY.getPosition(), kfZ.getPosition()};
}

PositionData KinematicsEngine::getVelocity() {
    return {kfX.getVelocity(), kfY.getVelocity(), kfZ.getVelocity()};
}
