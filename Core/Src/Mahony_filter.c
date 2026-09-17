//=====================================================================================================
// Mahony_filter.c
//=====================================================================================================
//
// Implementation of Mahony's explicit complementary filter (AHRS & IMU), viet thay the
// truc tiep cho Madgwick_filter.c. Nguyen ly: sai so giua vector trong luc/tu truong do
// duoc va vector trong luc/tu truong uoc luong tu quaternion hien tai duoc dua qua tich
// co huong (cross product) de ra mot vector loi (error). Loi nay duoc dung lam feedback
// (P + I) bom nguoc vao gyro truoc khi tich phan quaternion - tuong tu PI controller.
//
// So voi Madgwick (gradient descent), Mahony re hon ve CPU (khong can chuan hoa buoc
// gradient 4 chieu) va co khau tich phan (Ki) tu trieu tieu bias gyro rat tot khi
// dung yen lau - phu hop cho ung dung dinh vi vi tri (KinematicsEngine) vi bias gyro
// thap se giam troi quaternion, giam troi gia toc the gioi.
//
//=====================================================================================================

#include "Mahony_filter.h"
#include "DCM.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 100.0f

//---------------------------------------------------------------------------------------------------
// Variable definitions

// Gia tri mac dinh: Kp=1.0 (bam accel/mag kha nhanh), Ki=0.05 (triet bias gyro cham, on dinh)
// Neu rung xoc/gia toc tuyen tinh manh nhieu -> giam Kp de bot tin accel gia (gay ra do trung luc gia).
volatile float twoKp = 2.0f * 1.0f;
volatile float twoKi = 2.0f * 0.05f;
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
//extern float roll, pitch, yaw;

// Bo tich phan loi (integral feedback state) - tuong duong bias gyro uoc luong
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

//---------------------------------------------------------------------------------------------------
// Function declarations

static float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update (co magnetometer)

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Neu magnetometer khong hop le -> dung ban IMU (chi accel + gyro)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Chi tinh feedback neu accel hop le (tranh chia 0)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Chuan hoa accel
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // Chuan hoa mag
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // Cac bien phu de giam tinh toan lap lai
        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
        q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

        // Huong tu truong Trai Dat uoc luong tu quaternion (giong Madgwick)
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Uoc luong huong trong luc (v) va tu truong (w) trong body frame
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Loi = tich co huong giua vector do duoc va vector uoc luong (accel + mag)
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Khau tich phan (I) - tu trieu tieu bias gyro theo thoi gian
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        }

        // Khau ty le (P)
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Tich phan gyro (da hieu chinh) de ra quaternion moi
    gx *= (0.5f * (1.0f / sampleFreq));
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Chuan hoa quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update (khong magnetometer)

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Chuan hoa accel
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // Uoc luong huong trong luc trong body frame (nua cot Z cua ma tran xoay)
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Loi = tich co huong giua accel do duoc va trong luc uoc luong
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Khau tich phan (I)
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        }

        // Khau ty le (P)
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Tich phan gyro (da hieu chinh) de ra quaternion moi
    gx *= (0.5f * (1.0f / sampleFreq));
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Chuan hoa quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root (ban an toan, khong dung bit-hack de tranh sai so tren FPU M4/M7)

static float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}
//====================================================================================================
// END OF CODE
//====================================================================================================
