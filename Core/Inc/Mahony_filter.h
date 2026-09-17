/*
 * Mahony_filter.h
 *
 *  Mahony AHRS/IMU filter - thay thế trực tiếp cho Madgwick_filter.
 *
 *  QUAN TRONG: file nay dung chung ten bien toan cuc (q0,q1,q2,q3,
 *  roll,pitch,yaw) va ham computeAngles() voi Madgwick_filter.c de co
 *  the "cam" thang vao code hien tai (ICM_Bridge.cpp, main.cpp) ma
 *  khong phai sua noi nao khac. Vi vay KHONG duoc build ca 2 file
 *  Madgwick_filter.c va Mahony_filter.c cung luc (se bi loi linker
 *  "multiple definition"). Muon doi sang Mahony:
 *    1. Go Core/Src/Madgwick_filter.c ra khoi build (hoac xoa).
 *    2. Them Core/Src/Mahony_filter.c + Core/Inc/Mahony_filter.h vao project.
 *    3. Trong ICM_Bridge.cpp: doi include "Madgwick_filter.h" -> "Mahony_filter.h",
 *       va doi loi goi MadgwickAHRSupdate(...) -> MahonyAHRSupdate(...).
 */

#ifndef INC_MAHONY_FILTER_H_
#define INC_MAHONY_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------------------------------
// Variable declaration
//#define PI 3.141592
//#define R2D 180.00f/3.141592f

// twoKp = 2 * Kp (he so feedback ty le - gain "manh", giong beta cua Madgwick)
// twoKi = 2 * Ki (he so feedback tich phan - triet tieu bias gyro theo thoi gian)
extern volatile float twoKp;
extern volatile float twoKi;
//extern volatile float q0, q1, q2, q3;  // quaternion cua sensor frame so voi auxiliary frame
//extern float roll, pitch, yaw;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
//void computeAngles_Mah(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAHONY_FILTER_H_ */
