#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif

// =========================================================================
// 1. LINEAR KALMAN FILTER 1D (Cho Tích phân Vị trí - Vận tốc)
// =========================================================================
class Kalman1D {
public:
    Kalman1D();
    void init(float dt, float process_noise, float measure_noise);
    void reset();
    void predict(float u_accel);
    void update(float z_vel);

    float getPosition() const { return X_data[0]; }
    float getVelocity() const { return X_data[1]; }

private:
    float dt;
    arm_matrix_instance_f32 X, P, A, A_T, B, H, H_T, Q, R;
    float X_data[2], P_data[4], A_data[4], A_T_data[4], B_data[2];
    float H_data[2], H_T_data[2], Q_data[4], R_data[1];

    arm_matrix_instance_f32 temp2x2, temp2x1, temp1x1, temp2x2_2, temp1x2;
    float temp2x2_data[4], temp2x1_data[2], temp1x1_data[1], temp2x2_2_data[4], temp1x2_data[2];
};

// =========================================================================
// 2. EXTENDED KALMAN FILTER (EKF 9-DoF AHRS)
// Trạng thái (State): q0, q1, q2, q3 (Quaternion)
// =========================================================================
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    // Khởi tạo EKF với thời gian lấy mẫu (dt) và các hệ số nhiễu
    void init(float delta_t, float gyro_noise, float accel_noise, float mag_noise);

    void reset();

    // Bước 1: Dự đoán Quaternion mới dựa trên Gyroscope (gx, gy, gz tính bằng rad/s)
    void predict(float gx, float gy, float gz);

    // Bước 2: Cập nhật Quaternion dựa trên Accelerometer và Magnetometer
    // Đơn vị: ax, ay, az (g) ; mx, my, mz (uT hoặc normalized)
    void update(float ax, float ay, float az, float mx, float my, float mz);

    // Lấy Quaternion hiện tại
    void getQuaternion(float &q0, float &q1, float &q2, float &q3);

private:
    float dt;

    // Các ma trận lõi của EKF (Kích thước State = 4, Measurement = 6)
    arm_matrix_instance_f32 X;     // State: [q0, q1, q2, q3]^T (4x1)
    arm_matrix_instance_f32 P;     // Covariance (4x4)
    arm_matrix_instance_f32 F;     // State Transition Jacobian (4x4)
    arm_matrix_instance_f32 F_T;   // Transpose of F (4x4)
    arm_matrix_instance_f32 Q;     // Process Noise (Gyro) (4x4)

    arm_matrix_instance_f32 Z;     // Measurement: [ax, ay, az, mx, my, mz]^T (6x1)
    arm_matrix_instance_f32 H;     // Measurement Jacobian (6x4)
    arm_matrix_instance_f32 H_T;   // Transpose of H (4x6)
    arm_matrix_instance_f32 R;     // Measurement Noise (Accel + Mag) (6x6)

    // Các ma trận hỗ trợ tính toán Update
    arm_matrix_instance_f32 Y;     // Innovation / Error (6x1)
    arm_matrix_instance_f32 S;     // Innovation Covariance (6x6)
    arm_matrix_instance_f32 S_inv; // Inverse of S (6x6)
    arm_matrix_instance_f32 K;     // Kalman Gain (4x6)
    arm_matrix_instance_f32 I;     // Identity Matrix (4x4)

    // Vùng nhớ RAM cho các ma trận EKF
    float X_data[4];
    float P_data[16];
    float F_data[16];
    float F_T_data[16];
    float Q_data[16];

    float Z_data[6];
    float H_data[24];
    float H_T_data[24];
    float R_data[36];

    float Y_data[6];
    float S_data[36];
    float S_inv_data[36];
    float K_data[24];
    float I_data[16];

    // Các ma trận trung gian (Temp)
    arm_matrix_instance_f32 temp4x4_1, temp4x4_2;
    arm_matrix_instance_f32 temp6x4_1;
    arm_matrix_instance_f32 temp4x6_1;
    arm_matrix_instance_f32 temp6x6_1;
    arm_matrix_instance_f32 temp4x1_1, temp6x1_1;

    float t4x4_1[16], t4x4_2[16];
    float t6x4_1[24];
    float t4x6_1[24];
    float t6x6_1[36];
    float t4x1_1[4], t6x1_1[6];

    // Hàm chuẩn hóa Quaternion nội bộ
    void normalizeQuaternion();
};

#endif // KALMAN_H
