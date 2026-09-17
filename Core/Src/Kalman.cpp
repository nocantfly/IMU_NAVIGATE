#include "Kalman.h"

// =========================================================================
// LINEAR KALMAN FILTER 1D IMPLEMENTATION
// =========================================================================

Kalman1D::Kalman1D() {
    init(0.01f, 0.05f, 0.001f);
}

void Kalman1D::init(float delta_t, float process_noise, float measure_noise) {
    dt = delta_t;

    arm_mat_init_f32(&X, 2, 1, (float32_t *)X_data);
    arm_mat_init_f32(&P, 2, 2, (float32_t *)P_data);
    arm_mat_init_f32(&A, 2, 2, (float32_t *)A_data);
    arm_mat_init_f32(&A_T, 2, 2, (float32_t *)A_T_data);
    arm_mat_init_f32(&B, 2, 1, (float32_t *)B_data);
    arm_mat_init_f32(&H, 1, 2, (float32_t *)H_data);
    arm_mat_init_f32(&H_T, 2, 1, (float32_t *)H_T_data);
    arm_mat_init_f32(&Q, 2, 2, (float32_t *)Q_data);
    arm_mat_init_f32(&R, 1, 1, (float32_t *)R_data);

    arm_mat_init_f32(&temp2x2, 2, 2, (float32_t *)temp2x2_data);
    arm_mat_init_f32(&temp2x1, 2, 1, (float32_t *)temp2x1_data);
    arm_mat_init_f32(&temp1x1, 1, 1, (float32_t *)temp1x1_data);
    arm_mat_init_f32(&temp2x2_2, 2, 2, (float32_t *)temp2x2_2_data);
    arm_mat_init_f32(&temp1x2, 1, 2, (float32_t *)temp1x2_data);

    A_data[0] = 1.0f; A_data[1] = dt;
    A_data[2] = 0.0f; A_data[3] = 1.0f;
    arm_mat_trans_f32(&A, &A_T);

    B_data[0] = 0.5f * dt * dt;
    B_data[1] = dt;

    H_data[0] = 0.0f; H_data[1] = 1.0f;
    arm_mat_trans_f32(&H, &H_T);

    Q_data[0] = process_noise * 0.25f * dt * dt * dt * dt; Q_data[1] = process_noise * 0.5f * dt * dt * dt;
    Q_data[2] = process_noise * 0.5f * dt * dt * dt;       Q_data[3] = process_noise * dt * dt;

    R_data[0] = measure_noise;

    reset();
}

void Kalman1D::reset() {
    X_data[0] = 0.0f; X_data[1] = 0.0f;
    P_data[0] = 1.0f; P_data[1] = 0.0f;
    P_data[2] = 0.0f; P_data[3] = 1.0f;
}

void Kalman1D::predict(float u_accel) {
    arm_mat_mult_f32(&A, &X, &temp2x1);
    arm_mat_scale_f32(&B, u_accel, &X);
    arm_mat_add_f32(&temp2x1, &X, &X);

    arm_mat_mult_f32(&A, &P, &temp2x2);
    arm_mat_mult_f32(&temp2x2, &A_T, &temp2x2_2);
    arm_mat_add_f32(&temp2x2_2, &Q, &P);
}

void Kalman1D::update(float z_vel) {
    float y_err = z_vel - X_data[1];
    float S = P_data[3] + R_data[0];
    float S_inv = 1.0f / S;

    float K0 = P_data[1] * S_inv;
    float K1 = P_data[3] * S_inv;

    X_data[0] += K0 * y_err;
    X_data[1] += K1 * y_err;

    float P00 = P_data[0], P01 = P_data[1];
    float P10 = P_data[2], P11 = P_data[3];

    P_data[0] = P00 - K0 * P10;
    P_data[1] = P01 - K0 * P11;
    P_data[2] = P10 - K1 * P10;
    P_data[3] = P11 - K1 * P11;
}

// =========================================================================
// EXTENDED KALMAN FILTER IMPLEMENTATION (Template)
// =========================================================================

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    init(0.01f, 0.001f, 0.05f, 0.1f); // Mặc định 100Hz
}

void ExtendedKalmanFilter::init(float delta_t, float gyro_noise, float accel_noise, float mag_noise) {
    dt = delta_t;

    // Gắn vùng nhớ cho các ma trận
    arm_mat_init_f32(&X, 4, 1, X_data);
    arm_mat_init_f32(&P, 4, 4, P_data);
    arm_mat_init_f32(&F, 4, 4, F_data);
    arm_mat_init_f32(&F_T, 4, 4, F_T_data);
    arm_mat_init_f32(&Q, 4, 4, Q_data);

    arm_mat_init_f32(&Z, 6, 1, Z_data);
    arm_mat_init_f32(&H, 6, 4, H_data);
    arm_mat_init_f32(&H_T, 4, 6, H_T_data);
    arm_mat_init_f32(&R, 6, 6, R_data);

    arm_mat_init_f32(&Y, 6, 1, Y_data);
    arm_mat_init_f32(&S, 6, 6, S_data);
    arm_mat_init_f32(&S_inv, 6, 6, S_inv_data);
    arm_mat_init_f32(&K, 4, 6, K_data);
    arm_mat_init_f32(&I, 4, 4, I_data);

    // Ma trận tạm
    arm_mat_init_f32(&temp4x4_1, 4, 4, t4x4_1);
    arm_mat_init_f32(&temp4x4_2, 4, 4, t4x4_2);
    arm_mat_init_f32(&temp6x4_1, 6, 4, t6x4_1);
    arm_mat_init_f32(&temp4x6_1, 4, 6, t4x6_1);
    arm_mat_init_f32(&temp6x6_1, 6, 6, t6x6_1);
    arm_mat_init_f32(&temp4x1_1, 4, 1, t4x1_1);
    arm_mat_init_f32(&temp6x1_1, 6, 1, t6x1_1);

    // Khởi tạo Ma trận Nhiễu Hệ thống (Q) - Dựa trên Gyro
    for(int i=0; i<16; i++) Q_data[i] = 0.0f;
    Q_data[0] = Q_data[5] = Q_data[10] = Q_data[15] = gyro_noise;

    // Khởi tạo Ma trận Nhiễu Đo lường (R) - Nửa đầu Accel, nửa sau Mag
    for(int i=0; i<36; i++) R_data[i] = 0.0f;
    R_data[0] = R_data[7] = R_data[14] = accel_noise;
    R_data[21] = R_data[28] = R_data[35] = mag_noise;

    // Khởi tạo Ma trận Đơn vị (I)
    for(int i=0; i<16; i++) I_data[i] = 0.0f;
    I_data[0] = I_data[5] = I_data[10] = I_data[15] = 1.0f;

    reset();
}

void ExtendedKalmanFilter::reset() {
    // Trạng thái ban đầu: Không xoay (Quaternion = [1, 0, 0, 0])
    X_data[0] = 1.0f; X_data[1] = 0.0f; X_data[2] = 0.0f; X_data[3] = 0.0f;

    // Covariance P ban đầu nhỏ
    for(int i=0; i<16; i++) P_data[i] = 0.0f;
    P_data[0] = P_data[5] = P_data[10] = P_data[15] = 0.01f;
}

void ExtendedKalmanFilter::normalizeQuaternion() {
    float norm = sqrtf(X_data[0]*X_data[0] + X_data[1]*X_data[1] + X_data[2]*X_data[2] + X_data[3]*X_data[3]);
    if (norm > 0.0f) {
        X_data[0] /= norm;
        X_data[1] /= norm;
        X_data[2] /= norm;
        X_data[3] /= norm;
    }
}

// BƯỚC 1: DỰ ĐOÁN (PREDICT)
void ExtendedKalmanFilter::predict(float gx, float gy, float gz) {
    // Tính toán Jacobian F(x) = I + 0.5 * dt * Omega
    float half_dt = 0.5f * dt;

    F_data[0] = 1.0f;           F_data[1] = -half_dt * gx;  F_data[2] = -half_dt * gy;  F_data[3] = -half_dt * gz;
    F_data[4] = half_dt * gx;   F_data[5] = 1.0f;           F_data[6] =  half_dt * gz;  F_data[7] = -half_dt * gy;
    F_data[8] = half_dt * gy;   F_data[9] = -half_dt * gz;  F_data[10]= 1.0f;           F_data[11]=  half_dt * gx;
    F_data[12]= half_dt * gz;   F_data[13]=  half_dt * gy;  F_data[14]= -half_dt * gx;  F_data[15]= 1.0f;

    // 1. X_new = F * X (Tích phân Gyro vào Quaternion)
    arm_mat_mult_f32(&F, &X, &temp4x1_1);
    X_data[0] = temp4x1_1.pData[0];
    X_data[1] = temp4x1_1.pData[1];
    X_data[2] = temp4x1_1.pData[2];
    X_data[3] = temp4x1_1.pData[3];
    normalizeQuaternion(); // Bắt buộc chuẩn hóa

    // 2. P_new = F * P * F^T + Q
    arm_mat_trans_f32(&F, &F_T);
    arm_mat_mult_f32(&F, &P, &temp4x4_1);        // temp_1 = F*P
    arm_mat_mult_f32(&temp4x4_1, &F_T, &temp4x4_2); // temp_2 = F*P*F^T
    arm_mat_add_f32(&temp4x4_2, &Q, &P);         // P = temp_2 + Q
}

// BƯỚC 2: CẬP NHẬT (UPDATE)
void ExtendedKalmanFilter::update(float ax, float ay, float az, float mx, float my, float mz) {
    // Lấy Quaternion dự đoán
    float q0 = X_data[0], q1 = X_data[1], q2 = X_data[2], q3 = X_data[3];

    // 1. Tính Ma trận Đo lường Dự đoán h(x) (Predicted Measurement)
    // - Nửa đầu: Trọng lực g (Chỉ theo trục Z của Trái đất)
    float h_acc[3];
    h_acc[0] = 2.0f * (q1*q3 - q0*q2);
    h_acc[1] = 2.0f * (q0*q1 + q2*q3);
    h_acc[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // - Nửa sau: Từ trường Trái đất b (Xoay vector Mag về Earth Frame, tìm thành phần b_x và b_z, rồi xoay ngược lại)
    float hx = mx*q0*q0 - 2.0f*my*q0*q3 + 2.0f*mz*q0*q2 + mx*q1*q1 + 2.0f*my*q1*q2 + 2.0f*mz*q1*q3 - mx*q2*q2 - mx*q3*q3;
    float hy = 2.0f*mx*q0*q3 + my*q0*q0 - 2.0f*mz*q0*q1 + 2.0f*mx*q1*q2 - my*q1*q1 + my*q2*q2 + 2.0f*mz*q2*q3 - my*q3*q3;
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = -2.0f*mx*q0*q2 + 2.0f*my*q0*q1 + mz*q0*q0 + 2.0f*mx*q1*q3 - mz*q1*q1 + 2.0f*my*q2*q3 - mz*q2*q2 + mz*q3*q3;

    float h_mag[3];
    h_mag[0] = bx*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz*(2.0f*(q1*q3 - q0*q2));
    h_mag[1] = bx*(2.0f*(q1*q2 - q0*q3)) + bz*(2.0f*(q0*q1 + q2*q3));
    h_mag[2] = bx*(2.0f*(q0*q2 + q1*q3)) + bz*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

    // 2. Tính Y = Z_measure - h(x) (Innovation / Sai số đo lường)
    // Chú ý: Cần chuẩn hóa vector Accel và Mag trước khi đo
    float norm_a = sqrtf(ax*ax + ay*ay + az*az);
    if(norm_a > 0.0f) { ax /= norm_a; ay /= norm_a; az /= norm_a; }

    float norm_m = sqrtf(mx*mx + my*my + mz*mz);
    if(norm_m > 0.0f) { mx /= norm_m; my /= norm_m; mz /= norm_m; }

    Y_data[0] = ax - h_acc[0]; Y_data[1] = ay - h_acc[1]; Y_data[2] = az - h_acc[2];
    Y_data[3] = mx - h_mag[0]; Y_data[4] = my - h_mag[1]; Y_data[5] = mz - h_mag[2];

    // 3. Tính Jacobian H(x) (Đạo hàm riêng của h(x) theo q0, q1, q2, q3)
    // Jacobian Accel (3x4)
    H_data[0] = -2.0f*q2; H_data[1] = 2.0f*q3;  H_data[2] = -2.0f*q0; H_data[3] = 2.0f*q1;
    H_data[4] = 2.0f*q1;  H_data[5] = 2.0f*q0;  H_data[6] = 2.0f*q3;  H_data[7] = 2.0f*q2;
    H_data[8] = 2.0f*q0;  H_data[9] = -2.0f*q1; H_data[10]= -2.0f*q2; H_data[11]= 2.0f*q3;

    // Jacobian Mag (3x4)
    H_data[12] = 2.0f*bx*q0 - 2.0f*bz*q2; H_data[13] = 2.0f*bx*q1 + 2.0f*bz*q3; H_data[14] = -2.0f*bx*q2 - 2.0f*bz*q0; H_data[15] = -2.0f*bx*q3 + 2.0f*bz*q1;
    H_data[16] = -2.0f*bx*q3 + 2.0f*bz*q1;H_data[17] = 2.0f*bx*q2 + 2.0f*bz*q0; H_data[18] = 2.0f*bx*q1 + 2.0f*bz*q3;  H_data[19] = -2.0f*bx*q0 + 2.0f*bz*q2;
    H_data[20] = 2.0f*bx*q2 + 2.0f*bz*q0; H_data[21] = 2.0f*bx*q3 - 2.0f*bz*q1; H_data[22] = 2.0f*bx*q0 - 2.0f*bz*q2;  H_data[23] = 2.0f*bx*q1 + 2.0f*bz*q3;

    arm_mat_trans_f32(&H, &H_T);

    // 4. Tính S = H * P * H^T + R
    arm_mat_mult_f32(&H, &P, &temp6x4_1);        // temp6x4 = H*P
    arm_mat_mult_f32(&temp6x4_1, &H_T, &temp6x6_1); // temp6x6 = H*P*H^T
    arm_mat_add_f32(&temp6x6_1, &R, &S);         // S = temp6x6 + R

    // 5. Tính Kalman Gain K = P * H^T * S^-1
    arm_mat_inverse_f32(&S, &S_inv);             // Tính S^-1 (Rất tốn CPU, thư viện ARM lo phần này)
    arm_mat_mult_f32(&P, &H_T, &temp4x6_1);      // temp4x6 = P*H^T
    arm_mat_mult_f32(&temp4x6_1, &S_inv, &K);    // K = P*H^T * S^-1

    // 6. Cập nhật Trạng thái: X = X + K * Y
    arm_mat_mult_f32(&K, &Y, &temp4x1_1);        // temp4x1 = K*Y
    arm_mat_add_f32(&X, &temp4x1_1, &X);         // X = X + K*Y
    normalizeQuaternion();

    // 7. Cập nhật Covariance: P = (I - K*H) * P
    arm_mat_mult_f32(&K, &H, &temp4x4_1);        // temp4x4_1 = K*H
    arm_mat_sub_f32(&I, &temp4x4_1, &temp4x4_2); // temp4x4_2 = I - K*H

    // Backup P cũ để nhân (vì CMSIS-DSP không cho phép Output đè lên Input ở hàm mult)
    float P_backup[16];
    memcpy(P_backup, P_data, 16 * sizeof(float));
    arm_matrix_instance_f32 P_old;
    arm_mat_init_f32(&P_old, 4, 4, P_backup);

    arm_mat_mult_f32(&temp4x4_2, &P_old, &P);    // P = (I - K*H) * P_old
}

void ExtendedKalmanFilter::getQuaternion(float &q0, float &q1, float &q2, float &q3) {
    q0 = X_data[0];
    q1 = X_data[1];
    q2 = X_data[2];
    q3 = X_data[3];
}
