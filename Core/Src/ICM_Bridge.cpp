#include "ICM_Bridge.h"
#include "Adafruit_ICM20948.h"
#include "DCM.h"
#include "Madgwick_filter.h"

Adafruit_ICM20948 icm;
imu_sample sample;
IMU_Data_t imuData;

#include <math.h>

// Bắt lấy biến beta từ thư viện Madgwick_filter.c
#ifdef __cplusplus
extern "C" {
#endif
    extern volatile float beta;
#ifdef __cplusplus
}
#endif

#define GYRO_SCALE_FACTOR (90.000f / 89.750f)
// Biến đếm để xử lý Fast-Startup
static uint32_t madgwick_cycles = 0;

float get_dynamic_beta(float ax, float ay, float az, float gx_rad, float gy_rad, float gz_rad) {
    if (madgwick_cycles < 200) {
        madgwick_cycles++;
        return 2.5f;
    }

    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    float acc_error = fabsf(acc_mag - 1.0f);

    float gyro_mag_rad = sqrtf(gx_rad*gx_rad + gy_rad*gy_rad + gz_rad*gz_rad);
    float gyro_deg_sec = gyro_mag_rad * (180.0f / PI);

    // Nếu bị rung xóc tịnh tiến mạnh -> Chỉ tin Gyro
    if (acc_error > 0.2f) {
        return 0.015f;
    }
    // Nếu đang xoay tay -> Beta hơi tăng nhẹ để chống trễ
    else if (gyro_deg_sec > 10.0f) {
        return 0.1f;
    }
    // Đứng im tĩnh lặng -> Lọc mượt
    else {
        return 0.05f;
    }
}

// 1. BIAS CỦA ACCEL (Combined bias b)
static float acc_bias[3] = {-0.012699f, -0.036971f, -0.034054f};

static float acc_matrix[3][3] = {
    { 0.998711f,  0.001624f,  0.000653f},
    { 0.001624f,  0.996836f, -0.002039f},
    { 0.000653f, -0.002039f,  0.981116f}
};

// MAG_I2C_BIAS
//static float mag_bias[3] = {2.452717f, -173.116051f, 35.507810f};
//
//static float mag_matrix[3][3] = {
//    { 1.161587f,  -0.003090f,  0.016671f},
//    { -0.003090f,  1.077526f,  0.024262f},
//    { 0.016671f,  0.024262f,  1.101561f}
//};

static float mag_bias[3] = {-7.944105f, -144.542062f, 49.058884f};

static float mag_matrix[3][3] = {
    { 1.014933f, -0.010871f, 0.031214f},
    { -0.010871f, 0.892421f, 0.001520f},
    { 0.031214f, 0.001520f, 0.959793f}
};

#define ALPHA_ACC 0.925f
#define ALPHA_GYR 0.925f
#define ALPHA_MAG 1.000f
//#define ALPHA_MAG 0.550f


// EMA parameter
static float f_ax = 0, f_ay = 0, f_az = 0;
static float f_gx = 0, f_gy = 0, f_gz = 0;
static float f_mx = 0, f_my = 0, f_mz = 0;

void IMU_Init_SPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    // 1. Gán tham số để hàm cs_active hoạt động
    icm.setSPI_Pins(hspi, cs_port, cs_pin);

    // ========================================================
    // 2. NGHI THỨC ĐÁNH THỨC SPI BẮT BUỘC (SPI WAKE-UP SEQUENCE)
    // Phải lặp lại 2-3 lần để ép chip vào chế độ SPI
    // ========================================================
    HAL_Delay(10); // Chờ nguồn ổn định hoàn toàn

    if (icm.begin_SPI(hspi, cs_port, cs_pin)) {
        icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_50_4_HZ);
        icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_51_2_HZ);
        u_printf("SPI IMU INIT SUCCESS!\r\n");
    } else {
        u_printf("SPI IMU INIT FAILED!\r\n");
    }
}

void IMU_Init_I2C(I2C_HandleTypeDef *hi2c) {
    if (icm.begin_I2C(0x69, hi2c)) {
        icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_50_4_HZ);
        icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_51_2_HZ);
    }
}


static float dyn_g_off_x = 0;
static float dyn_g_off_y = 0;
static float dyn_g_off_z = 0;

void IMU_CalibGyro_Startup(void) {
    u_printf("\r\n[INFO] DANG CALIB GYRO. VUI LONG KHONG CHAM VAO MACH!!!\r\n");

    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int samples = 500;

    for (int i = 0; i < samples; i++) {
        icm.getSample(&sample);
        sum_gx += sample.gyroX;
        sum_gy += sample.gyroY;
        sum_gz += sample.gyroZ;
        HAL_Delay(5); // Chờ 5ms -> Lấy 500 mẫu mất 2.5 giây
    }

    dyn_g_off_x = sum_gx / samples;
    dyn_g_off_y = sum_gy / samples;
    dyn_g_off_z = sum_gz / samples;

}


void IMU_Update(void) {
    if (!icm.getSample(&sample)) return;

//	u_printf("%f %f %f\r\n", sample.magX, sample.magY, sample.magZ);
    float cal_gx = sample.gyroX - dyn_g_off_x;
    float cal_gy = sample.gyroY - dyn_g_off_y;
    float cal_gz = sample.gyroZ - dyn_g_off_z;

    float temp_ax = sample.accX - acc_bias[0];
    float temp_ay = sample.accY - acc_bias[1];
    float temp_az = sample.accZ - acc_bias[2];

    // Bước B: Nhân Ma trận 3x3 A^-1
    float cal_ax = acc_matrix[0][0]*temp_ax + acc_matrix[0][1]*temp_ay + acc_matrix[0][2]*temp_az;
    float cal_ay = acc_matrix[1][0]*temp_ax + acc_matrix[1][1]*temp_ay + acc_matrix[1][2]*temp_az;
    float cal_az = acc_matrix[2][0]*temp_ax + acc_matrix[2][1]*temp_ay + acc_matrix[2][2]*temp_az;

    //(Khử nhiễu Hard-Iron)
    float temp_mx = sample.magX - mag_bias[0];
    float temp_my = sample.magY - mag_bias[1];
    float temp_mz = sample.magZ - mag_bias[2];

    float cal_mx = mag_matrix[0][0]*temp_mx + mag_matrix[0][1]*temp_my + mag_matrix[0][2]*temp_mz;
    float cal_my = mag_matrix[1][0]*temp_mx + mag_matrix[1][1]*temp_my + mag_matrix[1][2]*temp_mz;
    float cal_mz = mag_matrix[2][0]*temp_mx + mag_matrix[2][1]*temp_my + mag_matrix[2][2]*temp_mz;

    //EMA
    if (f_ax == 0 && f_ay == 0 && f_az == 0) {
        f_ax = cal_ax; f_ay = cal_ay; f_az = cal_az;
        f_gx = cal_gx; f_gy = cal_gy; f_gz = cal_gz;
        f_mx = cal_mx; f_my = cal_my; f_mz = cal_mz;
    }

    f_ax = (ALPHA_ACC * cal_ax) + ((1.0f - ALPHA_ACC) * f_ax);
    f_ay = (ALPHA_ACC * cal_ay) + ((1.0f - ALPHA_ACC) * f_ay);
    f_az = (ALPHA_ACC * cal_az) + ((1.0f - ALPHA_ACC) * f_az);

    f_gx = (ALPHA_GYR * cal_gx) + ((1.0f - ALPHA_GYR) * f_gx);
    f_gy = (ALPHA_GYR * cal_gy) + ((1.0f - ALPHA_GYR) * f_gy);
    f_gz = (ALPHA_GYR * cal_gz) + ((1.0f - ALPHA_GYR) * f_gz);

    f_mx = (ALPHA_MAG * cal_mx) + ((1.0f - ALPHA_MAG) * f_mx);
    f_my = (ALPHA_MAG * cal_my) + ((1.0f - ALPHA_MAG) * f_my);
    f_mz = (ALPHA_MAG * cal_mz) + ((1.0f - ALPHA_MAG) * f_mz);

    // final update
    imuData.ax = f_ax; imuData.ay = f_ay; imuData.az = f_az;
    imuData.gx = f_gx; imuData.gy = f_gy; imuData.gz = f_gz;
    imuData.mx = f_mx; imuData.my = f_my; imuData.mz = f_mz;

    float gx_rad = f_gx * (PI / 180.0f);
    float gy_rad = f_gy * (PI / 180.0f);
    float gz_rad = f_gz * (PI / 180.0f) * GYRO_SCALE_FACTOR;

    beta = get_dynamic_beta(f_ax, f_ay, f_az, gx_rad, gy_rad, gz_rad);

    // CHẠY BƯỚC 2: MADGWICK FILTER
	MadgwickAHRSupdate(gx_rad, gy_rad, gz_rad, f_ax, f_ay, f_az, f_my, f_mx, -f_mz);
//	MadgwickAHRSupdateIMU(gx_rad, gy_rad, gz_rad, f_ax, f_ay, f_az);
    computeAngles(); // Tính Roll, Pitch, Yaw
    imuData.q0 = q0;
    imuData.q1 = q1;
    imuData.q2 = q2;
    imuData.q3 = q3;

}

IMU_Data_t IMU_GetData(void) {
    return imuData;
}
