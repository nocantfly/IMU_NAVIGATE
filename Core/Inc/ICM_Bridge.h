#ifndef ICM_BRIDGE_H
#define ICM_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// Struct chứa toàn bộ dữ liệu trả về cho main
typedef struct {
    float q0, q1, q2, q3;   // Quaternion
    float roll, pitch, yaw; // Góc Euler (Độ)
    float ax, ay, az;       // Gia tốc (g)
    float gx, gy, gz;       // Vận tốc góc (Độ/s)
    float mx, my, mz;       // Từ trường (uT)
} IMU_Data_t;
void IMU_CalibGyro_Startup(void);
void IMU_Init_SPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void IMU_Init_I2C(I2C_HandleTypeDef *hi2c);
void IMU_Update(void);
IMU_Data_t IMU_GetData(void);

#ifdef __cplusplus
}
#endif

#endif
