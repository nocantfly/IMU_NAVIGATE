/*!
 *  @file Adafruit_ICM20X.cpp
 *
 *  @mainpage Adafruit ICM20X family motion sensor library
 *  @see Adafruit_ICM20649, Adafruit_ICM20948
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit ICM20X Family of motion sensors
 *
 * 	This is a library for the Adafruit ICM20X breakouts:
 * 	* https://www.adafruit.com/product/4464
 *
 * 	* https://www.adafruit.com/product/4554
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 * * [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
 *
 * * [Adafruit Unified Sensor
 * Driver](https://github.com/adafruit/Adafruit_Sensor)
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *    See [the repository's  release page for the release
 * history](https://github.com/adafruit/Adafruit_ICM20X/releases)
 */

// good reference: https://os.mbed.com/teams/SiliconLabs/code/ICM20648/file/296308a935f5/ICM20648.cpp/

#include "Adafruit_ICM20X.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

#include "cmsis_os2.h"
#define delay osDelay
uint8_t debug=1;

uint8_t arr[4];
//#define MAX_FIFO_SIZE	4096
//static uint8_t tx_data[MAX_FIFO_SIZE + 1] = {0};
//static uint8_t rx_data[MAX_FIFO_SIZE + 1] = {0};
Adafruit_ICM20X::Adafruit_ICM20X(void) {
}

/*!
 *    @brief  Cleans up the ICM20X
 */
Adafruit_ICM20X::~Adafruit_ICM20X(void) {
	if (accel_sensor) delete accel_sensor;
	if (gyro_sensor) delete gyro_sensor;
	if (mag_sensor) delete mag_sensor;
	if (temp_sensor) delete temp_sensor;
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 */
bool Adafruit_ICM20X::begin_I2C(uint8_t i2c_addr_in, I2C_HandleTypeDef *i2c_handle, int32_t sensor_id) {
	if (i2c_handle == NULL) {
		return false;
	}

	i2c_han = i2c_handle;
	// HAL STM32 yêu cầu địa chỉ I2C phải dịch trái 1 bit (7-bit addr << 1)
	i2c_addr = i2c_addr_in << 1;

	return _init(sensor_id);
}

bool Adafruit_ICM20X::begin_SPI(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin, int32_t sensor_id) {
    if (spi_handle == NULL) return false;

    // Gán biến
    spi_han = spi_handle;
    _cs_port = cs_port;
    _cs_pin = cs_pin;

    // 1. Tát CS để ép vào SPI Mode
    HAL_Delay(100);
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
        HAL_Delay(10);
    }

    // 2. TEST SPI SỐNG CHẾT (Đọc WHO_AM_I trực tiếp)
    uint8_t id = readRegisterByte(ICM20X_B0_WHOAMI);
    debug = id;
    u_printf("\r\n>> SPI TEST WHO_AM_I: 0x%02X\r\n", id);
//    if (id != 0xEA) {
//        u_printf(">> SPI CHET! Khong the giao tiep voi InvenSense!\r\n");
//        return false;
//    }

    // ========================================================
    // 3. VŨ KHÍ HẠT NHÂN: ÉP MASTER I2C VÀ TẮT I2C SLAVE CÙNG 1 LÚC
    // ========================================================
    // Chuyển về Bank 0 cho chắc
    _setBank(0);
    // Bật I2C_MST_EN (Bit 5) và Tắt I2C Slave (Bit 4) TRONG CÙNG 1 LỆNH
    // Ghi 0x30 vào USER_CTRL (0x03)
    writeRegisterByte(ICM20X_B0_USER_CTRL, 0x30);
    HAL_Delay(50); // Chờ cho nó cấu hình xong

    // Reset lại Master I2C một lần nữa cho sạch
    uint8_t user_ctrl = readRegisterByte(ICM20X_B0_USER_CTRL);
    writeRegisterByte(ICM20X_B0_USER_CTRL, user_ctrl | 0x02); // Bật cờ Reset
    HAL_Delay(10);
    writeRegisterByte(ICM20X_B0_USER_CTRL, user_ctrl & ~0x02); // Tắt cờ Reset
    HAL_Delay(50);

    // ========================================================
    // 4. TIẾN HÀNH KHỞI TẠO BÌNH THƯỜNG
    // ========================================================
    bool init_success = _init(sensor_id);

    if (init_success) {
        // Lúc này I2C Master đã sống, setupMag sẽ chạy mượt!
        init_success = setupMag();
    }

    is_initialized = init_success;
    return init_success;
}
void Adafruit_ICM20X::updateGyroSettings(uint8_t gyroLPFEn, uint8_t gyroLPFCutoff, uint8_t gyroRange, uint8_t gyroSampleRate) {
	_gyroLPFEn = gyroLPFEn;
	_gyroLPFCutoff = gyroLPFCutoff;
	_gyroRange = gyroRange;
	_gyroSampleRate = gyroSampleRate;
}

void Adafruit_ICM20X::updateAccelSettings(uint8_t accelLPFEn, uint8_t accelLPFCutoff, uint8_t accelRange, uint8_t accelSampleRate) {
	_accelLPFEn = accelLPFEn;
	_accelLPFCutoff = accelLPFCutoff;
	_accelRange = accelRange;
	_accelSampleRate = accelSampleRate;
}

// I2C không dùng chân CS, để hàm trống
void Adafruit_ICM20X::cs_active(bool state) {
    if (state) {
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET); // Kéo LOW để nói chuyện
    } else {
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);   // Kéo HIGH để nghỉ
    }
}
/*!
 * @brief Reset the internal registers and restores the default settings
 */
void Adafruit_ICM20X::reset(void) {
	_setBank(0);
	modifyRegisterBit(ICM20X_B0_PWR_MGMT_1, 1, 7);
	delay(20);
	while (checkRegisterBit(ICM20X_B0_PWR_MGMT_1, 7)) {
		delay(10);
	};
	delay(50);
}

/*!  @brief Initilizes the sensor */
bool Adafruit_ICM20X::_init(int32_t sensor_id) {
    _setBank(0);

    uint8_t chip_id_ = readRegisterByte(ICM20X_B0_WHOAMI);

    if ((chip_id_ != ICM20649_CHIP_ID) && (chip_id_ != ICM20948_CHIP_ID)) {
        return false;
    }

    _sensorid_accel = sensor_id;
    _sensorid_gyro = sensor_id + 1;
    _sensorid_mag = sensor_id + 2;
    _sensorid_temp = sensor_id + 3;

    // Reset toàn bộ thanh ghi cảm biến về mặc định
    reset();

    // KHÔI PHỤC CỜ KHÓA I2C NGOÀI (I2C_IF_DIS) NGAY LẬP TỨC NẾU CHẠY SPI
    if (spi_han != NULL) {
        writeRegisterByte(ICM20X_B0_USER_CTRL, 0x10); // Ép cứng chế độ SPI để tránh nhiễu bus ngoài
        delay(10);
    }

    writeRegisterByte(ICM20X_B0_PWR_MGMT_1, 0x01); // Wake up & Auto-select clock
    delay(10);
    _setBank(0);
    delay(1);

    // (Giữ nguyên phần cấu hình FIFO và ngắt bên dưới của bạn...)
    modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 4);
    delay(1);
    modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 3);
    delay(1);
    modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 2);
    delay(1);
    modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 1);
    delay(1);

    modifyRegisterBit(ICM20X_BO_FIFO_EN_1, 1, 0);
    delay(1);

    setInt1ActiveLow(true);
    delay(1);
    setInt1Latch(false);
    delay(1);
    clearIntOnRead(true);
    delay(1);

    enableGyrolDLPF(_gyroLPFEn, (icm20x_gyro_cutoff_t) _gyroLPFCutoff);
    delay(1);
    writeGyroRange(_gyroRange);
    delay(1);

    enableAccelDLPF(_accelLPFEn, (icm20x_accel_cutoff_t) _accelLPFCutoff);
    delay(1);
    writeAccelRange(_accelRange);
    delay(1);

    setGyroRateDivisor(_gyroSampleRate);
    delay(1);

    setAccelRateDivisor(_accelSampleRate);
    delay(1);

    /* KÍCH HOẠT FIFO BẰNG READ-MODIFY-WRITE ĐỂ TRÁNH XÓA CÁC CỜ PHÍA TRÊN */
    _setBank(0);
    delay(1);
    writeRegisterByte(ICM20X_BO_FIFO_RST, 0x0F); // reset FIFO
    delay(1);
    writeRegisterByte(ICM20X_BO_FIFO_RST, 0x00);
    delay(1);

    // Sử dụng modifyRegisterBit để bảo toàn I2C_IF_DIS
    modifyRegisterBit(ICM20X_B0_USER_CTRL, 0, 7); // Disable DMP
    delay(1);
    modifyRegisterBit(ICM20X_B0_USER_CTRL, 1, 6); // Enable FIFO

    return true;
}

//bool Adafruit_ICM20X::_init(int32_t sensor_id) {
//	_setBank(0);
//
//	uint8_t chip_id_ = readRegisterByte(ICM20X_B0_WHOAMI);
//
//	// Kiểm tra xem có kết nối được chip không
//	if ((chip_id_ != ICM20649_CHIP_ID) && (chip_id_ != ICM20948_CHIP_ID)) {
//		return false;
//	}
//
//	_sensorid_accel = sensor_id;
//	_sensorid_gyro = sensor_id + 1;
//	_sensorid_mag = sensor_id + 2;
//	_sensorid_temp = sensor_id + 3;
//
//	reset();
//
//	writeRegisterByte(ICM20X_B0_PWR_MGMT_1, 0x01); // Wake up & Auto-select clock
//	delay(10);
//	_setBank(0);
//	delay(1);
//
//	modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 4); // Enable accelerometer data to be saved to FIFO
//	delay(1);
//	modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 3); // Enable gyro Z
//	delay(1);
//	modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 2); // Enable gyro Y
//	delay(1);
//	modifyRegisterBit(ICM20X_BO_FIFO_EN_2, 1, 1); // Enable gyro X
//	delay(1);
//
//	modifyRegisterBit(ICM20X_BO_FIFO_EN_1, 1, 0); // Enable I2C Slave 0 (Mag data) to be saved to FIFO
//	delay(1);
//
//	setInt1ActiveLow(true);
//	delay(1);
//	setInt1Latch(false);
//	delay(1);
//	clearIntOnRead(true);
//	delay(1);
//
//	enableGyrolDLPF(_gyroLPFEn, (icm20x_gyro_cutoff_t) _gyroLPFCutoff);
//	delay(1);
//	writeGyroRange(_gyroRange);
//	delay(1);
//
//	enableAccelDLPF(_accelLPFEn, (icm20x_accel_cutoff_t) _accelLPFCutoff);
//	delay(1);
//	writeAccelRange(_accelRange);
//	delay(1);
//
//	// 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
//	setGyroRateDivisor(_gyroSampleRate); //550hz
//	delay(1);
//
//	// 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
//	setAccelRateDivisor(_accelSampleRate); // 562.5Hz
//	delay(1);
//
//	/* ENABLE FIFO */
//	_setBank(0);
//	delay(1);
//	writeRegisterByte(ICM20X_BO_FIFO_RST, 0x0F); // reset FIFO
//	delay(1);
//	writeRegisterByte(ICM20X_BO_FIFO_RST, 0x00);
//	delay(1);
//	modifyRegisterBit(ICM20X_B0_USER_CTRL, 0, 7); // Currently not enable digital motion processor (dmp)(enable ìf use)
//	delay(1);
//	modifyRegisterBit(ICM20X_B0_USER_CTRL, 1, 6); // Enable FIFO
//
//	return true;
//}

bool Adafruit_ICM20X::getEvent(sensors_event_t *accel, sensors_event_t *gyro,
		sensors_event_t *temp, sensors_event_t *mag) {
	uint32_t t = HAL_GetTick();
	_read();

	fillAccelEvent(accel, t);
	fillGyroEvent(gyro, t);
	fillTempEvent(temp, t);
	if (mag) {
		fillMagEvent(mag, t);
	}
	return true;
}

void Adafruit_ICM20X::fillAccelEvent(sensors_event_t *accel, uint32_t timestamp) {
	memset(accel, 0, sizeof(sensors_event_t));
	accel->version = 1;
	accel->sensor_id = _sensorid_accel;
	accel->type = SENSOR_TYPE_ACCELEROMETER;
	accel->timestamp = timestamp;
	accel->acceleration.x = accX * SENSORS_GRAVITY_EARTH;
	accel->acceleration.y = accY * SENSORS_GRAVITY_EARTH;
	accel->acceleration.z = accZ * SENSORS_GRAVITY_EARTH;
}

void Adafruit_ICM20X::fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp) {
	memset(gyro, 0, sizeof(sensors_event_t));
	gyro->version = 1;
	gyro->sensor_id = _sensorid_gyro;
	gyro->type = SENSOR_TYPE_GYROSCOPE;
	gyro->timestamp = timestamp;
	gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS;
	gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
	gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
}

void Adafruit_ICM20X::fillMagEvent(sensors_event_t *mag, uint32_t timestamp) {
	memset(mag, 0, sizeof(sensors_event_t));
	mag->version = 1;
	mag->sensor_id = _sensorid_mag;
	mag->type = SENSOR_TYPE_MAGNETIC_FIELD;
	mag->timestamp = timestamp;
	mag->magnetic.x = magX;
	mag->magnetic.y = magY;
	mag->magnetic.z = magZ;
}

void Adafruit_ICM20X::fillTempEvent(sensors_event_t *temp, uint32_t timestamp) {
	memset(temp, 0, sizeof(sensors_event_t));
	temp->version = sizeof(sensors_event_t);
	temp->sensor_id = _sensorid_temp;
	temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
	temp->timestamp = timestamp;
	temp->temperature = (temperature / 333.87) + 21.0;
}

void Adafruit_ICM20X::_read_ByPass(void) {
    _setBank(0);

    uint8_t imu_buffer[14];
    readRegister(ICM20X_B0_ACCEL_XOUT_H, imu_buffer, 14);

    rawAccX = (imu_buffer[0] << 8) | imu_buffer[1];
    rawAccY = (imu_buffer[2] << 8) | imu_buffer[3];
    rawAccZ = (imu_buffer[4] << 8) | imu_buffer[5];

    rawGyroX = (imu_buffer[6] << 8) | imu_buffer[7];
    rawGyroY = (imu_buffer[8] << 8) | imu_buffer[9];
    rawGyroZ = (imu_buffer[10] << 8) | imu_buffer[11];

    temperature = (imu_buffer[12] << 8) | imu_buffer[13];


    uint8_t mag_buffer[9];
    uint8_t mag_addr = 0x0C << 1;

    // Đọc từ thanh ghi ST1 (0x10) quét 9 byte tới ST2 (0x18)
    if (HAL_I2C_Mem_Read(i2c_han, mag_addr, 0x10, 1, mag_buffer, 9, 10) == HAL_OK) {
        uint8_t ST1 = mag_buffer[0];
        //uint8_t ST2 = mag_buffer[8];

        // Kiểm tra bit DRDY (Data Ready)
        if (ST1 & 0x01) {
            // Dữ liệu Mag là Little Endian (Byte thấp đứng trước)
            rawMagX = (mag_buffer[2] << 8) | mag_buffer[1];
            rawMagY = (mag_buffer[4] << 8) | mag_buffer[3];
            rawMagZ = (mag_buffer[6] << 8) | mag_buffer[5];
        }
    }
//	u_printf("ST1: 0x%02X | ST2: 0x%02X | DATA: %02X %02X %02X %02X %02X %02X\r\n",
//			mag_buffer[0], mag_buffer[8], mag_buffer[1], mag_buffer[2], mag_buffer[3], mag_buffer[4], mag_buffer[5], mag_buffer[6]);
    scaleValues();
    _setBank(0);
}

void Adafruit_ICM20X::_read(void) {
    _setBank(0);

    // ========================================================
    // 1. ĐỌC ACCEL, GYRO VÀ NHIỆT ĐỘ (14 Byte)
    // Bắt đầu từ thanh ghi 0x2D
    // ========================================================
    uint8_t imu_buffer[14];
    readRegister(ICM20X_B0_ACCEL_XOUT_H, imu_buffer, 14);

    rawAccX = (imu_buffer[0] << 8) | imu_buffer[1];
    rawAccY = (imu_buffer[2] << 8) | imu_buffer[3];
    rawAccZ = (imu_buffer[4] << 8) | imu_buffer[5];

    rawGyroX = (imu_buffer[6] << 8) | imu_buffer[7];
    rawGyroY = (imu_buffer[8] << 8) | imu_buffer[9];
    rawGyroZ = (imu_buffer[10] << 8) | imu_buffer[11];

    temperature = (imu_buffer[12] << 8) | imu_buffer[13];

    // ========================================================
    // 2. ĐỌC TỪ KẾ MAG (9 Byte từ kênh SLV0)
    // Bắt đầu từ thanh ghi EXT_SLV_SENS_DATA_00 (0x3B)
    // ========================================================
    uint8_t mag_buffer[8]; // Chỉ cần 8 byte
    readRegister(0x3B, mag_buffer, 8);

    // Vì ta đọc bắt đầu từ HXL (0x11) và đỗ vào 0x3B, nên:
    // Byte 0, 1 = X | Byte 2, 3 = Y | Byte 4, 5 = Z
    // Byte 6 = TMPS (Nhiệt độ ảo)
    // Byte 7 = ST2

    uint8_t ST2 = mag_buffer[7];

    // KIỂM TRA LỖI TRÀN TỪ TRƯỜNG HOẶC LỖI OVERFLOW DATA (HOFL - Bit 3)
    if (!(ST2 & 0x08)) {

        // Gắn thẳng xương: [High Byte] << 8 | [Low Byte]
        rawMagX = (int16_t)((mag_buffer[1] << 8) | mag_buffer[0]);
        rawMagY = (int16_t)((mag_buffer[3] << 8) | mag_buffer[2]);
        rawMagZ = (int16_t)((mag_buffer[5] << 8) | mag_buffer[4]);

    } else {
        // Tùy chọn: In cảnh báo nếu từ kế bị câm hoặc báo lỗi tràn (HOFL)
         //u_printf(">> [WARNING] Mag ST1: 0x%02X | ST2: 0x%02X\r\n", ST1, ST2);
         u_printf(">> [WARNING] Mag ST2: 0x%02X\r\n", ST2);
    }
    scaleValues(); // Hàm này của bạn sẽ nhân chia với Bias và Scale
    _setBank(0);
}
void Adafruit_ICM20X::scaleValues(void) {
}

Adafruit_Sensor* Adafruit_ICM20X::getAccelerometerSensor(void) { return accel_sensor; }
Adafruit_Sensor* Adafruit_ICM20X::getGyroSensor(void) { return gyro_sensor; }
Adafruit_Sensor* Adafruit_ICM20X::getMagnetometerSensor(void) { return mag_sensor; }
Adafruit_Sensor* Adafruit_ICM20X::getTemperatureSensor(void) { return temp_sensor; }

void Adafruit_ICM20X::_setBank(uint8_t bank_number) {
	writeRegisterByte(ICM20X_B0_REG_BANK_SEL, (bank_number << 4) & 0x30);
}

uint8_t Adafruit_ICM20X::readAccelRange(void) {
	_setBank(2);
	uint8_t range = readRegisterBits(ICM20X_B2_ACCEL_CONFIG_1, 1, 2);
	_setBank(0);
	return range;
}

void Adafruit_ICM20X::writeAccelRange(uint8_t new_accel_range) {
	_setBank(2);
	modifyRegisterMultipleBit(ICM20X_B2_ACCEL_CONFIG_1, new_accel_range, 1, 2);
	current_accel_range = new_accel_range;
	_setBank(0);
}

uint8_t Adafruit_ICM20X::readGyroRange(void) {
	_setBank(2);
	uint8_t range = readRegisterBits(ICM20X_B2_GYRO_CONFIG_1, 1, 2);
	_setBank(0);
	return range;
}

void Adafruit_ICM20X::writeGyroRange(uint8_t new_gyro_range) {
	_setBank(2);
	modifyRegisterMultipleBit(ICM20X_B2_GYRO_CONFIG_1, new_gyro_range, 1, 2);
	current_gyro_range = new_gyro_range;
	_setBank(0);
}

uint16_t Adafruit_ICM20X::getAccelRateDivisor(void) {
	_setBank(2);
	uint16_t divisor_val = 0;
	readRegister(ICM20X_B2_ACCEL_SMPLRT_DIV_1, (uint8_t*) &divisor_val, 2);
	_setBank(0);
	return divisor_val;
}

void Adafruit_ICM20X::setAccelRateDivisor(uint16_t new_accel_divisor) {
	_setBank(2);
	uint8_t data[2];
	data[0] = new_accel_divisor >> 8;
	data[1] = new_accel_divisor & 0xFF;

	new_accel_divisor = new_accel_divisor & 0x07FF;
	writeRegister(ICM20X_B2_ACCEL_SMPLRT_DIV_1, data, 2);
	_setBank(0);
}

uint8_t Adafruit_ICM20X::getGyroRateDivisor(void) {
	_setBank(2);
	uint8_t divisor_val = readRegisterByte(ICM20X_B2_GYRO_SMPLRT_DIV);
	_setBank(0);
	return divisor_val;
}

void Adafruit_ICM20X::setGyroRateDivisor(uint8_t new_gyro_divisor) {
	_setBank(2);
	writeRegisterByte(ICM20X_B2_GYRO_SMPLRT_DIV, new_gyro_divisor);
	_setBank(0);
}

bool Adafruit_ICM20X::enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq) {
	_setBank(2);
	if (!modifyRegisterBit(ICM20X_B2_ACCEL_CONFIG_1, enable, 0)) return false;
	if (!enable) return true;
	if (!modifyRegisterMultipleBit(ICM20X_B2_ACCEL_CONFIG_1, cutoff_freq, 3, 3)) return false;
	return true;
}

bool Adafruit_ICM20X::enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq) {
	_setBank(2);
	if (!modifyRegisterBit(ICM20X_B2_ACCEL_CONFIG_1, enable, 0)) return false;
	if (!enable) return true;
	if (!modifyRegisterMultipleBit(ICM20X_B2_ACCEL_CONFIG_1, cutoff_freq, 3, 3)) return false;
	return true;
}

void Adafruit_ICM20X::setInt1ActiveLow(bool active_low) {
	_setBank(0);
	modifyRegisterBit(ICM20X_B0_REG_INT_PIN_CFG, true, 6); //open drain
	modifyRegisterBit(ICM20X_B0_REG_INT_PIN_CFG, active_low, 7); //active low
}

void Adafruit_ICM20X::setInt1Latch(bool latch) {
	_setBank(0);
	modifyRegisterBit(ICM20X_B0_REG_INT_PIN_CFG, latch, 5);
}

void Adafruit_ICM20X::clearIntOnRead(bool enable) {
	_setBank(0);
	modifyRegisterBit(ICM20X_B0_REG_INT_PIN_CFG, enable, 4);
}

void Adafruit_ICM20X::enableInt1(bool enable) {
	_setBank(0);
	modifyRegisterBit(ICM20X_B0_REG_INT_ENABLE_1, enable, 0);
}

void Adafruit_ICM20X::enableInt2(bool enable) {
	_setBank(0);
	modifyRegisterBit(ICM20X_B0_REG_INT_ENABLE_2, enable, 0);
}

//void Adafruit_ICM20X::setI2CBypass(bool bypass_i2c) {
//	_setBank(0);
//	modifyRegisterBit(ICM20X_B0_REG_INT_PIN_CFG, bypass_i2c, 1);
//}
void Adafruit_ICM20X::setI2CBypass(bool bypass_i2c) {
    _setBank(0);
    // Sử dụng Bit 2 cho cờ BYPASS_EN của ICM-20948 thay vì Bit 1
    modifyRegisterBit(ICM20X_B0_REG_INT_PIN_CFG, bypass_i2c, 2);
}

bool Adafruit_ICM20X::enableI2CMaster(bool enable_i2c_master) {
	_setBank(0);
	return modifyRegisterMultipleBit(ICM20X_B0_USER_CTRL, enable_i2c_master, 5, 1);
}

bool Adafruit_ICM20X::enableSPI(bool enable_spi_master) {
	_setBank(0);
	delay(1);
	return modifyRegisterMultipleBit(ICM20X_B0_USER_CTRL, enable_spi_master, 4, 1);
}

bool Adafruit_ICM20X::configureI2CMaster(void) {
	_setBank(3);
	return writeRegisterByte(ICM20X_B3_I2C_MST_CTRL, 0x07);
}

uint8_t Adafruit_ICM20X::readExternalRegister(uint8_t slv_addr, uint8_t reg_addr) {
	return auxillaryRegisterTransaction(true, slv_addr, reg_addr);
}

bool Adafruit_ICM20X::writeExternalRegister(uint8_t slv_addr, uint8_t reg_addr, uint8_t value) {
	return (bool) auxillaryRegisterTransaction(false, slv_addr, reg_addr, value);
}

uint8_t Adafruit_ICM20X::auxillaryRegisterTransaction(bool read, uint8_t slv_addr, uint8_t reg_addr, uint8_t value) {
	_setBank(3);

	if (read) {
		slv_addr |= 0x80;
	} else {
		if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_DO, value)) return 0;
	}

	if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_ADDR, slv_addr)) return 0;
	if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_REG, reg_addr)) return 0;

	_setBank(0);
	readRegisterByte(ICM20X_B0_I2C_MST_STATUS);

	_setBank(3);
	if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_CTRL, 0x80)) return 0;

	_setBank(0);
	uint8_t tries = 0;
	bool is_done = false;

	while (tries < 100) {
		uint8_t status = readRegisterByte(ICM20X_B0_I2C_MST_STATUS);
		if (status & 0x40) { // Bit 6 (DONE)
			is_done = true;
			if (status & 0x10) { // Bit 4 (NACK)
				u_printf("I2C NACK tu Mag! reg: 0x%02X\r\n", reg_addr);
			}
			break;
		}
		tries++;
		HAL_Delay(1);
	}

	if (!is_done) {
		u_printf("Loi TIMEOUT I2C Master (SLV4)! reg_addr: 0x%02X\r\n", reg_addr);
		// Tự động cứu vớt I2C Master để nó không làm treo lệnh tiếp theo
		resetI2CMaster();
		return 0;
	}

	uint8_t ret_val = 1;
	if (read) {
		_setBank(3);
		ret_val = readRegisterByte(ICM20X_B3_I2C_SLV4_DI);
	}

	// Thời gian nghỉ để Bus I2C ổn định trước lệnh tiếp theo
	HAL_Delay(2);
	return ret_val;
}

/*uint8_t Adafruit_ICM20X::auxillaryRegisterTransaction(bool read, uint8_t slv_addr, uint8_t reg_addr, uint8_t value) {
	_setBank(3);
	if (read) {
		slv_addr |= 0x80;
	} else {
		if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_DO, value)) return (uint8_t) false;
	}

	if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_ADDR, slv_addr)) return (uint8_t) false;
	if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_REG, reg_addr)) return (uint8_t) false;
	if (!writeRegisterByte(ICM20X_B3_I2C_SLV4_CTRL, 0x80)) return (uint8_t) false;

	_setBank(0);
	uint8_t tries = 0;
	while (readRegisterBits(ICM20X_B0_I2C_MST_STATUS, 6, 1) != true) {
		tries++;
		if (tries >= NUM_FINISHED_CHECKS) return (uint8_t) false;
	}
	if (read) {
		_setBank(3);
		return readRegisterByte(ICM20X_B3_I2C_SLV4_DI);
	}
	return (uint8_t) true;
}*/

/*
void Adafruit_ICM20X::resetI2CMaster(void) {
	_setBank(0);
	modifyRegisterBit(ICM20X_B0_USER_CTRL, true, 1);
	HAL_Delay(10);
	while (checkRegisterBit(ICM20X_B0_USER_CTRL, 1)) {
		delay(10);
	}
	enableI2CMaster(true); // Bật lại I2C_MST_EN (Bit 5) để chắc chắn nó đang chạy
	delay(100);
}
*/

bool Adafruit_ICM20X::writeMagRegisterSLV4(uint8_t reg, uint8_t value) {
    uint8_t val;
    _setBank(3);

    // 1. Nạp dữ liệu cần ghi
    writeRegisterByte(ICM20X_B3_I2C_SLV4_DO, value);

    // 2. Nạp địa chỉ AK09916 (Ghi -> Bit 7 = 0 -> 0x0C)
    writeRegisterByte(ICM20X_B3_I2C_SLV4_ADDR, 0x0C);

    // 3. Nạp thanh ghi đích
    writeRegisterByte(ICM20X_B3_I2C_SLV4_REG, reg);

    // 4. Bắn lệnh (Bật I2C_SLV4_EN)
    writeRegisterByte(ICM20X_B3_I2C_SLV4_CTRL, 0x80);

    // 5. Chờ cờ DONE
    _setBank(0);
    uint8_t tries = 0;
    while (tries < 100) {
        val = readRegisterByte(ICM20X_B0_I2C_MST_STATUS);
        if (val & 0x40) { // I2C_SLV4_DONE = 1
            // Kiểm tra lỗi NACK
            if (val & 0x10) {
                u_printf(">> NACK tai Mag Reg 0x%02X\r\n", reg);
                return false;
            }
            return true; // Ghi thành công!
        }
        HAL_Delay(1);
        tries++;
    }

    u_printf(">> Timeout khi ghi Mag Reg 0x%02X\r\n", reg);
    return false;
}

// Hàm này mô phỏng chính xác logic ghi của AK09916_SetContinuousMode cũ của bạn
bool Adafruit_ICM20X::writeMagRegisterSLV0(uint8_t reg, uint8_t value) {
    uint8_t val;
    _setBank(3);

    val = 0x0C; // Địa chỉ I2C của AK09916 (Chế độ Ghi)
    writeRegister(ICM20X_B3_I2C_SLV0_ADDR, &val, 1);

    val = reg;  // Thanh ghi đích
    writeRegister(ICM20X_B3_I2C_SLV0_REG, &val, 1);

    val = value; // Giá trị cần ghi
    writeRegister(ICM20X_B3_I2C_SLV0_DO, &val, 1);

    val = 0x81; // Kích hoạt SLV0, ghi 1 byte
    writeRegister(ICM20X_B3_I2C_SLV0_CTRL, &val, 1);

    _setBank(0);
    // Chỉ cần chờ, không được ép ghi 0x00 vào SLV0_CTRL để tắt nó!
    HAL_Delay(50);
    return true;}

void Adafruit_ICM20X::resetI2CMaster(void) {
    uint8_t user_ctrl = 0;
    uint8_t val = 0;

    _setBank(0);
    // Đọc trạng thái cấu hình hiện tại của USER_CTRL
    user_ctrl = readRegisterByte(ICM20X_B0_USER_CTRL);

    // 1. Gửi lệnh Reset I2C Master nội bộ (Ghi đè Bit 1)
    val = user_ctrl | 0x02;
    writeRegisterByte(ICM20X_B0_USER_CTRL, val);
    HAL_Delay(10);

    // 2. Xóa cờ Reset (Bảo toàn các bit chức năng khác)
    val = user_ctrl & ~0x02;
    writeRegisterByte(ICM20X_B0_USER_CTRL, val);
    HAL_Delay(10);

    // 3. Đảm bảo kích hoạt lại Master I2C
    val = user_ctrl | 0x20;
    writeRegisterByte(ICM20X_B0_USER_CTRL, val);
    HAL_Delay(10);

    // 4. KHÔNG SỬ DỤNG CHẾ ĐỘ MST_CYCLE KHI CONFIG VÀ ĐỌC BÌNH THƯỜNG
    writeRegisterByte(ICM20X_B0_LP_CONFIG, 0x00);
    HAL_Delay(10);

    // 5. Cài đặt lại tần số clock cho Master I2C
    _setBank(3);
    val = 0x0D; // 345.6 kHz
    writeRegisterByte(ICM20X_B3_I2C_MST_CTRL, val);
    HAL_Delay(10);

    _setBank(0);
}

//void Adafruit_ICM20X::resetI2CMaster(void) {
//    uint8_t user_ctrl = 0;
//    uint8_t val = 0;
//
//    _setBank(0);
//    // Lấy trạng thái hiện tại (Đảm bảo I2C_IF_DIS vẫn đang bật)
//    user_ctrl = readRegisterByte(ICM20X_B0_USER_CTRL);
//
//    // 1. Kích hoạt Hardware Reset cho I2C Master (Ghi bit 1)
//    val = user_ctrl | 0x02;
//    writeRegisterByte(ICM20X_B0_USER_CTRL, val);
//    HAL_Delay(10); // Đợi chip Reset xong
//
//    // 2. Xóa cờ Reset đi (Clear bit 1)
//    val = user_ctrl & ~0x02;
//    writeRegisterByte(ICM20X_B0_USER_CTRL, val);
//    HAL_Delay(10);
//
//    // 3. Bật I2C Master Enable (Bit 5)
//    val = user_ctrl | 0x20;
//    writeRegisterByte(ICM20X_B0_USER_CTRL, val);
//    HAL_Delay(10);
//
//    // ========================================================
//    // BẬT CHẾ ĐỘ I2C_MST_CYCLE ĐỂ MASTER CHẠY ĐỘC LẬP
//    // Cực kỳ quan trọng để chống dính Clock!
//    // ========================================================
//    val = 0x40; // Ghi bit I2C_MST_CYCLE vào LP_CONFIG (0x05)
//    writeRegisterByte(ICM20X_B0_LP_CONFIG, val);
//    HAL_Delay(10);
//
//    // 4. Cấu hình tốc độ Master Clock
//    _setBank(3);
//    // Thử 400kHz (0x07). Nếu sau này mạch vẫn lỗi Timeout, hãy thử đổi về 345.6kHz (0x17)
//    val = 0x07;
//    writeRegisterByte(ICM20X_B3_I2C_MST_CTRL, val);
//    HAL_Delay(10);
//
//    // Đưa về Bank 0 cho an toàn
//    _setBank(0);
//}

/*void Adafruit_ICM20X::resetI2CMaster(void) {
    _setBank(0);

    // 1. Tắt I2C Master trước cho an toàn
    uint8_t user_ctrl = readRegisterByte(ICM20X_B0_USER_CTRL);
    writeRegisterByte(ICM20X_B0_USER_CTRL, user_ctrl & ~0x20);
    HAL_Delay(10);

    // 2. Kích hoạt Hardware Reset Master
    writeRegisterByte(ICM20X_B0_USER_CTRL, user_ctrl | 0x02);
    HAL_Delay(10);

    // 3. Bật lại Master
    writeRegisterByte(ICM20X_B0_USER_CTRL, user_ctrl | 0x20);
    HAL_Delay(10);

    // 4. RẤT QUAN TRỌNG: Gọi lại cấu hình Clock vì Hardware Reset đã xóa nó mất rồi!
    configureI2CMaster();
    HAL_Delay(10);
}*/
// ---------------------------------------------------------
// LOW LEVEL SPI FUNCTIONS (Implemented for STM32 HAL)
// ---------------------------------------------------------
bool Adafruit_ICM20X::writeRegisterByte(uint8_t mem_addr, uint8_t val) {
    uint8_t tx_data[2];
    tx_data[0] = mem_addr & 0x7F; // Bit 7 = 0 (Write)
    tx_data[1] = val;

    cs_active(true);
    for (volatile int i = 0; i < 50; i++); // Trễ thiết lập phần cứng bắt buộc cho SPI

    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_han, tx_data, 2, 100);
    cs_active(false);

    if (status != HAL_OK) {
        u_printf("SPI Write Fail at Reg: 0x%02X\r\n", mem_addr);
        return false;
    }
    return true;
}

bool Adafruit_ICM20X::writeRegister(uint8_t mem_addr, uint8_t *val, uint16_t size) {
    uint8_t tx_addr = mem_addr & 0x7F; // Bit 7 = 0 (Write)

    cs_active(true);
    for (volatile int i = 0; i < 50; i++); // Trễ thiết lập phần cứng bắt buộc cho SPI

    HAL_SPI_Transmit(spi_han, &tx_addr, 1, 100);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_han, val, size, 100);
    cs_active(false);

    return (status == HAL_OK);
}

bool Adafruit_ICM20X::readRegister(uint16_t mem_addr, uint8_t *dest, uint16_t size) {
    uint8_t tx_data[size + 1];
    uint8_t rx_data[size + 1];

    memset(tx_data, 0x00, size + 1);
    tx_data[0] = (mem_addr & 0x7F) | 0x80; // Bit 7 = 1 (Read)

    cs_active(true);
    for (volatile int i = 0; i < 50; i++); // Trễ thiết lập phần cứng bắt buộc cho SPI

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(spi_han, tx_data, rx_data, size + 1, 100);
    cs_active(false);

    if (status == HAL_OK) {
        memcpy(dest, &rx_data[1], size);
        return true;
    }

    return false;
}
//bool Adafruit_ICM20X::writeRegister(uint8_t mem_addr, uint8_t *val, uint16_t size) {
//    uint8_t tx_addr = mem_addr & 0x7F; // Bit 7 = 0 (Write)
//    cs_active(true);
//    HAL_SPI_Transmit(spi_han, &tx_addr, 1, 100);
//    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_han, val, size, 100);
//    cs_active(false);
//    return (status == HAL_OK);
//}
//
//bool Adafruit_ICM20X::writeRegisterByte(uint8_t mem_addr, uint8_t val) {
//    uint8_t tx_data[2];
//    tx_data[0] = mem_addr & 0x7F; // Bit 7 = 0 (Write)
//    tx_data[1] = val;
//
//    cs_active(true);
//    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_han, tx_data, 2, 100);
//    cs_active(false);
//
//    if (status != HAL_OK) {
//        u_printf("SPI Write Fail at Reg: 0x%02X\r\n", mem_addr);
//        return false;
//    }
//    return true;
//}
//
//bool Adafruit_ICM20X::readRegister(uint16_t mem_addr, uint8_t *dest, uint16_t size) {
//    // 1. Tạo mảng TX đệm (Byte đầu là Địa chỉ Đọc, các byte sau là rác 0x00 để đẩy Clock)
//    uint8_t tx_data[size + 1];
//    uint8_t rx_data[size + 1];
//
//    memset(tx_data, 0x00, size + 1); // Xóa sạch mảng
//    tx_data[0] = (mem_addr & 0x7F) | 0x80; // Byte đầu là lệnh Read
//
//    // 2. Kéo CS xuống
//    cs_active(true);
//
//    // 3. Vừa Đẩy mảng TX đi, vừa Hứng mảng RX về (Đồng bộ Clock)
//    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(spi_han, tx_data, rx_data, size + 1, 100);
//
//    // 4. Kéo CS lên
//    cs_active(false);
//
//    // 5. Copy dữ liệu thật (Bỏ byte đầu tiên vì đó là lúc gửi địa chỉ)
//    if (status == HAL_OK) {
//        memcpy(dest, &rx_data[1], size);
//        return true;
//    }
//
//    return false;
//}

uint8_t Adafruit_ICM20X::readRegisterByte(uint16_t mem_addr) {
    // Để đọc 1 byte an toàn nhất qua SPI HAL: Dùng Mảng 2 byte
    uint8_t tx_data[2] = { (uint8_t)((mem_addr & 0x7F) | 0x80), 0x00 };
    uint8_t rx_data[2] = { 0x00, 0x00 };

    cs_active(true);
    for (volatile int i = 0; i < 50; i++);

    // Gửi 2 byte (Byte 1: Địa chỉ, Byte 2: Xung clock rỗng để hứng data)
    // Nhận 2 byte (Byte 1: Rác, Byte 2: Dữ liệu thật từ chip)
    HAL_SPI_TransmitReceive(spi_han, tx_data, rx_data, 2, 500);

    cs_active(false);
    arr[0] = tx_data[0];
    arr[1] = tx_data[1];
    arr[2] = rx_data[0];
    arr[3] = rx_data[1];

    return rx_data[1]; // Trả về byte dữ liệu thật (Byte thứ 2)
}
// ---------------------------------------------------------
// LOW LEVEL I2C FUNCTIONS (Implemented for STM32 HAL)
// ---------------------------------------------------------

//bool Adafruit_ICM20X::writeRegister(uint8_t mem_addr, uint8_t *val, uint16_t size) {
//	if (HAL_OK == HAL_I2C_Mem_Write(i2c_han, i2c_addr, mem_addr, 1, val, size, 100)) {
//		return true;
//	}
//	return false;
//}
//
///*bool Adafruit_ICM20X::writeRegisterByte(uint8_t mem_addr, uint8_t val) {
//	if (HAL_OK == HAL_I2C_Mem_Write(i2c_han, i2c_addr, mem_addr, 1, &val, 1, 100)) {
//		return true;
//	}
//	return false;
//}*/
//bool Adafruit_ICM20X::writeRegisterByte(uint8_t mem_addr, uint8_t val) {
//	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(i2c_han, i2c_addr, mem_addr, 1, &val, 1, 100);
//	if (status == HAL_OK) {
//		return true;
//	}
//
//	// In lỗi ra màn hình (1: HAL_ERROR, 2: HAL_BUSY, 3: HAL_TIMEOUT)
//	u_printf("I2C Fail: %d at Reg: 0x%02X\r\n", status, mem_addr);
//	return false;
//}
//
//bool Adafruit_ICM20X::readRegister(uint16_t mem_addr, uint8_t *dest, uint16_t size) {
//	if (HAL_OK == HAL_I2C_Mem_Read(i2c_han, i2c_addr, mem_addr, 1, dest, size, 100)) {
//		return true;
//	}
//	return false;
//}
//
//uint8_t Adafruit_ICM20X::readRegisterByte(uint16_t mem_addr) {
//	uint8_t data = 0;
//	HAL_I2C_Mem_Read(i2c_han, i2c_addr, mem_addr, 1, &data, 1, 100);
//	return data;
//}

// ---------------------------------------------------------

uint8_t Adafruit_ICM20X::modifyBitInByte(uint8_t var, uint8_t value, uint8_t pos) {
	uint8_t mask = 1 << pos;
	return ((var & ~mask) | (value << pos));
}

bool Adafruit_ICM20X::getFIFOcnt(uint16_t *cnt){
	uint8_t data[2];
	*cnt = 0;
	_setBank(0);
	if (readRegister(ICM20X_BO_FIFO_COUNTH,data,2)) {
		*cnt = ((data[0] & 0x1F) << 8) | data[1];
		return true;
	}
	return false;
}

bool Adafruit_ICM20X::getINTstatus(uint8_t *data){
	_setBank(0);
	if (readRegister(ICM20X_B0_REG_INT_STATUS,data,4)) {
		return true;
	}
	return false;
}

bool Adafruit_ICM20X::readFIFO(uint8_t *buffer, uint16_t size) {
	_setBank(0);
	// Đối với I2C, readFIFO chỉ đơn giản là đọc register FIFO_R_W
	if (readRegister(ICM20X_BO_FIFO_R_W, buffer, size)) {
		return true;
	}
	return false;
}

uint8_t Adafruit_ICM20X::checkRegisterBit(uint16_t reg, uint8_t pos) {
	return (uint8_t) ((readRegisterByte(reg) >> pos) & 0x01);
}

bool Adafruit_ICM20X::modifyRegisterBit(uint16_t reg, bool value, uint8_t pos) {
	uint8_t register_value = readRegisterByte(reg);
	register_value = modifyBitInByte(register_value, (uint8_t) value, pos);
	return writeRegisterByte(reg, register_value);
}

bool Adafruit_ICM20X::modifyRegisterMultipleBit(uint16_t reg, uint8_t value, uint8_t pos, uint8_t bits) {
	uint8_t register_value = readRegisterByte(reg);
	uint8_t mask = (1 << (bits)) - 1;
	value &= mask;
	mask <<= pos;
	register_value &= ~mask;
	register_value |= value << pos;
	return writeRegisterByte(reg, register_value);
}

/*
uint8_t Adafruit_ICM20X::readRegisterBits(uint16_t reg, uint8_t pos, uint8_t bits) {
	uint8_t register_value = readRegisterByte(reg);
	uint8_t mask = (1 << (bits)) - 1;
	mask <<= pos;
	register_value &= ~mask;
	return register_value >> pos;
}
*/
uint8_t Adafruit_ICM20X::readRegisterBits(uint16_t reg, uint8_t pos, uint8_t bits) {
	uint8_t register_value = readRegisterByte(reg);

	// 1. Dịch bit cần đọc xuống tận cùng bên phải
	register_value >>= pos;

	// 2. Tạo mặt nạ để giữ lại đúng số bit cần thiết (VD: bits=1 thì mask=0x01)
	uint8_t mask = (1 << bits) - 1;

	// 3. Che đi các bit rác, chỉ giữ lại bit cần lấy
	return (register_value & mask);
}

void Adafruit_ICM20X_Accelerometer::getSensor(sensor_t *sensor) {
	/* Clear the sensor_t object */
	memset(sensor, 0, sizeof(sensor_t));

	/* Insert the sensor name in the fixed length char array */
	strncpy(sensor->name, "ICM20X_A", sizeof(sensor->name) - 1);
	sensor->name[sizeof(sensor->name) - 1] = 0;
	sensor->version = 1;
	sensor->sensor_id = _sensorID;
	sensor->type = SENSOR_TYPE_ACCELEROMETER;
	sensor->min_delay = 0;
	sensor->min_value = -294.1995F; /*  -30g = 294.1995 m/s^2  */
	sensor->max_value = 294.1995F; /* 30g = 294.1995 m/s^2  */
	sensor->resolution = 0.122; /* 8192LSB/1000 mG -> 8.192 LSB/ mG => 0.122 mG/LSB at +-4g */
}

/**************************************************************************/
/*!
 @brief  Gets the accelerometer as a standard sensor event
 @param  event Sensor event object that will be populated
 @returns True
 */
/**************************************************************************/
bool Adafruit_ICM20X_Accelerometer::getEvent(sensors_event_t *event) {
	_theICM20X->_read();
	_theICM20X->fillAccelEvent(event, HAL_GetTick());
	return true;
}

/**************************************************************************/
/*!
 @brief  Gets the sensor_t data for the ICM20X's gyroscope sensor
 */
/**************************************************************************/
void Adafruit_ICM20X_Gyro::getSensor(sensor_t *sensor) {
	/* Clear the sensor_t object */
	memset(sensor, 0, sizeof(sensor_t));

	/* Insert the sensor name in the fixed length char array */
	strncpy(sensor->name, "ICM20X_G", sizeof(sensor->name) - 1);
	sensor->name[sizeof(sensor->name) - 1] = 0;
	sensor->version = 1;
	sensor->sensor_id = _sensorID;
	sensor->type = SENSOR_TYPE_GYROSCOPE;
	sensor->min_delay = 0;
	sensor->min_value = -69.81; /* -4000 dps -> rad/s (radians per second) */
	sensor->max_value = +69.81;
	sensor->resolution = 2.665e-7; /* 65.5 LSB/DPS */
}

/**************************************************************************/
/*!
 @brief  Gets the gyroscope as a standard sensor event
 @param  event Sensor event object that will be populated
 @returns True
 */
/**************************************************************************/
bool Adafruit_ICM20X_Gyro::getEvent(sensors_event_t *event) {
	_theICM20X->_read();
	_theICM20X->fillGyroEvent(event, HAL_GetTick());
	return true;
}

/**************************************************************************/
/*!
 @brief  Gets the sensor_t data for the ICM20X's magnetometer sensor
 */
/**************************************************************************/
void Adafruit_ICM20X_Magnetometer::getSensor(sensor_t *sensor) {
	/* Clear the sensor_t object */
	memset(sensor, 0, sizeof(sensor_t));

	/* Insert the sensor name in the fixed length char array */
	strncpy(sensor->name, "ICM20X_M", sizeof(sensor->name) - 1);
	sensor->name[sizeof(sensor->name) - 1] = 0;
	sensor->version = 1;
	sensor->sensor_id = _sensorID;
	sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
	sensor->min_delay = 0;
	sensor->min_value = -4900;
	sensor->max_value = 4900;
	sensor->resolution = 0.6667;
}

/**************************************************************************/
/*!
 @brief  Gets the magnetometer as a standard sensor event
 @param  event Sensor event object that will be populated
 @returns True
 */
/**************************************************************************/
bool Adafruit_ICM20X_Magnetometer::getEvent(sensors_event_t *event) {
	_theICM20X->_read();
	_theICM20X->fillMagEvent(event, HAL_GetTick());
	return true;
}

/**************************************************************************/
/*!
 @brief  Gets the sensor_t data for the ICM20X's tenperature
 */
/**************************************************************************/
void Adafruit_ICM20X_Temp::getSensor(sensor_t *sensor) {
	/* Clear the sensor_t object */
	memset(sensor, 0, sizeof(sensor_t));

	/* Insert the sensor name in the fixed length char array */
	strncpy(sensor->name, "ICM20X_T", sizeof(sensor->name) - 1);
	sensor->name[sizeof(sensor->name) - 1] = 0;
	sensor->version = 1;
	sensor->sensor_id = _sensorID;
	sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
	sensor->min_delay = 0;
	sensor->min_value = -40;
	sensor->max_value = 85;
	sensor->resolution = 0.0029952; /* 333.87 LSB/C => 1/333.87 C/LSB */
}

/**************************************************************************/
/*!
 @brief  Gets the temperature as a standard sensor event
 @param  event Sensor event object that will be populated
 @returns True
 */
/**************************************************************************/
bool Adafruit_ICM20X_Temp::getEvent(sensors_event_t *event) {
	_theICM20X->_read();
	_theICM20X->fillTempEvent(event, HAL_GetTick());
	return true;
}
