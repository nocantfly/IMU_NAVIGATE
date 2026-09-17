/*!   @file Adafruit_ICM20948.cpp
 */
//#include "Arduino.h"
//#include <Wire.h>
#include "Adafruit_ICM20948.h"
#include "Adafruit_ICM20X.h"
#include "main.h"
/*!
 *    @brief  Instantiates a new ICM20948 class!
 */

Adafruit_ICM20948::Adafruit_ICM20948(void) {
}
/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            An optional parameter to set the sensor ids to differentiate
 * similar sensors The passed value is assigned to the accelerometer and the
 * gyro get +1 and the temperature sensor +2.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_ICM20948::begin_I2C(uint8_t i2c_address, I2C_HandleTypeDef *i2c_handle, int32_t sensor_id) {


	i2c_han = i2c_handle;
	i2c_addr = i2c_address << 1;

	bool init_success = _init(sensor_id);
	u_printf("\r\nimu_done\r\n");
	if(init_success) init_success = setupMag_ByPass();
	// todo: the below function doesnt execute properly, not sure why yet (or if its needed)
	if (!init_success) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(1000);
		return false;
	}
	u_printf("mag_done\r\n");
	is_initialized = true;
	return init_success;

}

// A million thanks to the SparkFun folks for their library that I pillaged to
// write this method! See their Arduino library here:
// https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
bool Adafruit_ICM20948::auxI2CBusSetupFailed(void) {
	// check aux I2C bus connection by reading the magnetometer chip ID
	bool aux_i2c_setup_failed = true;
	for (int i = 0; i < I2C_MASTER_RESETS_BEFORE_FAIL; i++) {
		if (getMagId() != ICM20948_MAG_ID) {
			resetI2CMaster();
		} else {
			aux_i2c_setup_failed = false;
			u_printf("MAG_ID_TRUE\r\n");
			break;
		}
	}
	return aux_i2c_setup_failed;
}

uint8_t Adafruit_ICM20948::getMagId(void) {
	// verify the magnetometer id
	return readExternalRegister(0x0C, 0x01);
}

/*bool Adafruit_ICM20948::setupMag(void) {
	uint8_t buffer[2];

	setI2CBypass(false);
	HAL_Delay(10); // Cho thời gian bypass chuyển trạng thái

	resetI2CMaster(); // THÊM DÒNG NÀY ĐỂ RESET I2C MASTER BÊN TRONG
	configureI2CMaster();

	enableI2CMaster(true);
    HAL_Delay(10);

	if (auxI2CBusSetupFailed()) {
		u_printf("Aux I2C Setup Failed!\r\n");
		return false;
	}
	writeMagRegister(0x32, 0x01);
	HAL_Delay(100);
	if (!writeMagRegister(0x31, 0x08)) {
		u_printf("Loi ghi CNTL2 0x31!\r\n");
		return false;
	}
	HAL_Delay(10);


	// set mag data rate
	if (!setMagDataRate(AK09916_MAG_DATARATE_100_HZ)) {
//		Serial.println("Error setting magnetometer data rate on external bus");
		return false;
	}

	// TODO: extract method
	// Set up Slave0 to proxy Mag readings
	_setBank(3);
	// set up slave0 to proxy reads to mag
	buffer[0] = ICM20X_B3_I2C_SLV0_ADDR;
	buffer[1] = 0x8C;
	if (!writeRegister(ICM20X_B3_I2C_SLV0_ADDR, &buffer[1], 1)) {
		return false;
	}
	HAL_Delay(5);

	buffer[0] = ICM20X_B3_I2C_SLV0_REG;
	buffer[1] = 0x10;
	if (!writeRegister(ICM20X_B3_I2C_SLV0_REG, &buffer[1], 1)) {
		return false;
	}
	HAL_Delay(5);

	buffer[0] = ICM20X_B3_I2C_SLV0_CTRL;
	buffer[1] = 0x89; // enable, read 9 bytes
	if (!writeRegister(ICM20X_B3_I2C_SLV0_CTRL, &buffer[1], 1)) {
		return false;
	}
	HAL_Delay(5);

	return true;
}*/
bool Adafruit_ICM20948::setupMag_ByPass(void) {
    uint8_t val;
    HAL_StatusTypeDef status;
    uint8_t mag_addr = 0x0C << 1;

    enableI2CMaster(false);
    HAL_Delay(10);

    setI2CBypass(true);
    HAL_Delay(10);

    u_printf("Tien hanh Setup Mag bang BYPASS Vinh Vien...\r\n");

    uint8_t wia_val = 0;
    status = HAL_I2C_Mem_Read(i2c_han, mag_addr, 0x01, 1, &wia_val, 1, 100);
    if (status == HAL_OK && wia_val == 0x09) {
        u_printf(">> AK09916 San Sang!\r\n");
    } else {
        u_printf(">> Loi: Khong tim thay AK09916 hoac WIA sai!\r\n");
        return false;
    }

    val = 0x00; // Sleep Mode
    HAL_I2C_Mem_Write(i2c_han, mag_addr, 0x31, 1, &val, 1, 100);
    HAL_Delay(20);

    val = 0x01; // Soft Reset
    HAL_I2C_Mem_Write(i2c_han, mag_addr, 0x32, 1, &val, 1, 100);
    HAL_Delay(50);

    val = 0x08; // Continuous Mode 4 (100Hz)
    HAL_I2C_Mem_Write(i2c_han, mag_addr, 0x31, 1, &val, 1, 100);
    HAL_Delay(20);

    u_printf("Hoan tat Setup Mag!\r\n");
    return true;
}
//bool Adafruit_ICM20948::setupMag(void) {
//    uint8_t val;
//    u_printf("\r\n[INFO] Tien hanh Setup Mag qua I2C Master noi bo (SPI Mode)...\r\n");
//
//    // =========================================================================
//    // GIAI ĐOẠN 1: DỌN DẸP "RÁC" PHẦN CỨNG (CỰC KỲ QUAN TRỌNG)
//    // =========================================================================
//    _setBank(0);
//
//    // 1. Ép chip dùng xung Clock nội bộ tốt nhất (Auto-select PLL)
//    writeRegisterByte(ICM20X_B0_PWR_MGMT_1, 0x01);
//    HAL_Delay(20);
//
//    // 2. Tắt I2C Master VÀ Tắt I2C Slave (SPI Only)
//    // Ghi 0x10 vào USER_CTRL (Tắt Master = Bit 5=0, Tắt I2C = Bit 4=1)
//    writeRegisterByte(ICM20X_B0_USER_CTRL, 0x10);
//    HAL_Delay(20);
//
//    // =========================================================================
//    // GIAI ĐOẠN 2: RESET VÀ KHỞI ĐỘNG LẠI I2C MASTER TỪ ĐẦU
//    // =========================================================================
//    // 3. Reset Hardware cho I2C Master (Ghi bit 1)
//    val = readRegisterByte(ICM20X_B0_USER_CTRL);
//    writeRegisterByte(ICM20X_B0_USER_CTRL, val | 0x02);
//    HAL_Delay(20);
//
//    // 4. Bật lại I2C Master (Ghi bit 5 = 1)
//    // Đảm bảo SPI Only (Bit 4 = 1) vẫn được giữ nguyên
//    val = readRegisterByte(ICM20X_B0_USER_CTRL);
//    writeRegisterByte(ICM20X_B0_USER_CTRL, val | 0x20 | 0x10);
//    HAL_Delay(50); // CHỜ 50ms LÀ BẮT BUỘC ĐỂ I2C MASTER SẠC ĐẦY NĂNG LƯỢNG!
//
//    // BẬT CHẾ ĐỘ I2C_MST_CYCLE ĐỂ MASTER CHẠY ĐỘC LẬP
//    writeRegisterByte(ICM20X_B0_LP_CONFIG, 0x40);
//    HAL_Delay(10);
//
//    // =========================================================================
//    // GIAI ĐOẠN 3: CẤU HÌNH TỐC ĐỘ CLOCK I2C NỘI BỘ
//    // =========================================================================
//    _setBank(3);
//
//    // Tốc độ bus I2C nội bộ: 400kHz (0x07)
//    writeRegisterByte(ICM20X_B3_I2C_MST_CTRL, 0x07);
//    HAL_Delay(20);
//
//    // =========================================================================
//    // GIAI ĐOẠN 4: TEST KẾT NỐI AK09916 (SLV4) - ĐỌC WIA
//    // =========================================================================
//    writeRegisterByte(ICM20X_B3_I2C_SLV4_REG, 0x01); // Thanh ghi 0x01 (WIA)
//    writeRegisterByte(ICM20X_B3_I2C_SLV4_ADDR, 0x8C); // 0x0C | 0x80 (Read)
//    writeRegisterByte(ICM20X_B3_I2C_SLV4_CTRL, 0x80); // Kích hoạt SLV4 (Bit 7 = 1)
//
//    // Chờ I2C Master hoàn thành nhiệm vụ
//    _setBank(0);
//    uint8_t tries = 0;
//    bool mag_alive = false;
//
//    while (tries < 100) {
//        val = readRegisterByte(ICM20X_B0_I2C_MST_STATUS);
//
//        // KIỂM TRA LỖI NACK TRƯỚC (Bit 4)
//        if (val & 0x10) {
//            u_printf(">> [CẢNH BÁO] I2C Master chay nhung AK09916 tu choi tra loi (NACK)!\r\n");
//            break;
//        }
//
//        // KIỂM TRA CỜ DONE (Bit 6)
//        if (val & 0x40) {
//            mag_alive = true;
//            break;
//        }
//
//        HAL_Delay(1);
//        tries++;
//    }
//
//    if (!mag_alive) {
//        u_printf(">> [LOI TAI HAI] I2C Master bi treo cung (Timeout SLV4)!\r\n");
//        return false;
//    }
//
//    // Lấy kết quả từ SLV4_DI
//    _setBank(3);
//    uint8_t mag_id = readRegisterByte(ICM20X_B3_I2C_SLV4_DI);
//    u_printf(">> SPI Doc Tu Ke WIA: 0x%02X (Ky Vong: 0x09)\r\n", mag_id);
//
//    if (mag_id != 0x09) {
//        u_printf(">> [LOI] Sai ma ID Tu ke. Huy khoi tao Mag!\r\n");
//        return false;
//    }
//
//    // =========================================================================
//    // GIAI ĐOẠN 5: CẤU HÌNH TỪ KẾ CHẠY CONTINUOUS MODE 100Hz
//    // =========================================================================
//    // Ghi 0x01 vào 0x32 (CNTL3 - Soft Reset Mag)
//    if (!writeMagRegisterSLV4(0x32, 0x01)) return false;
//    HAL_Delay(100);
//
//    // Ghi 0x08 vào 0x31 (CNTL2 - Continuous Mode 4: 100Hz)
//    if (!writeMagRegisterSLV4(0x31, 0x08)) return false;
//    HAL_Delay(20);
//
//    // =========================================================================
//    // GIAI ĐOẠN 6: ỦY QUYỀN CHO SLV0 TỰ ĐỘNG ĐỌC 9 BYTE DỮ LIỆU
//    // =========================================================================
//    _setBank(3);
//    writeRegisterByte(ICM20X_B3_I2C_SLV0_ADDR, 0x8C); // Mag (0x0C) + Read (0x80)
//
//    // ĐIỂM CHẾT LỖI BYTE LỆCH: BẮT BUỘC ĐỌC TỪ ST1 (0x10) THAY VÌ HXL (0x11)
//    writeRegisterByte(ICM20X_B3_I2C_SLV0_REG, 0x11);
//
//    writeRegisterByte(ICM20X_B3_I2C_SLV0_CTRL, 0x88); // Bật SLV0 (EN) + Đọc 9 byte
//    HAL_Delay(10);
//
//    _setBank(0);
//
//    u_printf("[OK] Hoan tat Setup Mag SPI thanh cong ruc ro!\r\n");
//    return true;
//}

bool Adafruit_ICM20948::setupMag(void) {
    uint8_t val;
    u_printf("\r\n[INFO] Tien hanh Setup Mag qua I2C Master noi bo (SPI Mode)...\r\n");

    // =========================================================================
    // GIAI ĐOẠN 1: KHỞI ĐỘNG HỆ THỐNG VÀ BẬT BYPASS ĐỂ MƯỢN TRỞ KÉO
    // =========================================================================
    _setBank(0);
    writeRegisterByte(ICM20X_B0_PWR_MGMT_1, 0x01); // Wake up & PLL Clock
    HAL_Delay(100);

    // BẬT BYPASS (true) ĐỂ THÔNG MẠCH SỬ DỤNG ĐIỆN TRỞ KÉO LÊN CỦA BUS CHÍNH!
    setI2CBypass(true);
    HAL_Delay(15);

    _setBank(0);
    uint8_t check_bypass = readRegisterByte(ICM20X_B0_REG_INT_PIN_CFG);
    u_printf(">> [DIAG] Doc nguoc INT_PIN_CFG: 0x%02X (Ky vong Bit 2 bang 1, vi du: 0xD4)\r\n", check_bypass);

    // =========================================================================
    // GIAI ĐOẠN 2: CHUỖI RESET VÀ KHỞI ĐỘNG MASTER CHUẨN ARDUPILOT
    // =========================================================================
    writeRegisterByte(ICM20X_B0_USER_CTRL, 0x30); // Enable Master, Disable Host I2C
    HAL_Delay(15);

    writeRegisterByte(ICM20X_B0_USER_CTRL, 0x32); // Enable Master, Disable Host I2C, Reset Master
    HAL_Delay(20);

    writeRegisterByte(ICM20X_B0_USER_CTRL, 0x30); // Release Reset
    HAL_Delay(30);

    uint8_t check_ctrl = readRegisterByte(ICM20X_B0_USER_CTRL);
    u_printf(">> [DIAG] Doc nguoc USER_CTRL: 0x%02X (Ky vong: 0x30)\r\n", check_ctrl);

    writeRegisterByte(ICM20X_B0_LP_CONFIG, 0x40); // Bật Clock Master I2C
    HAL_Delay(10);

    // =========================================================================
    // GIAI ĐOẠN 3: CẤU HÌNH TỐC ĐỘ BUS PHỤ VÀ STOP CONDITION (0x17)
    // =========================================================================
    _setBank(3);
    writeRegisterByte(ICM20X_B3_I2C_MST_CTRL, 0x17);
    writeRegisterByte(ICM20X_B3_I2C_MST_DELAY_CTRL, 0x00);
    HAL_Delay(15);

    // =========================================================================
    // GIAI ĐOẠN 4: CHƯƠNG TRÌNH QUÉT ĐỊA CHỈ I2C NỘI BỘ (I2C SCANNER OVER SPI)
    // =========================================================================
    u_printf("\r\n===============================================\r\n");
    u_printf(">> BAT DAU QUET TOAN BO BUS I2C PHU TREN CHIP (BYPASS ON)...\r\n");
    u_printf("===============================================\r\n");

    uint8_t devices_found = 0;

    for (uint8_t addr = 0x01; addr < 128; addr++) {
        _setBank(3);
        // Cấu hình SLV4 đọc: Địa chỉ ghi dịch trái và bật bit Read (addr | 0x80)
        writeRegisterByte(ICM20X_B3_I2C_SLV4_ADDR, addr | 0x80);
        writeRegisterByte(ICM20X_B3_I2C_SLV4_REG, 0x01);  // Thử truy vấn thanh ghi WIA2
        writeRegisterByte(ICM20X_B3_I2C_SLV4_CTRL, 0x80); // Kích hoạt giao dịch đơn lẻ
        HAL_Delay(1);

        _setBank(0);
        uint8_t tries = 0;
        bool done = false;
        uint8_t status_val = 0;

        while (tries < 20) {
            status_val = readRegisterByte(ICM20X_B0_I2C_MST_STATUS);
            if (status_val & 0x40) { // DONE = 1
                done = true;
                break;
            }
            HAL_Delay(1);
            tries++;
        }

        if (done) {
            if (!(status_val & 0x10)) { // NACK = 0
                u_printf("  -> [FOUND] Tim thay thiet bi tai dia chi: 0x%02X (STATUS: 0x%02X)\r\n", addr, status_val);
                devices_found++;
            }
        } else {
            resetI2CMaster();
        }
        HAL_Delay(2);
    }

    u_printf("===============================================\r\n");
    u_printf(">> QUET HOAN TAT! Tim thay tong cong: %d thiet bi.\r\n", devices_found);
    u_printf("===============================================\r\n\r\n");

    return false;
}
/*bool Adafruit_ICM20948::setupMag(void) {
    uint8_t val;
    HAL_StatusTypeDef status;
    uint8_t mag_addr = 0x0C << 1; // Địa chỉ HAL I2C của La bàn (0x18)

    enableI2CMaster(false);
    HAL_Delay(10);

    setI2CBypass(true);
    HAL_Delay(10);

    u_printf("Tien hanh Setup Mag bang BYPASS (STM32 Truc tiep)...\r\n");

    u_printf(">> Scanning I2C Bus...\r\n");
    bool mag_found = false;
    for (uint8_t i = 1; i < 128; i++) {
        // Dịch trái 1 bit vì HAL STM32 yêu cầu địa chỉ 8-bit
        if (HAL_I2C_IsDeviceReady(i2c_han, (uint16_t)(i << 1), 2, 10) == HAL_OK) {
            u_printf("   -> Tim thay thiet bi tai dia chi: 0x%02X\r\n", i);
            if (i == 0x0C) mag_found = true;
        }
    }
    if (!mag_found) {
        u_printf("   -> CANH BAO: Khong tim thay Mag AK09916 tai 0x0C!\r\n");
    }

    // =========================================================
    // THÊM MỚI: ĐỌC THANH GHI WHO_AM_I (WIA) CỦA AK09916
    // =========================================================
    uint8_t wia_val = 0;
    // Đọc 1 byte từ thanh ghi 0x01 của thiết bị 0x0C
    status = HAL_I2C_Mem_Read(i2c_han, mag_addr, 0x01, 1, &wia_val, 1, 100);
    if (status == HAL_OK) {
        u_printf(">> Mag WIA (WHO_AM_I) = 0x%02X ", wia_val);
        if (wia_val == 0x09) {
            u_printf("(AK09916 Chuan!)\r\n");
        } else {
            u_printf("(Sai ma ID!)\r\n");
        }
    } else {
        u_printf(">> Loi Bypass: Khong the doc WIA cua Mag!\r\n");
    }
    HAL_Delay(10);

    val = 0x00;
    status = HAL_I2C_Mem_Write(i2c_han, mag_addr, 0x31, 1, &val, 1, 100);
    if(status != HAL_OK) u_printf("Loi Bypass: Khong the dua Mag ve Sleep!\r\n");
    HAL_Delay(20);

    val = 0x01;
    status = HAL_I2C_Mem_Write(i2c_han, mag_addr, 0x32, 1, &val, 1, 100);
    if(status != HAL_OK) u_printf("Loi Bypass: Khong the Reset Mag!\r\n");
    HAL_Delay(50);

    val = 0x06;
    status = HAL_I2C_Mem_Write(i2c_han, mag_addr, 0x31, 1, &val, 1, 100);
    if(status != HAL_OK) u_printf("Loi Bypass: Khong the Set Mode 50Hz!\r\n");
    HAL_Delay(20);


    setI2CBypass(false);
    HAL_Delay(10);

    resetI2CMaster();

    _setBank(3);
    val = 0x8C; writeRegister(ICM20X_B3_I2C_SLV0_ADDR, &val, 1);
    val = 0x10; writeRegister(ICM20X_B3_I2C_SLV0_REG, &val, 1);
    val = 0x89; writeRegister(ICM20X_B3_I2C_SLV0_CTRL, &val, 1);
    HAL_Delay(10);
    _setBank(0);

    u_printf("Hoan tat Setup Mag!\r\n");
    return true;
}*/
/**
 * @brief
 *
 * @param slv_addr
 * @param mag_reg_addr
 * @param num_finished_checks
 * @return uint8_t
 */
uint8_t Adafruit_ICM20948::readMagRegister(uint8_t mag_reg_addr) {
	return readExternalRegister(0x8C, mag_reg_addr);
}

bool Adafruit_ICM20948::writeMagRegister(uint8_t mag_reg_addr, uint8_t value) {
	return writeExternalRegister(0x0C, mag_reg_addr, value);
}

void Adafruit_ICM20948::scaleValues(void) {

	icm20948_gyro_range_t gyro_range = (icm20948_gyro_range_t) current_gyro_range;
	icm20948_accel_range_t accel_range =
			(icm20948_accel_range_t) current_accel_range;

	float accel_scale = 1.0;
	float gyro_scale = 1.0;

	if (gyro_range == ICM20948_GYRO_RANGE_250_DPS)
		gyro_scale = 131.0;
	if (gyro_range == ICM20948_GYRO_RANGE_500_DPS)
		gyro_scale = 65.5;
	if (gyro_range == ICM20948_GYRO_RANGE_1000_DPS)
		gyro_scale = 32.8;
	if (gyro_range == ICM20948_GYRO_RANGE_2000_DPS)
		gyro_scale = 16.4;

	if (accel_range == ICM20948_ACCEL_RANGE_2_G)
		accel_scale = 16384.0;
	if (accel_range == ICM20948_ACCEL_RANGE_4_G)
		accel_scale = 8192.0;
	if (accel_range == ICM20948_ACCEL_RANGE_8_G)
		accel_scale = 4096.0;
	if (accel_range == ICM20948_ACCEL_RANGE_16_G)
		accel_scale = 2048.0;

	gyroX = rawGyroX / gyro_scale;
	gyroY = rawGyroY / gyro_scale;
	gyroZ = rawGyroZ / gyro_scale;

	accX = rawAccX / accel_scale;
	accY = rawAccY / accel_scale;
	accZ = rawAccZ / accel_scale;

	magX = rawMagX * ICM20948_UT_PER_LSB;
	magY = rawMagY * ICM20948_UT_PER_LSB;
	magZ = rawMagZ * ICM20948_UT_PER_LSB;
}

/**************************************************************************/
/*!
 @brief Get the accelerometer's measurement range.
 @returns The accelerometer's measurement range (`icm20948_accel_range_t`).
 */
icm20948_accel_range_t Adafruit_ICM20948::getAccelRange(void) {
	return (icm20948_accel_range_t) readAccelRange();
}

/**************************************************************************/
/*!

 @brief Sets the accelerometer's measurement range.
 @param  new_accel_range
 Measurement range to be set. Must be an
 `icm20948_accel_range_t`.
 */
void Adafruit_ICM20948::setAccelRange(icm20948_accel_range_t new_accel_range) {
	writeAccelRange((uint8_t) new_accel_range);
}

/**************************************************************************/
/*!
 @brief Get the gyro's measurement range.
 @returns The gyro's measurement range (`icm20948_gyro_range_t`).
 */
icm20948_gyro_range_t Adafruit_ICM20948::getGyroRange(void) {
	return (icm20948_gyro_range_t) readGyroRange();
}

/**************************************************************************/
/*!

 @brief Sets the gyro's measurement range.
 @param  new_gyro_range
 Measurement range to be set. Must be an
 `icm20948_gyro_range_t`.
 */
void Adafruit_ICM20948::setGyroRange(icm20948_gyro_range_t new_gyro_range) {
	writeGyroRange((uint8_t) new_gyro_range);
}

/**
 * @brief Get the current magnetometer measurement rate
 *
 * @return ak09916_data_rate_t the current rate
 */
ak09916_data_rate_t Adafruit_ICM20948::getMagDataRate(void) {

	uint8_t raw_mag_rate = readMagRegister(AK09916_CNTL2);
	return (ak09916_data_rate_t)(raw_mag_rate);
}
/**
 * @brief Set the magnetometer measurement rate
 *
 * @param rate The rate to set.
 *
 * @return true: success false: failure
 */
bool Adafruit_ICM20948::setMagDataRate(ak09916_data_rate_t rate) {
	/*
	 * Following the datasheet, the sensor will be set to
	 * AK09916_MAG_DATARATE_SHUTDOWN followed by a 100ms delay, followed by
	 * setting the new data rate.
	 *
	 * See page 9 of https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf
	 */

	// don't need to read/mask because there's nothing else in the register and
	// it's right justified
	bool success = writeMagRegister(AK09916_CNTL2, AK09916_MAG_DATARATE_SHUTDOWN);
	HAL_Delay(1);
	return writeMagRegister(AK09916_CNTL2, rate) && success;
}


bool Adafruit_ICM20948::getSample_byPass(imu_sample* data) {

	if (!is_initialized)
	{
		return false;
	}
	data->timestamp = HAL_GetTick();
	_read_ByPass();

	data->gyroX = gyroX;
	data->gyroY = gyroY;
	data->gyroZ = gyroZ;

	data->magX = magX;
	data->magY = magY;
	data->magZ = magZ;

	data->accX = accX;
	data->accY = accY;
	data->accZ = accZ;

	data->temperature = (temperature / 333.87) + 21.0;

	return true;
}

bool Adafruit_ICM20948::getSample(imu_sample* data) {

	if (!is_initialized)
	{
		return false;
	}
	data->timestamp = HAL_GetTick();
	_read();

	data->gyroX = gyroX;
	data->gyroY = gyroY;
	data->gyroZ = gyroZ;

	data->magX = magX;
	data->magY = magY;
	data->magZ = magZ;

	data->accX = accX;
	data->accY = accY;
	data->accZ = accZ;

	data->temperature = (temperature / 333.87) + 21.0;

	return true;
}

