#pragma once

#include <esp_log.h>
#include <stdio.h>

#include "IIMU.h"
#include "Vector3.h"
#include "i2c.h"

//#define G (9.8)
#define RtA 57.324841
#define AtR 0.0174533
#define Gyro_Gr 0.0010653

namespace ESPIDF {

enum AccelScale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum GyroScale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum MagScale {
	MFS_4GAUSS = 0,
	MFS_8GAUSS,
	MFS_12GAUSS,
	MFS_16GAUSS
};

class LSM9DS1 : public IIMU {
    public:
	/// <summary>
	/// コンストラクタ
	/// </summary>
	/// <param name="i2c">I2Cオブジェクト</param>
	/// <param name="i2c">I2Cアドレスチャンネル channel = 0 : (SDOAG, SDOM) = High, channel = 1 : (SDOAG, SDOM) = Low</param>
	/// <returns>備考</returns>
	LSM9DS1(I2CMaster* i2c, uint8_t channel);

	Vector3<int16_t> getAccelAdc();
	Vector3<float> getAccel();
	Vector3<int16_t> getGyroAdc();
	Vector3<float> getGyro();
	Vector3<int16_t> getMagAdc();
	Vector3<float> getMag();

	int16_t getTemp();
	int16_t getTempAndGyroAdc(Vector3<int16_t>* gyro);

	void getAccelAdc(Vector3<int16_t>* accel);
	void getGyroAdc(Vector3<int16_t>* gyro);
	void getMagAdc(Vector3<int16_t>* mag);

	void calibrateZeroBias(uint8_t count, int32_t accel_threshould, int32_t gyro_threshould);
	void updateAhrs(Quaternion* result);

	void* getI2CMaster();

    private:
	float accel_resolution, gyro_resolution, mag_resolution;
	AccelScale accel_scale;
	GyroScale gyro_scale;
	MagScale mag_scale;
	void getAccelResolution();
	void getGyroResolution();
	void getMagResolution();
	I2CMaster* i2c;

	uint8_t channel;

	static const uint8_t AG_ADDRESS[];
	static const uint8_t M_ADDRESS[];
};

const uint8_t LSM9DS1::AG_ADDRESS[] = {0x6b, 0x6a};  // SDOAG: High(default), Low
const uint8_t LSM9DS1::M_ADDRESS[]	 = {0x1e, 0x1c};  // SDOM: High(default), Low

#define LSM9DS1_ADDRESS_AG (AG_ADDRESS[channel])
#define LSM9DS1_ADDRESS_M (M_ADDRESS[channel])

#define LSM9DS1_WHOAMI 0x0f
#define LSM9DS1_WHOAMI_AG_RSP 0x68
#define LSM9DS1_WHOAMI_M_RSP 0x3d

inline void* LSM9DS1::getI2CMaster() { return i2c; }

#define LSM9DS1_CTRL_REG1_G 0x10
#define LSM9DS1_CTRL_REG2_G 0x11
#define LSM9DS1_CTRL_REG3_G 0x12
#define LSM9DS1_ORIENT_CFG_G 0x13

#define LSM9DS1_CTRL_REG5_XL 0x1f
#define LSM9DS1_CTRL_REG6_XL 0x20
#define LSM9DS1_CTRL_REG7_XL 0x21

#define LSM9DS1_CTRL_REG1_M 0x20
#define LSM9DS1_CTRL_REG2_M 0x21
#define LSM9DS1_CTRL_REG3_M 0x22
#define LSM9DS1_CTRL_REG4_M 0x23
#define LSM9DS1_CTRL_REG5_M 0x24

#define LSM9DS1_OUT_X_G 0x18
#define LSM9DS1_OUT_Y_G 0x1a
#define LSM9DS1_OUT_Z_G 0x1c

#define LSM9DS1_OUT_X_XL 0x28
#define LSM9DS1_OUT_Y_XL 0x2a
#define LSM9DS1_OUT_Z_XL 0x2c

#define LSM9DS1_OUT_X_L_M 0x28
#define LSM9DS1_OUT_Y_L_M 0x2a
#define LSM9DS1_OUT_Z_L_M 0x2c

#define STATUS_REG 0x17
#define STATUS_REG_M 0x27

LSM9DS1::LSM9DS1(I2CMaster* i2c, uint8_t channel) {
	gyro_scale  = GFS_2000DPS;
	accel_scale = AFS_8G;
	mag_scale	  = MFS_4GAUSS;

	this->i2c	    = i2c;
	this->channel = channel;

	uint8_t who = i2c->read(LSM9DS1_ADDRESS_AG, LSM9DS1_WHOAMI);
	printf("who AG %d\n", who);
	if (who != LSM9DS1_WHOAMI_AG_RSP) return;
	who = i2c->read(LSM9DS1_ADDRESS_M, LSM9DS1_WHOAMI);
	printf("who M %d\n", who);
	if (who != LSM9DS1_WHOAMI_M_RSP) return;

	vTaskDelay(5 / portTICK_PERIOD_MS);

	/// Gyro configure
	// ODR 952Hz, 2000 dps, Cutoff 33Hz ※HPF, LPF2は使わないのでCutoffは意味なし
	i2c->write(LSM9DS1_ADDRESS_AG, LSM9DS1_CTRL_REG1_G, 0b11011011);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// LPF2 disable
	i2c->write(LSM9DS1_ADDRESS_AG, LSM9DS1_CTRL_REG2_G, 0b00000001);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// Low power mode: disable, HPF: disable
	i2c->write(LSM9DS1_ADDRESS_AG, LSM9DS1_CTRL_REG3_G, 0b01001001);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	//
	i2c->write(LSM9DS1_ADDRESS_AG, LSM9DS1_ORIENT_CFG_G, 0b00000000);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	/// Accel configure
	// enable
	i2c->write(LSM9DS1_ADDRESS_AG, LSM9DS1_CTRL_REG5_XL, 0b00111000);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// ODR 952Hz, full-scale 8g, Bandwidth 408Hz
	i2c->write(LSM9DS1_ADDRESS_AG, LSM9DS1_CTRL_REG6_XL, 0b11011000);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// Filter bypassed
	i2c->write(LSM9DS1_ADDRESS_AG, LSM9DS1_CTRL_REG7_XL, 0b00000000);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	/// Magnetic sensor configure
	// Temperature compensation enabled, XY ultra high performance mode, ODR 80Hz, fast odr, no self-test
	i2c->write(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0b01111100);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// ±4Gauss,
	i2c->write(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0b00000000);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// i2c enabled, low-power disabled, SPI disabled, continous mode
	i2c->write(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0b00000000);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// z ultra high performance mode, Little endian
	i2c->write(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG4_M, 0b00001100);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// Fast-read, continous update
	i2c->write(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG5_M, 0b00000000);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	//////////////////////////

	getGyroResolution();
	getAccelResolution();
	getMagResolution();

	uint8_t ag_status = i2c->read(LSM9DS1_ADDRESS_AG, STATUS_REG);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	uint8_t m_status = i2c->read(LSM9DS1_ADDRESS_M, STATUS_REG_M);
	// Check Available
	printf("Status %x %x\n", ag_status, m_status);
}

void LSM9DS1::getGyroResolution() {
	switch (gyro_scale) {
			// Possible gyro scales (and their register bit settings) are:
		case GFS_250DPS:
			gyro_resolution = 250.0 / 32768.0;
			break;
		case GFS_500DPS:
			gyro_resolution = 500.0 / 32768.0;
			break;
		case GFS_1000DPS:
			gyro_resolution = 1000.0 / 32768.0;
			break;
		case GFS_2000DPS:
			gyro_resolution = 2000.0 / 32768.0;
			break;
	}
}

void LSM9DS1::getAccelResolution() {
	switch (accel_scale) {
			// Possible accelerometer scales (and their register bit settings) are:
			// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			accel_resolution = 2.0 / 32768.0;
			break;
		case AFS_4G:
			accel_resolution = 4.0 / 32768.0;
			break;
		case AFS_8G:
			accel_resolution = 8.0 / 32768.0;
			break;
		case AFS_16G:
			accel_resolution = 16.0 / 32768.0;
			break;
	}
}

void LSM9DS1::getMagResolution() {
	switch (mag_scale) {
		case MFS_4GAUSS:
			mag_resolution = 4.0 / 32768.0;
			break;
		case MFS_8GAUSS:
			mag_resolution = 8.0 / 32768.0;
			break;
		case MFS_12GAUSS:
			mag_resolution = 12.0 / 32768.0;
			break;
		case MFS_16GAUSS:
			mag_resolution = 16.0 / 32768.0;
			break;
	}
}

inline Vector3<float> LSM9DS1::getAccel() { return getAccelAdc() * accel_resolution; }
inline Vector3<float> LSM9DS1::getGyro() { return getGyroAdc() * gyro_resolution; }
inline Vector3<float> LSM9DS1::getMag() { return getMagAdc() * mag_resolution; }

Vector3<int16_t> LSM9DS1::getAccelAdc() {
	Vector3<int16_t> accel;
	i2c->read_bytes(LSM9DS1_ADDRESS_AG, LSM9DS1_OUT_X_XL, (uint8_t *)&accel, 6);
	return accel;
}

Vector3<int16_t> LSM9DS1::getGyroAdc() {
	Vector3<int16_t> gyro;
	i2c->read_bytes(LSM9DS1_ADDRESS_AG, LSM9DS1_OUT_X_G, (uint8_t *)&gyro, 6);
	return gyro;
}

Vector3<int16_t> LSM9DS1::getMagAdc() {
	Vector3<int16_t> mag;
	i2c->read_bytes(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t *)&mag, 6);
	return mag;
}

void LSM9DS1::getAccelAdc(Vector3<int16_t>* accel) {
	i2c->read_bytes(LSM9DS1_ADDRESS_AG, LSM9DS1_OUT_X_XL, (uint8_t *)accel, 6);
}

void LSM9DS1::getGyroAdc(Vector3<int16_t>* gyro) {
	i2c->read_bytes(LSM9DS1_ADDRESS_AG, LSM9DS1_OUT_X_G, (uint8_t*)gyro, 6);
}

void LSM9DS1::getMagAdc(Vector3<int16_t>* mag) {
	i2c->read_bytes(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t *)mag, 6);
}

}  // namespace ESPIDF
