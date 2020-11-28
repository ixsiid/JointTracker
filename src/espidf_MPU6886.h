#pragma once

#include <esp_log.h>
#include <stdio.h>

#include "IIMU.h"
#include "Vector3.h"
#include "i2c.h"

#define MPU6886_ADDRESS 0x68
#define MPU6886_WHOAMI 0x75
#define MPU6886_ACCEL_INTEL_CTRL 0x69
#define MPU6886_SMPLRT_DIV 0x19
#define MPU6886_INT_PIN_CFG 0x37
#define MPU6886_INT_ENABLE 0x38
#define MPU6886_ACCEL_XOUT_H 0x3B
#define MPU6886_ACCEL_XOUT_L 0x3C
#define MPU6886_ACCEL_YOUT_H 0x3D
#define MPU6886_ACCEL_YOUT_L 0x3E
#define MPU6886_ACCEL_ZOUT_H 0x3F
#define MPU6886_ACCEL_ZOUT_L 0x40

#define MPU6886_TEMP_OUT_H 0x41
#define MPU6886_TEMP_OUT_L 0x42

#define MPU6886_GYRO_XOUT_H 0x43
#define MPU6886_GYRO_XOUT_L 0x44
#define MPU6886_GYRO_YOUT_H 0x45
#define MPU6886_GYRO_YOUT_L 0x46
#define MPU6886_GYRO_ZOUT_H 0x47
#define MPU6886_GYRO_ZOUT_L 0x48

#define MPU6886_USER_CTRL 0x6A
#define MPU6886_PWR_MGMT_1 0x6B
#define MPU6886_PWR_MGMT_2 0x6C
#define MPU6886_CONFIG 0x1A
#define MPU6886_GYRO_CONFIG 0x1B
#define MPU6886_ACCEL_CONFIG 0x1C
#define MPU6886_ACCEL_CONFIG2 0x1D
#define MPU6886_FIFO_EN 0x23

#define MPU6886_FIFO_ENABLE 0x23
#define MPU6886_FIFO_CONUTH 0x72
#define MPU6886_FIFO_CONUTL 0x73
#define MPU6886_FIFO_R_W 0x74

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

class MPU6886 : public IIMU {
    public:
	MPU6886(I2CMaster* i2c);

	Vector3<int16_t> getAccelAdc();
	Vector3<float> getAccel();
	Vector3<int16_t> getGyroAdc();
	Vector3<float> getGyro();

	int16_t getTemp();
	int16_t getTempAndGyroAdc(Vector3<int16_t>* gyro);
	int16_t getData(Vector3<int16_t>* accel, Vector3<int16_t>* gyro);

	void getAccelAdc(Vector3<int16_t>* accel);
	void getGyroAdc(Vector3<int16_t>* gyro);

	void calibrateZeroBias(uint8_t count, int32_t accel_threshould, int32_t gyro_threshould);
	void updateAhrs(Quaternion* result);

    private:
	float accel_resolution, gyro_resolution;
	AccelScale accel_scale;
	GyroScale gyro_scale;
	void getAccelResolution();
	void getGyroResolution();
	I2CMaster* i2c;
};

MPU6886::MPU6886(I2CMaster* i2c) {
	gyro_scale = GFS_2000DPS;
	accel_scale = AFS_8G;

	this->i2c = i2c;

	uint8_t who = i2c->read(MPU6886_ADDRESS, MPU6886_WHOAMI);
	if (who != 0x19) return;

	vTaskDelay(5 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 0x00);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 0x01 << 7);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 0x01 << 0);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 0x10);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 0x18);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_CONFIG, 0x01);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 0x05);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 0x00);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 0x00);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_USER_CTRL, 0x00);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_FIFO_EN, 0x00);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 0x22);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c->write(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 0x01);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	getGyroResolution();
	getAccelResolution();
}

void MPU6886::getGyroResolution() {
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

void MPU6886::getAccelResolution() {
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

Vector3<int16_t> MPU6886::getAccelAdc() {
	uint8_t buffer[6];

	i2c->read_bytes(MPU6886_ADDRESS, MPU6886_ACCEL_XOUT_H, buffer, 6);

	Vector3<int16_t> accel;
	accel.x = ((int16_t)buffer[0] << 8) | buffer[1];
	accel.y = ((int16_t)buffer[2] << 8) | buffer[3];
	accel.z = ((int16_t)buffer[4] << 8) | buffer[5];

	return accel;
}

Vector3<float> MPU6886::getAccel() {
	return getAccelAdc() * accel_resolution;
}

Vector3<int16_t> MPU6886::getGyroAdc() {
	uint8_t buffer[6];

	i2c->read_bytes(MPU6886_ADDRESS, MPU6886_GYRO_XOUT_H, buffer, 6);

	Vector3<int16_t> accel;
	accel.x = ((int16_t)buffer[0] << 8) | buffer[1];
	accel.y = ((int16_t)buffer[2] << 8) | buffer[3];
	accel.z = ((int16_t)buffer[4] << 8) | buffer[5];

	return accel;
}

Vector3<float> MPU6886::getGyro() {
	return getGyroAdc() * gyro_resolution;
}



void MPU6886::getAccelAdc(Vector3<int16_t> * accel) {
	uint8_t buffer[6];

	i2c->read_bytes(MPU6886_ADDRESS, MPU6886_ACCEL_XOUT_H, buffer, 6);

	accel->x = ((int16_t)buffer[0] << 8) | buffer[1];
	accel->y = ((int16_t)buffer[2] << 8) | buffer[3];
	accel->z = ((int16_t)buffer[4] << 8) | buffer[5];
}

void MPU6886::getGyroAdc(Vector3<int16_t> * gyro) {
	uint8_t buffer[6];

	i2c->read_bytes(MPU6886_ADDRESS, MPU6886_GYRO_XOUT_H, buffer, 6);
	// こうやったらどうなる？？？エンディアン合わないかな
	// i2c->read_bytes(MPU6886_ADDRESS, MPU6886_GYRO_XOUT_H, (uint8_t *)gyro, 6);

	gyro->x = ((int16_t)buffer[0] << 8) | buffer[1];
	gyro->y = ((int16_t)buffer[2] << 8) | buffer[3];
	gyro->z = ((int16_t)buffer[4] << 8) | buffer[5];
}

int16_t MPU6886::getTemp() {
	uint8_t buffer[2];
	i2c->read_bytes(MPU6886_ADDRESS, MPU6886_TEMP_OUT_H, buffer, 2);
	return ((int16_t)buffer[0] << 8) | buffer[1];
}


int16_t MPU6886::getTempAndGyroAdc(Vector3<int16_t> * gyro) {
	uint8_t buffer[8];

	i2c->read_bytes(MPU6886_ADDRESS, MPU6886_TEMP_OUT_H, buffer, 8);

	gyro->x = ((int16_t)buffer[2] << 8) | buffer[3];
	gyro->y = ((int16_t)buffer[4] << 8) | buffer[5];
	gyro->z = ((int16_t)buffer[6] << 8) | buffer[7];

	return ((int16_t)buffer[0] << 8) | buffer[1];
}


int16_t MPU6886::getData(Vector3<int16_t> * accel, Vector3<int16_t> * gyro) {
	uint8_t buffer[14];
	i2c->read_bytes(MPU6886_ADDRESS, MPU6886_ACCEL_XOUT_H, buffer, 6);

	accel->x = ((int16_t)buffer[0] << 8) | buffer[1];
	accel->y = ((int16_t)buffer[2] << 8) | buffer[3];
	accel->z = ((int16_t)buffer[4] << 8) | buffer[5];

	gyro->x = ((int16_t)buffer[8] << 8) | buffer[9];
	gyro->y = ((int16_t)buffer[10] << 8) | buffer[11];
	gyro->z = ((int16_t)buffer[12] << 8) | buffer[13];
	
	return ((int16_t)buffer[6] << 8) | buffer[7];
}

}  // namespace ESPIDF
