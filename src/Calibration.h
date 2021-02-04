#pragma once

#include <stdint.h>

#include "IIMU.h"
#include "Vector3.h"

struct temper_chara_t {
	Vector3<float> a, b;
};

class Calibration {
    public:
	enum Mode {
		None		  = 0,
		Gyro		  = 1 << 1,
		Accelemeter = 1 << 2,
	};

	/// count数分のサンプリングからゼロバイアスを推定します
	Calibration(IIMU *imu, int32_t count, uint32_t gyro_threshould = 100, uint32_t accel_threshould = 360);
	/// 測定済みの温度特性からゼロバイアスを推定します
	Calibration(IIMU *imu, temper_chara_t chara);
	~Calibration();
	void regist(Mode mode);
	void regist(int mode);
	bool proccess();
	void getStatus(char *buffer);

	void getGyroAdc(Vector3<int32_t> *value);
	void getAccelAdc(Vector3<int32_t> *value);

	void getGyroAdcWithCalibrate(Vector3<int32_t> *value);

	void getData(Vector3<int32_t> *accel, Vector3<int32_t> *gyro);
	Vector3<int32_t> a, g;
	Vector3<int32_t> gyro_a, gyro_b;

    private:
	void calcOffsetByTemperCharacteristics();

	IIMU *sensor;
	int32_t count_limit;
	uint32_t gyro_threshould, accel_threshould;
	Vector3<int16_t> *raw;
	int32_t count;
	Mode mode;
	Vector3<int32_t> sum;
};

inline void Calibration::regist(int mode) { this->mode = static_cast<Mode>(mode); }

Calibration::Calibration(IIMU *imu, int count, uint32_t gyro_threshould, uint32_t accel_threshould) {
	this->sensor		   = imu;
	this->count_limit	   = count;
	this->gyro_threshould  = gyro_threshould * gyro_threshould;
	this->accel_threshould = accel_threshould * accel_threshould;

	raw = new Vector3<int16_t>[count];

	a = {0, 0, 0};
	g = {0, 0, 0};

	this->count = 0;
	this->mode  = Mode::None;
}

Calibration::Calibration(IIMU *imu, temper_chara_t chara) {
	this->sensor	   = imu;
	this->count_limit = -1;

	a = {0, 0, 0};

	gyro_a = chara.a * 0x10000;
	gyro_b = chara.b * 0x10000;

	calcOffsetByTemperCharacteristics();

	this->mode = Mode::None;
}

void Calibration::calcOffsetByTemperCharacteristics() {
	int32_t temper = sensor->getTemp();

	g = (gyro_a * temper + gyro_b) / 0x10000;
}

void Calibration::regist(Mode mode) {
	this->mode = mode;
}

bool Calibration::proccess() {
	if (!mode) return true;
	if (count_limit <= 0) return true;

	if (count >= count_limit) {
		sum = {0, 0, 0};

		Vector3<int32_t> min = {raw[0].x, raw[0].y, raw[0].z};
		Vector3<int32_t> max = {raw[0].x, raw[0].y, raw[0].z};

		for (int i = 1; i < count_limit; i++) {
			if (raw[i].x > max.x) max.x = raw[i].x;
			if (raw[i].y > max.x) max.y = raw[i].y;
			if (raw[i].z > max.x) max.z = raw[i].z;
			if (raw[i].x < min.x) min.x = raw[i].x;
			if (raw[i].y < min.x) min.y = raw[i].y;
			if (raw[i].z < min.x) min.z = raw[i].z;

			sum += raw[i];
		}

		int32_t dm = (max - min).Dot2();

		count = 0;
		if (mode & Mode::Gyro) {
			if (dm < gyro_threshould) {
				g = -sum / count_limit;

				mode = static_cast<Mode>(mode & ~Mode::Gyro);
			}
		} else if (mode & Mode::Accelemeter) {
			if (dm < accel_threshould) {
				a = -sum / count_limit;

				mode = static_cast<Mode>(mode & ~Mode::Accelemeter);
			}
		}
	} else {
		if (mode & Mode::Gyro) {
			sensor->getGyroAdc(&raw[count]);
		} else if (mode & Mode::Accelemeter) {
			sensor->getAccelAdc(&raw[count]);
			raw[count].z -= 4096;  // 8G Scale (range 32768)の1G
		}
		count++;
	}

	return false;
}

void Calibration::getStatus(char *buffer) {
	if (!mode) {
		sprintf(buffer, "Done. ");
	} else {
		if (mode & Mode::Gyro) {
			sprintf(buffer, "Gyro processing");
		} else if (mode & Mode::Accelemeter) {
			sprintf(buffer, "Accelemeter processing");
		} else {
			sprintf(buffer, "Unknown");
		}
	}
}

Calibration::~Calibration() {
	if (count_limit > 0) delete[] raw;
}

void Calibration::getGyroAdc(Vector3<int32_t> *value) {
	Vector3<int16_t> gyro;
	sensor->getGyroAdc(&gyro);

	value->x = g.x + gyro.x;
	value->y = g.y + gyro.y;
	value->z = g.z + gyro.z;
}

void Calibration::getGyroAdcWithCalibrate(Vector3<int32_t> *value) {
	Vector3<int16_t> gyro;
	sensor->getGyroAdc(&gyro);

	if (count >= count_limit) count = 0;
	sum -= raw[count];
	raw[count] = gyro;
	sum += gyro;

	Vector3<int32_t> min = {raw[0].x, raw[0].y, raw[0].z};
	Vector3<int32_t> max = {raw[0].x, raw[0].y, raw[0].z};

	for (int i = 1; i < count_limit; i++) {
		max.Larger(raw[i]);
		min.Smaller(raw[i]);
	}

	int32_t dm = (max - min).Dot2();

	if (dm < gyro_threshould) {
		printf("Calibrate: [%d, %d, %d] -> ", g.x, g.y, g.z);
		g = -sum / count_limit;
		printf("[%d, %d, %d]\n", g.x, g.y, g.z);
	}

	count++;

	value->x = g.x + gyro.x;
	value->y = g.y + gyro.y;
	value->z = g.z + gyro.z;
}

void Calibration::getAccelAdc(Vector3<int32_t> *value) {
	Vector3<int16_t> accel;
	sensor->getAccelAdc(&accel);

	value->x = a.x + accel.x;
	value->y = a.y + accel.y;
	value->z = a.z + accel.z;
}

void Calibration::getData(Vector3<int32_t> *accel, Vector3<int32_t> *gyro) {
	Vector3<int16_t> ac, gy;
	sensor->getGyroAdc(&gy);
	sensor->getAccelAdc(&ac);

	accel->setWithAdd(a, ac);
	gyro->setWithAdd(g, gy);
}
