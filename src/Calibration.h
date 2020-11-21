#pragma once

#include <stdint.h>

#include "Vector3.h"
#include "IIMU.h"

class Calibration {
    public:
	enum Mode {
		None		  = 0,
		Gyro		  = 1 << 1,
		Accelemeter = 1 << 2,
	};

	Calibration(IIMU *imu, int32_t count, uint32_t gyro_threshould = 100, uint32_t accel_threshould = 360);
	~Calibration();
	void regist(Mode mode);
	void regist(int mode);
	bool proccess();
	void getStatus(char *buffer);

	void getGyroAdc(Vector3<int32_t> *value);
	void getAccelAdc(Vector3<int32_t> *value);

	Vector3<int16_t> g, a;
	Vector3<int32_t> da;
	int32_t dm;

    private:
	IIMU *sensor;
	int32_t count_limit;
	uint32_t gyro_threshould, accel_threshould;
	Vector3<int16_t> *raw;
	int32_t count;
	Mode mode;
};

Calibration::Calibration(IIMU *imu, int count, uint32_t gyro_threshould, uint32_t accel_threshould) {
	this->sensor		   = imu;
	this->count_limit	   = count;
	this->gyro_threshould  = gyro_threshould * gyro_threshould;
	this->accel_threshould = accel_threshould * accel_threshould;

	raw = new Vector3<int16_t>[count];
	
	g.x = g.y = g.z = 0;
	a.x = a.y = a.z = 0;
	da.x = da.y = da.z = dm = 0;

	this->count = 0;
	this->mode  = Mode::None;
}

void Calibration::regist(Mode mode) {
	this->mode = mode;
}

void Calibration::regist(int mode) {
	this->mode = static_cast<Mode>(mode);
}

bool Calibration::proccess() {
	if (!mode) return true;

	if (count >= count_limit) {
		Vector3<int32_t> sum = {0, 0, 0};
		Vector3<int32_t> min = {raw[0].x, raw[0].y, raw[0].z};
		Vector3<int32_t> max = {raw[0].x, raw[0].y, raw[0].z};

		for (int i = 1; i < count_limit; i++) {
			if(raw[i].x > max.x) max.x = raw[i].x;
			if(raw[i].y > max.x) max.y = raw[i].y;
			if(raw[i].z > max.x) max.z = raw[i].z;
			if(raw[i].x < min.x) min.x = raw[i].x;
			if(raw[i].y < min.x) min.y = raw[i].y;
			if(raw[i].z < min.x) min.z = raw[i].z;

			sum += raw[i];
		}

		da = max - min;
		dm = da.Dot2();

		count = 0;
		sum /= count_limit;
		if (mode & Mode::Gyro) {
			if (dm < gyro_threshould) {
				g.x = -sum.x;
				g.y = -sum.y;
				g.z = -sum.z;

				mode = static_cast<Mode>(mode & ~Mode::Gyro);
			}
		} else if (mode & Mode::Accelemeter) {
			if (dm < accel_threshould) {
				a.x = -sum.x;
				a.y = -sum.y;
				a.z = -sum.z;

				mode = static_cast<Mode>(mode & ~Mode::Accelemeter);
			}
		}
	} else {
		if (mode & Mode::Gyro) {
			sensor->getGyroAdc(&raw[count]);
		} else if (mode & Mode::Accelemeter) {
			sensor->getAccelAdc(&raw[count]);
			raw[count].z -= 4096; // 8G Scale (range 32768)の1G
		}
		count++;
	}

	return false;
}

void Calibration::getStatus(char *buffer) {
	if (!mode) {
		sprintf(buffer, "Done. ");
	} else {
		int t = 0;
		if (mode & Mode::Gyro) {
			t = sprintf(buffer, "Gyro");
		} else if (mode & Mode::Accelemeter) {
			t = sprintf(buffer, "Accelemeter");
		} else {
			t = sprintf(buffer, "Unknown");
		}
		sprintf(buffer + t, " proccessing: [%d, %d, %d] (%d), %d", da.x, da.y, da.z, dm, count);
	}
}

Calibration::~Calibration() {
	delete[] raw;
}

void Calibration::getGyroAdc(Vector3<int32_t> *value) {
	Vector3<int16_t> gyro;
	sensor->getGyroAdc(&gyro);
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
