#pragma once

#include "IAHRS.h"

class MadgwickAHRS : public IAHRS {
    public:
	MadgwickAHRS(float beta);
	virtual void update(Vector3<float> gyro, Vector3<float> accel, Vector3<float> magnitude);
	virtual void update(Vector3<float> gyro, Vector3<float> accel);

	virtual void reset();

    private:
	float beta;
	int64_t time;
};
