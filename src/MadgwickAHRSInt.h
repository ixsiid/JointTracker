#pragma once

#include "IAHRS.h"

class MadgwickAHRSInt : public IAHRS {
    public:
	MadgwickAHRSInt(float beta);
	virtual void update(Vector3<int16_t> gyro, Vector3<int16_t> accel, Vector3<int16_t> magnitude);
	virtual void update(Vector3<int16_t> gyro, Vector3<int16_t> accel);

	virtual void reset();

    private:
	float beta;
	int64_t time;
};
