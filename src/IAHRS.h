#pragma once

#include "Vector3.h"

class IAHRS {
    public:
	virtual ~IAHRS() {}

	Quaternion q;
	virtual void update(Vector3<float> gyro, Vector3<float> accel, Vector3<float> magnitude) = 0;
	virtual void update(Vector3<float> gyro, Vector3<float> accel) = 0;

	virtual void reset();

	void rotate(Vector3<float> * p);
	void inverse_rotate(Vector3<float> * p);
};

inline void IAHRS::reset() {
	q = Quaternion::identify();
}
