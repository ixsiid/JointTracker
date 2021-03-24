#pragma once

#include "Vector3.h"

class IIMU {
    public:
	virtual ~IIMU(){};

	virtual Vector3<int16_t> getAccelAdc() = 0;
	virtual Vector3<float> getAccel()	    = 0;
	virtual Vector3<int16_t> getGyroAdc()  = 0;
	virtual Vector3<float> getGyro()	    = 0;
	virtual Vector3<int16_t> getMagAdc()   = 0;
	virtual Vector3<float> getMag()	    = 0;

	virtual void getAccelAdc(Vector3<int16_t>* accel) = 0;
	virtual void getGyroAdc(Vector3<int16_t>* gyro)	= 0;
	virtual void getMagAdc(Vector3<int16_t>* mag)	= 0;

	virtual void* getI2CMaster() = 0;
};
