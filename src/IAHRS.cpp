#include "IAHRS.h"

void IAHRS::rotate(Vector3<float> * p) {
	float px = p->x;
	float py = p->y;
	float pz = p->z;

	float ww = q.w * q.w;
	float xx = q.x * q.x;
	float yy = q.y * q.y;
	float zz = q.z * q.z;

	float wx = q.w * q.x;
	float wy = q.w * q.y;
	float wz = q.w * q.z;
	float xy = q.x * q.y;
	float xz = q.x * q.z;
	float yz = q.y * q.z;

	float a00 = ww - xx - yy + zz;
	float a01 = 2.0f * (wx - yz);
	float a02 = 2.0f * (wy + xz);
	float a10 = 2.0f * (wx + yz);
	float a11 = -ww + xx - yy + zz;
	float a12 = 2.0f * (xy - wz);
	float a20 = 2.0f * (wy - xz);
	float a21 = 2.0f * (xy + wz);
	float a22 = -ww - xx + yy + zz;

	p->x = a00 * px + a01 * py + a02 * pz;
	p->y = a10 * px + a11 * py + a12 * pz;
	p->z = a20 * px + a21 * py + a22 * pz;
}

void IAHRS::inverse_rotate(Vector3<float> * p) {
	float px = p->x;
	float py = p->y;
	float pz = p->z;

	float ww = q.w * q.w;
	float xx = q.x * q.x;
	float yy = q.y * q.y;
	float zz = q.z * q.z;

	float wx = q.w * q.x;
	float wy = q.w * q.y;
	float wz = q.w * q.z;
	float xy = q.x * q.y;
	float xz = q.x * q.z;
	float yz = q.y * q.z;

	float a00 = ww - xx - yy + zz;
	float a01 = 2.0f * (wx + yz);
	float a02 = 2.0f * (wy - xz);
	float a10 = 2.0f * (wx - yz);
	float a11 = -ww + xx - yy + zz;
	float a12 = 2.0f * (xy + wz);
	float a20 = 2.0f * (wy + xz);
	float a21 = 2.0f * (xy - wz);
	float a22 = -ww - xx + yy + zz;

	p->x = a00 * px + a01 * py + a02 * pz;
	p->y = a10 * px + a11 * py + a12 * pz;
	p->z = a20 * px + a21 * py + a22 * pz;
}
