#include "MadgwickAHRS.h"

#include <esp_timer.h>
#include <fastmath.h>

MadgwickAHRS::MadgwickAHRS(float beta) {
	this->beta = beta;

	reset();
	time = esp_timer_get_time();
}

void MadgwickAHRS::update(Vector3<float> g, Vector3<float> a, Vector3<float> m) {
	float anorm = a.Dot2();
	float mnorm = m.Dot2();
	if (anorm < 0.002f || mnorm < 0.0002f) return;
	float norm;

	norm = 1.0f / sqrt(anorm);
	a *= norm;

	norm = 1.0f / sqrt(mnorm);
	m *= norm;

	int64_t t = esp_timer_get_time();
	float dt	= (t - time) / 1000000.0f;
	time		= t;

	// 再利用変数の初期化
	float ww	= q.w * q.w;
	float xx	= q.x * q.x;
	float yy	= q.y * q.y;
	float zz	= q.z * q.z;
	float _2w = 2.0f * q.w;
	float _2x = 2.0f * q.x;
	float _2y = 2.0f * q.y;
	float _2z = 2.0f * q.z;

	float wx	 = q.w * q.x;
	float wy	 = q.w * q.y;
	float wz	 = q.w * q.z;
	float xy	 = q.x * q.y;
	float xz	 = q.x * q.z;
	float yz	 = q.y * q.z;
	float _2wy = 2.0f * q.w * q.y;
	float _2yz = 2.0f * q.y * q.z;

	// Reference direction of Earth's magnetic field
	float _2w_mx = 2.0f * q.w * m.x;
	float _2w_my = 2.0f * q.w * m.y;
	float _2w_mz = 2.0f * q.w * m.z;
	float _2x_mx = 2.0f * q.x * m.x;

	float hx = m.x * ww - _2w_my * q.z + _2w_mz * q.y + m.x * xx + _2x * m.y * q.y + _2x * m.z * q.z - m.x * yy - m.x * zz;
	float hy = _2w_mx * q.z + m.y * ww - _2w_mz * q.x + _2x_mx * q.y - m.y * xx + m.y * yy + _2y * m.z * q.z - m.y * zz;

	float _2bx = sqrt(hx * hx + hy * hy);
	float _2bz = -_2w_mx * q.y + _2w_my * q.x + m.z * ww + _2x_mx * q.z - m.z * xx + _2y * m.y * q.z - m.z * yy + m.z * zz;
	float _4bx = 2.0f * _2bx;
	float _4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	Quaternion s = {+_2z * (2.0f * xz - _2wy - a.x) + _2w * (2.0f * wx + _2yz - a.y) - 4.0f * q.x * (1 - 2.0f * xx - 2.0f * yy - a.z) + _2bz * q.z * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (_2bx * q.y + _2bz * q.w) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + (_2bx * q.z - _4bz * q.x) * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z),
				 -_2w * (2.0f * xz - _2wy - a.x) + _2z * (2.0f * wx + _2yz - a.y) - 4.0f * q.y * (1 - 2.0f * xx - 2.0f * yy - a.z) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (_2bx * q.x + _2bz * q.z) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + (_2bx * q.w - _4bz * q.y) * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z),
				 +_2x * (2.0f * xz - _2wy - a.x) + _2y * (2.0f * wx + _2yz - a.y) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + _2bx * q.x * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z),
				 -_2y * (2.0f * xz - _2wy - a.x) + _2x * (2.0f * wx + _2yz - a.y) - _2bz * q.y * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + _2bx * q.y * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z)};
	s.normalize();

	// Compute rate of change of quaternion
	Quaternion qdot = {+q.w * g.x + q.y * g.z - q.z * g.y,
				    +q.w * g.y - q.x * g.z + q.z * g.x,
				    +q.w * g.z + q.x * g.y - q.y * g.x,
				    -q.x * g.x - q.y * g.y - q.z * g.z};

	qdot = qdot * 0.5f - s * beta;

	// Integrate to yield quaternion
	q += qdot * dt;
	q.normalize(true);
}

void MadgwickAHRS::update(Vector3<float> g, Vector3<float> a) {
	// Normalise accelerometer measurement
	float norm = sqrtf(a.Dot2());
	if (norm <= 0.002f) return;  // handle NaN
	norm = 1.0f / norm;		    // use reciprocal for division
	a *= norm;

	int64_t t = esp_timer_get_time();
	float dt	= (t - time) / 1000000.0f;
	time		= t;

	// Auxiliary variables to avoid repeated arithmetic
	Quaternion qq = q.Dot(q);

	float qqxy = qq.x + qq.y;
	float qqwz = qq.w + qq.z;

	// Gradient decent algorithm corrective step
	Quaternion s = {
	    q.x * qqwz - 0.5f * (q.z * a.x + q.w * a.y) + q.x * (-1.0f + 2.0f * qq.x + 2.0f * qq.y + a.z),
	    q.y * qqwz - 0.5f * (q.z * a.y - q.w * a.x) + q.y * (-1.0f + 2.0f * qq.x + 2.0f * qq.y + a.z),
	    q.z * qqxy - 0.5f * (q.x * a.x + q.y * a.y),
	    q.w * qqxy - 0.5f * (q.x * a.y - q.y * a.x)};
	s.normalize();

	// Compute rate of change of quaternion
	Quaternion qdot = {+q.w * g.x + q.y * g.z - q.z * g.y,
				    +q.w * g.y - q.x * g.z + q.z * g.x,
				    +q.w * g.z + q.x * g.y - q.y * g.x,
				    -q.x * g.x - q.y * g.y - q.z * g.z};

	qdot = qdot * 0.5f - s * beta;

	// Integrate to yield quaternion
	q += qdot * dt;
	q.normalize(true);
}

void MadgwickAHRS::reset() {
	IAHRS::reset();
	time = esp_timer_get_time();
}
