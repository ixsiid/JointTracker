#include "MadgwickAHRSInt.h"

#include <esp_timer.h>
#include <fastmath.h>

MadgwickAHRSInt::MadgwickAHRSInt(float beta) {
	this->beta = beta;

	reset();
	time = esp_timer_get_time();
}

void MadgwickAHRSInt::update(Vector3<int16_t> g, Vector3<int16_t> a, Vector3<int16_t> m) {
	float anorm = a.Dot2();
	float mnorm = m.Dot2();
	if (anorm < 0.002f || mnorm < 0.0002f) return;
	float norm;

	norm = 1.0f / sqrt(anorm);
	a *= norm;

	norm = 1.0f / sqrt(mnorm);
	m *= norm;

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
	float s1 = -_2y * (2.0f * xz - _2wy - a.x) + _2x * (2.0f * wx + _2yz - a.y) - _2bz * q.y * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + _2bx * q.y * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z);
	float s2 = +_2z * (2.0f * xz - _2wy - a.x) + _2w * (2.0f * wx + _2yz - a.y) - 4.0f * q.x * (1 - 2.0f * xx - 2.0f * yy - a.z) + _2bz * q.z * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (_2bx * q.y + _2bz * q.w) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + (_2bx * q.z - _4bz * q.x) * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z);
	float s3 = -_2w * (2.0f * xz - _2wy - a.x) + _2z * (2.0f * wx + _2yz - a.y) - 4.0f * q.y * (1 - 2.0f * xx - 2.0f * yy - a.z) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (_2bx * q.x + _2bz * q.z) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + (_2bx * q.w - _4bz * q.y) * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z);
	float s4 = +_2x * (2.0f * xz - _2wy - a.x) + _2y * (2.0f * wx + _2yz - a.y) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - yy - zz) + _2bz * (xz - wy) - m.x) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + _2bx * q.x * (_2bx * (wy + xz) + _2bz * (0.5f - xx - yy) - m.z);

	norm = 1.0f / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);	// normalise step m
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	float qDotw = 0.5f * (-q.x * g.x - q.y * g.y - q.z * g.z) - beta * s1;
	float qDotx = 0.5f * (+q.w * g.x + q.y * g.z - q.z * g.y) - beta * s2;
	float qDoty = 0.5f * (+q.w * g.y - q.x * g.z + q.z * g.x) - beta * s3;
	float qDotz = 0.5f * (+q.w * g.z + q.x * g.y - q.y * g.x) - beta * s4;

	int64_t t		    = esp_timer_get_time();
	float samplePeriod = (t - time) / 1000000.0f;
	time			    = t;
	// Integrate to yield quaternion
	q.w += qDotw * samplePeriod;
	q.x += qDotx * samplePeriod;
	q.y += qDoty * samplePeriod;
	q.z += qDotz * samplePeriod;
	norm = 1.0f / sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

	q.w = q.w * norm;
	q.x = q.x * norm;
	q.y = q.y * norm;
	q.z = q.z * norm;
}

inline int32_t GetMsbPos(int32_t uVal) 
{ 
	// https://jp.quora.com/%E6%9C%80%E4%B8%8A%E4%BD%8D%E3%83%93%E3%83%83%E3%83%88-MSB-%E3%81%AE%E4%BD%8D%E7%BD%AE%E3%82%92%E6%A4%9C%E5%87%BA%E3%81%99%E3%82%8B%E5%8A%B9%E6%9E%9C%E7%9A%84%E3%81%AA%E6%96%B9%E6%B3%95%E3%81%AF%E3%81%82%E3%82%8A
	/* Propagates the MSB to all the lower bits. */ 
	uVal |= (uVal >> 1); 
	uVal |= (uVal >> 2); 
	uVal |= (uVal >> 4); 
	uVal |= (uVal >> 8); 
	uVal |= (uVal >>16); 
 
	/* Counts 1-bits. */ 
	uVal = (uVal & 0x55555555) + ((uVal >> 1) & 0x55555555); /*uVal<=2*/ 
	uVal = (uVal & 0x33333333) + ((uVal >> 2) & 0x33333333); /*uVal<=4*/ 
	uVal = (uVal & 0x0F0F0F0F) + ((uVal >> 4) & 0x0F0F0F0F); 
	uVal = (uVal             ) + ((uVal >> 8)); 
	uVal = (uVal             ) + ((uVal >>16)); 
 
	return (uVal & 0x1F) - 1; 
}

inline int32_t sqrtint(int32_t value) {
	// https://itchyny.hatenablog.com/entry/20101222/1293028538
	int32_t msb = GetMsbPos(value);

	int32_t a = 0, c = 0, y = 0, x = value;
	for (int i = msb + (msb & 1); i; i-= 2) {
		c = (y << 1 | 1) <= x >> i;
		a = a << 1 | c;
		y = y << 1 | c;
		x -= c * y << i;
		y += c;
	}
	return a;
}

void MadgwickAHRSInt::update(Vector3<int16_t> g, Vector3<int16_t> a) {
	// 内部はint32で処理する、入出力はint16で処理される
	int32_t t;
	int32_t norm = 0;
	t = a.x;
	norm += t * t;
	t = a.y;
	norm += t * t;
	t = a.z;
	norm += t * t;

/*
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
	// max: 0x7fff * 0x7fff = 3fff0001 ( *2 / 10000 + 1)
	// 3fff * 3fff = fff8001

	float qqxy = qq.x + qq.y;
	float qqwz = qq.w + qq.z;
	// 1fff0002

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
	*/
}

void MadgwickAHRSInt::reset() {
	IAHRS::reset();
	time = esp_timer_get_time();
}
