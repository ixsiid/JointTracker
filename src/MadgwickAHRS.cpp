#include "MadgwickAHRS.h"

#include <esp_timer.h>
#include <fastmath.h>

MadgwickAHRS::MadgwickAHRS(float beta) {
	this->beta = beta;

	reset();
	time = esp_timer_get_time();
}


void MadgwickAHRS::update(Vector3<float> g, Vector3<float> a, Vector3<float> m) {
	// https://github.com/jsjolund/f4/blob/master/src/madgwick_ahrs.rs
	// LSM9DS1用、座標系が違うと式展開が変わる
	int64_t t = esp_timer_get_time();
	float dt	= (t - time) / 1000000.0f;
	time		= t;

	// ジャイロデータの積算分
	Quaternion qdot = Quaternion::xyzw(-q.y * g.x - q.z * g.y - q.w * g.z,
				                    +q.x * g.x + q.z * g.z - q.w * g.y,
				                    +q.x * g.y - q.y * g.z + q.w * g.x,
				                    +q.x * g.z + q.y * g.y - q.z * g.x) * 0.5f;

	// 加速度データを処理するか否か
	float anorm = sqrtf(a.Dot2());
	float mnorm = sqrtf(m.Dot2());
	if (anorm > 0.9f && anorm < 1.1f) {
		// 加速度データの補正
		a *= 1.0f / anorm;

		// Auxiliary variables to avoid repeated arithmetic
		Quaternion qq = q.Dot(q);

		float _12qqyz = - 1.0f + 2.0f * qq.y + 2.0f * qq.z;

		// 加速度の補正項
		Quaternion s = {
			+ 0.5f * (q.z * a.x - q.y * a.y),
			- 0.5f * (q.w * a.x + q.x * a.y) + q.y * a.z + q.y * _12qqyz,
			+ 0.5f * (q.x * a.x - q.w * a.y) + q.z * a.z + q.z * _12qqyz,
			- 0.5f * (q.y * a.x + q.z * a.y),
		};
		
		// 加速度のみの場合
		float qq_yz = qq.y + qq.z;
		float qq_xw = qq.x + qq.w;
		s += {
			q.x * qq_yz,
			q.y * qq_xw,
			q.z * qq_xw,
			q.w * qq_yz,
		};

		float cos_t = a.Dot(m) / (anorm * mnorm);
		cos_t *= cos_t;
		/// 磁気センサーが地磁気のレンジに収まっていない or 加速度と地磁気の成す角が90°ではない
		// if (mnorm < 0.25f || mnorm > 0.7f || cos_t > 0.01f) {
		if (true) {
			m *= 1.0f / mnorm;
			
			// Reference direction of Earth's magnetic field
			float hx = m.x * (qq.x + qq.y - qq.z * qq.w) + 2.0f * m.y * (q.y * q.z - q.x * q.w) + 2.0f * m.z * (q.x * q.z + q.y * q.w);
			float hy = 2.0f * m.x * (q.x * q.w + q.y * q.z) + m.y * (qq.x - qq.y + qq.z - qq.w) + 2.0f * m.z * (q.z * q.w - q.x * q.y);
			float bx = sqrtf(hx * hx + hy * hy) * 0.5f;
			float bz = m.x * (q.y * q.w - q.x * q.z) + m.y * (q.x * q.y + q.z* q.w) + 0.5f * m.z * (qq.x - qq.y - qq.z + qq.w);

			s += {
				-              bz * q.z  * (bx * (0.5f - qq.z - qq.w)    + bz * (q.y * q.w - q.x * q.z) - 0.5f * m.x)
				+ (-bx * q.w + bz * q.y) * (bx * (q.y * q.z - q.x * q.w) + bz * (q.x * q.y + q.z * q.w) - 0.5f * m.y)
				+   bx * q.z             * (bx * (q.x * q.z + q.y * q.w) + bz * (0.5f - qq.y - qq.z)    - 0.5f * m.z),

				+                    bz * q.w  * (bx * (0.5f - qq.z - qq.w)    + bz * (q.y * q.w - q.x * q.z) - 0.5f * m.x)
				+ (bx * q.z +        bz * q.x) * (bx * (q.y * q.z - q.x * q.w) + bz * (q.x * q.y + q.z * q.w) - 0.5f * m.y)
				+ (bx * q.w - 2.0f * bz * q.y) * (bx * (q.x * q.z + q.y * q.w) + bz * (0.5f - qq.y - qq.z)    - 0.5f * m.z),

				+ (-2.0f * bx * q.z        - bz * q.x) * (bx * (0.5f - qq.z - qq.w)    + bz * (q.y * q.w - q.x * q.z) - 0.5f * m.x)
				+ (        bx * q.y        + bz * q.w) * (bx * (q.y * q.z - q.x * q.w) + bz * (q.x * q.y + q.z * q.w) - 0.5f * m.y)
				+ (        bx * q.x - 2.0f * bz * q.z) * (bx * (q.x * q.z + q.y * q.w) + bz * (0.5f - qq.y - qq.z)    - 0.5f * m.z),

				+ (-2.0f * bx * q.w + bz * q.y) * (bx * (0.5f - qq.z - qq.w)    + bz * (q.y * q.w - q.x * q.z) - 0.5f * m.x)
				+ (       -bx * q.x + bz * q.z) * (bx * (q.y * q.z - q.x * q.w) + bz * (q.x * q.y + q.z * q.w) - 0.5f * m.y)
				+          bx * q.y             * (bx * (q.x * q.z + q.y * q.w) + bz * (0.5f - qq.y - qq.z)    - 0.5f * m.z),
			};
		}

		s.normalize();
		qdot -= s * beta;
	}

	// Integrate to yield quaternion
	q += qdot * dt;
	q.normalize(true);
}

/*
void MadgwickAHRS::update(Vector3<float> g, Vector3<float> a, Vector3<float> m) {
	int64_t t = esp_timer_get_time();
	float dt	= (t - time) / 1000000.0f;
	time		= t;

	// ジャイロデータの積算分
	Quaternion qdot = Quaternion::xyzw(+q.w * g.x + q.y * g.z - q.z * g.y,
				                    +q.w * g.y - q.x * g.z + q.z * g.x,
				                    +q.w * g.z + q.x * g.y - q.y * g.x,
				                    -q.x * g.x - q.y * g.y - q.z * g.z) * 0.5f;

	// 加速度データを処理するか否か
	float anorm = sqrtf(a.Dot2());
	//if (anorm > 0.9f && anorm < 1.1f) {
	if (true) {
		// 加速度データの補正
		a *= 1.0f / anorm;

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

		// 磁気データを処理するか否か
		float mnorm = sqrtf(m.Dot2());
		// if (mnorm > 0.2f && mnorm < 0.7f) {
		if (false) {
			m *= 1.0f / mnorm;

			// 再利用変数の初期化
			float _2x = 2.0f * q.x;
			float _2y = 2.0f * q.y;

			float wx	 = q.w * q.x;
			float wy	 = q.w * q.y;
			float wz	 = q.w * q.z;
			float xy	 = q.x * q.y;
			float xz	 = q.x * q.z;
			float yz	 = q.y * q.z;

			// Reference direction of Earth's magnetic field
			float _2w_mx = 2.0f * q.w * m.x;
			float _2w_my = 2.0f * q.w * m.y;
			float _2w_mz = 2.0f * q.w * m.z;
			float _2x_mx = 2.0f * q.x * m.x;

			float hx = m.x * qq.w - _2w_my * q.z + _2w_mz * q.y + m.x * qq.x + _2x * m.y * q.y + _2x * m.z * q.z - m.x * qq.y - m.x * qq.z;
			float hy = _2w_mx * q.z + m.y * qq.w - _2w_mz * q.x + _2x_mx * q.y - m.y * qq.x + m.y * qq.y + _2y * m.z * q.z - m.y * qq.z;

			float _2bx = sqrt(hx * hx + hy * hy);
			float _2bz = -_2w_mx * q.y + _2w_my * q.x + m.z * qq.w + _2x_mx * q.z - m.z * qq.x + _2y * m.y * q.z - m.z * qq.y + m.z * qq.z;
			float _4bx = 2.0f * _2bx;
			float _4bz = 2.0f * _2bz;

			// if magnetic field
			// Gradient decent algorithm corrective step
			s *= 4.0f;
			s += {+_2bz * q.z * (_2bx * (0.5f - qq.y - qq.z) + _2bz * (xz - wy) - m.x) + (_2bx * q.y + _2bz * q.w) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + (_2bx * q.z - _4bz * q.x) * (_2bx * (wy + xz) + _2bz * (0.5f - qq.x - qq.y) - m.z),
				 +(-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - qq.y - qq.z) + _2bz * (xz - wy) - m.x) + (_2bx * q.x + _2bz * q.z) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + (_2bx * q.w - _4bz * q.y) * (_2bx * (wy + xz) + _2bz * (0.5f - qq.x - qq.y) - m.z),
				 +(-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - qq.y - qq.z) + _2bz * (xz - wy) - m.x) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + _2bx * q.x * (_2bx * (wy + xz) + _2bz * (0.5f - qq.x - qq.y) - m.z),
				 -_2bz * q.y * (_2bx * (0.5f - qq.y - qq.z) + _2bz * (xz - wy) - m.x) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (xy - wz) + _2bz * (wx + yz) - m.y) + _2bx * q.y * (_2bx * (wy + xz) + _2bz * (0.5f - qq.x - qq.y) - m.z)};
		}

		s.normalize();
		qdot -= s * beta;
	}

	// Integrate to yield quaternion
	q += qdot * dt;
	q.normalize(true);
}
*/

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
