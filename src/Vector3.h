#pragma once

#include <fastmath.h>
#include <stdint.h>
#include <stdio.h>

template <typename T>
struct Vector3 {
	T x, y, z;

	inline static Vector3 xyz(T x, T y, T z) { return {x, y, z}; }

	Vector3<T> add(Vector3<T> value) {
		x += value.x;
		y += value.y;
		z += value.z;
		return this;
	}

	/// x^2 + y^2 + z^2
	T Dot2() {
		return x * x + y * y + z * z;
	}

	template <typename S, typename U>
	void setWithAdd(Vector3<S> a, Vector3<U> b) {
		x = a.x + b.x;
		y = a.y + b.y;
		z = a.z + b.z;
	}

	template <typename S>
	Vector3<S> operator*(S value) {
		Vector3<S> r;
		r.x = (S)(x)*value;
		r.y = (S)(y)*value;
		r.z = (S)(z)*value;
		return r;
	}

	Vector3<T> operator/(T value) {
		Vector3<T> r;
		r.x = x / value;
		r.y = y / value;
		r.z = z / value;
		return r;
	}

	Vector3<T> operator-(Vector3<T> value) {
		Vector3<T> r;
		r.x = x - value.x;
		r.y = y - value.y;
		r.z = z - value.z;
		return r;
	}

	Vector3<T> operator-() {
		Vector3<T> r;
		r.x = -x;
		r.y = -y;
		r.z = -z;
		return r;
	}

	template <typename S>
	Vector3<T> operator+(Vector3<S> value) {
		Vector3<T> r;
		r.x = x + value.x;
		r.y = y + value.y;
		r.z = z + value.z;
		return r;
	}

	void operator+=(Vector3<T> value) {
		this->x += value.x;
		this->y += value.y;
		this->z += value.z;
	}

	void operator-=(Vector3<T> value) {
		this->x -= value.x;
		this->y -= value.y;
		this->z -= value.z;
	}

	template <typename S>
	void operator+=(Vector3<S> value) {
		this->x += value.x;
		this->y += value.y;
		this->z += value.z;
	}

	template <typename S>
	void operator-=(Vector3<S> value) {
		this->x -= value.x;
		this->y -= value.y;
		this->z -= value.z;
	}

	void operator*=(T value) {
		this->x *= value;
		this->y *= value;
		this->z *= value;
	}

	void operator/=(T value) {
		this->x /= value;
		this->y /= value;
		this->z /= value;
	}

	inline T Dot(Vector3<T> a) {
		return x * a.x + y * a.y + z * a.z;
	}

	template <typename S>
	inline void Larger(Vector3<S> a) {
		if (this->x < a.x) this->x = a.x;
		if (this->y < a.y) this->y = a.y;
		if (this->z < a.z) this->z = a.z;
	}

	template <typename S>
	inline void Smaller(Vector3<S> a) {
		if (this->x > a.x) this->x = a.x;
		if (this->y > a.y) this->y = a.y;
		if (this->z > a.z) this->z = a.z;
	}
};

struct Matrix3x3 {
	Vector3<float> columns[3];

	Matrix3x3() {
		columns[0].x = columns[1].y = columns[2].z = 1.0f;
		columns[0].y = columns[0].z = columns[1].x = columns[1].z = columns[2].x = columns[2].y = 0.0f;
	}

	Matrix3x3(Vector3<float> column0, Vector3<float> column1, Vector3<float> column2) {
		columns[0] = column0;
		columns[1] = column1;
		columns[2] = column2;
	}

	Matrix3x3(float m00, float m01, float m02,
			float m10, float m11, float m12,
			float m20, float m21, float m22) {
		columns[0].x = m00;
		columns[0].y = m01;
		columns[0].z = m02;

		columns[1].x = m10;
		columns[1].y = m11;
		columns[1].z = m12;

		columns[2].x = m20;
		columns[2].y = m21;
		columns[2].z = m22;
	}

	Vector3<float> rotate(Vector3<float> point) {
		Vector3<float> r;
		r.x = columns[0].x * point.x + columns[0].y * point.y + columns[0].z * point.z;
		r.y = columns[1].x * point.x + columns[1].y * point.y + columns[1].z * point.z;
		r.z = columns[2].x * point.x + columns[2].y * point.y + columns[2].z * point.z;
		return r;
	}

	/*
	Vector3<float> rotate(Vector3<float> point) {
		Vector3<float> r;
		r.x = columns[0].x * point.x + columns[1].x * point.y + columns[2].x * point.z;
		r.y = columns[0].y * point.x + columns[1].y * point.y + columns[2].y * point.z;
		r.z = columns[0].z * point.x + columns[1].z * point.y + columns[2].z * point.z;
	}
	*/
};

struct Quaternion {
	float x;
	float y;
	float z;
	float w;

	inline static Quaternion identify() { return {0.0f, 0.0f, 0.0f, 1.0f}; }

	inline static Quaternion xyzw(float x, float y, float z, float w) { return {x, y, z, w}; }

	void normalize() {
		float n = 1.0f / sqrtf(x * x + y * y + z * z + w * w);
		x *= n;
		y *= n;
		z *= n;
		w *= n;
	}

	void normalize(bool positive_w) {
		float n = 1.0f / sqrtf(x * x + y * y + z * z + w * w);
		if (positive_w && w < 0.0f) n = -n;
		x *= n;
		y *= n;
		z *= n;
		w *= n;
	}

	Quaternion operator-(Quaternion value) {
		return Quaternion::xyzw(x - value.x, y - value.y, z - value.z, w - value.w);
	}

	Quaternion operator*(float value) {
		return Quaternion::xyzw(x * value, y * value, z * value, w * value);
	}

	void operator+=(Quaternion value) {
		x += value.x;
		y += value.y;
		z += value.z;
		w += value.w;
	}

	void operator *=(float value) {
		x *= value;
		y *= value;
		z *= value;
		w *= value;
	}

	void operator -=(Quaternion value) {
		x -= value.x;
		y -= value.y;
		z -= value.z;
		w -= value.w;
	}

	Quaternion operator*(Quaternion p) {
		// https://www.mss.co.jp/technology/report/pdf/18-07.pdf
		// 資料は [w, x, y, z] の順番なので注意
		return {x * p.w + w * p.x - z * p.y + y * p.z,
			   y * p.w + z * p.x + w * p.y - x * p.z,
			   z * p.w - y * p.x + x * p.y + w * p.z,
			   w * p.w - x * p.x - y * p.y - z * p.z};
	}

	Quaternion inverse() {
		return {-x, -y, -z, w};
	}

	Vector3<float> operator*(Vector3<float> p) {
		Quaternion qq = this->Dot(*this);

		float wx = w * x;
		float wy = w * y;
		float wz = w * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;

		return {(qq.w + qq.x - qq.y - qq.z) * p.x + 2.0f * (xy - wz) * p.y + 2.0f * (xz + wy) * p.z,
			   2.0f * (xy + wz) * p.x + (qq.w - qq.x + qq.y - qq.z) * p.y + 2.0f * (yz - wx) * p.z,
			   2.0f * (xz - wy) * p.x + 2.0f * (yz + wx) * p.y + (qq.w - qq.x - qq.y + qq.z) * p.z};
	}

	Vector3<float> rotate(Vector3<float> p) {
		Quaternion qq = this->Dot(*this);

		float wx = w * x;
		float wy = w * y;
		float wz = w * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;

		return {(qq.w + qq.x - qq.y - qq.z) * p.x + 2.0f * (xy + wz) * p.y + 2.0f * (xz - wy) * p.z,
			   2.0f * (xy - wz) * p.x + (qq.w - qq.x + qq.y - qq.z) * p.y + 2.0f * (yz + wx) * p.z,
			   2.0f * (xz + wy) * p.x + 2.0f * (yz - wx) * p.y + (qq.w - qq.x - qq.y + qq.z) * p.z};
	}

	Quaternion Dot(Quaternion q) {
		return {x * q.x,
			   y * q.y,
			   z * q.z,
			   w * q.w};
	}
};
