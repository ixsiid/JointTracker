var s = {
x: -_2qz * (2.0 * qyqw - _2qxqz - a.x) + _2qy * (2.0 * qxqy + _2qxqw - a.y)
- _2bz * q.z * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
+ (-_2bx * q.w + _2bz * q.y)
    * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
+ _2bx * q.z * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
y: _2qw * (2.0 * qyqw - _2qxqz - a.x) + _2qx * (2.0 * qxqy + _2qxqw - a.y)
- 4.0 * q.y * (1.0 - 2.0 * qyqy - 2.0 * qzqz - a.z)
+ _2bz * q.w * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
+ (_2bx * q.z + _2bz * q.x)
    * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
+ (_2bx * q.w - _4bz * q.y)
    * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
z: -_2qx * (2.0 * qyqw - _2qxqz - a.x) + _2qw * (2.0 * qxqy + _2qxqw - a.y)
- 4.0 * q.z * (1.0 - 2.0 * qyqy - 2.0 * qzqz - a.z)
+ (-_4bx * q.z - _2bz * q.x)
    * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
+ (_2bx * q.y + _2bz * q.w)
    * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
+ (_2bx * q.x - _4bz * q.z)
    * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
w: _2qy * (2.0 * qyqw - _2qxqz - a.x) + _2qz * (2.0 * qxqy + _2qxqw - a.y)
+ (-_4bx * q.w + _2bz * q.y)
    * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
+ (-_2bx * q.x + _2bz * q.z)
    * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
+ _2bx * q.y * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
}


var d = {
	x: _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y,
	y: _4q1 * q3q3 - _2q3 * a.x + 4.0 * q0q0 * q.y - _2q0 * a.y - _4q1 + _8q1 * q1q1
	    + _8q1 * q2q2 + _4q1 * a.z,
	z: 4.0 * q0q0 * q.z + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1
	    + _8q2 * q2q2 + _4q2 * a.z,
	w: 4.0 * q1q1 * q.w - _2q1 * a.x + 4.0 * q2q2 * q.w - _2q2 * a.y,
 };