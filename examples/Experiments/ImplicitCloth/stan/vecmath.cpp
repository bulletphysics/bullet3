//
//
//  Typical 3d vector math code.
//  By S Melax 1998-2008
//
//
//

#include "vecmath.h"
#include <memory.h>  // for memcpy
#include <float.h>

float squared(float a) { return a * a; }
float clamp(float a, const float minval, const float maxval) { return Min(maxval, Max(minval, a)); }
int clamp(int a, const int minval, const int maxval) { return Min(maxval, Max(minval, a)); }

float Round(float a, float precision)
{
	return floorf(0.5f + a / precision) * precision;
}

float Interpolate(const float &f0, const float &f1, float alpha)
{
	return f0 * (1 - alpha) + f1 * alpha;
}

int argmin(const float a[], int n)
{
	int r = 0;
	for (int i = 1; i < n; i++)
	{
		if (a[i] < a[r])
		{
			r = i;
		}
	}
	return r;
}

int argmax(const float a[], int n)
{
	int r = 0;
	for (int i = 1; i < n; i++)
	{
		if (a[i] > a[r])
		{
			r = i;
		}
	}
	return r;
}

//------------ float3 (3D) --------------

float3 vabs(const float3 &v)
{
	return float3(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}

float3 safenormalize(const float3 &v)
{
	if (magnitude(v) <= 0.0f)
	{
		return float3(1, 0, 0);
	}
	return normalize(v);
}

float3 Round(const float3 &a, float precision)
{
	return float3(Round(a.x, precision), Round(a.y, precision), Round(a.z, precision));
}

float3 Interpolate(const float3 &v0, const float3 &v1, float alpha)
{
	return v0 * (1 - alpha) + v1 * alpha;
}

float3 Orth(const float3 &v)
{
	float3 absv = vabs(v);
	float3 u(1, 1, 1);
	u[argmax(&absv[0], 3)] = 0.0f;
	return normalize(cross(u, v));
}

void BoxLimits(const float3 *verts, int verts_count, float3 &bmin, float3 &bmax)
{
	bmin = float3(FLT_MAX, FLT_MAX, FLT_MAX);
	bmax = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (int i = 0; i < verts_count; i++)
	{
		bmin = VectorMin(bmin, verts[i]);
		bmax = VectorMax(bmax, verts[i]);
	}
}
void BoxLimits(const float4 *verts, int verts_count, float3 &bmin, float3 &bmax)
{
	bmin = float3(FLT_MAX, FLT_MAX, FLT_MAX);
	bmax = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (int i = 0; i < verts_count; i++)
	{
		bmin = VectorMin(bmin, verts[i].xyz());
		bmax = VectorMax(bmax, verts[i].xyz());
	}
}
int overlap(const float3 &bmina, const float3 &bmaxa, const float3 &bminb, const float3 &bmaxb)
{
	for (int j = 0; j < 3; j++)
	{
		if (bmina[j] > bmaxb[j]) return 0;
		if (bminb[j] > bmaxa[j]) return 0;
	}
	return 1;
}

// the statement v1*v2 is ambiguous since there are 3 types
// of vector multiplication
//  - componantwise (for example combining colors)
//  - dot product
//  - cross product
// Therefore we never declare/implement this function.
// So we will never see:  float3 operator*(float3 a,float3 b)

//------------ float3x3 ---------------
float Determinant(const float3x3 &m)
{
	return m.x.x * m.y.y * m.z.z + m.y.x * m.z.y * m.x.z + m.z.x * m.x.y * m.y.z - m.x.x * m.z.y * m.y.z - m.y.x * m.x.y * m.z.z - m.z.x * m.y.y * m.x.z;
}

float3x3 Inverse(const float3x3 &a)
{
	float3x3 b;
	float d = Determinant(a);
	assert(d != 0);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int i1 = (i + 1) % 3;
			int i2 = (i + 2) % 3;
			int j1 = (j + 1) % 3;
			int j2 = (j + 2) % 3;
			// reverse indexs i&j to take transpose
			b[j][i] = (a[i1][j1] * a[i2][j2] - a[i1][j2] * a[i2][j1]) / d;
		}
	}
	// Matrix check=a*b; // Matrix 'check' should be the identity (or close to it)
	return b;
}

float3x3 Transpose(const float3x3 &m)
{
	return float3x3(float3(m.x.x, m.y.x, m.z.x),
					float3(m.x.y, m.y.y, m.z.y),
					float3(m.x.z, m.y.z, m.z.z));
}

float3 operator*(const float3 &v, const float3x3 &m)
{
	return float3((m.x.x * v.x + m.y.x * v.y + m.z.x * v.z),
				  (m.x.y * v.x + m.y.y * v.y + m.z.y * v.z),
				  (m.x.z * v.x + m.y.z * v.y + m.z.z * v.z));
}
float3 operator*(const float3x3 &m, const float3 &v)
{
	return float3(dot(m.x, v), dot(m.y, v), dot(m.z, v));
}

float3x3 operator*(const float3x3 &a, const float3x3 &b)
{
	return float3x3(a.x * b, a.y * b, a.z * b);
}

float3x3 operator*(const float3x3 &a, const float &s)
{
	return float3x3(a.x * s, a.y * s, a.z * s);
}
float3x3 operator/(const float3x3 &a, const float &s)
{
	float t = 1 / s;
	return float3x3(a.x * t, a.y * t, a.z * t);
}
float3x3 operator+(const float3x3 &a, const float3x3 &b)
{
	return float3x3(a.x + b.x, a.y + b.y, a.z + b.z);
}
float3x3 operator-(const float3x3 &a, const float3x3 &b)
{
	return float3x3(a.x - b.x, a.y - b.y, a.z - b.z);
}
float3x3 &operator+=(float3x3 &a, const float3x3 &b)
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}
float3x3 &operator-=(float3x3 &a, const float3x3 &b)
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return a;
}
float3x3 &operator*=(float3x3 &a, const float &s)
{
	a.x *= s;
	a.y *= s;
	a.z *= s;
	return a;
}

float3x3 outerprod(const float3 &a, const float3 &b)
{
	return float3x3(a.x * b, a.y * b, a.z * b);  // a is a column vector b is a row vector
}

//--------------- 4D ----------------

float4 operator*(const float4 &v, const float4x4 &m)
{
	return v.x * m.x + v.y * m.y + v.z * m.z + v.w * m.w;  // yes this actually works
}

//  Dont implement m*v for now, since that might confuse us
//  All our transforms are based on multiplying the "row" vector on the left
//float4   operator*(const float4x4& m , const float4&   v )
//{
//	return float4(dot(v,m.x),dot(v,m.y),dot(v,m.z),dot(v,m.w));
//}

float4x4 operator*(const float4x4 &a, const float4x4 &b)
{
	return float4x4(a.x * b, a.y * b, a.z * b, a.w * b);
}

float4x4 MatrixTranspose(const float4x4 &m)
{
	return float4x4(
		m.x.x, m.y.x, m.z.x, m.w.x,
		m.x.y, m.y.y, m.z.y, m.w.y,
		m.x.z, m.y.z, m.z.z, m.w.z,
		m.x.w, m.y.w, m.z.w, m.w.w);
}

float4x4 MatrixRigidInverse(const float4x4 &m)
{
	float4x4 trans_inverse = MatrixTranslation(-m.w.xyz());
	float4x4 rot = m;
	rot.w = float4(0, 0, 0, 1);
	return trans_inverse * MatrixTranspose(rot);
}

float4x4 MatrixPerspectiveFov(float fovy, float aspect, float zn, float zf)
{
	float h = 1.0f / tanf(fovy / 2.0f);  // view space height
	float w = h / aspect;                // view space width
	return float4x4(
		w, 0, 0, 0,
		0, h, 0, 0,
		0, 0, zf / (zn - zf), -1,
		0, 0, zn * zf / (zn - zf), 0);
}

float4x4 MatrixLookAt(const float3 &eye, const float3 &at, const float3 &up)
{
	float4x4 m;
	m.w.w = 1.0f;
	m.w.xyz() = eye;
	m.z.xyz() = normalize(eye - at);
	m.x.xyz() = normalize(cross(up, m.z.xyz()));
	m.y.xyz() = cross(m.z.xyz(), m.x.xyz());
	return MatrixRigidInverse(m);
}

float4x4 MatrixTranslation(const float3 &t)
{
	return float4x4(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		t.x, t.y, t.z, 1);
}

float4x4 MatrixRotationZ(const float angle_radians)
{
	float s = sinf(angle_radians);
	float c = cosf(angle_radians);
	return float4x4(
		c, s, 0, 0,
		-s, c, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
}

int operator==(const float4x4 &a, const float4x4 &b)
{
	return (a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w);
}

float4x4 Inverse(const float4x4 &m)
{
	float4x4 d;
	float *dst = &d.x.x;
	float tmp[12]; /* temp array for pairs */
	float src[16]; /* array of transpose source matrix */
	float det;     /* determinant */
	/* transpose matrix */
	for (int i = 0; i < 4; i++)
	{
		src[i] = m(i, 0);
		src[i + 4] = m(i, 1);
		src[i + 8] = m(i, 2);
		src[i + 12] = m(i, 3);
	}
	/* calculate pairs for first 8 elements (cofactors) */
	tmp[0] = src[10] * src[15];
	tmp[1] = src[11] * src[14];
	tmp[2] = src[9] * src[15];
	tmp[3] = src[11] * src[13];
	tmp[4] = src[9] * src[14];
	tmp[5] = src[10] * src[13];
	tmp[6] = src[8] * src[15];
	tmp[7] = src[11] * src[12];
	tmp[8] = src[8] * src[14];
	tmp[9] = src[10] * src[12];
	tmp[10] = src[8] * src[13];
	tmp[11] = src[9] * src[12];
	/* calculate first 8 elements (cofactors) */
	dst[0] = tmp[0] * src[5] + tmp[3] * src[6] + tmp[4] * src[7];
	dst[0] -= tmp[1] * src[5] + tmp[2] * src[6] + tmp[5] * src[7];
	dst[1] = tmp[1] * src[4] + tmp[6] * src[6] + tmp[9] * src[7];
	dst[1] -= tmp[0] * src[4] + tmp[7] * src[6] + tmp[8] * src[7];
	dst[2] = tmp[2] * src[4] + tmp[7] * src[5] + tmp[10] * src[7];
	dst[2] -= tmp[3] * src[4] + tmp[6] * src[5] + tmp[11] * src[7];
	dst[3] = tmp[5] * src[4] + tmp[8] * src[5] + tmp[11] * src[6];
	dst[3] -= tmp[4] * src[4] + tmp[9] * src[5] + tmp[10] * src[6];
	dst[4] = tmp[1] * src[1] + tmp[2] * src[2] + tmp[5] * src[3];
	dst[4] -= tmp[0] * src[1] + tmp[3] * src[2] + tmp[4] * src[3];
	dst[5] = tmp[0] * src[0] + tmp[7] * src[2] + tmp[8] * src[3];
	dst[5] -= tmp[1] * src[0] + tmp[6] * src[2] + tmp[9] * src[3];
	dst[6] = tmp[3] * src[0] + tmp[6] * src[1] + tmp[11] * src[3];
	dst[6] -= tmp[2] * src[0] + tmp[7] * src[1] + tmp[10] * src[3];
	dst[7] = tmp[4] * src[0] + tmp[9] * src[1] + tmp[10] * src[2];
	dst[7] -= tmp[5] * src[0] + tmp[8] * src[1] + tmp[11] * src[2];
	/* calculate pairs for second 8 elements (cofactors) */
	tmp[0] = src[2] * src[7];
	tmp[1] = src[3] * src[6];
	tmp[2] = src[1] * src[7];
	tmp[3] = src[3] * src[5];
	tmp[4] = src[1] * src[6];
	tmp[5] = src[2] * src[5];
	tmp[6] = src[0] * src[7];
	tmp[7] = src[3] * src[4];
	tmp[8] = src[0] * src[6];
	tmp[9] = src[2] * src[4];
	tmp[10] = src[0] * src[5];
	tmp[11] = src[1] * src[4];
	/* calculate second 8 elements (cofactors) */
	dst[8] = tmp[0] * src[13] + tmp[3] * src[14] + tmp[4] * src[15];
	dst[8] -= tmp[1] * src[13] + tmp[2] * src[14] + tmp[5] * src[15];
	dst[9] = tmp[1] * src[12] + tmp[6] * src[14] + tmp[9] * src[15];
	dst[9] -= tmp[0] * src[12] + tmp[7] * src[14] + tmp[8] * src[15];
	dst[10] = tmp[2] * src[12] + tmp[7] * src[13] + tmp[10] * src[15];
	dst[10] -= tmp[3] * src[12] + tmp[6] * src[13] + tmp[11] * src[15];
	dst[11] = tmp[5] * src[12] + tmp[8] * src[13] + tmp[11] * src[14];
	dst[11] -= tmp[4] * src[12] + tmp[9] * src[13] + tmp[10] * src[14];
	dst[12] = tmp[2] * src[10] + tmp[5] * src[11] + tmp[1] * src[9];
	dst[12] -= tmp[4] * src[11] + tmp[0] * src[9] + tmp[3] * src[10];
	dst[13] = tmp[8] * src[11] + tmp[0] * src[8] + tmp[7] * src[10];
	dst[13] -= tmp[6] * src[10] + tmp[9] * src[11] + tmp[1] * src[8];
	dst[14] = tmp[6] * src[9] + tmp[11] * src[11] + tmp[3] * src[8];
	dst[14] -= tmp[10] * src[11] + tmp[2] * src[8] + tmp[7] * src[9];
	dst[15] = tmp[10] * src[10] + tmp[4] * src[8] + tmp[9] * src[9];
	dst[15] -= tmp[8] * src[9] + tmp[11] * src[10] + tmp[5] * src[8];
	/* calculate determinant */
	det = src[0] * dst[0] + src[1] * dst[1] + src[2] * dst[2] + src[3] * dst[3];
	/* calculate matrix inverse */
	det = 1 / det;
	for (int j = 0; j < 16; j++)
		dst[j] *= det;
	return d;
}

//--------- Quaternion --------------

template <>
void Quaternion::Normalize()
{
	float m = sqrtf(squared(w) + squared(x) + squared(y) + squared(z));
	if (m < 0.000000001f)
	{
		w = 1.0f;
		x = y = z = 0.0f;
		return;
	}
	(*this) *= (1.0f / m);
}

float3 rotate(const Quaternion &q, const float3 &v)
{
	// The following is equivalent to:
	//return (q.getmatrix() * v);
	float qx2 = q.x * q.x;
	float qy2 = q.y * q.y;
	float qz2 = q.z * q.z;

	float qxqy = q.x * q.y;
	float qxqz = q.x * q.z;
	float qxqw = q.x * q.w;
	float qyqz = q.y * q.z;
	float qyqw = q.y * q.w;
	float qzqw = q.z * q.w;
	return float3(
		(1 - 2 * (qy2 + qz2)) * v.x + (2 * (qxqy - qzqw)) * v.y + (2 * (qxqz + qyqw)) * v.z,
		(2 * (qxqy + qzqw)) * v.x + (1 - 2 * (qx2 + qz2)) * v.y + (2 * (qyqz - qxqw)) * v.z,
		(2 * (qxqz - qyqw)) * v.x + (2 * (qyqz + qxqw)) * v.y + (1 - 2 * (qx2 + qy2)) * v.z);
}

Quaternion slerp(const Quaternion &_a, const Quaternion &b, float interp)
{
	Quaternion a = _a;
	if (dot(a, b) < 0.0)
	{
		a.w = -a.w;
		a.x = -a.x;
		a.y = -a.y;
		a.z = -a.z;
	}
	float d = dot(a, b);
	if (d >= 1.0)
	{
		return a;
	}
	float theta = acosf(d);
	if (theta == 0.0f)
	{
		return (a);
	}
	return a * (sinf(theta - interp * theta) / sinf(theta)) + b * (sinf(interp * theta) / sinf(theta));
}

Quaternion Interpolate(const Quaternion &q0, const Quaternion &q1, float alpha)
{
	return slerp(q0, q1, alpha);
}

Quaternion YawPitchRoll(float yaw, float pitch, float roll)
{
	return QuatFromAxisAngle(float3(0.0f, 0.0f, 1.0f), DegToRad(yaw)) * QuatFromAxisAngle(float3(1.0f, 0.0f, 0.0f), DegToRad(pitch)) * QuatFromAxisAngle(float3(0.0f, 1.0f, 0.0f), DegToRad(roll));
}

float Yaw(const Quaternion &q)
{
	static float3 v;
	v = q.ydir();
	return (v.y == 0.0 && v.x == 0.0) ? 0.0f : RadToDeg(atan2f(-v.x, v.y));
}

float Pitch(const Quaternion &q)
{
	static float3 v;
	v = q.ydir();
	return RadToDeg(atan2f(v.z, sqrtf(squared(v.x) + squared(v.y))));
}

float Roll(const Quaternion &_q)
{
	Quaternion q = _q;
	q = QuatFromAxisAngle(float3(0.0f, 0.0f, 1.0f), -DegToRad(Yaw(q))) * q;
	q = QuatFromAxisAngle(float3(1.0f, 0.0f, 0.0f), -DegToRad(Pitch(q))) * q;
	return RadToDeg(atan2f(-q.xdir().z, q.xdir().x));
}

float Yaw(const float3 &v)
{
	return (v.y == 0.0 && v.x == 0.0) ? 0.0f : RadToDeg(atan2f(-v.x, v.y));
}

float Pitch(const float3 &v)
{
	return RadToDeg(atan2f(v.z, sqrtf(squared(v.x) + squared(v.y))));
}

//--------- utility functions -------------

//        RotationArc()
// Given two vectors v0 and v1 this function
// returns quaternion q where q*v0==v1.
// Routine taken from game programming gems.
Quaternion RotationArc(float3 v0, float3 v1)
{
	static Quaternion q;
	v0 = normalize(v0);  // Comment these two lines out if you know its not needed.
	v1 = normalize(v1);  // If vector is already unit length then why do it again?
	float3 c = cross(v0, v1);
	float d = dot(v0, v1);
	if (d <= -1.0f)
	{
		float3 a = Orth(v0);
		return Quaternion(a.x, a.y, a.z, 0);
	}  // 180 about any orthogonal axis axis
	float s = sqrtf((1 + d) * 2);
	q.x = c.x / s;
	q.y = c.y / s;
	q.z = c.z / s;
	q.w = s / 2.0f;
	return q;
}

float4x4 MatrixFromQuatVec(const Quaternion &q, const float3 &v)
{
	// builds a 4x4 transformation matrix based on orientation q and translation v
	float qx2 = q.x * q.x;
	float qy2 = q.y * q.y;
	float qz2 = q.z * q.z;

	float qxqy = q.x * q.y;
	float qxqz = q.x * q.z;
	float qxqw = q.x * q.w;
	float qyqz = q.y * q.z;
	float qyqw = q.y * q.w;
	float qzqw = q.z * q.w;

	return float4x4(
		1 - 2 * (qy2 + qz2),
		2 * (qxqy + qzqw),
		2 * (qxqz - qyqw),
		0,
		2 * (qxqy - qzqw),
		1 - 2 * (qx2 + qz2),
		2 * (qyqz + qxqw),
		0,
		2 * (qxqz + qyqw),
		2 * (qyqz - qxqw),
		1 - 2 * (qx2 + qy2),
		0,
		v.x,
		v.y,
		v.z,
		1.0f);
}

float3 PlaneLineIntersection(const float3 &normal, const float dist, const float3 &p0, const float3 &p1)
{
	// returns the point where the line p0-p1 intersects the plane n&d
	float3 dif;
	dif = p1 - p0;
	float dn = dot(normal, dif);
	float t = -(dist + dot(normal, p0)) / dn;
	return p0 + (dif * t);
}

float3 LineProject(const float3 &p0, const float3 &p1, const float3 &a)
{
	// project point a on segment [p0,p1]
	float3 d = p1 - p0;
	float t = dot(d, (a - p0)) / dot(d, d);
	return p0 + d * t;
}

float LineProjectTime(const float3 &p0, const float3 &p1, const float3 &a)
{
	// project point a on segment [p0,p1]
	float3 d = p1 - p0;
	float t = dot(d, (a - p0)) / dot(d, d);
	return t;
}

float3 TriNormal(const float3 &v0, const float3 &v1, const float3 &v2)
{
	// return the normal of the triangle
	// inscribed by v0, v1, and v2
	float3 cp = cross(v1 - v0, v2 - v1);
	float m = magnitude(cp);
	if (m == 0) return float3(1, 0, 0);
	return cp * (1.0f / m);
}

int BoxInside(const float3 &p, const float3 &bmin, const float3 &bmax)
{
	return (p.x >= bmin.x && p.x <= bmax.x &&
			p.y >= bmin.y && p.y <= bmax.y &&
			p.z >= bmin.z && p.z <= bmax.z);
}

int BoxIntersect(const float3 &v0, const float3 &v1, const float3 &bmin, const float3 &bmax, float3 *impact)
{
	if (BoxInside(v0, bmin, bmax))
	{
		*impact = v0;
		return 1;
	}
	if (v0.x <= bmin.x && v1.x >= bmin.x)
	{
		float a = (bmin.x - v0.x) / (v1.x - v0.x);
		//v.x = bmin.x;
		float vy = (1 - a) * v0.y + a * v1.y;
		float vz = (1 - a) * v0.z + a * v1.z;
		if (vy >= bmin.y && vy <= bmax.y && vz >= bmin.z && vz <= bmax.z)
		{
			impact->x = bmin.x;
			impact->y = vy;
			impact->z = vz;
			return 1;
		}
	}
	else if (v0.x >= bmax.x && v1.x <= bmax.x)
	{
		float a = (bmax.x - v0.x) / (v1.x - v0.x);
		//v.x = bmax.x;
		float vy = (1 - a) * v0.y + a * v1.y;
		float vz = (1 - a) * v0.z + a * v1.z;
		if (vy >= bmin.y && vy <= bmax.y && vz >= bmin.z && vz <= bmax.z)
		{
			impact->x = bmax.x;
			impact->y = vy;
			impact->z = vz;
			return 1;
		}
	}
	if (v0.y <= bmin.y && v1.y >= bmin.y)
	{
		float a = (bmin.y - v0.y) / (v1.y - v0.y);
		float vx = (1 - a) * v0.x + a * v1.x;
		//v.y = bmin.y;
		float vz = (1 - a) * v0.z + a * v1.z;
		if (vx >= bmin.x && vx <= bmax.x && vz >= bmin.z && vz <= bmax.z)
		{
			impact->x = vx;
			impact->y = bmin.y;
			impact->z = vz;
			return 1;
		}
	}
	else if (v0.y >= bmax.y && v1.y <= bmax.y)
	{
		float a = (bmax.y - v0.y) / (v1.y - v0.y);
		float vx = (1 - a) * v0.x + a * v1.x;
		// vy = bmax.y;
		float vz = (1 - a) * v0.z + a * v1.z;
		if (vx >= bmin.x && vx <= bmax.x && vz >= bmin.z && vz <= bmax.z)
		{
			impact->x = vx;
			impact->y = bmax.y;
			impact->z = vz;
			return 1;
		}
	}
	if (v0.z <= bmin.z && v1.z >= bmin.z)
	{
		float a = (bmin.z - v0.z) / (v1.z - v0.z);
		float vx = (1 - a) * v0.x + a * v1.x;
		float vy = (1 - a) * v0.y + a * v1.y;
		// v.z = bmin.z;
		if (vy >= bmin.y && vy <= bmax.y && vx >= bmin.x && vx <= bmax.x)
		{
			impact->x = vx;
			impact->y = vy;
			impact->z = bmin.z;
			return 1;
		}
	}
	else if (v0.z >= bmax.z && v1.z <= bmax.z)
	{
		float a = (bmax.z - v0.z) / (v1.z - v0.z);
		float vx = (1 - a) * v0.x + a * v1.x;
		float vy = (1 - a) * v0.y + a * v1.y;
		// v.z = bmax.z;
		if (vy >= bmin.y && vy <= bmax.y && vx >= bmin.x && vx <= bmax.x)
		{
			impact->x = vx;
			impact->y = vy;
			impact->z = bmax.z;
			return 1;
		}
	}
	return 0;
}

float DistanceBetweenLines(const float3 &ustart, const float3 &udir, const float3 &vstart, const float3 &vdir, float3 *upoint, float3 *vpoint)
{
	static float3 cp;
	cp = normalize(cross(udir, vdir));

	float distu = -dot(cp, ustart);
	float distv = -dot(cp, vstart);
	float dist = (float)fabs(distu - distv);
	if (upoint)
	{
		float3 normal = normalize(cross(vdir, cp));
		*upoint = PlaneLineIntersection(normal, -dot(normal, vstart), ustart, ustart + udir);
	}
	if (vpoint)
	{
		float3 normal = normalize(cross(udir, cp));
		*vpoint = PlaneLineIntersection(normal, -dot(normal, ustart), vstart, vstart + vdir);
	}
	return dist;
}

Quaternion VirtualTrackBall(const float3 &cop, const float3 &cor, const float3 &dir1, const float3 &dir2)
{
	// routine taken from game programming gems.
	// Implement track ball functionality to spin stuf on the screen
	//  cop   center of projection
	//  cor   center of rotation
	//  dir1  old mouse direction
	//  dir2  new mouse direction
	// pretend there is a sphere around cor.  Then find the points
	// where dir1 and dir2 intersect that sphere.  Find the
	// rotation that takes the first point to the second.
	float m;
	// compute plane
	float3 nrml = cor - cop;
	float fudgefactor = 1.0f / (magnitude(nrml) * 0.25f);  // since trackball proportional to distance from cop
	nrml = normalize(nrml);
	float dist = -dot(nrml, cor);
	float3 u = PlaneLineIntersection(nrml, dist, cop, cop + dir1);
	u = u - cor;
	u = u * fudgefactor;
	m = magnitude(u);
	if (m > 1)
	{
		u /= m;
	}
	else
	{
		u = u - (nrml * sqrtf(1 - m * m));
	}
	float3 v = PlaneLineIntersection(nrml, dist, cop, cop + dir2);
	v = v - cor;
	v = v * fudgefactor;
	m = magnitude(v);
	if (m > 1)
	{
		v /= m;
	}
	else
	{
		v = v - (nrml * sqrtf(1 - m * m));
	}
	return RotationArc(u, v);
}

int countpolyhit = 0;
int HitCheckPoly(const float3 *vert, const int n, const float3 &v0, const float3 &v1, float3 *impact, float3 *normal)
{
	countpolyhit++;
	int i;
	float3 nrml(0, 0, 0);
	for (i = 0; i < n; i++)
	{
		int i1 = (i + 1) % n;
		int i2 = (i + 2) % n;
		nrml = nrml + cross(vert[i1] - vert[i], vert[i2] - vert[i1]);
	}

	float m = magnitude(nrml);
	if (m == 0.0)
	{
		return 0;
	}
	nrml = nrml * (1.0f / m);
	float dist = -dot(nrml, vert[0]);
	float d0, d1;
	if ((d0 = dot(v0, nrml) + dist) < 0 || (d1 = dot(v1, nrml) + dist) > 0)
	{
		return 0;
	}

	static float3 the_point;
	// By using the cached plane distances d0 and d1
	// we can optimize the following:
	//     the_point = planelineintersection(nrml,dist,v0,v1);
	float a = d0 / (d0 - d1);
	the_point = v0 * (1 - a) + v1 * a;

	int inside = 1;
	for (int j = 0; inside && j < n; j++)
	{
		// let inside = 0 if outside
		float3 pp1, pp2, side;
		pp1 = vert[j];
		pp2 = vert[(j + 1) % n];
		side = cross((pp2 - pp1), (the_point - pp1));
		inside = (dot(nrml, side) >= 0.0);
	}
	if (inside)
	{
		if (normal)
		{
			*normal = nrml;
		}
		if (impact)
		{
			*impact = the_point;
		}
	}
	return inside;
}

int SolveQuadratic(float a, float b, float c, float *ta, float *tb)  // if true returns roots ta,tb where ta<=tb
{
	assert(ta);
	assert(tb);
	float d = b * b - 4.0f * a * c;  // discriminant
	if (d < 0.0f) return 0;
	float sqd = sqrtf(d);
	*ta = (-b - sqd) / (2.0f * a);
	*tb = (-b + sqd) / (2.0f * a);
	return 1;
}

int HitCheckRaySphere(const float3 &sphereposition, float radius, const float3 &_v0, const float3 &_v1, float3 *impact, float3 *normal)
{
	assert(impact);
	assert(normal);
	float3 dv = _v1 - _v0;
	float3 v0 = _v0 - sphereposition;            // solve in coord system of the sphere
	if (radius <= 0.0f || _v0 == _v1) return 0;  // only true if point moves from outside to inside sphere.
	float a = dot(dv, dv);
	float b = 2.0f * dot(dv, v0);
	float c = dot(v0, v0) - radius * radius;
	if (c < 0.0f) return 0;  // we are already inside the sphere.

	float ta, tb;
	int doesIntersect = SolveQuadratic(a, b, c, &ta, &tb);

	if (!doesIntersect) return 0;

	if (ta >= 0.0f && ta <= 1.0f && (ta <= tb || tb <= 0.0f))
	{
		*impact = _v0 + dv * ta;
		*normal = (v0 + dv * ta) / radius;
		return 1;
	}
	if (tb >= 0.0f && tb <= 1.0f)
	{
		assert(tb <= ta || ta <= 0.0f);  // tb must be better than ta
		*impact = _v0 + dv * tb;
		*normal = (v0 + dv * tb) / radius;
		return 1;
	}
	return 0;
}

int HitCheckRayCylinder(const float3 &p0, const float3 &p1, float radius, const float3 &_v0, const float3 &_v1, float3 *impact, float3 *normal)
{
	assert(impact);
	assert(normal);
	// only concerned about hitting the sides, not the caps for now
	float3x3 m = RotationArc(p1 - p0, float3(0, 0, 1.0f)).getmatrix();
	float h = ((p1 - p0) * m).z;
	float3 v0 = (_v0 - p0) * m;
	float3 v1 = (_v1 - p0) * m;
	if (v0.z <= 0.0f && v1.z <= 0.0f) return 0;                                  // entirely below cylinder
	if (v0.z >= h && v1.z >= h) return 0;                                        // ray is above cylinder
	if (v0.z < 0.0f) v0 = PlaneLineIntersection(float3(0, 0, 1.0f), 0, v0, v1);  // crop to cylinder range
	if (v1.z < 0.0f) v1 = PlaneLineIntersection(float3(0, 0, 1.0f), 0, v0, v1);
	if (v0.z > h) v0 = PlaneLineIntersection(float3(0, 0, 1.0f), -h, v0, v1);
	if (v1.z > h) v1 = PlaneLineIntersection(float3(0, 0, 1.0f), -h, v0, v1);
	if (v0.x == v1.x && v0.y == v1.y) return 0;
	float3 dv = v1 - v0;

	float a = dv.x * dv.x + dv.y * dv.y;
	float b = 2.0f * (dv.x * v0.x + dv.y * v0.y);
	float c = (v0.x * v0.x + v0.y * v0.y) - radius * radius;
	if (c < 0.0f) return 0;  // we are already inside the cylinder .

	float ta, tb;
	int doesIntersect = SolveQuadratic(a, b, c, &ta, &tb);

	if (!doesIntersect) return 0;

	if (ta >= 0.0f && ta <= 1.0f && (ta <= tb || tb <= 0.0f))
	{
		*impact = (v0 + dv * ta) * Transpose(m) + p0;
		*normal = (float3(v0.x, v0.y, 0.0f) + float3(dv.x, dv.y, 0) * ta) / radius * Transpose(m);
		return 1;
	}
	if (tb >= 0.0f && tb <= 1.0f)
	{
		assert(tb <= ta || ta <= 0.0f);                // tb must be better than ta
		*impact = (v0 + dv * tb) * Transpose(m) + p0;  // compute intersection in original space
		*normal = (float3(v0.x, v0.y, 0.0f) + float3(dv.x, dv.y, 0) * tb) / radius * Transpose(m);
		return 1;
	}
	return 0;
}

int HitCheckSweptSphereTri(const float3 &p0, const float3 &p1, const float3 &p2, float radius, const float3 &v0, const float3 &_v1, float3 *impact, float3 *normal)
{
	float3 unused;
	if (!normal) normal = &unused;
	float3 v1 = _v1;  // so we can update v1 after each sub intersection test if necessary
	int hit = 0;
	float3 cp = cross(p1 - p0, p2 - p0);
	if (dot(cp, v1 - v0) >= 0.0f) return 0;  // coming from behind and/or moving away
	float3 n = normalize(cp);
	float3 tv[3];
	tv[0] = p0 + n * radius;
	tv[1] = p1 + n * radius;
	tv[2] = p2 + n * radius;
	hit += HitCheckPoly(tv, 3, v0, v1, &v1, normal);
	hit += HitCheckRayCylinder(p0, p1, radius, v0, v1, &v1, normal);
	hit += HitCheckRayCylinder(p1, p2, radius, v0, v1, &v1, normal);
	hit += HitCheckRayCylinder(p2, p0, radius, v0, v1, &v1, normal);
	hit += HitCheckRaySphere(p0, radius, v0, v1, &v1, normal);
	hit += HitCheckRaySphere(p1, radius, v0, v1, &v1, normal);
	hit += HitCheckRaySphere(p2, radius, v0, v1, &v1, normal);
	if (hit && impact) *impact = v1 + *normal * 0.001f;
	return hit;
}

float3 PlanesIntersection(const Plane &p0, const Plane &p1, const Plane &p2)
{
	float3x3 mp = Transpose(float3x3(p0.normal(), p1.normal(), p2.normal()));
	float3x3 mi = Inverse(mp);
	float3 b(p0.dist(), p1.dist(), p2.dist());
	return -b * mi;
}

float3 PlanesIntersection(const Plane *planes, int planes_count, const float3 &seed)
{
	int i;
	float3x3 A;  // gets initilized to 0 matrix
	float3 b(0, 0, 0);
	for (i = 0; i < planes_count; i++)
	{
		const Plane &p = planes[i];
		A += outerprod(p.normal(), p.normal());
		b += p.normal() * -p.dist();
	}
	float3x3 evecs = Diagonalizer(A).getmatrix();           // eigenvectors
	float3 evals = Diagonal(evecs * A * Transpose(evecs));  // eigenvalues
	for (i = 0; i < 3; i++)
	{
		if (fabsf(evals[i]) < 1.0f)  // not sure if they are necessarily positive
		{
			Plane p;
			p.normal() = evecs[i] * squared(1.0f - evals[i]);
			p.dist() = -dot(seed, p.normal());
			A += outerprod(p.normal(), p.normal());
			b += p.normal() * -p.dist();
		}
	}
	return Inverse(A) * b;
}

Plane Transform(const Plane &p, const float3 &translation, const Quaternion &rotation)
{
	//   Transforms the plane by the given translation/rotation.
	float3 newnormal = rotate(rotation, p.normal());
	return Plane(newnormal, p.dist() - dot(newnormal, translation));
}

float3 PlaneProject(const Plane &plane, const float3 &point)
{
	return point - plane.normal() * (dot(point, plane.normal()) + plane.dist());
}
float3 PlaneLineIntersection(const Plane &plane, const float3 &p0, const float3 &p1)
{
	// returns the point where the line p0-p1 intersects the plane n&d
	float3 dif;
	dif = p1 - p0;
	float dn = dot(plane.normal(), dif);
	float t = -(plane.dist() + dot(plane.normal(), p0)) / dn;
	return p0 + (dif * t);
}

int Clip(const float3 &plane_normal, float plane_dist, const float3 *verts_in, int count_in, float3 *verts_out)
{
	// clips a polygon specified by the non-indexed vertex list verts_in.
	// verts_out must be preallocated with a size >= count+1
	assert(verts_out);
	int n = 0;
	int prev_status = (dot(plane_normal, verts_in[count_in - 1]) + plane_dist > 0);
	for (int i = 0; i < count_in; i++)
	{
		int status = (dot(plane_normal, verts_in[i]) + plane_dist > 0);
		if (status != prev_status)
		{
			verts_out[n++] = PlaneLineIntersection(plane_normal, plane_dist, verts_in[(i == 0) ? count_in - 1 : i - 1], verts_in[i]);
		}
		if (status == 0)  // under
		{
			verts_out[n++] = verts_in[i];
		}
	}
	assert(n <= count_in + 1);  // remove if intention to use this routine on convex polygons
	return n;
}

int ClipPolyPoly(const float3 &normal, const float3 *clipper, int clipper_count, const float3 *verts_in, int in_count, float3 *scratch)
{
	// clips polys against each other.
	// requires sufficiently allocated temporary memory in scratch buffer
	// function returns final number of vertices in clipped polygon.
	// Resulting vertices are returned in the scratch buffer.
	// if the arrays are the same &verts_in==&scratch the routine should still work anyways.
	// the first argument (normal) is the normal of polygon clipper.
	// its generally assumed both are convex polygons.
	assert(scratch);  // size should be >= 2*(clipper_count+in_count)
	int i;
	int bsize = clipper_count + in_count;
	int count = in_count;
	for (i = 0; i < clipper_count; i++)
	{
		int i1 = (i + 1) % clipper_count;
		float3 n = cross(clipper[i1] - clipper[i], normal);
		if (n == float3(0, 0, 0)) continue;
		n = normalize(n);
		count = Clip(n, -dot(clipper[i], n), (i == 0) ? verts_in : (i % 2) ? scratch : scratch + bsize, count, (i % 2) ? scratch + bsize : scratch);
		assert(count < bsize);
	}
	if (clipper_count % 2) memcpy(scratch, scratch + bsize, count * sizeof(float3));
	return count;
}

float Volume(const float3 *vertices, const int3 *tris, const int count)
{
	// count is the number of triangles (tris)
	float volume = 0;
	for (int i = 0; i < count; i++)  // for each triangle
	{
		volume += Determinant(float3x3(vertices[tris[i][0]], vertices[tris[i][1]], vertices[tris[i][2]]));  //divide by 6 later for efficiency
	}
	return volume / 6.0f;  // since the determinant give 6 times tetra volume
}

float3 CenterOfMass(const float3 *vertices, const int3 *tris, const int count)
{
	// count is the number of triangles (tris)
	float3 com(0, 0, 0);
	float volume = 0;                // actually accumulates the volume*6
	for (int i = 0; i < count; i++)  // for each triangle
	{
		float3x3 A(vertices[tris[i][0]], vertices[tris[i][1]], vertices[tris[i][2]]);
		float vol = Determinant(A);      // dont bother to divide by 6
		com += vol * (A.x + A.y + A.z);  // divide by 4 at end
		volume += vol;
	}
	com /= volume * 4.0f;
	return com;
}
float3x3 Inertia(const float3 *vertices, const int3 *tris, const int count, const float3 &com /* =float3(0,0,0) */)
{
	// count is the number of triangles (tris)
	// The moments are calculated based on the center of rotation (com) which defaults to [0,0,0] if unsupplied
	// assume mass==1.0  you can multiply by mass later.
	// for improved accuracy the next 3 variables, the determinant d, and its calculation should be changed to double
	float volume = 0;                // technically this variable accumulates the volume times 6
	float3 diag(0, 0, 0);            // accumulate matrix main diagonal integrals [x*x, y*y, z*z]
	float3 offd(0, 0, 0);            // accumulate matrix off-diagonal  integrals [y*z, x*z, x*y]
	for (int i = 0; i < count; i++)  // for each triangle
	{
		float3x3 A(vertices[tris[i][0]] - com, vertices[tris[i][1]] - com, vertices[tris[i][2]] - com);  // matrix trick for volume calc by taking determinant
		float d = Determinant(A);                                                                        // vol of tiny parallelapiped= d * dr * ds * dt (the 3 partials of my tetral triple integral equasion)
		volume += d;                                                                                     // add vol of current tetra (note it could be negative - that's ok we need that sometimes)
		for (int j = 0; j < 3; j++)
		{
			int j1 = (j + 1) % 3;
			int j2 = (j + 2) % 3;
			diag[j] += (A[0][j] * A[1][j] + A[1][j] * A[2][j] + A[2][j] * A[0][j] +
						A[0][j] * A[0][j] + A[1][j] * A[1][j] + A[2][j] * A[2][j]) *
					   d;  // divide by 60.0f later;
			offd[j] += (A[0][j1] * A[1][j2] + A[1][j1] * A[2][j2] + A[2][j1] * A[0][j2] +
						A[0][j1] * A[2][j2] + A[1][j1] * A[0][j2] + A[2][j1] * A[1][j2] +
						A[0][j1] * A[0][j2] * 2 + A[1][j1] * A[1][j2] * 2 + A[2][j1] * A[2][j2] * 2) *
					   d;  // divide by 120.0f later
		}
	}
	diag /= volume * (60.0f / 6.0f);  // divide by total volume (vol/6) since density=1/volume
	offd /= volume * (120.0f / 6.0f);
	return float3x3(diag.y + diag.z, -offd.z, -offd.y,
					-offd.z, diag.x + diag.z, -offd.x,
					-offd.y, -offd.x, diag.x + diag.y);
}

float3x3 ShapeInertiaContrib(const float3 &cor, const float3 &position, const Quaternion &orientation,
							 const float3 &shape_com, const float3x3 &shape_inertia, float shape_mass)
{
	// transforms 3x3 inertia tensor from local reference frame to a more global one.
	// essentially returns the contribution of a subshape to the inertia of a larger rigid body
	// typical usage:
	//         foreach shape s { totalinertia += InertiaContribution(...); }
	// cor - new center of rotation that we are translating to.
	// This could be the center of mass of the compound object.
	// Another application is when an object is attached to something (nail-joint) that is static, in which
	// one easy way to implement this is to lock the center of rotation and adjust the inertia accordingly.
	// position & orientation - is the current pose or transform of the shape.
	// Obviously position, orientation and cor are all described wrt the same reference frame.
	// shape_com and shape_inertia are the center of mass and the inertia of the shape in the local coordinate system of that shape.
	// To clarify, if a shape happened to be located somewhere else then position or orientation would be different, but
	// com and inertia would be the same.
	float3x3 Identity(1.0f, 0, 0, 0, 1.0f, 0, 0, 0, 1.0f);
	float3x3 R = orientation.getmatrix();
	float3 r = (shape_com * R + position) - cor;
	return Transpose(R) * shape_inertia * R + (Identity * dot(r, r) - outerprod(r, r)) * shape_mass;
}

Quaternion Diagonalizer(const float3x3 &A)
{
	// A must be a symmetric matrix.
	// returns quaternion q such that its corresponding matrix Q
	// can be used to Diagonalize A
	// Diagonal matrix D = Q * A * Transpose(Q);  and  A = QT*D*Q
	// The rows of q are the eigenvectors D's diagonal is the eigenvalues
	// As per 'row' convention if float3x3 Q = q.getmatrix(); then v*Q = q*v*conj(q)
	int maxsteps = 24;  // certainly wont need that many.
	int i;
	Quaternion q(0, 0, 0, 1);
	for (i = 0; i < maxsteps; i++)
	{
		float3x3 Q = q.getmatrix();                                        // v*Q == q*v*conj(q)
		float3x3 D = Q * A * Transpose(Q);                                 // A = Q^T*D*Q
		float3 offdiag(D[1][2], D[0][2], D[0][1]);                         // elements not on the diagonal
		float3 om(fabsf(offdiag.x), fabsf(offdiag.y), fabsf(offdiag.z));   // mag of each offdiag elem
		int k = (om.x > om.y && om.x > om.z) ? 0 : (om.y > om.z) ? 1 : 2;  // index of largest element of offdiag
		int k1 = (k + 1) % 3;
		int k2 = (k + 2) % 3;
		if (offdiag[k] == 0.0f) break;  // diagonal already
		float thet = (D[k2][k2] - D[k1][k1]) / (2.0f * offdiag[k]);
		float sgn = (thet > 0.0f) ? 1.0f : -1.0f;
		thet *= sgn;                                                                     // make it positive
		float t = sgn / (thet + ((thet < 1.E6f) ? sqrtf(squared(thet) + 1.0f) : thet));  // sign(T)/(|T|+sqrt(T^2+1))
		float c = 1.0f / sqrtf(squared(t) + 1.0f);                                       //  c= 1/(t^2+1) , t=s/c
		if (c == 1.0f) break;                                                            // no room for improvement - reached machine precision.
		Quaternion jr(0, 0, 0, 0);                                                       // jacobi rotation for this iteration.
		jr[k] = sgn * sqrtf((1.0f - c) / 2.0f);                                          // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)
		jr[k] *= -1.0f;                                                                  // since our quat-to-matrix convention was for v*M instead of M*v
		jr.w = sqrtf(1.0f - squared(jr[k]));
		if (jr.w == 1.0f) break;  // reached limits of floating point precision
		q = q * jr;
		q.Normalize();
	}
	return q;
}

float3 Diagonal(const float3x3 &M)
{
	return float3(M[0][0], M[1][1], M[2][2]);
}
