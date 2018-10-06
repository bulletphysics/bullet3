//
//
//  Typical 3d vector math code.
//  By S Melax 1998-2008
//
//

#ifndef SM_VEC_MATH_H
#define SM_VEC_MATH_H

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <xmmintrin.h>

#define M_PIf (3.1415926535897932384626433832795f)

inline float DegToRad(float angle_degrees) { return angle_degrees * M_PIf / 180.0f; }  // returns Radians.
inline float RadToDeg(float angle_radians) { return angle_radians * 180.0f / M_PIf; }  // returns Degrees.

#define OFFSET(Class, Member) (((char *)(&(((Class *)NULL)->Member))) - ((char *)NULL))

int argmin(const float a[], int n);
int argmax(const float a[], int n);
float squared(float a);
float clamp(float a, const float minval = 0.0f, const float maxval = 1.0f);
int clamp(int a, const int minval, const int maxval);
float Round(float a, float precision);
float Interpolate(const float &f0, const float &f1, float alpha);

template <class T>
void Swap(T &a, T &b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

template <class T>
T Max(const T &a, const T &b)
{
	return (a > b) ? a : b;
}

template <class T>
T Min(const T &a, const T &b)
{
	return (a < b) ? a : b;
}

//for template normalize functions:
inline float squareroot(float a) { return sqrtf(a); }
inline double squareroot(double a) { return sqrt(a); }

//----------------------------------

//-------- 2D --------

template <class T>
class vec2
{
public:
	T x, y;
	inline vec2()
	{
		x = 0;
		y = 0;
	}
	inline vec2(const T &_x, const T &_y)
	{
		x = _x;
		y = _y;
	}
	inline T &operator[](int i) { return ((T *)this)[i]; }
	inline const T &operator[](int i) const { return ((T *)this)[i]; }
};

typedef vec2<int> int2;
typedef vec2<float> float2;

template <class T>
inline int operator==(const vec2<T> &a, const vec2<T> &b)
{
	return (a.x == b.x && a.y == b.y);
}
template <class T>
inline vec2<T> operator-(const vec2<T> &a, const vec2<T> &b)
{
	return vec2<T>(a.x - b.x, a.y - b.y);
}
template <class T>
inline vec2<T> operator+(const vec2<T> &a, const vec2<T> &b)
{
	return float2(a.x + b.x, a.y + b.y);
}

//--------- 3D ---------

template <class T>
class vec3
{
public:
	T x, y, z;
	inline vec3()
	{
		x = 0;
		y = 0;
		z = 0;
	};
	inline vec3(const T &_x, const T &_y, const T &_z)
	{
		x = _x;
		y = _y;
		z = _z;
	};
	inline T &operator[](int i) { return ((T *)this)[i]; }
	inline const T &operator[](int i) const { return ((T *)this)[i]; }
};

typedef vec3<int> int3;
typedef vec3<short> short3;
typedef vec3<float> float3;

// due to ambiguity there is no overloaded operators for v3*v3 use dot,cross,outerprod,cmul
template <class T>
inline int operator==(const vec3<T> &a, const vec3<T> &b)
{
	return (a.x == b.x && a.y == b.y && a.z == b.z);
}
template <class T>
inline int operator!=(const vec3<T> &a, const vec3<T> &b)
{
	return !(a == b);
}
template <class T>
inline vec3<T> operator+(const vec3<T> &a, const vec3<T> &b)
{
	return vec3<T>(a.x + b.x, a.y + b.y, a.z + b.z);
}
template <class T>
inline vec3<T> operator-(const vec3<T> &a, const vec3<T> &b)
{
	return vec3<T>(a.x - b.x, a.y - b.y, a.z - b.z);
}
template <class T>
inline vec3<T> operator-(const vec3<T> &v)
{
	return vec3<T>(-v.x, -v.y, -v.z);
}
template <class T>
inline vec3<T> operator*(const vec3<T> &v, const T &s)
{
	return vec3<T>(v.x * s, v.y * s, v.z * s);
}
template <class T>
inline vec3<T> operator*(T s, const vec3<T> &v)
{
	return v * s;
}
template <class T>
inline vec3<T> operator/(const vec3<T> &v, T s)
{
	return vec3<T>(v.x / s, v.y / s, v.z / s);
}
template <class T>
inline T dot(const vec3<T> &a, const vec3<T> &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
template <class T>
inline vec3<T> cmul(const vec3<T> &a, const vec3<T> &b)
{
	return vec3<T>(a.x * b.x, a.y * b.y, a.z * b.z);
}
template <class T>
inline vec3<T> cross(const vec3<T> &a, const vec3<T> &b)
{
	return vec3<T>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
template <class T>
inline T magnitude(const vec3<T> &v)
{
	return squareroot(dot(v, v));
}
template <class T>
inline vec3<T> normalize(const vec3<T> &v)
{
	return v / magnitude(v);
}
template <class T>
inline vec3<T> &operator+=(vec3<T> &a, const vec3<T> &b)
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}
template <class T>
inline vec3<T> &operator-=(vec3<T> &a, const vec3<T> &b)
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return a;
}
template <class T>
inline vec3<T> &operator*=(vec3<T> &v, T s)
{
	v.x *= s;
	v.y *= s;
	v.z *= s;
	return v;
}
template <class T>
inline vec3<T> &operator/=(vec3<T> &v, T s)
{
	v.x /= s;
	v.y /= s;
	v.z /= s;
	return v;
}

float3 safenormalize(const float3 &v);
float3 vabs(const float3 &v);
float3 Interpolate(const float3 &v0, const float3 &v1, float alpha);
float3 Round(const float3 &a, float precision);
template <class T>
inline vec3<T> VectorMin(const vec3<T> &a, const vec3<T> &b)
{
	return vec3<T>(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}
template <class T>
inline vec3<T> VectorMax(const vec3<T> &a, const vec3<T> &b)
{
	return vec3<T>(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}
int overlap(const float3 &bmina, const float3 &bmaxa, const float3 &bminb, const float3 &bmaxb);

template <class T>
class mat3x3
{
public:
	vec3<T> x, y, z;  // the 3 rows of the Matrix
	inline mat3x3() {}
	inline mat3x3(const T &xx, const T &xy, const T &xz, const T &yx, const T &yy, const T &yz, const T &zx, const T &zy, const T &zz) : x(xx, xy, xz), y(yx, yy, yz), z(zx, zy, zz) {}
	inline mat3x3(const vec3<T> &_x, const vec3<T> &_y, const vec3<T> &_z) : x(_x), y(_y), z(_z) {}
	inline vec3<T> &operator[](int i) { return (&x)[i]; }
	inline const vec3<T> &operator[](int i) const { return (&x)[i]; }
	inline T &operator()(int r, int c) { return ((&x)[r])[c]; }
	inline const T &operator()(int r, int c) const { return ((&x)[r])[c]; }
};
typedef mat3x3<float> float3x3;

float3x3 Transpose(const float3x3 &m);
template <class T>
vec3<T> operator*(const vec3<T> &v, const mat3x3<T> &m)
{
	return vec3<T>((m.x.x * v.x + m.y.x * v.y + m.z.x * v.z),
				   (m.x.y * v.x + m.y.y * v.y + m.z.y * v.z),
				   (m.x.z * v.x + m.y.z * v.y + m.z.z * v.z));
}

float3 operator*(const float3x3 &m, const float3 &v);
float3x3 operator*(const float3x3 &m, const float &s);
float3x3 operator*(const float3x3 &ma, const float3x3 &mb);
float3x3 operator/(const float3x3 &a, const float &s);
float3x3 operator+(const float3x3 &a, const float3x3 &b);
float3x3 operator-(const float3x3 &a, const float3x3 &b);
float3x3 &operator+=(float3x3 &a, const float3x3 &b);
float3x3 &operator-=(float3x3 &a, const float3x3 &b);
float3x3 &operator*=(float3x3 &a, const float &s);
float Determinant(const float3x3 &m);
float3x3 Inverse(const float3x3 &a);  // its just 3x3 so we simply do that cofactor method
float3x3 outerprod(const float3 &a, const float3 &b);

//-------- 4D Math --------

template <class T>
class vec4
{
public:
	T x, y, z, w;
	inline vec4()
	{
		x = 0;
		y = 0;
		z = 0;
		w = 0;
	};
	inline vec4(const T &_x, const T &_y, const T &_z, const T &_w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}
	inline vec4(const vec3<T> &v, const T &_w)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = _w;
	}
	//operator float *() { return &x;};
	T &operator[](int i) { return ((T *)this)[i]; }
	const T &operator[](int i) const { return ((T *)this)[i]; }
	inline const vec3<T> &xyz() const { return *((vec3<T> *)this); }
	inline vec3<T> &xyz() { return *((vec3<T> *)this); }
};

typedef vec4<float> float4;
typedef vec4<int> int4;
typedef vec4<unsigned char> byte4;

template <class T>
inline int operator==(const vec4<T> &a, const vec4<T> &b)
{
	return (a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w);
}
template <class T>
inline int operator!=(const vec4<T> &a, const vec4<T> &b)
{
	return !(a == b);
}
template <class T>
inline vec4<T> operator+(const vec4<T> &a, const vec4<T> &b)
{
	return vec4<T>(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
template <class T>
inline vec4<T> operator-(const vec4<T> &a, const vec4<T> &b)
{
	return vec4<T>(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}
template <class T>
inline vec4<T> operator-(const vec4<T> &v)
{
	return vec4<T>(-v.x, -v.y, -v.z, -v.w);
}
template <class T>
inline vec4<T> operator*(const vec4<T> &v, const T &s)
{
	return vec4<T>(v.x * s, v.y * s, v.z * s, v.w * s);
}
template <class T>
inline vec4<T> operator*(T s, const vec4<T> &v)
{
	return v * s;
}
template <class T>
inline vec4<T> operator/(const vec4<T> &v, T s)
{
	return vec4<T>(v.x / s, v.y / s, v.z / s, v.w / s);
}
template <class T>
inline T dot(const vec4<T> &a, const vec4<T> &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
template <class T>
inline vec4<T> cmul(const vec4<T> &a, const vec4<T> &b)
{
	return vec4<T>(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}
template <class T>
inline vec4<T> &operator+=(vec4<T> &a, const vec4<T> &b)
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	a.w += b.w;
	return a;
}
template <class T>
inline vec4<T> &operator-=(vec4<T> &a, const vec4<T> &b)
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	a.w -= b.w;
	return a;
}
template <class T>
inline vec4<T> &operator*=(vec4<T> &v, T s)
{
	v.x *= s;
	v.y *= s;
	v.z *= s;
	v.w *= s;
	return v;
}
template <class T>
inline vec4<T> &operator/=(vec4<T> &v, T s)
{
	v.x /= s;
	v.y /= s;
	v.z /= s;
	v.w /= s;
	return v;
}
template <class T>
inline T magnitude(const vec4<T> &v)
{
	return squareroot(dot(v, v));
}
template <class T>
inline vec4<T> normalize(const vec4<T> &v)
{
	return v / magnitude(v);
}

struct D3DXMATRIX;

template <class T>
class mat4x4
{
public:
	vec4<T> x, y, z, w;  // the 4 rows
	inline mat4x4() {}
	inline mat4x4(const vec4<T> &_x, const vec4<T> &_y, const vec4<T> &_z, const vec4<T> &_w) : x(_x), y(_y), z(_z), w(_w) {}
	inline mat4x4(const T &m00, const T &m01, const T &m02, const T &m03,
				  const T &m10, const T &m11, const T &m12, const T &m13,
				  const T &m20, const T &m21, const T &m22, const T &m23,
				  const T &m30, const T &m31, const T &m32, const T &m33)
		: x(m00, m01, m02, m03), y(m10, m11, m12, m13), z(m20, m21, m22, m23), w(m30, m31, m32, m33) {}
	inline vec4<T> &operator[](int i)
	{
		assert(i >= 0 && i < 4);
		return (&x)[i];
	}
	inline const vec4<T> &operator[](int i) const
	{
		assert(i >= 0 && i < 4);
		return (&x)[i];
	}
	inline T &operator()(int r, int c)
	{
		assert(r >= 0 && r < 4 && c >= 0 && c < 4);
		return ((&x)[r])[c];
	}
	inline const T &operator()(int r, int c) const
	{
		assert(r >= 0 && r < 4 && c >= 0 && c < 4);
		return ((&x)[r])[c];
	}
	inline operator T *() { return &x.x; }
	inline operator const T *() const { return &x.x; }
	operator struct D3DXMATRIX *() { return (struct D3DXMATRIX *)this; }
	operator const struct D3DXMATRIX *() const { return (struct D3DXMATRIX *)this; }
};

typedef mat4x4<float> float4x4;

float4x4 operator*(const float4x4 &a, const float4x4 &b);
float4 operator*(const float4 &v, const float4x4 &m);
float4x4 Inverse(const float4x4 &m);
float4x4 MatrixRigidInverse(const float4x4 &m);
float4x4 MatrixTranspose(const float4x4 &m);
float4x4 MatrixPerspectiveFov(float fovy, float Aspect, float zn, float zf);
float4x4 MatrixTranslation(const float3 &t);
float4x4 MatrixRotationZ(const float angle_radians);
float4x4 MatrixLookAt(const float3 &eye, const float3 &at, const float3 &up);
int operator==(const float4x4 &a, const float4x4 &b);

//-------- Quaternion ------------

template <class T>
class quaternion : public vec4<T>
{
public:
	inline quaternion()
	{
		this->x = this->y = this->z = 0.0f;
		this->w = 1.0f;
	}
	inline quaternion(const T &_x, const T &_y, const T &_z, const T &_w)
	{
		this->x = _x;
		this->y = _y;
		this->z = _z;
		this->w = _w;
	}
	inline explicit quaternion(const vec4<T> &v) : vec4<T>(v) {}
	T angle() const { return acosf(this->w) * 2.0f; }
	vec3<T> axis() const
	{
		vec3<T> a(this->x, this->y, this->z);
		if (fabsf(angle()) < 0.0000001f) return vec3<T>(1, 0, 0);
		return a * (1 / sinf(angle() / 2.0f));
	}
	inline vec3<T> xdir() const { return vec3<T>(1 - 2 * (this->y * this->y + this->z * this->z), 2 * (this->x * this->y + this->w * this->z),
												 2 * (this->x * this->z - this->w * this->y)); }
	inline vec3<T> ydir() const { return vec3<T>(2 * (this->x * this->y - this->w * this->z), 1 - 2 * (this->x * this->x + this->z * this->z), 2 * (this->y * this->z + this->w * this->x)); }
	inline vec3<T> zdir() const { return vec3<T>(2 * (this->x * this->z + this->w * this->y),
												 2 * (this->y * this->z - this->w * this->x), 1 - 2 * (this->x * this->x + this->y * this->y)); }
	inline mat3x3<T> getmatrix() const { return mat3x3<T>(xdir(), ydir(), zdir()); }
	//operator float3x3() { return getmatrix(); }
	void Normalize();
};

template <class T>
inline quaternion<T> quatfrommat(const mat3x3<T> &m)
{
	T magw = m[0][0] + m[1][1] + m[2][2];
	T magxy;
	T magzw;
	vec3<T> pre;
	vec3<T> prexy;
	vec3<T> prezw;
	quaternion<T> postxy;
	quaternion<T> postzw;
	quaternion<T> post;
	int wvsz = (magw > m[2][2]);
	magzw = (wvsz) ? magw : m[2][2];
	prezw = (wvsz) ? vec3<T>(1.0f, 1.0f, 1.0f) : vec3<T>(-1.0f, -1.0f, 1.0f);
	postzw = (wvsz) ? quaternion<T>(0.0f, 0.0f, 0.0f, 1.0f) : quaternion<T>(0.0f, 0.0f, 1.0f, 0.0f);
	int xvsy = (m[0][0] > m[1][1]);
	magxy = (xvsy) ? m[0][0] : m[1][1];
	prexy = (xvsy) ? vec3<T>(1.0f, -1.0f, -1.0f) : vec3<T>(-1.0f, 1.0f, -1.0f);
	postxy = (xvsy) ? quaternion<T>(1.0f, 0.0f, 0.0f, 0.0f) : quaternion<T>(0.0f, 1.0f, 0.0f, 0.0f);
	int zwvsxy = (magzw > magxy);
	pre = (zwvsxy) ? prezw : prexy;
	post = (zwvsxy) ? postzw : postxy;

	T t = pre.x * m[0][0] + pre.y * m[1][1] + pre.z * m[2][2] + 1.0f;
	T s = 1 / sqrt(t) * 0.5f;
	quaternion<T> qp;
	qp.x = (pre.y * m[1][2] - pre.z * m[2][1]) * s;
	qp.y = (pre.z * m[2][0] - pre.x * m[0][2]) * s;
	qp.z = (pre.x * m[0][1] - pre.y * m[1][0]) * s;
	qp.w = t * s;
	return qp * post;
}

typedef quaternion<float> Quaternion;

inline Quaternion QuatFromAxisAngle(const float3 &_v, float angle_radians)
{
	float3 v = normalize(_v) * sinf(angle_radians / 2.0f);
	return Quaternion(v.x, v.y, v.z, cosf(angle_radians / 2.0f));
}

template <class T>
inline quaternion<T> Conjugate(const quaternion<T> &q)
{
	return quaternion<T>(-q.x, -q.y, -q.z, q.w);
}
template <class T>
inline quaternion<T> Inverse(const quaternion<T> &q)
{
	return Conjugate(q);
}
template <class T>
inline quaternion<T> normalize(const quaternion<T> &a)
{
	return quaternion<T>(normalize((vec4<T> &)a));
}
template <class T>
inline quaternion<T> &operator*=(quaternion<T> &a, T s)
{
	return (quaternion<T> &)((vec4<T> &)a *= s);
}
template <class T>
inline quaternion<T> operator*(const quaternion<T> &a, float s)
{
	return quaternion<T>((vec4<T> &)a * s);
}
template <class T>
inline quaternion<T> operator+(const quaternion<T> &a, const quaternion<T> &b)
{
	return quaternion<T>((vec4<T> &)a + (vec4<T> &)b);
}
template <class T>
inline quaternion<T> operator-(const quaternion<T> &a, const quaternion<T> &b)
{
	return quaternion<T>((vec4<T> &)a - (vec4<T> &)b);
}
template <class T>
inline quaternion<T> operator-(const quaternion<T> &b)
{
	return quaternion<T>(-(vec4<T> &)b);
}
template <class T>
inline quaternion<T> operator*(const quaternion<T> &a, const quaternion<T> &b)
{
	return quaternion<T>(
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,   //x
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,   //y
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,   //z
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);  //w
}

float3 rotate(const Quaternion &q, const float3 &v);
//float3		operator*( const Quaternion& q, const float3& v );
//float3		operator*( const float3& v, const Quaternion& q );

Quaternion slerp(const Quaternion &a, const Quaternion &b, float t);
Quaternion Interpolate(const Quaternion &q0, const Quaternion &q1, float t);
Quaternion RotationArc(float3 v0, float3 v1);  // returns quat q where q*v0*q^-1=v1
float4x4 MatrixFromQuatVec(const Quaternion &q, const float3 &v);

inline Quaternion QuatFromMat(const float3 &t, const float3 &b, const float3 &n)
{
	return normalize(quatfrommat<float>(float3x3(t, b, n)));
}

//---------------- Pose ------------------

class Pose
{
public:
	float3 position;
	Quaternion orientation;
	Pose() {}
	Pose(const float3 &p, const Quaternion &q) : position(p), orientation(q) {}
	Pose &pose() { return *this; }
	const Pose &pose() const { return *this; }
};

inline float3 operator*(const Pose &a, const float3 &v)
{
	return a.position + rotate(a.orientation, v);
}

inline Pose operator*(const Pose &a, const Pose &b)
{
	return Pose(a.position + rotate(a.orientation, b.position), a.orientation * b.orientation);
}

inline Pose Inverse(const Pose &a)
{
	Quaternion q = Inverse(a.orientation);
	return Pose(rotate(q, -a.position), q);
}

inline Pose slerp(const Pose &p0, const Pose &p1, float t)
{
	return Pose(p0.position * (1.0f - t) + p1.position * t, slerp(p0.orientation, p1.orientation, t));
}

inline float4x4 MatrixFromPose(const Pose &pose)
{
	return MatrixFromQuatVec(pose.orientation, pose.position);
}

//------ Euler Angle -----

Quaternion YawPitchRoll(float yaw, float pitch, float roll);
float Yaw(const Quaternion &q);
float Pitch(const Quaternion &q);
float Roll(const Quaternion &q);
float Yaw(const float3 &v);
float Pitch(const float3 &v);

//------- Plane ----------
class Plane : public float4
{
public:
	float3 &normal() { return xyz(); }
	const float3 &normal() const { return xyz(); }
	float &dist() { return w; }              // distance below origin - the D from plane equasion Ax+By+Cz+D=0
	const float &dist() const { return w; }  // distance below origin - the D from plane equasion Ax+By+Cz+D=0
	Plane(const float3 &n, float d) : float4(n, d) {}
	Plane() { dist() = 0; }
	explicit Plane(const float4 &v) : float4(v) {}
};

Plane Transform(const Plane &p, const float3 &translation, const Quaternion &rotation);

inline Plane PlaneFlip(const Plane &p) { return Plane(-p.normal(), -p.dist()); }
inline int operator==(const Plane &a, const Plane &b) { return (a.normal() == b.normal() && a.dist() == b.dist()); }
inline int coplanar(const Plane &a, const Plane &b) { return (a == b || a == PlaneFlip(b)); }

float3 PlaneLineIntersection(const Plane &plane, const float3 &p0, const float3 &p1);
float3 PlaneProject(const Plane &plane, const float3 &point);
float3 PlanesIntersection(const Plane &p0, const Plane &p1, const Plane &p2);
float3 PlanesIntersection(const Plane *planes, int planes_count, const float3 &seed = float3(0, 0, 0));

int Clip(const Plane &p, const float3 *verts_in, int count, float *verts_out);                                                            // verts_out must be preallocated with sufficient size >= count+1 or more if concave
int ClipPolyPoly(const float3 &normal, const float3 *clipper, int clipper_count, const float3 *verts_in, int in_count, float3 *scratch);  //scratch must be preallocated

//--------- Utility Functions ------

float3 PlaneLineIntersection(const float3 &normal, const float dist, const float3 &p0, const float3 &p1);
float3 LineProject(const float3 &p0, const float3 &p1, const float3 &a);  // projects a onto infinite line p0p1
float LineProjectTime(const float3 &p0, const float3 &p1, const float3 &a);
int BoxInside(const float3 &p, const float3 &bmin, const float3 &bmax);
int BoxIntersect(const float3 &v0, const float3 &v1, const float3 &bmin, const float3 &bmax, float3 *impact);
float DistanceBetweenLines(const float3 &ustart, const float3 &udir, const float3 &vstart, const float3 &vdir, float3 *upoint = NULL, float3 *vpoint = NULL);
float3 TriNormal(const float3 &v0, const float3 &v1, const float3 &v2);
float3 NormalOf(const float3 *vert, const int n);
Quaternion VirtualTrackBall(const float3 &cop, const float3 &cor, const float3 &dir0, const float3 &dir1);
int Clip(const float3 &plane_normal, float plane_dist, const float3 *verts_in, int count, float *verts_out);                              // verts_out must be preallocated with sufficient size >= count+1 or more if concave
int ClipPolyPoly(const float3 &normal, const float3 *clipper, int clipper_count, const float3 *verts_in, int in_count, float3 *scratch);  //scratch must be preallocated
float3 Diagonal(const float3x3 &M);
Quaternion Diagonalizer(const float3x3 &A);
float3 Orth(const float3 &v);
int SolveQuadratic(float a, float b, float c, float *ta, float *tb);  // if true returns roots ta,tb where ta<=tb
int HitCheckPoly(const float3 *vert, const int n, const float3 &v0, const float3 &v1, float3 *impact = NULL, float3 *normal = NULL);
int HitCheckRaySphere(const float3 &sphereposition, float radius, const float3 &_v0, const float3 &_v1, float3 *impact, float3 *normal);
int HitCheckRayCylinder(const float3 &p0, const float3 &p1, float radius, const float3 &_v0, const float3 &_v1, float3 *impact, float3 *normal);
int HitCheckSweptSphereTri(const float3 &p0, const float3 &p1, const float3 &p2, float radius, const float3 &v0, const float3 &_v1, float3 *impact, float3 *normal);
void BoxLimits(const float3 *verts, int verts_count, float3 &bmin_out, float3 &bmax_out);
void BoxLimits(const float4 *verts, int verts_count, float3 &bmin_out, float3 &bmax_out);

template <class T>
inline int maxdir(const T *p, int count, const T &dir)
{
	assert(count);
	int m = 0;
	for (int i = 1; i < count; i++)
	{
		if (dot(p[i], dir) > dot(p[m], dir)) m = i;
	}
	return m;
}

float3 CenterOfMass(const float3 *vertices, const int3 *tris, const int count);
float3x3 Inertia(const float3 *vertices, const int3 *tris, const int count, const float3 &com = float3(0, 0, 0));
float Volume(const float3 *vertices, const int3 *tris, const int count);
int calchull(float3 *verts, int verts_count, int3 *&tris_out, int &tris_count, int vlimit);  // computes convex hull see hull.cpp

#endif  // VEC_MATH_H
