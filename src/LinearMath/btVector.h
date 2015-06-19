/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_VECTOR3_H
#define BT_VECTOR3_H

//#include <stdint.h>
#include <string.h>//for memset()
#include "btScalar.h"
#include "btMinMax.h"
#include "btAlignedAllocator.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define btVector3Data btVector3DoubleData
#define btVector3DataName "btVector3DoubleData"
#define btVectorData btVectorDoubleData
#define btVectorDataName "btVectorDoubleData"
#else
#define btVector3Data btVector3FloatData
#define btVector3DataName "btVector3FloatData"
#define btVectorData btVectorFloatData
#define btVectorDataName "btVectorFloatData"
#endif //BT_USE_DOUBLE_PRECISION

#if defined BT_USE_SSE

//typedef  uint32_t __m128i __attribute__ ((vector_size(16)));

#ifdef _MSC_VER
#pragma warning(disable: 4556) // value of intrinsic immediate argument '4294967239' is out of range '0 - 255'
#endif


#define BT_SHUFFLE(x,y,z,w) ((w)<<6 | (z)<<4 | (y)<<2 | (x))
//#define bt_pshufd_ps( _a, _mask ) (__m128) _mm_shuffle_epi32((__m128i)(_a), (_mask) )
#define bt_pshufd_ps( _a, _mask ) _mm_shuffle_ps((_a), (_a), (_mask) )
#define bt_splat3_ps( _a, _i ) bt_pshufd_ps((_a), BT_SHUFFLE(_i,_i,_i, 3) )
#define bt_splat_ps( _a, _i )  bt_pshufd_ps((_a), BT_SHUFFLE(_i,_i,_i,_i) )

#define btv3AbsiMask (_mm_set_epi32(0x00000000, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))
#define btvAbsMask (_mm_set_epi32( 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))
#define btvFFF0Mask (_mm_set_epi32(0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))
#define btvFFFFMask (_mm_set_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))
#define btv3AbsfMask btCastiTo128f(btv3AbsiMask)
#define btvFFF0fMask btCastiTo128f(btvFFF0Mask)
#define btvxyzMaskf btvFFF0fMask
#define btvFFFFfMask btCastiTo128f(btvFFFFMask)
#define btvxyzwMaskf btvFFFFfMask
#define btvAbsfMask btCastiTo128f(btvAbsMask)

//there is an issue with XCode 3.2 (LCx errors)
#define btvMzeroMask (_mm_set_ps(-0.0f, -0.0f, -0.0f, -0.0f))
#define v1110		 (_mm_set_ps(0.0f, 1.0f, 1.0f, 1.0f))
#define vHalf		 (_mm_set_ps(0.5f, 0.5f, 0.5f, 0.5f))
#define v1_5		 (_mm_set_ps(1.5f, 1.5f, 1.5f, 1.5f))

//const __m128 ATTRIBUTE_ALIGNED16(btvMzeroMask) = {-0.0f, -0.0f, -0.0f, -0.0f};
//const __m128 ATTRIBUTE_ALIGNED16(v1110) = {1.0f, 1.0f, 1.0f, 0.0f};
//const __m128 ATTRIBUTE_ALIGNED16(vHalf) = {0.5f, 0.5f, 0.5f, 0.5f};
//const __m128 ATTRIBUTE_ALIGNED16(v1_5)  = {1.5f, 1.5f, 1.5f, 1.5f};

#endif

#ifdef BT_USE_NEON

const float32x4_t ATTRIBUTE_ALIGNED16(btvMzeroMask) = (float32x4_t){-0.0f, -0.0f, -0.0f, -0.0f};
const int32x4_t ATTRIBUTE_ALIGNED16(btvFFF0Mask) = (int32x4_t){static_cast<int32_t>(0xFFFFFFFF),
	static_cast<int32_t>(0xFFFFFFFF), static_cast<int32_t>(0xFFFFFFFF), 0x0};
const int32x4_t ATTRIBUTE_ALIGNED16(btvAbsMask) = (int32x4_t){0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF};
const int32x4_t ATTRIBUTE_ALIGNED16(btv3AbsMask) = (int32x4_t){0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x0};

#endif

// A mini-version of btVector for non-C++ language.
#ifdef __cplusplus
ATTRIBUTE_ALIGNED16(struct) btVector
#else
typedef ATTRIBUTE_ALIGNED16(struct)
#endif
{
#if defined (__SPU__) && defined (__CELLOS_LV2__)
		btScalar	m_floats[4];

#ifdef __cplusplus
	SIMD_FORCE_INLINE const vec_float4&	get128() const
	{
		return *((const vec_float4*)&m_floats[0]);
	}
#endif

#else //__CELLOS_LV2__ __SPU__
    #if defined (BT_USE_SSE) || defined(BT_USE_NEON) // _WIN32 || ARM
        union {
            btSimdFloat4      mVec128;
            btScalar	m_floats[4];
        };
        
#ifdef __cplusplus
        SIMD_FORCE_INLINE	btSimdFloat4	get128() const
        {
            return mVec128;
        }
        SIMD_FORCE_INLINE	void	set128(btSimdFloat4 v128)
        {
            mVec128 = v128;
        }
#endif

    #else
        btScalar	m_floats[4];
    #endif
#endif //__CELLOS_LV2__ __SPU__

// C++ only function, constructors and operators
#ifdef __cplusplus
  /**@brief No initialization constructor */
	SIMD_FORCE_INLINE btVector() 
	{

	}
	
  /**@brief Constructor from scalars 
   * @param x X value
   * @param y Y value 
   * @param z Z value 
   */
	SIMD_FORCE_INLINE btVector(const btScalar& _x, const btScalar& _y, const btScalar& _z, const btScalar& _w)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = _w;
	}

#if (defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE) )|| defined (BT_USE_NEON)
	// Set Vector 
	SIMD_FORCE_INLINE btVector(btSimdFloat4 v)
	{
		mVec128 = v;
	}

	// Notice: Copy constructor and assignment Operator aren't needed, since the
	//  union contain only POD members, so it's a trivial bitwise copy/assignment.
	// Proof: (under Implicitly-defined copy constructor)
	//  http://en.cppreference.com/w/cpp/language/copy_constructor
#endif // #if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)

	SIMD_FORCE_INLINE bool	operator==(const btVector& other) const;
	
	SIMD_FORCE_INLINE bool	operator!=(const btVector& other) const
	{
		return !(*this == other);
	}

  /**@brief Return the x value */
	SIMD_FORCE_INLINE const btScalar& getX() const { return m_floats[0]; }
  /**@brief Return the y value */
	SIMD_FORCE_INLINE const btScalar& getY() const { return m_floats[1]; }
  /**@brief Return the z value */
	SIMD_FORCE_INLINE const btScalar& getZ() const { return m_floats[2]; }
  /**@brief Return the w value */
	SIMD_FORCE_INLINE const btScalar& getW() const { return m_floats[3]; }
	
  /**@brief Set the x value */
	SIMD_FORCE_INLINE void	setX(btScalar _x) { m_floats[0] = _x; }
  /**@brief Set the y value */
	SIMD_FORCE_INLINE void	setY(btScalar _y) { m_floats[1] = _y; }
  /**@brief Set the z value */
	SIMD_FORCE_INLINE void	setZ(btScalar _z) { m_floats[2] = _z; }
  /**@brief Set the w value */
	SIMD_FORCE_INLINE void	setW(btScalar _w) { m_floats[3] = _w; }
	
  /**@brief Return the x value */
	SIMD_FORCE_INLINE const btScalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
	SIMD_FORCE_INLINE const btScalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
	SIMD_FORCE_INLINE const btScalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
	SIMD_FORCE_INLINE const btScalar& w() const { return m_floats[3]; }

	//SIMD_FORCE_INLINE btScalar&       operator[](int i)       { return (&m_floats[0])[i];	}      
	//SIMD_FORCE_INLINE const btScalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator btScalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
	SIMD_FORCE_INLINE	operator       btScalar *()       { return &m_floats[0]; }
	SIMD_FORCE_INLINE	operator const btScalar *() const { return &m_floats[0]; }

  /**@brief Set each element to the max of the current values and the values of another btVector
   * @param other The other btVector to compare with 
   */
	SIMD_FORCE_INLINE void	setMax(const btVector& other);

  /**@brief Set each element to the min of the current values and the values of another btVector
   * @param other The other btVector to compare with 
   */
	SIMD_FORCE_INLINE void	setMin(const btVector& other);

	SIMD_FORCE_INLINE void 	setValue(const btScalar& _x, const btScalar& _y, const btScalar& _z, const btScalar& _w)
	{
		m_floats[0]=_x;
		m_floats[1]=_y;
		m_floats[2]=_z;
		m_floats[3]=_w;
	}
	
	SIMD_FORCE_INLINE void setZero();

	SIMD_FORCE_INLINE bool isZero() const;

	SIMD_FORCE_INLINE bool fuzzyZero() const ;

	// Serialization functions.
	// TODO: Get rid somehow from the "3" number in the names.
	SIMD_FORCE_INLINE	void	serialize(struct	btVector3Data& dataOut) const;

	SIMD_FORCE_INLINE	void	deSerialize(const struct	btVector3Data& dataIn);

	SIMD_FORCE_INLINE	void	serializeFloat(struct	btVector3FloatData& dataOut) const;

	SIMD_FORCE_INLINE	void	deSerializeFloat(const struct	btVector3FloatData& dataIn);

	SIMD_FORCE_INLINE	void	serializeDouble(struct	btVector3DoubleData& dataOut) const;

	SIMD_FORCE_INLINE	void	deSerializeDouble(const struct	btVector3DoubleData& dataIn);

#endif
// end of C++ only function, constructors and operators

#ifdef __cplusplus
};

#else
} btVector;

typedef btVector btVector3;
typedef btVector btVector4;
#endif

typedef enum {
	BT_VEC3_MODE = 0,
	BT_VEC4_MODE = 1
} btVectorMode;

// btVector functions
static SIMD_FORCE_INLINE btVector btVector_create(btScalar _x, btScalar _y, btScalar _z, btScalar _w) {
	btVector result;
	
	result.m_floats[0] = _x;
	result.m_floats[1] = _y;
	result.m_floats[2] = _z;
	result.m_floats[3] = _w;
	
	return result;
}

#ifndef __cplusplus
// Fake constructor for non-C++
#define btVector(x, y, z, w) btVector_create(x, y, z, w)

#define btVector3(x, y, z) btVector(x, y, z, 0)
#define btVector4(x, y, z, w) btVector(x, y, z, w)

#endif

#define btVector_copy(target, src) memcpy(target, src, sizeof(btVector))

#if (defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE) )|| defined (BT_USE_NEON)
static SIMD_FORCE_INLINE btVector btVector_fromSimd(btSimdFloat4 v) {
	btVector result;
	result.mVec128 = v;
	return result;
}
#endif // #if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)

static SIMD_FORCE_INLINE btBool btVector_cmp(const btVector* v1, const btVector* v2) {
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
    return (0xf == _mm_movemask_ps((__m128)_mm_cmpeq_ps(v1->mVec128, v2->mVec128)));
#else 
	return ((v1->m_floats[3]==v2->m_floats[3]) && 
            (v1->m_floats[2]==v2->m_floats[2]) && 
            (v1->m_floats[1]==v2->m_floats[1]) && 
            (v1->m_floats[0]==v2->m_floats[0]));
#endif
}

static SIMD_FORCE_INLINE void btVector_add(btVector* BT_RESTRICT self, const btVector* BT_RESTRICT v, btVectorMode mode) {
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	self->mVec128 = _mm_add_ps(self->mVec128, vec->mVec128);
#elif defined(BT_USE_NEON)
	self->mVec128 = vaddq_f32(self->mVec128, vec->mVec128);
#else
	self->m_floats[0] += v->m_floats[0];
	self->m_floats[1] += v->m_floats[1];
	self->m_floats[2] += v->m_floats[2];
	if (mode == BT_VEC4_MODE)
		self->m_floats[3] += v->m_floats[3];
#endif
}

static SIMD_FORCE_INLINE void btVector_subtract(btVector* BT_RESTRICT self, const btVector* BT_RESTRICT v, btVectorMode mode) {
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	self->mVec128 = _mm_sub_ps(self->mVec128, vec->mVec128);
#elif defined(BT_USE_NEON)
	self->mVec128 = vsubq_f32(self->mVec128, vec->mVec128);
#else
	self->m_floats[0] -= v->m_floats[0];
	self->m_floats[1] -= v->m_floats[1];
	self->m_floats[2] -= v->m_floats[2];
	if (mode == BT_VEC4_MODE)
		self->m_floats[3] -= v->m_floats[3];
#endif
}

static SIMD_FORCE_INLINE void btVector_scale(btVector* self, btScalar s, btVectorMode mode) {
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	__m128	vs = _mm_load_ss(&s);	//	(S 0 0 0)
	vs = bt_pshufd_ps(vs, 0x00);	//	(S S S S)
	mVec128 = _mm_mul_ps(self->mVec128, vs);
#elif defined(BT_USE_NEON)
	mVec128 = vmulq_n_f32(self->mVec128, s);
#else
	self->m_floats[0] *= s;
	self->m_floats[1] *= s;
	self->m_floats[2] *= s;
	if (mode == BT_VEC4_MODE)
		self->m_floats[3] *= s;
#endif
}

static SIMD_FORCE_INLINE void btVector_divide(btVector* self, btScalar s, btVectorMode mode) {
	btFullAssert(s != btScalar(0.0));

#if (defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)) || defined(BT_USE_NEON)
	btVector_scale(self, btScalar(1.0) / s, mode);
#else
	self->m_floats[0] /= s;
	self->m_floats[1] /= s;
	self->m_floats[2] /= s;
	if (mode == BT_VEC4_MODE)
		self->m_floats[3] /= s;
#endif
}

/**@brief Elementwise multiply this vector by the other 
 * @param v The other vector */
static SIMD_FORCE_INLINE void btVector_multiply(btVector* BT_RESTRICT self, const btVector* BT_RESTRICT v, btVectorMode mode)
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	mVec128 = _mm_mul_ps(self->mVec128, v->mVec128);
#elif defined(BT_USE_NEON)
	mVec128 = vmulq_f32(selfmVec128, v->mVec128);
#else	
	self->m_floats[0] *= v->m_floats[0]; 
	self->m_floats[1] *= v->m_floats[1];
	self->m_floats[2] *= v->m_floats[2];
	if (mode == BT_VEC4_MODE)
		self->m_floats[3] *= v->m_floats[3];
#endif
}

static SIMD_FORCE_INLINE btScalar btVector_dot(const btVector* a, const btVector* b, btVectorMode mode) {
#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	// First, multiply each component
	__m128 vd = _mm_mul_ps(a->mVec128, b->mVec128);
	
	// We sum the component in the lower 32 bits
	__m128 t = _mm_movehl_ps(vd, vd);
	vd = _mm_add_ps(vd, t);
	t = _mm_shuffle_ps(vd, vd, 0x55);
	vd = _mm_add_ss(vd, t);
	
	return _mm_cvtss_f32(vd);
#elif defined(BT_USE_NEON)
	float32x4_t vd = vmulq_f32(a->mVec128, b->mVec128);
	float32x2_t x = vpadd_f32(vget_low_f32(vd), vget_high_f32(vd));  
	x = vpadd_f32(x, x);
	return vget_lane_f32(x, 0);
#else
	double sum =
		a->m_floats[0] * b->m_floats[0] +
		a->m_floats[1] * b->m_floats[1] +
		a->m_floats[2] * b->m_floats[2];
	if (mode == BT_VEC4_MODE)
		sum += a->m_floats[3] * b->m_floats[3];
	
	return sum;
#endif
}

// We can't put here as macro, since the "self" parameter would be executed twice.
static SIMD_FORCE_INLINE btScalar btVector_length2(const btVector* self, btVectorMode mode) {
	return btVector_dot(self, self, mode);
}

#define btVector_length(self, mode) btSqrt(btVector_length2(self, mode))

#define btVector_norm(self, mode) btVector_length(self, mode)

static SIMD_FORCE_INLINE btVector btVector_diff(const btVector* a, const btVector* b, btVectorMode mode)
{
	#if defined BT_USE_SIMD_VECTOR3 && (defined(BT_USE_SSE_IN_API)  && defined(BT_USE_SSE))
		//	without _mm_and_ps this code causes slowdown in Concave moving
		__m128 r = _mm_sub_ps(a->mVec128, b->mVec128);
		if (mode == BT_VEC3_MODE)
			return btVector_fromSimd(_mm_and_ps(r, btvxyzMaskf));
		else
			return btVector_fromSimd(r);
	#elif defined(BT_USE_NEON)
		float32x4_t r = vsubq_f32(a->mVec128, b->mVec128);
		if (mode == BT_VEC3_MODE)
			return btVector_fromSimd((float32x4_t)vandq_s32(r, btvxyzMaskf));
		else
			return btVector_fromSimd(r);
	#else
		return btVector(
				a->m_floats[0] - b->m_floats[0], 
				a->m_floats[1] - b->m_floats[1], 
				a->m_floats[2] - b->m_floats[2],
				(mode == BT_VEC4_MODE) ? (a->m_floats[3] - b->m_floats[3]) : 0);
	#endif
}

static SIMD_FORCE_INLINE btScalar btVector_distance2(const btVector* a, const btVector* b, btVectorMode mode) {
	btVector diff = btVector_diff(a, b, mode);
	return btVector_length2(&diff, mode);
}

#define btVector_distance(a, b, mode) btSqrt(btVector_distance2(a, b, mode))
	
/**@brief Return a vector will the absolute values of each element */
static SIMD_FORCE_INLINE btVector btVector_absolute(const btVector* self, btVectorMode mode)
{
#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE) 
	return btVector_fromSimd(_mm_and_ps(self->mVec128, (mode == BT_VEC4_MODE) ? btvAbsMask : btv3AbsfMask));
#elif defined(BT_USE_NEON)
	return btVector_fromSimd(vabsq_f32(self->mVec128));
#else	
	return btVector(
		btFabs(self->m_floats[0]), 
		btFabs(self->m_floats[1]), 
		btFabs(self->m_floats[2]),
		(mode == BT_VEC4_MODE) ? btFabs(self->m_floats[3]) : 0);
#endif
}

/**@brief Return the axis with the smallest value 
* Note return values are 0,1,2,3 for x, y, z or w */
static SIMD_FORCE_INLINE int btVector_minAxis(const btVector* self, btVectorMode mode)
{
	const int min3 = self->m_floats[0] < self->m_floats[1] ?
		(self->m_floats[0] < self->m_floats[2] ? 0 : 2) :
		(self->m_floats[1] < self->m_floats[2] ? 1 : 2);
	
	if (mode == BT_VEC3_MODE)
		return min3;
	else
		return (self->m_floats[min3] < self->m_floats[3]) ? min3 : 3;
}

/**@brief Return the axis with the largest value 
* Note return values are 0,1,2,3 for x, y, z or w */
static SIMD_FORCE_INLINE int btVector_maxAxis(const btVector* self, btVectorMode mode)
{
	const int max3 = self->m_floats[0] < self->m_floats[1] ?
		(self->m_floats[1] < self->m_floats[2] ? 2 : 1) :
		(self->m_floats[0] < self->m_floats[2] ? 2 : 0);
	
	if (mode == BT_VEC3_MODE)
		return max3;
	else
		return (self->m_floats[max3] < self->m_floats[3]) ? 3 : max3;
}

static SIMD_FORCE_INLINE int btVector_furthestAxis(const btVector* self, btVectorMode mode) {
	btVector abs = btVector_absolute(self, mode);
	return btVector_minAxis(&abs, mode);
}

static SIMD_FORCE_INLINE int btVector_closestAxis(const btVector* self, btVectorMode mode) {
	btVector abs = btVector_absolute(self, mode);
	return btVector_maxAxis(&abs, mode);
}

static SIMD_FORCE_INLINE void btVector_safeNormalize(btVector* self, btVectorMode mode) 
{
	btVector absVec = btVector_absolute(self, mode);
	int maxIndex = btVector_maxAxis(&absVec, mode);
	btScalar maxValue = absVec.m_floats[maxIndex];
	if (maxValue > 0)
	{
		btVector_divide(self, maxValue, mode);
		btScalar length = btVector_length(self, mode);
		
		btVector_divide(self, length, mode);
	} else {
		self->m_floats[0] = 1;
		self->m_floats[1] = 0;
		self->m_floats[2] = 0;
		if (mode == BT_VEC4_MODE)
			self->m_floats[3] = 0;
	}
}

#define btVector_isZero(self, mode) \
	(self->m_floats[0] == btScalar(0) && \
	self->m_floats[1] == btScalar(0) && \
	self->m_floats[2] == btScalar(0) && \
	((mode == BT_VEC4_MODE) ? self->m_floats[2] == btScalar(0) : btTrue))


#define btVector_fuzzyZero(self, mode) \
	(btVector_length2(self, mode) < SIMD_EPSILON*SIMD_EPSILON)

/**@brief Return the angle between this and another vector
 * @param v The other vector */
SIMD_FORCE_INLINE btScalar btVector_angle(const btVector* v1, const btVector* v2, btVectorMode mode)
{
	btScalar s = btSqrt(btVector_length2(v1, mode) * btVector_length2(v2, mode));
	btFullAssert(s != btScalar(0.0));
	return btAcos(btVector_dot(v1, v2, mode) / s);
}

/**@brief Normalize this vector 
* x^2 + y^2 + z^2 + w^2 = 1 */
static SIMD_FORCE_INLINE void btVector_normalize(btVector* self, btVectorMode mode) 
{
	
	btAssert(!btVector_fuzzyZero(self, mode));

#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	if (mode == BT_VEC4_MODE)
	{
		__m128	vd;
		
		vd = _mm_mul_ps(self->mVec128, self->mVec128);
		
        __m128 t = _mm_movehl_ps(vd, vd);
		vd = _mm_add_ps(vd, t);
		t = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, t);

		vd = _mm_sqrt_ss(vd);
		vd = _mm_div_ss(vOnes, vd);
        vd = bt_pshufd_ps(vd, 0); // splat
		self->mVec128 = _mm_mul_ps(self->mVec128, vd);
	}
	else
	{
		// dot product first
		__m128 vd = _mm_mul_ps(self->mVec128, self->mVec128);
		__m128 z = _mm_movehl_ps(vd, vd);
		__m128 y = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, y);
		vd = _mm_add_ss(vd, z);
	
		#if 0
		vd = _mm_sqrt_ss(vd);
		vd = _mm_div_ss(v1110, vd);
		vd = bt_splat_ps(vd, 0x80);
		self->mVec128 = _mm_mul_ps(self->mVec128, vd);
		#else
		
		// NR step 1/sqrt(x) - vd is x, y is output 
		y = _mm_rsqrt_ss(vd); // estimate 
		
		//  one step NR 
		z = v1_5;
		vd = _mm_mul_ss(vd, vHalf); // vd * 0.5	
		//x2 = vd;
		vd = _mm_mul_ss(vd, y); // vd * 0.5 * y0
		vd = _mm_mul_ss(vd, y); // vd * 0.5 * y0 * y0
		z = _mm_sub_ss(z, vd);  // 1.5 - vd * 0.5 * y0 * y0 

		y = _mm_mul_ss(y, z);   // y0 * (1.5 - vd * 0.5 * y0 * y0)

		y = bt_splat_ps(y, 0x80);
		self->mVec128 = _mm_mul_ps(self->mVec128, y);

		#endif
    }
#else
	btVector_divide(self, btVector_length(self, mode), mode);
#endif
}

/**@brief Return a normalized version of this vector */
static SIMD_FORCE_INLINE btVector btVector_normalized(const btVector* self, btVectorMode mode)
{
	btVector nrm;
	btVector_copy(&nrm, &self);
	
	btVector_normalize(&nrm, mode);

	return nrm;
}

static SIMD_FORCE_INLINE void btVector_setInterpolate3(btVector* self, const btVector* v0, const btVector* v1, btScalar rt, btVectorMode mode)
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	const unsigned char mask = (mode == BT_VEC3_MODE) ? 0x80 : 0x00;
	if (mode == BT_VEC3_MODE)
	__m128	vrt = _mm_load_ss(&rt);	//	(rt 0 0 0)
	btScalar s = btScalar(1.0) - rt;
	__m128	vs = _mm_load_ss(&s);	//	(S 0 0 0)
	vs = bt_pshufd_ps(vs, mask);	//	(S S S 0.0) or (S S S S)
	__m128 r0 = _mm_mul_ps(v0->mVec128, vs);
	vrt = bt_pshufd_ps(vrt, mask);	//	(rt rt rt 0.0) or (rt rt rt rt)
	__m128 r1 = _mm_mul_ps(v1->mVec128, vrt);
	self->mVec128 = _mm_add_ps(r0,r1);
#elif defined(BT_USE_NEON)
	float32x4_t vl = vsubq_f32(v1->mVec128, v0->mVec128);
	vl = vmulq_n_f32(vl, rt);
	self->mVec128 = vaddq_f32(vl, v0->mVec128);
#else	
	btScalar s = btScalar(1.0) - rt;
	self->m_floats[0] = s * v0->m_floats[0] + rt * v1->m_floats[0];
	self->m_floats[1] = s * v0->m_floats[1] + rt * v1->m_floats[1];
	self->m_floats[2] = s * v0->m_floats[2] + rt * v1->m_floats[2];
	if (mode == BT_VEC4_MODE)
		self->m_floats[3] = s * v0->m_floats[3] + rt * v1->m_floats[3];
#endif
}

/**@brief Return the linear interpolation between this and another vector 
 * @param v The other vector 
 * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
static SIMD_FORCE_INLINE btVector btVector_lerp(const btVector* self, const btVector* v, const btScalar t, btVectorMode mode)
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	const unsigned char mask = (mode == BT_VEC3_MODE) ? 0x80 : 0x00;
	__m128	vt = _mm_load_ss(&t);	//	(t 0 0 0)
	vt = bt_pshufd_ps(vt, mask);	//	(t t t 0.0) or (t t t t)
	__m128 vl = _mm_sub_ps(v->mVec128, self->mVec128);
	vl = _mm_mul_ps(vl, vt);
	vl = _mm_add_ps(vl, self->mVec128);
	
	return btVector(vl);
#elif defined(BT_USE_NEON)
	float32x4_t vl = vsubq_f32(v->mVec128, self->mVec128);
	vl = vmulq_n_f32(vl, t);
	vl = vaddq_f32(vl, self->mVec128);
	
	return btVector(vl);
#else
	return
		btVector(self->m_floats[0] + (v->m_floats[0] - self->m_floats[0]) * t,
				self->m_floats[1] + (v->m_floats[1] - self->m_floats[1]) * t,
				self->m_floats[2] + (v->m_floats[2] - self->m_floats[2]) * t,
				(mode == BT_VEC4_MODE) ? (self->m_floats[3] + (v->m_floats[3] - self->m_floats[3]) * t) : 0);
#endif
}

/**@brief Set each element to the max of the current values and the values of another btVector3
 * @param other The other btVector3 to compare with 
*/
static SIMD_FORCE_INLINE void btVector_setMax(btVector* BT_RESTRICT self, const btVector* BT_RESTRICT other, btVectorMode mode)
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	self->mVec128 = _mm_max_ps(self->mVec128, other->mVec128);
#elif defined(BT_USE_NEON)
	self->mVec128 = vmaxq_f32(self->mVec128, other->mVec128);
#else
	btSetMax_unsafe(self->m_floats[0], other->m_floats[0]);
	btSetMax_unsafe(self->m_floats[1], other->m_floats[1]);
	btSetMax_unsafe(self->m_floats[2], other->m_floats[2]);
	if (mode == BT_VEC4_MODE)
		btSetMax_unsafe(self->m_floats[3], other->m_floats[3]);
#endif
}

/**@brief Set each element to the min of the current values and the values of another btVector3
 * @param other The other btVector3 to compare with 
*/
static SIMD_FORCE_INLINE void btVector_setMin(btVector* BT_RESTRICT self, const btVector* BT_RESTRICT other, btVectorMode mode)
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	self->mVec128 = _mm_min_ps(self->mVec128, other->mVec128);
#elif defined(BT_USE_NEON)
	self->Vec128 = vminq_f32(self->mVec128, other->mVec128);
#else
	btSetMin_unsafe(self->m_floats[0], other->m_floats[0]);
	btSetMin_unsafe(self->m_floats[1], other->m_floats[1]);
	btSetMin_unsafe(self->m_floats[2], other->m_floats[2]);
	if (mode == BT_VEC4_MODE)
		btSetMin_unsafe(self->m_floats[3], other->m_floats[3]);
#endif
}

#define btVector_setValue(self, x, y, z, w) \
	self->m_floats[0] = x; \
	self->m_floats[1] = y; \
	self->m_floats[2] = z; \
	self->m_floats[3] = w

// btVector_setZero
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
#define btVector_setZero(self) self->mVec128 = (__m128)_mm_xor_ps(self->mVec128, self->mVec128)
#elif defined(BT_USE_NEON)
#define btVector_setZero(self) self->mVec128 = vreinterpretq_f32_s32(vdupq_n_s32(0));
#else	
#define btVector_setZero(self) memset(self, 0, sizeof(btVector))
#endif


// Serialization

typedef struct btVector3FloatData// Named struct for compatibility
{
	float	m_floats[4];
} btVectorFloatData;


typedef struct btVector3DoubleData// Named struct for compatibility
{
	double	m_floats[4];
} btVectorDoubleData;


static SIMD_FORCE_INLINE void btVector_serializeFloat(const btVector* BT_RESTRICT self, btVectorFloatData* BT_RESTRICT dataOut)
{
	///could also do a memcpy, check if it is worth it
	int i;
	for (i=0;i<4;i++)
		dataOut->m_floats[i] = (float)self->m_floats[i];
}

static SIMD_FORCE_INLINE void btVector_deSerializeFloat(btVector* BT_RESTRICT self, const btVectorFloatData* BT_RESTRICT dataIn)
{
	int i;
	for (i=0;i<4;i++)
		self->m_floats[i] = (btScalar)dataIn->m_floats[i];
}


static SIMD_FORCE_INLINE void btVector_serializeDouble(const btVector* BT_RESTRICT self, btVectorDoubleData* BT_RESTRICT dataOut)
{
	///could also do a memcpy, check if it is worth it
	int i;
	for (i=0;i<4;i++)
		dataOut->m_floats[i] = (double)self->m_floats[i];
}

static SIMD_FORCE_INLINE void btVector_deSerializeDouble(btVector* BT_RESTRICT self, const btVectorDoubleData* BT_RESTRICT dataIn)
{
	int i;
	for (i=0;i<4;i++)
		self->m_floats[i] = (btScalar)dataIn->m_floats[i];
}


static SIMD_FORCE_INLINE void btVector_serialize(const btVector* BT_RESTRICT self, btVectorData* BT_RESTRICT dataOut)
{
	///could also do a memcpy, check if it is worth it
	int i;
	for (i=0;i<4;i++)
		dataOut->m_floats[i] = self->m_floats[i];
}

static SIMD_FORCE_INLINE void btVector_deSerialize(btVector* BT_RESTRICT self, const btVectorData* BT_RESTRICT dataIn)
{
	int i;
	for (i=0;i<4;i++)
		self->m_floats[i] = dataIn->m_floats[i];
}


#ifdef __cplusplus
SIMD_FORCE_INLINE bool	btVector::operator==(const btVector& other) const
{
	return btVector_cmp(this, &other);
}

SIMD_FORCE_INLINE void	btVector::setMax(const btVector& other)
{
	btVector_setMax(this, &other, BT_VEC4_MODE);
}

SIMD_FORCE_INLINE void	btVector::setMin(const btVector& other)
{
	btVector_setMin(this, &other, BT_VEC4_MODE);
}

SIMD_FORCE_INLINE void btVector::setZero()
{
	btVector_setZero(this);
}

SIMD_FORCE_INLINE bool btVector::isZero() const 
{
	return btVector_isZero(this, BT_VEC4_MODE);
}

SIMD_FORCE_INLINE bool btVector::fuzzyZero() const 
{
	return btVector_fuzzyZero(this, BT_VEC4_MODE);
}

SIMD_FORCE_INLINE	void	btVector::serializeFloat(btVectorFloatData& dataOut) const
{
	btVector_serializeFloat(this, &dataOut);
}

SIMD_FORCE_INLINE void	btVector::deSerializeFloat(const btVectorFloatData& dataIn)
{
	btVector_deSerializeFloat(this, &dataIn);
}


SIMD_FORCE_INLINE	void	btVector::serializeDouble(btVectorDoubleData& dataOut) const
{
	btVector_serializeDouble(this, &dataOut);
}

SIMD_FORCE_INLINE void	btVector::deSerializeDouble(const btVectorDoubleData& dataIn)
{
	btVector_deSerializeDouble(this, &dataIn);
}


SIMD_FORCE_INLINE	void	btVector::serialize(btVectorData& dataOut) const
{
	btVector_serialize(this, &dataOut);
}

SIMD_FORCE_INLINE void	btVector::deSerialize(const btVectorData& dataIn)
{
	btVector_deSerialize(this, &dataIn);
}
#endif//__cplusplus


/**@brief btVector3 can be used to represent 3D points and vectors.
 * It has an un-used w component to suit 16-byte alignment when btVector3 is stored in containers. This extra component can be used by derived classes (Quaternion?) or by user
 * Ideally, this class should be replaced by a platform optimized SIMD version that keeps the data in registers
 */
#ifdef __cplusplus
ATTRIBUTE_ALIGNED16(class) btVector3 : public btVector
{
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

  /**@brief No initialization constructor */
	SIMD_FORCE_INLINE btVector3() 
	{

	}

 
	
  /**@brief Constructor from scalars 
   * @param x X value
   * @param y Y value 
   * @param z Z value 
   */
	SIMD_FORCE_INLINE btVector3(const btScalar& _x, const btScalar& _y, const btScalar& _z) :
		btVector(_x, _y, _z, 0.f)
	{
	}

#if (defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE) )|| defined (BT_USE_NEON)
	// Set Vector 
	SIMD_FORCE_INLINE btVector3( btSimdFloat4 v) : btVector(v)
	{
	}

	SIMD_FORCE_INLINE btVector3(const btVector& rhs)
	{
		mVec128 = rhs.mVec128;
	}
#else
	SIMD_FORCE_INLINE btVector3(const btVector& v) : btVector(v) {
	}
#endif // #if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON) 
    
/**@brief Add a vector to this one 
 * @param The vector to add to this one */
	SIMD_FORCE_INLINE btVector3& operator+=(const btVector3& v)
	{
		btVector_add(this, &v, BT_VEC3_MODE);
		
		return *this;
	}


  /**@brief Subtract a vector from this one
   * @param The vector to subtract */
	SIMD_FORCE_INLINE btVector3& operator-=(const btVector3& v) 
	{
		btVector_subtract(this, &v, BT_VEC3_MODE);
		
		return *this;
	}
	
  /**@brief Scale the vector
   * @param s Scale factor */
	SIMD_FORCE_INLINE btVector3& operator*=(btScalar s)
	{
		btVector_scale(this, s, BT_VEC3_MODE);
		
		return *this;
	}

  /**@brief Inversely scale the vector 
   * @param s Scale factor to divide by */
	SIMD_FORCE_INLINE btVector3& operator/=(btScalar s) 
	{
		btVector_divide(this, s, BT_VEC3_MODE);
		
		return *this;
	}

  /**@brief Return the dot product
   * @param v The other vector in the dot product */
	SIMD_FORCE_INLINE btScalar dot(const btVector3& v) const
	{
		return btVector_dot(this, &v, BT_VEC3_MODE);
	}

  /**@brief Return the length of the vector squared */
	SIMD_FORCE_INLINE btScalar length2() const
	{
		return btVector_length2(this, BT_VEC3_MODE);
	}

  /**@brief Return the length of the vector */
	SIMD_FORCE_INLINE btScalar length() const
	{
		return btVector_length(this, BT_VEC3_MODE);
	}

	/**@brief Return the norm (length) of the vector */
	SIMD_FORCE_INLINE btScalar norm() const
	{
		return btVector_norm(this, BT_VEC3_MODE);
	}
	
	SIMD_FORCE_INLINE btVector3 diff(const btVector3& v2) const
	{
		return btVector_diff(this, &v2, BT_VEC3_MODE);
	}

  /**@brief Return the distance squared between the ends of this and another vector
   * This is symantically treating the vector like a point */
	SIMD_FORCE_INLINE btScalar distance2(const btVector3& v) const {
		return btVector_distance2(this, &v, BT_VEC3_MODE);
	}

  /**@brief Return the distance between the ends of this and another vector
   * This is symantically treating the vector like a point */
	SIMD_FORCE_INLINE btScalar distance(const btVector3& v) const {
		return btVector_distance(this, &v, BT_VEC3_MODE);
	}

	SIMD_FORCE_INLINE btVector3& safeNormalize() 
	{
		btVector_safeNormalize(this, BT_VEC3_MODE);
		
		return *this;
	}

  /**@brief Normalize this vector 
   * x^2 + y^2 + z^2 = 1 */
	SIMD_FORCE_INLINE btVector3& normalize() 
	{
		btVector_normalize(this, BT_VEC3_MODE);
		
		return *this;
	}

  /**@brief Return a normalized version of this vector */
	SIMD_FORCE_INLINE btVector3 normalized() const
	{
		btVector3 nrm = *this;

		return nrm.normalize();
	}

  /**@brief Return a rotated version of this vector
   * @param wAxis The axis to rotate about 
   * @param angle The angle to rotate by */
	SIMD_FORCE_INLINE btVector3 rotate( const btVector3& wAxis, const btScalar angle ) const;

  /**@brief Return the angle between this and another vector
   * @param v The other vector */
	SIMD_FORCE_INLINE btScalar angle(const btVector3& v) const 
	{
		return btVector_angle(this, &v, BT_VEC3_MODE);
	}
	
  /**@brief Return a vector will the absolute values of each element */
	SIMD_FORCE_INLINE btVector3 absolute() const 
	{
		return btVector_absolute(this, BT_VEC3_MODE);
	}
	
  /**@brief Return the cross product between this and another vector 
   * @param v The other vector */
	SIMD_FORCE_INLINE btVector3 cross(const btVector3& v) const;

	SIMD_FORCE_INLINE btScalar triple(const btVector3& v1, const btVector3& v2) const;

  /**@brief Return the axis with the smallest value 
   * Note return values are 0,1,2 for x, y, or z */
	SIMD_FORCE_INLINE int minAxis() const
	{
		return btVector_minAxis(this, BT_VEC3_MODE);
	}

  /**@brief Return the axis with the largest value 
   * Note return values are 0,1,2 for x, y, or z */
	SIMD_FORCE_INLINE int maxAxis() const 
	{
		return btVector_maxAxis(this, BT_VEC3_MODE);
	}

	SIMD_FORCE_INLINE int furthestAxis() const
	{
		return btVector_furthestAxis(this, BT_VEC3_MODE);
	}

	SIMD_FORCE_INLINE int closestAxis() const 
	{
		return btVector_closestAxis(this, BT_VEC3_MODE);
	}

	// Notice: The self vector(this), v0 and v1 can be memory aligned
	SIMD_FORCE_INLINE void setInterpolate3(const btVector3& v0, const btVector3& v1, btScalar rt)
	{
		return btVector_setInterpolate3(this, &v0, &v1, rt, BT_VEC3_MODE);
	}

  /**@brief Return the linear interpolation between this and another vector 
   * @param v The other vector 
   * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
	SIMD_FORCE_INLINE btVector3 lerp(const btVector3& v, const btScalar& t) const 
	{
		return btVector_lerp(this, &v, t, BT_VEC3_MODE);
	}

  /**@brief Elementwise multiply this vector by the other 
   * @param v The other vector */
	SIMD_FORCE_INLINE btVector3& operator*=(const btVector3& v)
	{
		btVector_multiply(this, &v, BT_VEC3_MODE);
		return *this;
	}

  /**@brief Set each element to the max of the current values and the values of another btVector3
   * @param other The other btVector3 to compare with 
   */
	SIMD_FORCE_INLINE void	setMax(const btVector3& other)
	{
		btVector_setMax(this, &other, BT_VEC3_MODE);
	}

  /**@brief Set each element to the min of the current values and the values of another btVector3
   * @param other The other btVector3 to compare with 
   */
	SIMD_FORCE_INLINE void	setMin(const btVector3& other)
	{
		btVector_setMin(this, &other, BT_VEC3_MODE);
	}

	SIMD_FORCE_INLINE void 	setValue(const btScalar& _x, const btScalar& _y, const btScalar& _z)
	{
		m_floats[0]=_x;
		m_floats[1]=_y;
		m_floats[2]=_z;
		m_floats[3] = btScalar(0.f);
	}

	SIMD_FORCE_INLINE void	getSkewSymmetricMatrix(btVector3* v0,btVector3* v1,btVector3* v2) const;

	SIMD_FORCE_INLINE bool isZero() const 
	{
		return btVector_isZero(this, BT_VEC3_MODE);
	}


	SIMD_FORCE_INLINE bool fuzzyZero() const 
	{
		return btVector_fuzzyZero(this, BT_VEC3_MODE);
	}
    
    /**@brief returns index of maximum dot product between this and vectors in array[]
     * @param array The other vectors 
     * @param array_count The number of other vectors 
     * @param dotOut The maximum dot product */
    SIMD_FORCE_INLINE   long    maxDot( const btVector3 *array, long array_count, btScalar &dotOut ) const; 

    /**@brief returns index of minimum dot product between this and vectors in array[]
     * @param array The other vectors 
     * @param array_count The number of other vectors 
     * @param dotOut The minimum dot product */    
    SIMD_FORCE_INLINE   long    minDot( const btVector3 *array, long array_count, btScalar &dotOut ) const; 

    /* create a vector as  btVector3( this->dot( btVector3 v0 ), this->dot( btVector3 v1), this->dot( btVector3 v2 ))  */
    SIMD_FORCE_INLINE btVector3  dot3( const btVector3 &v0, const btVector3 &v1, const btVector3 &v2 ) const
    {
#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)

        __m128 a0 = _mm_mul_ps( v0.mVec128, this->mVec128 );
        __m128 a1 = _mm_mul_ps( v1.mVec128, this->mVec128 );
        __m128 a2 = _mm_mul_ps( v2.mVec128, this->mVec128 );
        __m128 b0 = _mm_unpacklo_ps( a0, a1 );
        __m128 b1 = _mm_unpackhi_ps( a0, a1 );
        __m128 b2 = _mm_unpacklo_ps( a2, _mm_setzero_ps() );
        __m128 r = _mm_movelh_ps( b0, b2 );
        r = _mm_add_ps( r, _mm_movehl_ps( b2, b0 ));
        a2 = _mm_and_ps( a2, btvxyzMaskf);
        r = _mm_add_ps( r, btCastdTo128f (_mm_move_sd( btCastfTo128d(a2), btCastfTo128d(b1) )));
        return btVector3(r);
        
#elif defined(BT_USE_NEON)
        static const uint32x4_t xyzMask = (const uint32x4_t){ static_cast<uint32_t>(-1), static_cast<uint32_t>(-1), static_cast<uint32_t>(-1), 0 };
        float32x4_t a0 = vmulq_f32( v0.mVec128, this->mVec128);
        float32x4_t a1 = vmulq_f32( v1.mVec128, this->mVec128);
        float32x4_t a2 = vmulq_f32( v2.mVec128, this->mVec128);
        float32x2x2_t zLo = vtrn_f32( vget_high_f32(a0), vget_high_f32(a1));
        a2 = (float32x4_t) vandq_u32((uint32x4_t) a2, xyzMask );
        float32x2_t b0 = vadd_f32( vpadd_f32( vget_low_f32(a0), vget_low_f32(a1)), zLo.val[0] );
        float32x2_t b1 = vpadd_f32( vpadd_f32( vget_low_f32(a2), vget_high_f32(a2)), vdup_n_f32(0.0f));
        return btVector3( vcombine_f32(b0, b1) );
#else	
		return btVector3( dot(v0), dot(v1), dot(v2));
#endif
    }
};
#endif//__cplusplus

// Common Vec3 function

#define btVector3_copy(target, src) btVector_copy(target, src)

#if (defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE) )|| defined (BT_USE_NEON)
#define btVector3_fromSimd(v) btVector_fromSimd(v)
#endif // #if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)

#define btVector3_cmp(v1, v2) btVector_cmp(v1, v2)

#define btVector3_add(self, v) btVector_add(self, v, BT_VEC3_MODE)

#define btVector3_subtract(self, v) btVector_subtract(self, v, BT_VEC3_MODE)

#define btVector3_scale(self, s) btVector_scale(self, s, BT_VEC3_MODE)

#define btVector3_divide(self, s) btVector_divide(self, s, BT_VEC3_MODE)

#define btVector3_multiply(self, v) btVector_multiply(self, v, BT_VEC3_MODE)

#define btVector3_dot(a, b) btVector_dot(a, b, BT_VEC3_MODE)

#define btVector3_length2(self) btVector_length2(self, BT_VEC3_MODE)

#define btVector3_length(self) btVector_length(self, BT_VEC3_MODE)

#define btVector3_norm(self) btVector_norm(self, BT_VEC3_MODE)

#define btVector3_diff(a, b) btVector_diff(a, b, BT_VEC3_MODE)

#define btVector3_distance2(a, b) btVector_distance2(a, b, BT_VEC3_MODE)

#define btVector3_distance(a, b) btVector_distance(a, b, BT_VEC3_MODE)

#define btVector3_absolute(self) btVector_absolute(self, BT_VEC3_MODE)

#define btVector3_minAxis(self) btVector_minAxis(self, BT_VEC3_MODE)

#define btVector3_maxAxis(self) btVector_maxAxis(self, BT_VEC3_MODE)

#define btVector3_furthestAxis(self) btVector_furthestAxis(self, BT_VEC3_MODE)

#define btVector3_closestAxis(self) btVector_closestAxis(self, BT_VEC3_MODE)

#define btVector3_safeNormalize(self) btVector_safeNormalize(self, BT_VEC3_MODE)

#define btVector3_isZero(self) btVector_isZero(self, BT_VEC3_MODE)

#define btVector3_fuzzyZero(self) btVector_fuzzyZero(self, BT_VEC3_MODE)

#define btVector3_angle(v1, v2) btVector_angle(v1, v2, BT_VEC3_MODE)

#define btVector3_normalize(self) btVector_normalize(self, BT_VEC3_MODE)

#define btVector3_normalized(self) btVector_normalized(self, BT_VEC3_MODE)

#define btVector3_setInterpolate3(self, v0, v1, rt) btVector_setInterpolate3(self, v0, v1, rt, BT_VEC3_MODE)

#define btVector3_lerp(self, v, t) btVector_lerp(self, v, t, BT_VEC3_MODE)

#define btVector3_setMax(self, other) btVector_setMax(self, other, BT_VEC3_MODE)

#define btVector3_setMin(self, other) btVector_setMin(self, other, BT_VEC3_MODE)

#define btVector3_setValue(self, x, y, z) btVector_setValue(self, x, y, z, 0)

#define btVector3_setZero(self) btVector_setZero(self)

/**@brief Return the cross product between this and another vector 
 * @param v The other vector */
static SIMD_FORCE_INLINE btVector3 btVector3_cross(const btVector3* a, const btVector3* b)
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	__m128	T, V;
	
	T = bt_pshufd_ps(a->mVec128, BT_SHUFFLE(1, 2, 0, 3));	//	(Y Z X 0)
	V = bt_pshufd_ps(b->mVec128, BT_SHUFFLE(1, 2, 0, 3));	//	(Y Z X 0)
	
	V = _mm_mul_ps(V, a->mVec128);
	T = _mm_mul_ps(T, b->mVec128);
	V = _mm_sub_ps(V, T);
	
	V = bt_pshufd_ps(V, BT_SHUFFLE(1, 2, 0, 3));
	return btVector_fromSimd(V);
#elif defined(BT_USE_NEON)
	float32x4_t T, V;
	// form (Y, Z, X, _) of a->mVec128 and b->mVec128
	float32x2_t Tlow = vget_low_f32(a->mVec128);
	float32x2_t Vlow = vget_low_f32(b->mVec128);
	T = vcombine_f32(vext_f32(Tlow, vget_high_f32(a->mVec128), 1), Tlow);
	V = vcombine_f32(vext_f32(Vlow, vget_high_f32(b->mVec128), 1), Vlow);
	
	V = vmulq_f32(V, a->mVec128);
	T = vmulq_f32(T, b->mVec128);
	V = vsubq_f32(V, T);
	Vlow = vget_low_f32(V);
	// form (Y, Z, X, _);
	V = vcombine_f32(vext_f32(Vlow, vget_high_f32(V), 1), Vlow);
	V = (float32x4_t)vandq_s32((int32x4_t)V, btvFFF0Mask);
	
	return btVector_fromSimd(V);
#else
	return btVector3(
		a->m_floats[1] * b->m_floats[2] - a->m_floats[2] * b->m_floats[1],
		a->m_floats[2] * b->m_floats[0] - a->m_floats[0] * b->m_floats[2],
		a->m_floats[0] * b->m_floats[1] - a->m_floats[1] * b->m_floats[0]);
#endif
}

static SIMD_FORCE_INLINE btScalar btVector3_triple(const btVector3* self, const btVector3* v1, const btVector3* v2)
{
#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	// cross:
	__m128 T = _mm_shuffle_ps(v1->mVec128, v1->mVec128, BT_SHUFFLE(1, 2, 0, 3));	//	(Y Z X 0)
	__m128 V = _mm_shuffle_ps(v2->mVec128, v2->mVec128, BT_SHUFFLE(1, 2, 0, 3));	//	(Y Z X 0)
	
	V = _mm_mul_ps(V, v1->mVec128);
	T = _mm_mul_ps(T, v2->mVec128);
	V = _mm_sub_ps(V, T);
	
	V = _mm_shuffle_ps(V, V, BT_SHUFFLE(1, 2, 0, 3));

	// dot: 
	V = _mm_mul_ps(V, self->mVec128);
	__m128 z = _mm_movehl_ps(V, V);
	__m128 y = _mm_shuffle_ps(V, V, 0x55);
	V = _mm_add_ss(V, y);
	V = _mm_add_ss(V, z);
	return _mm_cvtss_f32(V);

#elif defined(BT_USE_NEON)
	// cross:
	float32x4_t T, V;
	// form (Y, Z, X, _) of mVec128 and v.mVec128
	float32x2_t Tlow = vget_low_f32(v1->mVec128);
	float32x2_t Vlow = vget_low_f32(v2->mVec128);
	T = vcombine_f32(vext_f32(Tlow, vget_high_f32(v1->mVec128), 1), Tlow);
	V = vcombine_f32(vext_f32(Vlow, vget_high_f32(v2->mVec128), 1), Vlow);
	
	V = vmulq_f32(V, v1->mVec128);
	T = vmulq_f32(T, v2->mVec128);
	V = vsubq_f32(V, T);
	Vlow = vget_low_f32(V);
	// form (Y, Z, X, _);
	V = vcombine_f32(vext_f32(Vlow, vget_high_f32(V), 1), Vlow);

	// dot: 
	V = vmulq_f32(self->mVec128, V);
	float32x2_t x = vpadd_f32(vget_low_f32(V), vget_low_f32(V));  
	x = vadd_f32(x, vget_high_f32(V));
	return vget_lane_f32(x, 0);
#else
	return 
		self->m_floats[0] * (v1->m_floats[1] * v2->m_floats[2] - v1->m_floats[2] * v2->m_floats[1]) + 
		self->m_floats[1] * (v1->m_floats[2] * v2->m_floats[0] - v1->m_floats[0] * v2->m_floats[2]) + 
		self->m_floats[2] * (v1->m_floats[0] * v2->m_floats[1] - v1->m_floats[1] * v2->m_floats[0]);
#endif
}

/**@brief Return a rotated version of this vector
 * @param wAxis The axis to rotate about 
 * @param angle The angle to rotate by */
static SIMD_FORCE_INLINE btVector3 btVector3_rotate(const btVector3* self, const btVector3* wAxis, const btScalar _angle)
{
	// wAxis must be a unit lenght vector

#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)

    __m128 O = _mm_mul_ps(wAxis->mVec128, self->mVec128);
	btScalar ssin = btSin( _angle );
	btVector3 cross = btVector3_cross(wAxis, self);
    __m128 C = cross->mVec128;
	O = _mm_and_ps(O, btvFFF0fMask);
    btScalar scos = btCos( _angle );
	
	__m128 vsin = _mm_load_ss(&ssin);	//	(S 0 0 0)
    __m128 vcos = _mm_load_ss(&scos);	//	(S 0 0 0)
	
	__m128 Y = bt_pshufd_ps(O, 0xC9);	//	(Y Z X 0)
	__m128 Z = bt_pshufd_ps(O, 0xD2);	//	(Z X Y 0)
	O = _mm_add_ps(O, Y);
	vsin = bt_pshufd_ps(vsin, 0x80);	//	(S S S 0)
	O = _mm_add_ps(O, Z);
    vcos = bt_pshufd_ps(vcos, 0x80);	//	(S S S 0)
	
    vsin = _mm_mul_ps(vsin, C);
	O = _mm_mul_ps(O, wAxis->mVec128);
	__m128 X = _mm_sub_ps(self->mVec128, O);
	
    O = _mm_add_ps(O, vsin);
	vcos = _mm_mul_ps(vcos, X);
	O = _mm_add_ps(O, vcos);
	
	return btVector_fromSimd(O);
#else
	btVector3 o;
	btVector_copy(&o, wAxis);
	btVector3_scale(&o, btVector3_dot(wAxis, self));
	
	btVector3 _x;
	btVector_copy(&_x, self);
	btVector3_subtract(&_x, &o);
	
	btVector3 _y = btVector3_cross(wAxis, self);
	
	// Summing up
	// First, multiply:
	btVector3_scale(&_x, btCos( _angle ));
	btVector3_scale(&_y, btSin( _angle ));
	
	// Then, sum:
	btVector3_add(&o, &_x);
	btVector3_add(&o, &_y);

	return o;
#endif
}

static void btVector3_getSkewSymmetricMatrix(const btVector3* BT_RESTRICT self, btVector3* BT_RESTRICT v0, btVector3* BT_RESTRICT v1, btVector3* BT_RESTRICT v2)
{
#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	__m128 V  = _mm_and_ps(self->mVec128, btvFFF0fMask);
	__m128 V0 = _mm_xor_ps(btvMzeroMask, V);
	__m128 V2 = _mm_movelh_ps(V0, V);
	
	__m128 V1 = _mm_shuffle_ps(V, V0, 0xCE);
	
    V0 = _mm_shuffle_ps(V0, V, 0xDB);
	V2 = _mm_shuffle_ps(V2, V, 0xF9);
	
	v0->mVec128 = V0;
	v1->mVec128 = V1;
	v2->mVec128 = V2;
#else
	const btScalar x = self->m_floats[0];
	const btScalar y = self->m_floats[1];
	const btScalar z = self->m_floats[2];
	
	btVector3_setValue(v0,  0., -z,  y);
	btVector3_setValue(v1,  z,  0., -x);
	btVector3_setValue(v2, -y,   x, 0.);
#endif
}

#if (defined BT_USE_SSE && defined BT_USE_SIMD_VECTOR3 && defined BT_USE_SSE_IN_API) || defined (BT_USE_NEON)
    #if defined _WIN32 || defined (BT_USE_SSE)
        long _maxdot_large( const float* BT_RESTRICT vv, const float* BT_RESTRICT vec, unsigned long count, float*  BT_RESTRICT dotResult );
        long _mindot_large( const float *array, const float *vec, unsigned long array_count, float *dotOut );
    #elif defined BT_USE_NEON
        extern long (*_maxdot_large)( const float* BT_RESTRICT vv, const float* BT_RESTRICT vec, unsigned long count, float* BT_RESTRICT dotResult );
        extern long (*_mindot_large)( const float *array, const float *vec, unsigned long array_count, float *dotOut );
    #endif
#endif

/**@brief returns index of maximum dot product between self and vectors in array[]
  * @param self The self vector to be tested
  * @param array The other vectors
  * @param array_count The number of other vectors
  * @param dotOut The maximum dot product, mustn't be NULL */
static SIMD_FORCE_INLINE   long    btVector3_maxDot(const btVector3* BT_RESTRICT self, const btVector3* BT_RESTRICT array, long array_count, btScalar* BT_RESTRICT dotOut)
{
#if (defined BT_USE_SSE && defined BT_USE_SIMD_VECTOR3 && defined BT_USE_SSE_IN_API) || defined (BT_USE_NEON)
    #if defined _WIN32 || defined (BT_USE_SSE)
        const long scalar_cutoff = 10;
    #elif defined BT_USE_NEON
        const long scalar_cutoff = 4;
    #endif
    if( array_count < scalar_cutoff )	
#endif
    {
        btScalar maxDot1 = -SIMD_INFINITY;
        int i = 0;
        int ptIndex = -1;
        for( i = 0; i < array_count; i++ )
        {
            btScalar dot = btVector3_dot(self, &array[i]);
            
            if( dot > maxDot1 )
            {
                maxDot1 = dot;
                ptIndex = i;
            }
        }
        
        *dotOut = maxDot1;
        return ptIndex;
    }
#if (defined BT_USE_SSE && defined BT_USE_SIMD_VECTOR3 && defined BT_USE_SSE_IN_API) || defined (BT_USE_NEON)
    return _maxdot_large( (float*) &self.m_floats[0], (float*) &array[0].m_floats[0], array_count, &dotOut );
#endif
}

/**@brief returns index of minimum dot product between self and vectors in array[]
  * @param self The self vector to be tested
  * @param array The other vectors
  * @param array_count The number of other vectors
  * @param dotOut The minimum dot product, mustn't be NULL */
SIMD_FORCE_INLINE   long    btVector3_minDot(const btVector3* BT_RESTRICT self, const btVector3* BT_RESTRICT array, long array_count, btScalar* BT_RESTRICT dotOut)
{
#if (defined BT_USE_SSE && defined BT_USE_SIMD_VECTOR3 && defined BT_USE_SSE_IN_API) || defined (BT_USE_NEON)
    #if defined BT_USE_SSE
        const long scalar_cutoff = 10;
    #elif defined BT_USE_NEON
        const long scalar_cutoff = 4;
    #else
        #error unhandled arch!
    #endif
    
    if( array_count < scalar_cutoff )
#endif
    {
        btScalar  minDot = SIMD_INFINITY;
        int i = 0;
        int ptIndex = -1;
        
        for( i = 0; i < array_count; i++ )
        {
            btScalar dot = btVector3_dot(self, &array[i]);
            
            if( dot < minDot )
            {
                minDot = dot;
                ptIndex = i;
            }
        }
        
        *dotOut = minDot;
        
        return ptIndex;
    }
#if (defined BT_USE_SSE && defined BT_USE_SIMD_VECTOR3 && defined BT_USE_SSE_IN_API) || defined (BT_USE_NEON)
    return _mindot_large( (float*) &self.m_floats[0], (float*) &array[0].m_floats[0], array_count, &dotOut );
#endif//BT_USE_SIMD_VECTOR3
}


#ifdef __cplusplus
/**@brief Return the sum of two vectors (Point symantics)*/
SIMD_FORCE_INLINE btVector3 
operator+(const btVector3& v1, const btVector3& v2) 
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	return btVector3(_mm_add_ps(v1.mVec128, v2.mVec128));
#elif defined(BT_USE_NEON)
	return btVector3(vaddq_f32(v1.mVec128, v2.mVec128));
#else
	return btVector3(
			v1.m_floats[0] + v2.m_floats[0], 
			v1.m_floats[1] + v2.m_floats[1], 
			v1.m_floats[2] + v2.m_floats[2]);
#endif
}

/**@brief Return the elementwise product of two vectors */
SIMD_FORCE_INLINE btVector3 
operator*(const btVector3& v1, const btVector3& v2) 
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	return btVector3(_mm_mul_ps(v1.mVec128, v2.mVec128));
#elif defined(BT_USE_NEON)
	return btVector3(vmulq_f32(v1.mVec128, v2.mVec128));
#else
	return btVector3(
			v1.m_floats[0] * v2.m_floats[0], 
			v1.m_floats[1] * v2.m_floats[1], 
			v1.m_floats[2] * v2.m_floats[2]);
#endif
}

/**@brief Return the difference between two vectors */
SIMD_FORCE_INLINE btVector3 
operator-(const btVector3& v1, const btVector3& v2)
{
	return v1.diff(v2);
}

/**@brief Return the negative of the vector */
SIMD_FORCE_INLINE btVector3 
operator-(const btVector3& v)
{
#if defined BT_USE_SIMD_VECTOR3 && (defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE))
	__m128 r = _mm_xor_ps(v.mVec128, btvMzeroMask);
	return btVector3(_mm_and_ps(r, btvFFF0fMask)); 
#elif defined(BT_USE_NEON)
	return btVector3((btSimdFloat4)veorq_s32((int32x4_t)v.mVec128, (int32x4_t)btvMzeroMask));
#else	
	return btVector3(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2]);
#endif
}

/**@brief Return the vector scaled by s */
SIMD_FORCE_INLINE btVector3 
operator*(const btVector3& v, const btScalar& s)
{
#if defined(BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	__m128	vs = _mm_load_ss(&s);	//	(S 0 0 0)
	vs = bt_pshufd_ps(vs, 0x80);	//	(S S S 0.0)
	return btVector3(_mm_mul_ps(v.mVec128, vs));
#elif defined(BT_USE_NEON)
	float32x4_t r = vmulq_n_f32(v.mVec128, s);
	return btVector3((float32x4_t)vandq_s32((int32x4_t)r, btvFFF0Mask));
#else
	return btVector3(v.m_floats[0] * s, v.m_floats[1] * s, v.m_floats[2] * s);
#endif
}

/**@brief Return the vector scaled by s */
SIMD_FORCE_INLINE btVector3 
operator*(const btScalar& s, const btVector3& v)
{ 
	return v * s; 
}

/**@brief Return the vector inversely scaled by s */
SIMD_FORCE_INLINE btVector3
operator/(const btVector3& v, const btScalar& s)
{
	btFullAssert(s != btScalar(0.0));
#if 0 //defined(BT_USE_SSE_IN_API)
// this code is not faster !
	__m128 vs = _mm_load_ss(&s);
    vs = _mm_div_ss(v1110, vs);
	vs = bt_pshufd_ps(vs, 0x00);	//	(S S S S)

	return btVector3(_mm_mul_ps(v.mVec128, vs));
#else
	return v * (btScalar(1.0) / s);
#endif
}

/**@brief Return the vector inversely scaled by s */
SIMD_FORCE_INLINE btVector3
operator/(const btVector3& v1, const btVector3& v2)
{
#if defined BT_USE_SIMD_VECTOR3 && (defined(BT_USE_SSE_IN_API)&& defined (BT_USE_SSE))
	__m128 vec = _mm_div_ps(v1.mVec128, v2.mVec128);
	vec = _mm_and_ps(vec, btvFFF0fMask);
	return btVector3(vec); 
#elif defined(BT_USE_NEON)
	float32x4_t x, y, v, m;

	x = v1.mVec128;
	y = v2.mVec128;
	
	v = vrecpeq_f32(y);			// v ~ 1/y
	m = vrecpsq_f32(y, v);		// m = (2-v*y)
	v = vmulq_f32(v, m);		// vv = v*m ~~ 1/y
	m = vrecpsq_f32(y, v);		// mm = (2-vv*y)
	v = vmulq_f32(v, x);		// x*vv
	v = vmulq_f32(v, m);		// (x*vv)*(2-vv*y) = x*(vv(2-vv*y)) ~~~ x/y

	return btVector3(v);
#else
	return btVector3(
			v1.m_floats[0] / v2.m_floats[0], 
			v1.m_floats[1] / v2.m_floats[1],
			v1.m_floats[2] / v2.m_floats[2]);
#endif
}

/**@brief Return the dot product between two vectors */
SIMD_FORCE_INLINE btScalar 
btDot(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.dot(v2); 
}


/**@brief Return the distance squared between two vectors */
SIMD_FORCE_INLINE btScalar
btDistance2(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.distance2(v2); 
}


/**@brief Return the distance between two vectors */
SIMD_FORCE_INLINE btScalar
btDistance(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.distance(v2); 
}

/**@brief Return the angle between two vectors */
SIMD_FORCE_INLINE btScalar
btAngle(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.angle(v2); 
}

/**@brief Return the cross product of two vectors */
SIMD_FORCE_INLINE btVector3 
btCross(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.cross(v2); 
}

SIMD_FORCE_INLINE btScalar
btTriple(const btVector3& v1, const btVector3& v2, const btVector3& v3)
{
	return v1.triple(v2, v3);
}

/**@brief Return the linear interpolation between two vectors
 * @param v1 One vector 
 * @param v2 The other vector 
 * @param t The ration of this to v (t = 0 => return v1, t=1 => return v2) */
SIMD_FORCE_INLINE btVector3 
lerp(const btVector3& v1, const btVector3& v2, const btScalar& t)
{
	return v1.lerp(v2, t);
}

SIMD_FORCE_INLINE btVector3 btVector3::rotate( const btVector3& wAxis, const btScalar _angle ) const
{
	return btVector3_rotate(this, &wAxis, _angle);
}

/**@brief Return the cross product between this and another vector 
 * @param v The other vector */
SIMD_FORCE_INLINE btVector3 btVector3::cross(const btVector3& v) const
{
	return btVector3_cross(this, &v);
}

SIMD_FORCE_INLINE btScalar btVector3::triple(const btVector3& v1, const btVector3& v2) const
{
	return btVector3_triple(this, &v1, &v2);
}

SIMD_FORCE_INLINE void btVector3::getSkewSymmetricMatrix(btVector3* v0,btVector3* v1,btVector3* v2) const {
	btVector3_getSkewSymmetricMatrix(this, v0, v1, v2);
}

SIMD_FORCE_INLINE   long    btVector3::maxDot( const btVector3 *array, long array_count, btScalar &dotOut ) const
{
	return btVector3_maxDot(this, array, array_count, &dotOut);
}

SIMD_FORCE_INLINE   long    btVector3::minDot( const btVector3 *array, long array_count, btScalar &dotOut ) const
{
	return btVector3_minDot(this, array, array_count, &dotOut);
}
#endif//__cplusplus


#ifdef __cplusplus
class btVector4 : public btVector3
{
public:

	SIMD_FORCE_INLINE btVector4() {}


	SIMD_FORCE_INLINE btVector4(const btScalar& _x, const btScalar& _y, const btScalar& _z,const btScalar& _w) 
		: btVector3(_x,_y,_z)
	{
		m_floats[3] = _w;
	}

#if (defined (BT_USE_SSE_IN_API)&& defined (BT_USE_SSE)) || defined (BT_USE_NEON) 
	SIMD_FORCE_INLINE btVector4(const btSimdFloat4 vec)
	{
		mVec128 = vec;
	}

	SIMD_FORCE_INLINE btVector4(const btVector& rhs)
	{
		mVec128 = rhs.mVec128;
	}
#else
	SIMD_FORCE_INLINE btVector4(const btVector& v) : btVector3(v) {
	}
#endif // #if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON) 

	SIMD_FORCE_INLINE btVector4 absolute4() const 
	{
		return btVector_absolute(this, BT_VEC4_MODE);
	}

	SIMD_FORCE_INLINE int maxAxis4() const
	{
		return btVector_maxAxis(this, BT_VEC4_MODE);
	}


	SIMD_FORCE_INLINE int minAxis4() const
	{
		return btVector_minAxis(this, BT_VEC4_MODE);
	}


	SIMD_FORCE_INLINE int closestAxis4() const 
	{
		return btVector_closestAxis(this, BT_VEC4_MODE);
	}

  /**@brief Set each element to the max of the current values and the values of another btVector4
   * @param other The other btVector4 to compare with 
   */
	SIMD_FORCE_INLINE void	setMax(const btVector4& other)
	{
		btVector_setMax(this, &other, BT_VEC4_MODE);
	}

  /**@brief Set each element to the min of the current values and the values of another btVector4
   * @param other The other btVector4 to compare with 
   */
	SIMD_FORCE_INLINE void	setMin(const btVector4& other)
	{
		btVector_setMin(this, &other, BT_VEC4_MODE);
	}

  /**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
		

/*		void getValue(btScalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] =m_floats[2];
		}
*/
  /**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
	SIMD_FORCE_INLINE void	setValue(const btScalar& _x, const btScalar& _y, const btScalar& _z,const btScalar& _w)
	{
		m_floats[0]=_x;
		m_floats[1]=_y;
		m_floats[2]=_z;
		m_floats[3]=_w;
	}

	SIMD_FORCE_INLINE bool isZero() const 
	{
		return btVector_isZero(this, BT_VEC4_MODE);
	}

	SIMD_FORCE_INLINE bool fuzzyZero() const 
	{
		return btVector_fuzzyZero(this, BT_VEC4_MODE);
	}
};
#endif//__cplusplus

// Common btVector4 functions



#define btVector4_copy(target, src) btVector_copy(target, src)

#if (defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE) )|| defined (BT_USE_NEON)
#define btVector4_fromSimd(v) btVector_fromSimd(v)
#endif // #if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)

#define btVector4_cmp(v1, v2) btVector_cmp(v1, v2)

#define btVector4_add(self, v) btVector_add(self, v, BT_VEC4_MODE)

#define btVector4_subtract(self, v) btVector_subtract(self, v, BT_VEC4_MODE)

#define btVector4_scale(self, s) btVector_scale(self, s, BT_VEC4_MODE)

#define btVector4_divide(self, s) btVector_divide(self, s, BT_VEC4_MODE)

#define btVector4_multiply(self, v) btVector_multiply(self, v, BT_VEC4_MODE)

#define btVector4_dot(a, b) btVector_dot(a, b, BT_VEC4_MODE)

#define btVector4_length2(self) btVector_length2(self, BT_VEC4_MODE)

#define btVector4_length(self) btVector_length(self, BT_VEC4_MODE)

#define btVector4_norm(self) btVector_norm(self, BT_VEC4_MODE)

#define btVector4_diff(a, b) btVector_diff(a, b, BT_VEC4_MODE)

#define btVector4_distance2(a, b) btVector_distance2(a, b, BT_VEC4_MODE)

#define btVector4_distance(a, b) btVector_distance(a, b, BT_VEC4_MODE)

#define btVector4_absolute(self) btVector_absolute(self, BT_VEC4_MODE)

#define btVector4_minAxis(self) btVector_minAxis(self, BT_VEC4_MODE)

#define btVector4_maxAxis(self) btVector_maxAxis(self, BT_VEC4_MODE)

#define btVector4_furthestAxis(self) btVector_furthestAxis(self, BT_VEC4_MODE)

#define btVector4_closestAxis(self) btVector_closestAxis(self, BT_VEC4_MODE)

#define btVector4_safeNormalize(self) btVector_safeNormalize(self, BT_VEC4_MODE)

#define btVector4_isZero(self) btVector_isZero(self, BT_VEC4_MODE)

#define btVector4_fuzzyZero(self) btVector_fuzzyZero(self, BT_VEC4_MODE)

#define btVector4_angle(v1, v2) btVector_angle(v1, v2, BT_VEC4_MODE)

#define btVector4_normalize(self) btVector_normalize(self, BT_VEC4_MODE)

#define btVector4_normalized(self) btVector_normalized(self, BT_VEC4_MODE)

#define btVector4_setInterpolate3(self, v0, v1, rt) btVector_setInterpolate3(self, v0, v1, rt, BT_VEC4_MODE)

#define btVector4_lerp(self, v, t) btVector_lerp(self, v, t, BT_VEC4_MODE)

#define btVector4_setMax(self, other) btVector_setMax(self, other, BT_VEC4_MODE)

#define btVector4_setMin(self, other) btVector_setMin(self, other, BT_VEC4_MODE)

#define btVector4_setValue(self, x, y, z, w) btVector_setValue(self, x, y, z, w)

#define btVector4_setZero(self) btVector_setZero(self)

///btSwapScalarEndianPtr swaps scalar endianness, useful for network and cross-platform serialization
static SIMD_FORCE_INLINE void	btSwapScalarEndianPtr(const btScalar* BT_RESTRICT src, btScalar* BT_RESTRICT dest)
{
#ifdef BT_USE_DOUBLE_PRECISION
	dest[0] = src[7];
    dest[1] = src[6];
    dest[2] = src[5];
    dest[3] = src[4];
    dest[4] = src[3];
    dest[5] = src[2];
    dest[6] = src[1];
    dest[7] = src[0];
#else
	dest[0] = src[3];
    dest[1] = src[2];
    dest[2] = src[1];
    dest[3] = src[0];
#endif //BT_USE_DOUBLE_PRECISION
}

///btSwapVectorEndianPtr swaps vector endianness, useful for network and cross-platform serialization
static SIMD_FORCE_INLINE void	btSwapVectorEndianPtr(const btVector* BT_RESTRICT sourceVec, btVector* BT_RESTRICT destVec)
{
	int i;
	for (i=0;i<4;i++)
	{
		btSwapScalarEndianPtr(&sourceVec->m_floats[i], &destVec->m_floats[i]);
	}
}

///btUnSwapVectorEndianPtr swaps vector endianness, useful for network and cross-platform serialization
static SIMD_FORCE_INLINE void	btUnSwapVectorEndianPtr(btVector* vector)
{
	btVector swappedVec = *vector;
	int i;
	for (i=0;i<4;i++)
	{
		btSwapScalarEndianPtr(&swappedVec.m_floats[i], &vector->m_floats[i]);
	}
}

#ifdef __cplusplus
SIMD_FORCE_INLINE void	btSwapScalarEndian(const btScalar& sourceVal, btScalar& destVal)
{
	return btSwapScalarEndianPtr(&sourceVal, &destVal);
}

///btSwapVectorEndian swaps vector endianness, useful for network and cross-platform serialization
SIMD_FORCE_INLINE void	btSwapVectorEndian(const btVector& sourceVec, btVector& destVec)
{
	return btSwapVectorEndianPtr(&sourceVec, &destVec);
}

///same as btSwapVectorEndian, only for compatibility support
SIMD_FORCE_INLINE void	btSwapVector3Endian(const btVector& sourceVec, btVector& destVec)
{
	return btSwapVectorEndian(sourceVec, destVec);
}

///btUnSwapVectorEndian swaps vector endianness, useful for network and cross-platform serialization
SIMD_FORCE_INLINE void	btUnSwapVectorEndian(btVector& vector)
{
	return btUnSwapVectorEndianPtr(&vector);
}

///same as btUnSwapVectorEndian, only for compatibility support
SIMD_FORCE_INLINE void	btUnSwapVector3Endian(btVector& vector)
{
	return btUnSwapVectorEndian(vector);
}

template <class T>
SIMD_FORCE_INLINE void btPlaneSpace1 (const T& n, T& p, T& q)
{
  if (btFabs(n[2]) > SIMDSQRT12) {
    // choose p in y-z plane
    btScalar a = n[1]*n[1] + n[2]*n[2];
    btScalar k = btRecipSqrt (a);
    p[0] = 0;
	p[1] = -n[2]*k;
	p[2] = n[1]*k;
    // set q = n x p
    q[0] = a*k;
	q[1] = -n[0]*p[2];
	q[2] = n[0]*p[1];
  }
  else {
    // choose p in x-y plane
    btScalar a = n[0]*n[0] + n[1]*n[1];
    btScalar k = btRecipSqrt (a);
    p[0] = -n[1]*k;
	p[1] = n[0]*k;
	p[2] = 0;
    // set q = n x p
    q[0] = -n[2]*p[1];
	q[1] = n[2]*p[0];
	q[2] = a*k;
  }
}
#endif//__cplusplus

#endif //BT_VECTOR3_H
