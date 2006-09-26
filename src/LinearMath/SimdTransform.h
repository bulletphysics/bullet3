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



#ifndef SimdTransform_H
#define SimdTransform_H

#include "LinearMath/SimdVector3.h"
#include "LinearMath/SimdMatrix3x3.h"


#define IGNORE_TYPE 1

class SimdTransform {
	

public:
	
	enum { 
		TRANSLATION = 0x01,
		ROTATION    = 0x02,
		RIGID       = TRANSLATION | ROTATION,  
		SCALING     = 0x04,
		LINEAR      = ROTATION | SCALING,
		AFFINE      = TRANSLATION | LINEAR
	};

	SimdTransform() {}

	explicit SIMD_FORCE_INLINE SimdTransform(const SimdQuaternion& q, 
		const SimdVector3& c = SimdVector3(SimdScalar(0), SimdScalar(0), SimdScalar(0))) 
		: m_basis(q),
		m_origin(c)
#ifndef IGNORE_TYPE
		,	m_type(RIGID)
#endif //IGNORE_TYPE
	{}

	explicit SIMD_FORCE_INLINE SimdTransform(const SimdMatrix3x3& b, 
		const SimdVector3& c = SimdVector3(SimdScalar(0), SimdScalar(0), SimdScalar(0)), 
		unsigned int type = AFFINE)
		: m_basis(b),
		m_origin(c)
#ifndef IGNORE_TYPE
		,	m_type(type)
#endif //IGNORE_TYPE
	{}


		SIMD_FORCE_INLINE void mult(const SimdTransform& t1, const SimdTransform& t2) {
			m_basis = t1.m_basis * t2.m_basis;
			m_origin = t1(t2.m_origin);
#ifndef IGNORE_TYPE
			m_type = t1.m_type | t2.m_type;
#endif //IGNORE_TYPE
		}

		void multInverseLeft(const SimdTransform& t1, const SimdTransform& t2) {
			SimdVector3 v = t2.m_origin - t1.m_origin;
#ifndef  IGNORE_TYPE
			if (t1.m_type & SCALING) {
				SimdMatrix3x3 inv = t1.m_basis.inverse();
				m_basis = inv * t2.m_basis;
				m_origin = inv * v;
			}
			else 
#else
			{
				m_basis = SimdMultTransposeLeft(t1.m_basis, t2.m_basis);
				m_origin = v * t1.m_basis;
#endif //IGNORE_TYPE
			}
#ifndef IGNORE_TYPE
			m_type = t1.m_type | t2.m_type;
#endif //IGNORE_TYPE
		}

	SIMD_FORCE_INLINE SimdVector3 operator()(const SimdVector3& x) const
	{
		return SimdVector3(m_basis[0].dot(x) + m_origin[0], 
			m_basis[1].dot(x) + m_origin[1], 
			m_basis[2].dot(x) + m_origin[2]);
	}

	SIMD_FORCE_INLINE SimdVector3 operator*(const SimdVector3& x) const
	{
		return (*this)(x);
	}

	SIMD_FORCE_INLINE SimdMatrix3x3&       getBasis()          { return m_basis; }
	SIMD_FORCE_INLINE const SimdMatrix3x3& getBasis()    const { return m_basis; }

	SIMD_FORCE_INLINE SimdVector3&         getOrigin()         { return m_origin; }
	SIMD_FORCE_INLINE const SimdVector3&   getOrigin()   const { return m_origin; }

	SimdQuaternion getRotation() const { 
		SimdQuaternion q;
		m_basis.getRotation(q);
		return q;
	}
	template <typename Scalar2>
		void setValue(const Scalar2 *m) 
	{
		m_basis.setValue(m);
		m_origin.setValue(&m[12]);
#ifndef IGNORE_TYPE
		m_type = AFFINE;
#endif //IGNORE_TYPE
	}

	
	void setFromOpenGLMatrix(const SimdScalar *m)
	{
		m_basis.setFromOpenGLSubMatrix(m);
		m_origin[0] = m[12];
		m_origin[1] = m[13];
		m_origin[2] = m[14];
	}

	void getOpenGLMatrix(SimdScalar *m) const 
	{
		m_basis.getOpenGLSubMatrix(m);
		m[12] = m_origin[0];
		m[13] = m_origin[1];
		m[14] = m_origin[2];
		m[15] = SimdScalar(1.0f);
	}

	SIMD_FORCE_INLINE void setOrigin(const SimdVector3& origin) 
	{ 
		m_origin = origin;
#ifndef IGNORE_TYPE
		m_type |= TRANSLATION;
#endif //IGNORE_TYPE
	}

	SIMD_FORCE_INLINE SimdVector3 invXform(const SimdVector3& inVec) const;



	SIMD_FORCE_INLINE void setBasis(const SimdMatrix3x3& basis)
	{ 
		m_basis = basis;
#ifndef IGNORE_TYPE
		m_type |= LINEAR;
#endif //IGNORE_TYPE
	}

	SIMD_FORCE_INLINE void setRotation(const SimdQuaternion& q)
	{
		m_basis.setRotation(q);
#ifndef IGNORE_TYPE
		m_type = (m_type & ~LINEAR) | ROTATION;
#endif //IGNORE_TYPE
	}

	SIMD_FORCE_INLINE void scale(const SimdVector3& scaling)
	{
		m_basis = m_basis.scaled(scaling);
#ifndef IGNORE_TYPE
		m_type |= SCALING;
#endif //IGNORE_TYPE
	}

	void setIdentity()
	{
		m_basis.setIdentity();
		m_origin.setValue(SimdScalar(0.0), SimdScalar(0.0), SimdScalar(0.0));
#ifndef IGNORE_TYPE
		m_type = 0x0;
#endif //IGNORE_TYPE
	}

	SIMD_FORCE_INLINE bool isIdentity() const { 
#ifdef IGNORE_TYPE
		return false;
#else
		return m_type == 0x0; 
#endif
	}

	SimdTransform& operator*=(const SimdTransform& t) 
	{
		m_origin += m_basis * t.m_origin;
		m_basis *= t.m_basis;
#ifndef IGNORE_TYPE
		m_type |= t.m_type; 
#endif //IGNORE_TYPE
		return *this;
	}

	SimdTransform inverse() const
	{ 
#ifdef IGNORE_TYPE
		SimdMatrix3x3 inv = m_basis.transpose();
		return SimdTransform(inv, inv * -m_origin, 0);
#else
		if (m_type)
		{
			SimdMatrix3x3 inv = (m_type & SCALING) ? 
				m_basis.inverse() : 
			m_basis.transpose();

			return SimdTransform(inv, inv * -m_origin, m_type);
		}

		return *this;
#endif //IGNORE_TYPE
	}

	SimdTransform inverseTimes(const SimdTransform& t) const;  

	SimdTransform operator*(const SimdTransform& t) const;

private:

	SimdMatrix3x3 m_basis;
	SimdVector3   m_origin;
#ifndef IGNORE_TYPE
	unsigned int      m_type;
#endif //IGNORE_TYPE
};


SIMD_FORCE_INLINE SimdVector3
SimdTransform::invXform(const SimdVector3& inVec) const
{
	SimdVector3 v = inVec - m_origin;
	return (m_basis.transpose() * v);
}

SIMD_FORCE_INLINE SimdTransform 
SimdTransform::inverseTimes(const SimdTransform& t) const  
{
	SimdVector3 v = t.getOrigin() - m_origin;
#ifndef IGNORE_TYPE
	if (m_type & SCALING) 
	{
		SimdMatrix3x3 inv = m_basis.inverse();
		return SimdTransform(inv * t.getBasis(), inv * v, 
			m_type | t.m_type);
	}
	else 
#else
	{
		return SimdTransform(m_basis.transposeTimes(t.m_basis),
			v * m_basis, 0);
	}
#endif //IGNORE_TYPE
}

SIMD_FORCE_INLINE SimdTransform 
SimdTransform::operator*(const SimdTransform& t) const
{
	return SimdTransform(m_basis * t.m_basis, 
		(*this)(t.m_origin), 
#ifndef IGNORE_TYPE
		m_type | t.m_type);
#else
		0);
#endif
}	



#endif





