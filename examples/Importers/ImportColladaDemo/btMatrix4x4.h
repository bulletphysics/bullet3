/*
Bullet Collision Detection and Physics Library http://bulletphysics.org
This file is Copyright (c) 2014 Google Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

//original author: Erwin Coumans
*/

#ifndef MATRIX4x4_H
#define MATRIX4x4_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

///This 4x4 matrix class is extremely limited, just created for the purpose of accumulating transform matrices in COLLADA .dae files
ATTRIBUTE_ALIGNED16(class)
btMatrix4x4
{
	btVector4 m_el[4];

public:
	btMatrix4x4()
	{
	}
	btMatrix4x4(const btScalar& xx, const btScalar& xy, const btScalar& xz, const btScalar& xw,
				const btScalar& yx, const btScalar& yy, const btScalar& yz, const btScalar& yw,
				const btScalar& zx, const btScalar& zy, const btScalar& zz, const btScalar& zw,
				const btScalar& wx, const btScalar& wy, const btScalar& wz, const btScalar& ww)
	{
		setValue(xx, xy, xz, xw,
				 yx, yy, yz, yw,
				 zx, zy, zz, zw,
				 wx, wy, wz, ww);
	}

	~btMatrix4x4()
	{
	}

	inline void setValue(const btScalar& xx, const btScalar& xy, const btScalar& xz, const btScalar& xw,
						 const btScalar& yx, const btScalar& yy, const btScalar& yz, const btScalar& yw,
						 const btScalar& zx, const btScalar& zy, const btScalar& zz, const btScalar& zw,
						 const btScalar& wx, const btScalar& wy, const btScalar& wz, const btScalar& ww)
	{
		m_el[0].setValue(xx, xy, xz, xw);
		m_el[1].setValue(yx, yy, yz, yw);
		m_el[2].setValue(zx, zy, zz, zw);
		m_el[3].setValue(wx, wy, wz, ww);
	}

	inline void setIdentity()
	{
		m_el[0].setValue(1, 0, 0, 0);
		m_el[1].setValue(0, 1, 0, 0);
		m_el[2].setValue(0, 0, 1, 0);
		m_el[3].setValue(0, 0, 0, 1);
	}
	inline void setPureRotation(const btQuaternion& orn)
	{
		setIdentity();

		btMatrix3x3 m3(orn);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m_el[i][j] = m3[i][j];
			}
		}
	}

	inline void setPureScaling(const btVector3& scale)
	{
		m_el[0].setValue(scale[0], 0, 0, 0);
		m_el[1].setValue(0, scale[1], 0, 0);
		m_el[2].setValue(0, 0, scale[2], 0);
		m_el[3].setValue(0, 0, 0, 1);
	}

	inline void setPureTranslation(const btVector3& pos)
	{
		m_el[0].setValue(1, 0, 0, pos[0]);
		m_el[1].setValue(0, 1, 0, pos[1]);
		m_el[2].setValue(0, 0, 1, pos[2]);
		m_el[3].setValue(0, 0, 0, 1);
	}
	SIMD_FORCE_INLINE const btVector4& operator[](int i) const
	{
		btFullAssert(0 <= i && i < 3);
		return m_el[i];
	}

	SIMD_FORCE_INLINE btScalar tdotx(const btVector4& v) const
	{
		return m_el[0].x() * v.x() + m_el[1].x() * v.y() + m_el[2].x() * v.z() + m_el[3].x() * v.w();
	}
	SIMD_FORCE_INLINE btScalar tdoty(const btVector4& v) const
	{
		return m_el[0].y() * v.x() + m_el[1].y() * v.y() + m_el[2].y() * v.z() + m_el[3].y() * v.w();
	}
	SIMD_FORCE_INLINE btScalar tdotz(const btVector4& v) const
	{
		return m_el[0].z() * v.x() + m_el[1].z() * v.y() + m_el[2].z() * v.z() + m_el[3].z() * v.w();
	}
	SIMD_FORCE_INLINE btScalar tdotw(const btVector4& v) const
	{
		return m_el[0].w() * v.x() + m_el[1].w() * v.y() + m_el[2].w() * v.z() + m_el[3].w() * v.w();
	}

	SIMD_FORCE_INLINE btMatrix4x4&
	operator*=(const btMatrix4x4& m)
	{
		setValue(
			m.tdotx(m_el[0]), m.tdoty(m_el[0]), m.tdotz(m_el[0]), m.tdotw(m_el[0]),
			m.tdotx(m_el[1]), m.tdoty(m_el[1]), m.tdotz(m_el[1]), m.tdotw(m_el[1]),
			m.tdotx(m_el[2]), m.tdoty(m_el[2]), m.tdotz(m_el[2]), m.tdotw(m_el[2]),
			m.tdotx(m_el[3]), m.tdoty(m_el[3]), m.tdotz(m_el[3]), m.tdotw(m_el[3]));
		return *this;
	}
};

inline btScalar btDot4(const btVector4& v0, const btVector4& v1)
{
	return v0.x() * v1.x() + v0.y() * v1.y() + v0.z() * v1.z() + v0.w() * v1.w();
}
SIMD_FORCE_INLINE btVector3
operator*(const btMatrix4x4& m, const btVector3& v1)
{
	btVector4 v(v1[0], v1[1], v1[2], 1);
	return btVector3(btDot4(m[0], v), btDot4(m[1], v), btDot4(m[2], v));
}

SIMD_FORCE_INLINE btMatrix4x4
operator*(const btMatrix4x4& m1, btMatrix4x4& m2)
{
	return btMatrix4x4(
		m2.tdotx(m1[0]), m2.tdoty(m1[0]), m2.tdotz(m1[0]), m2.tdotw(m1[0]),
		m2.tdotx(m1[1]), m2.tdoty(m1[1]), m2.tdotz(m1[1]), m2.tdotw(m1[1]),
		m2.tdotx(m1[2]), m2.tdoty(m1[2]), m2.tdotz(m1[2]), m2.tdotw(m1[2]),
		m2.tdotx(m1[3]), m2.tdoty(m1[3]), m2.tdotz(m1[3]), m2.tdotw(m1[3]));
}

#endif  //MATRIX4x4_H
