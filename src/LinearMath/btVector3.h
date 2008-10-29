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



#ifndef SIMD__VECTOR3_H
#define SIMD__VECTOR3_H

#include "btQuadWord.h"

/**@brief btVector3 can be used to represent 3D points and vectors.
 * It has an un-used w component to suit 16-byte alignment when btVector3 is stored in containers. This extra component can be used by derived classes (Quaternion?) or by user
 * Ideally, this class should be replaced by a platform optimized SIMD version that keeps the data in registers
 */
class	btVector3 : public btQuadWord {

public:
  /**@brief No initialization constructor */
	SIMD_FORCE_INLINE btVector3() {}

  /**@brief Constructor from btQuadWordStorage (btVector3 inherits from this so is also valid)
   * Note: Vector3 derives from btQuadWordStorage*/
	SIMD_FORCE_INLINE btVector3(const btQuadWordStorage& q) 
		: btQuadWord(q)
	{
	}
	
  /**@brief Constructor from scalars 
   * @param x X value
   * @param y Y value 
   * @param z Z value 
   */
	SIMD_FORCE_INLINE btVector3(const btScalar& x, const btScalar& y, const btScalar& z) 
		:btQuadWord(x,y,z,btScalar(0.))
	{
	}

//	SIMD_FORCE_INLINE btVector3(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w) 
//		: btQuadWord(x,y,z,w)
//	{
//	}

	
/**@brief Add a vector to this one 
 * @param The vector to add to this one */
	SIMD_FORCE_INLINE btVector3& operator+=(const btVector3& v)
	{

		m_floats[0] += v.x(); m_floats[1] += v.y(); m_floats[2] += v.z();
		return *this;
	}


  /**@brief Subtract a vector from this one
   * @param The vector to subtract */
	SIMD_FORCE_INLINE btVector3& operator-=(const btVector3& v) 
	{
		m_floats[0] -= v.x(); m_floats[1] -= v.y(); m_floats[2] -= v.z();
		return *this;
	}
  /**@brief Scale the vector
   * @param s Scale factor */
	SIMD_FORCE_INLINE btVector3& operator*=(const btScalar& s)
	{
		m_floats[0] *= s; m_floats[1] *= s; m_floats[2] *= s;
		return *this;
	}

  /**@brief Inversely scale the vector 
   * @param s Scale factor to divide by */
	SIMD_FORCE_INLINE btVector3& operator/=(const btScalar& s) 
	{
		btFullAssert(s != btScalar(0.0));
		return *this *= btScalar(1.0) / s;
	}

  /**@brief Return the dot product
   * @param v The other vector in the dot product */
	SIMD_FORCE_INLINE btScalar dot(const btVector3& v) const
	{
		return m_floats[0] * v.x() + m_floats[1] * v.y() + m_floats[2] * v.z();
	}

  /**@brief Return the length of the vector squared */
	SIMD_FORCE_INLINE btScalar length2() const
	{
		return dot(*this);
	}

  /**@brief Return the length of the vector */
	SIMD_FORCE_INLINE btScalar length() const
	{
		return btSqrt(length2());
	}

  /**@brief Return the distance squared between the ends of this and another vector
   * This is symantically treating the vector like a point */
	SIMD_FORCE_INLINE btScalar distance2(const btVector3& v) const;

  /**@brief Return the distance between the ends of this and another vector
   * This is symantically treating the vector like a point */
	SIMD_FORCE_INLINE btScalar distance(const btVector3& v) const;

  /**@brief Normalize this vector 
   * x^2 + y^2 + z^2 = 1 */
	SIMD_FORCE_INLINE btVector3& normalize() 
	{
		return *this /= length();
	}

  /**@brief Return a normalized version of this vector */
	SIMD_FORCE_INLINE btVector3 normalized() const;

  /**@brief Rotate this vector 
   * @param wAxis The axis to rotate about 
   * @param angle The angle to rotate by */
	SIMD_FORCE_INLINE btVector3 rotate( const btVector3& wAxis, const btScalar angle );

  /**@brief Return the angle between this and another vector
   * @param v The other vector */
	SIMD_FORCE_INLINE btScalar angle(const btVector3& v) const 
	{
		btScalar s = btSqrt(length2() * v.length2());
		btFullAssert(s != btScalar(0.0));
		return btAcos(dot(v) / s);
	}
  /**@brief Return a vector will the absolute values of each element */
	SIMD_FORCE_INLINE btVector3 absolute() const 
	{
		return btVector3(
			btFabs(m_floats[0]), 
			btFabs(m_floats[1]), 
			btFabs(m_floats[2]));
	}
  /**@brief Return the cross product between this and another vector 
   * @param v The other vector */
	SIMD_FORCE_INLINE btVector3 cross(const btVector3& v) const
	{
		return btVector3(
			m_floats[1] * v.z() - m_floats[2] * v.y(),
			m_floats[2] * v.x() - m_floats[0] * v.z(),
			m_floats[0] * v.y() - m_floats[1] * v.x());
	}

	SIMD_FORCE_INLINE btScalar triple(const btVector3& v1, const btVector3& v2) const
	{
		return m_floats[0] * (v1.y() * v2.z() - v1.z() * v2.y()) + 
			m_floats[1] * (v1.z() * v2.x() - v1.x() * v2.z()) + 
			m_floats[2] * (v1.x() * v2.y() - v1.y() * v2.x());
	}

  /**@brief Return the axis with the smallest value 
   * Note return values are 0,1,2 for x, y, or z */
	SIMD_FORCE_INLINE int minAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[0] < m_floats[2] ? 0 : 2) : (m_floats[1] < m_floats[2] ? 1 : 2);
	}

  /**@brief Return the axis with the largest value 
   * Note return values are 0,1,2 for x, y, or z */
	SIMD_FORCE_INLINE int maxAxis() const 
	{
		return m_floats[0] < m_floats[1] ? (m_floats[1] < m_floats[2] ? 2 : 1) : (m_floats[0] < m_floats[2] ? 2 : 0);
	}

	SIMD_FORCE_INLINE int furthestAxis() const
	{
		return absolute().minAxis();
	}

	SIMD_FORCE_INLINE int closestAxis() const 
	{
		return absolute().maxAxis();
	}

	SIMD_FORCE_INLINE void setInterpolate3(const btVector3& v0, const btVector3& v1, btScalar rt)
	{
		btScalar s = btScalar(1.0) - rt;
		m_floats[0] = s * v0.x() + rt * v1.x();
		m_floats[1] = s * v0.y() + rt * v1.y();
		m_floats[2] = s * v0.z() + rt * v1.z();
		//don't do the unused w component
		//		m_co[3] = s * v0[3] + rt * v1[3];
	}

  /**@brief Return the linear interpolation between this and another vector 
   * @param v The other vector 
   * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
	SIMD_FORCE_INLINE btVector3 lerp(const btVector3& v, const btScalar& t) const 
	{
		return btVector3(m_floats[0] + (v.x() - m_floats[0]) * t,
			m_floats[1] + (v.y() - m_floats[1]) * t,
			m_floats[2] + (v.z() - m_floats[2]) * t);
	}

  /**@brief Elementwise multiply this vector by the other 
   * @param v The other vector */
	SIMD_FORCE_INLINE btVector3& operator*=(const btVector3& v)
	{
		m_floats[0] *= v.x(); m_floats[1] *= v.y(); m_floats[2] *= v.z();
		return *this;
	}

	

};

/**@brief Return the sum of two vectors (Point symantics)*/
SIMD_FORCE_INLINE btVector3 
operator+(const btVector3& v1, const btVector3& v2) 
{
	return btVector3(v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z());
}

/**@brief Return the elementwise product of two vectors */
SIMD_FORCE_INLINE btVector3 
operator*(const btVector3& v1, const btVector3& v2) 
{
	return btVector3(v1.x() * v2.x(), v1.y() * v2.y(), v1.z() * v2.z());
}

/**@brief Return the difference between two vectors */
SIMD_FORCE_INLINE btVector3 
operator-(const btVector3& v1, const btVector3& v2)
{
	return btVector3(v1.x() - v2.x(), v1.y() - v2.y(), v1.z() - v2.z());
}
/**@brief Return the negative of the vector */
SIMD_FORCE_INLINE btVector3 
operator-(const btVector3& v)
{
	return btVector3(-v.x(), -v.y(), -v.z());
}

/**@brief Return the vector scaled by s */
SIMD_FORCE_INLINE btVector3 
operator*(const btVector3& v, const btScalar& s)
{
	return btVector3(v.x() * s, v.y() * s, v.z() * s);
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
	return v * (btScalar(1.0) / s);
}

/**@brief Return the vector inversely scaled by s */
SIMD_FORCE_INLINE btVector3
operator/(const btVector3& v1, const btVector3& v2)
{
	return btVector3(v1.x() / v2.x(),v1.y() / v2.y(),v1.z() / v2.z());
}

/**@brief Return the dot product between two vectors */
SIMD_FORCE_INLINE btScalar 
dot(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.dot(v2); 
}


/**@brief Return the distance squared between two vectors */
SIMD_FORCE_INLINE btScalar
distance2(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.distance2(v2); 
}


/**@brief Return the distance between two vectors */
SIMD_FORCE_INLINE btScalar
distance(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.distance(v2); 
}

/**@brief Return the angle between two vectors */
SIMD_FORCE_INLINE btScalar
angle(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.angle(v2); 
}

/**@brief Return the cross product of two vectors */
SIMD_FORCE_INLINE btVector3 
cross(const btVector3& v1, const btVector3& v2) 
{ 
	return v1.cross(v2); 
}

SIMD_FORCE_INLINE btScalar
triple(const btVector3& v1, const btVector3& v2, const btVector3& v3)
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

/**@brief Test if each element of the vector is equivalent */
SIMD_FORCE_INLINE bool operator==(const btVector3& p1, const btVector3& p2) 
{
	return p1.x() == p2.x() && p1.y() == p2.y() && p1.z() == p2.z();
}

SIMD_FORCE_INLINE btScalar btVector3::distance2(const btVector3& v) const
{
	return (v - *this).length2();
}

SIMD_FORCE_INLINE btScalar btVector3::distance(const btVector3& v) const
{
	return (v - *this).length();
}

SIMD_FORCE_INLINE btVector3 btVector3::normalized() const
{
	return *this / length();
} 

SIMD_FORCE_INLINE btVector3 btVector3::rotate( const btVector3& wAxis, const btScalar angle )
{
	// wAxis must be a unit lenght vector

	btVector3 o = wAxis * wAxis.dot( *this );
	btVector3 x = *this - o;
	btVector3 y;

	y = wAxis.cross( *this );

	return ( o + x * btCos( angle ) + y * btSin( angle ) );
}

class btVector4 : public btVector3
{
public:

	SIMD_FORCE_INLINE btVector4() {}


	SIMD_FORCE_INLINE btVector4(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w) 
		: btVector3(x,y,z)
	{
		m_floats[3] = w;
	}


	SIMD_FORCE_INLINE btVector4 absolute4() const 
	{
		return btVector4(
			btFabs(m_floats[0]), 
			btFabs(m_floats[1]), 
			btFabs(m_floats[2]),
			btFabs(m_floats[3]));
	}



	btScalar	getW() const { return m_floats[3];}


		SIMD_FORCE_INLINE int maxAxis4() const
	{
		int maxIndex = -1;
		btScalar maxVal = btScalar(-1e30);
		if (m_floats[0] > maxVal)
		{
			maxIndex = 0;
			maxVal = m_floats[0];
		}
		if (m_floats[1] > maxVal)
		{
			maxIndex = 1;
			maxVal = m_floats[1];
		}
		if (m_floats[2] > maxVal)
		{
			maxIndex = 2;
			maxVal = m_floats[2];
		}
		if (m_floats[3] > maxVal)
		{
			maxIndex = 3;
			maxVal = m_floats[3];
		}
		
		
		

		return maxIndex;

	}


	SIMD_FORCE_INLINE int minAxis4() const
	{
		int minIndex = -1;
		btScalar minVal = btScalar(1e30);
		if (m_floats[0] < minVal)
		{
			minIndex = 0;
			minVal = m_floats[0];
		}
		if (m_floats[1] < minVal)
		{
			minIndex = 1;
			minVal = m_floats[1];
		}
		if (m_floats[2] < minVal)
		{
			minIndex = 2;
			minVal = m_floats[2];
		}
		if (m_floats[3] < minVal)
		{
			minIndex = 3;
			minVal = m_floats[3];
		}
		
		return minIndex;

	}


	SIMD_FORCE_INLINE int closestAxis4() const 
	{
		return absolute4().maxAxis4();
	}

};


///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
SIMD_FORCE_INLINE void	btSwapScalarEndian(const btScalar& sourceVal, btScalar& destVal)
{
	#ifdef BT_USE_DOUBLE_PRECISION
	unsigned char* dest = (unsigned char*) &destVal;
	unsigned char* src  = (unsigned char*) &sourceVal;
	dest[0] = src[7];
    dest[1] = src[6];
    dest[2] = src[5];
    dest[3] = src[4];
    dest[4] = src[3];
    dest[5] = src[2];
    dest[6] = src[1];
    dest[7] = src[0];
#else
	unsigned char* dest = (unsigned char*) &destVal;
	unsigned char* src  = (unsigned char*) &sourceVal;
	dest[0] = src[3];
    dest[1] = src[2];
    dest[2] = src[1];
    dest[3] = src[0];
#endif //BT_USE_DOUBLE_PRECISION
}
///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
SIMD_FORCE_INLINE void	btSwapVector3Endian(const btVector3& sourceVec, btVector3& destVec)
{
	for (int i=0;i<4;i++)
	{
		btSwapScalarEndian(sourceVec[i],destVec[i]);
	}

}

///btUnSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
SIMD_FORCE_INLINE void	btUnSwapVector3Endian(btVector3& vector)
{

	btVector3	swappedVec;
	for (int i=0;i<4;i++)
	{
		btSwapScalarEndian(vector[i],swappedVec[i]);
	}
	vector = swappedVec;
}

#endif //SIMD__VECTOR3_H
