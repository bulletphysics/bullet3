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


#ifndef SIMD_QUADWORD_H
#define SIMD_QUADWORD_H

#include "btScalar.h"
#include "btSimdMinMax.h"

class btQuadWordStorage
{
protected:
#ifdef BT_USE_DOUBLE_PRECISION
	union { btScalar	m_x;  unsigned char m_charx[8] ;};
	union { btScalar	m_y;  unsigned char m_chary[8] ; };
	union { btScalar	m_z;  unsigned char m_charz[8] ; };
	union { btScalar	m_unusedW;  unsigned char m_charw[8] ; };
#else
	union { btScalar	m_x; unsigned int m_intx; unsigned char m_charx[4] ;};
	union { btScalar	m_y; unsigned int m_inty; unsigned char m_chary[4] ; };
	union { btScalar	m_z; unsigned int m_intz; unsigned char m_charz[4] ; };
	union { btScalar	m_unusedW; unsigned int m_intw; unsigned char m_charw[4] ; };
#endif //BT_USE_DOUBLE_PRECISION
};


///btQuadWord is base-class for vectors, points
class	btQuadWord : public btQuadWordStorage
{
	public:
	
//		SIMD_FORCE_INLINE btScalar&       operator[](int i)       { return (&m_x)[i];	}      
//		SIMD_FORCE_INLINE const btScalar& operator[](int i) const { return (&m_x)[i]; }

		SIMD_FORCE_INLINE const btScalar& getX() const { return m_x; }

		SIMD_FORCE_INLINE const btScalar& getY() const { return m_y; }

		SIMD_FORCE_INLINE const btScalar& getZ() const { return m_z; }

		SIMD_FORCE_INLINE void	setX(btScalar x) { m_x = x;};

		SIMD_FORCE_INLINE void	setY(btScalar y) { m_y = y;};

		SIMD_FORCE_INLINE void	setZ(btScalar z) { m_z = z;};

		SIMD_FORCE_INLINE void	setW(btScalar w) { m_unusedW = w;};

		SIMD_FORCE_INLINE const btScalar& x() const { return m_x; }

		SIMD_FORCE_INLINE const btScalar& y() const { return m_y; }

		SIMD_FORCE_INLINE const btScalar& z() const { return m_z; }

		SIMD_FORCE_INLINE const btScalar& w() const { return m_unusedW; }


		SIMD_FORCE_INLINE	operator       btScalar *()       { return &m_x; }
		SIMD_FORCE_INLINE	operator const btScalar *() const { return &m_x; }

#ifdef BT_USE_DOUBLE_PRECISION
		SIMD_FORCE_INLINE const unsigned char* getLongIntXValue() const
		{
			return &m_charx[0];
		}
		SIMD_FORCE_INLINE const unsigned char* getLongIntYValue() const
		{
			return &m_chary[0];;
		}
		SIMD_FORCE_INLINE const unsigned char* getLongIntZValue() const
		{
			return &m_charz[0];;
		}
		SIMD_FORCE_INLINE const unsigned char* getLongIntWValue() const
		{
			return &m_charw[0];;
		}
		SIMD_FORCE_INLINE void 	setXValueByLongInt(unsigned char* intval)
		{
			int i;
			for (i=0;i<8;i++)
				m_charx[i] = intval[i];
		}

		SIMD_FORCE_INLINE void 	setYValueByLongInt(unsigned char* intval)
		{
			int i;
			for (i=0;i<8;i++)
				m_chary[i] = intval[i];
		}

		SIMD_FORCE_INLINE void 	setZValueByLongInt(unsigned char* intval)
		{
			int i;
			for (i=0;i<8;i++)
				m_charz[i] = intval[i];
		}
		SIMD_FORCE_INLINE void 	setWValueByLongInt(unsigned char* intval)
		{
			int i;
			for (i=0;i<8;i++)
				m_charw[i] = intval[i];
		}
#else
		SIMD_FORCE_INLINE unsigned int getIntXValue() const
		{
			return m_intx;
		}
		SIMD_FORCE_INLINE unsigned  int getIntYValue() const
		{
			return m_inty;
		}
		SIMD_FORCE_INLINE unsigned int getIntZValue() const
		{
			return m_intz;
		}
		SIMD_FORCE_INLINE unsigned int getIntWValue() const
		{
			return m_intw;
		}
		SIMD_FORCE_INLINE void 	setXValueByInt(unsigned int intval)
		{
			m_intx = intval;
		}

		SIMD_FORCE_INLINE void 	setYValueByInt(unsigned int intval)
		{
			m_inty = intval;
		}

		SIMD_FORCE_INLINE void 	setZValueByInt(unsigned int intval)
		{
			m_intz = intval;
		}
		SIMD_FORCE_INLINE void 	setWValueByInt(unsigned int intval)
		{
			m_intw = intval;
		}

#endif//BT_USE_DOUBLE_PRECISION

		SIMD_FORCE_INLINE void 	setValue(const btScalar& x, const btScalar& y, const btScalar& z)
		{
			m_x=x;
			m_y=y;
			m_z=z;
			m_unusedW = 0.f;
		}

/*		void getValue(btScalar *m) const 
		{
			m[0] = m_x;
			m[1] = m_y;
			m[2] = m_z;
		}
*/
		SIMD_FORCE_INLINE void	setValue(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w)
		{
			m_x=x;
			m_y=y;
			m_z=z;
			m_unusedW=w;
		}

		SIMD_FORCE_INLINE btQuadWord()
		//	:m_x(btScalar(0.)),m_y(btScalar(0.)),m_z(btScalar(0.)),m_unusedW(btScalar(0.))
		{
		}

		SIMD_FORCE_INLINE btQuadWord(const btQuadWordStorage& q)
		{
			*((btQuadWordStorage*)this) = q;
		}

		SIMD_FORCE_INLINE btQuadWord(const btScalar& x, const btScalar& y, const btScalar& z)		
		{
			m_x = x, m_y = y, m_z = z, m_unusedW = 0.0f;
		}

		SIMD_FORCE_INLINE btQuadWord(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w) 
		{
			m_x = x, m_y = y, m_z = z, m_unusedW = w;
		}


		SIMD_FORCE_INLINE void	setMax(const btQuadWord& other)
		{
			btSetMax(m_x, other.m_x);
			btSetMax(m_y, other.m_y);
			btSetMax(m_z, other.m_z);
			btSetMax(m_unusedW, other.m_unusedW);
		}

		SIMD_FORCE_INLINE void	setMin(const btQuadWord& other)
		{
			btSetMin(m_x, other.m_x);
			btSetMin(m_y, other.m_y);
			btSetMin(m_z, other.m_z);
			btSetMin(m_unusedW, other.m_unusedW);
		}



};

#endif //SIMD_QUADWORD_H
