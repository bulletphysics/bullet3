/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

EPA Copyright (c) Ricardo Padrela 2006 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef EPA_VERTEX_H
#define EPA_VERTEX_H

class EpaHalfEdge;

//! Note : This class is not supposed to be a base class
class EpaVertex
{
	private :

		//! Prevents copying objects from this class
		EpaVertex( const EpaVertex& epaVertex );
		const EpaVertex&	operator = ( const EpaVertex& epaVertex );

	public :

		EpaVertex( const SimdPoint3& point ) : /*m_pHalfEdge( 0 ),*/ m_point( point )
		{
		}

		EpaVertex( const SimdPoint3& point,
				   const SimdPoint3& wSupportPointOnA,
				   const SimdPoint3& wSupportPointOnB ) : /*m_pHalfEdge( 0 ),*/ m_point( point ),
														  m_wSupportPointOnA( wSupportPointOnA ),
														  m_wSupportPointOnB( wSupportPointOnB )
		{
		}

		~EpaVertex()
		{
		}

	public :

		//! This is not necessary
		//EpaHalfEdge*		m_pHalfEdge;

		SimdPoint3			m_point;

		SimdPoint3			m_wSupportPointOnA;
		SimdPoint3			m_wSupportPointOnB;
};

#endif

