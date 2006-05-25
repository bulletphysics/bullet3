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
#ifndef EPA_HALF_EDGE_H
#define EPA_HALF_EDGE_H

class EpaFace;
class EpaVertex;

//! Note : This class is not supposed to be a base class
class EpaHalfEdge
{
	private :

		//! Prevents copying objects from this class
		EpaHalfEdge( const EpaHalfEdge& epaHalfEdge );
		const EpaHalfEdge&		operator = ( const EpaHalfEdge& epaHalfEdge );

	public :

		EpaHalfEdge() : m_pTwin( 0 ), m_pNextCCW( 0 ), m_pFace( 0 ), m_pVertex( 0 )
		{
		}

		~EpaHalfEdge()
		{
		}

	public :

		//! Twin half-edge link
		EpaHalfEdge*			m_pTwin;

		//! Next half-edge in counter clock-wise ( CCW ) order
		EpaHalfEdge*			m_pNextCCW;

		//! Parent face link
		EpaFace*				m_pFace;

		//! Origin vertex link
		EpaVertex*				m_pVertex;
};

#endif

