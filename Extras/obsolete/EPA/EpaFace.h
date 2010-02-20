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
#ifndef EPA_FACE_H
#define EPA_FACE_H

class EpaVertex;
class EpaHalfEdge;

#ifdef EPA_POLYHEDRON_USE_PLANES
extern btScalar PLANE_THICKNESS;
#endif

//! Note : This class is not supposed to be a base class
class EpaFace
{
	private :

		//! Prevents copying objects from this class
		EpaFace( const EpaFace& epaFace );
		const EpaFace&	operator = ( const EpaFace& epaFace );

	public :

		EpaFace();
		~EpaFace();

		bool			Initialize();

#ifdef EPA_POLYHEDRON_USE_PLANES
		bool			CalculatePlane();
#endif
		void			CalcClosestPoint();
		void			CalcClosestPointOnA( btVector3& closestPointOnA );
		void			CalcClosestPointOnB( btVector3& closestPointOnB );

		bool			IsAffinelyDependent()							const;
		bool			IsClosestPointInternal()						const;

		void			CollectVertices( EpaVertex** ppVertices );

		//void			FixOrder();

	public :

		EpaHalfEdge*	m_pHalfEdge;

		// We keep vertices here so we don't need to call CollectVertices
		// every time we need them
		EpaVertex*		m_pVertices[ 3 ];

#ifdef EPA_POLYHEDRON_USE_PLANES
		btVector3		m_planeNormal;
		btScalar		m_planeDistance;
			
		//btVector3		m_robustPlaneNormal;
		//btScalar		m_robustPlaneDistance;
#endif

		btVector3		m_v;
		btScalar		m_vSqrd;

		btScalar		m_determinant;
		btScalar		m_lambdas[ 2 ];

		bool			m_deleted;
};

#endif

