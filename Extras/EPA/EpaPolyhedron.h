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
#ifndef EPA_POLYHEDRON_H
#define EPA_POLYHEDRON_H

class EpaFace;
class EpaVertex;

//! Note : This class is not supposed to be a base class
class EpaPolyhedron
{
	private :

		//! Prevents copying objects from this class
		EpaPolyhedron( const EpaPolyhedron& epaPolyhedron );
		const EpaPolyhedron& operator = ( const EpaPolyhedron& epaPolyhedron );

	public :

		EpaPolyhedron();
		~EpaPolyhedron();

		bool					Create( SimdPoint3* pInitialPoints,
										SimdPoint3* pSupportPointsOnA, SimdPoint3* pSupportPointsOnB,
										const int nbInitialPoints );
		void					Destroy();

		EpaFace*				CreateFace();
		EpaHalfEdge*			CreateHalfEdge();
		EpaVertex*				CreateVertex( const SimdPoint3& wSupportPoint,
											  const SimdPoint3& wSupportPointOnA,
											  const SimdPoint3& wSupportPointOnB );

		void					DeleteFace( EpaFace* pFace );

		void					DestroyAllFaces();
		void					DestroyAllHalfEdges();
		void					DestroyAllVertices();

		bool					Expand( const SimdPoint3& wSupportPoint,
										const SimdPoint3& wSupportPointOnA,
										const SimdPoint3& wSupportPointOnB,
										EpaFace* pFace, std::list< EpaFace* >& newFaces );

		std::list< EpaFace* >&	GetFaces();
		int						GetNbFaces()		const;

	private :

		void					DeleteVisibleFaces( const SimdPoint3& point, EpaFace* pFace,
													std::list< EpaHalfEdge* >& coneBaseTwinHalfEdges );

		void					CreateCone( EpaVertex* pAppexVertex, std::list< EpaHalfEdge* >& baseTwinHalfEdges,
											std::list< EpaFace* >& newFaces );
		EpaFace*				CreateConeFace( EpaVertex* pAppexVertex, EpaHalfEdge* pBaseTwinHalfEdge,
												std::list< EpaHalfEdge* >& halfEdgesToLink );

#ifdef _DEBUG
	public :
		//! Please don't remove this method, it will help debugging if some problems arise in the future
		bool					_dbgSaveToFile( const char* pFileName );
#endif

	private :

		//! This is the number of valid faces, m_faces list also contain deleted faces
		int							m_nbFaces;

		std::list< EpaFace* >		m_faces;
		std::list< EpaHalfEdge* >	m_halfEdges;
		std::list< EpaVertex* >		m_vertices;
};

#endif

