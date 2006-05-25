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
#ifndef EPA_H
#define EPA_H

#define EPA_MAX_FACE_ENTRIES 256

extern const SimdScalar EPA_MAX_RELATIVE_ERROR;
extern const SimdScalar EPA_MAX_RELATIVE_ERROR_SQRD;

class Epa
{
	private :

		//! Prevents copying objects from this class
		Epa( const Epa& epa );
		const Epa&			operator = ( const Epa& epa );

	public :

		Epa( ConvexShape* pConvexShapeA, ConvexShape* pConvexShapeB,
			 const SimdTransform& transformA, const SimdTransform& transformB );
		~Epa();

		bool				Initialize( SimplexSolverInterface& simplexSolver );

		SimdScalar			CalcPenDepth( SimdPoint3& wWitnessOnA, SimdPoint3& wWitnessOnB );

	private :

		bool				TetrahedronContainsOrigin( SimdPoint3* pPoints );
		bool				TetrahedronContainsOrigin( const SimdPoint3& point0, const SimdPoint3& point1,
													   const SimdPoint3& point2, const SimdPoint3& point3 );

	private :

		//! Priority queue
		std::vector< EpaFace* >	m_faceEntries;

		ConvexShape*			m_pConvexShapeA;
		ConvexShape*			m_pConvexShapeB;

		SimdTransform			m_transformA;
		SimdTransform			m_transformB;

		EpaPolyhedron			m_polyhedron;
};

extern bool CompareEpaFaceEntries( EpaFace* pFaceA, EpaFace* pFaceB );

#endif

