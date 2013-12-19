/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef GL_SIMPLEX_1TO4_H
#define GL_SIMPLEX_1TO4_H

#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"

#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"

///GL_Simplex1to4 is a class to debug a Simplex Solver with 1 to 4 points. 
///Can be used by GJK.
class GL_Simplex1to4 : public btBU_Simplex1to4
{
	btSimplexSolverInterface*	m_simplexSolver;

	public:

	GL_Simplex1to4();
	virtual ~GL_Simplex1to4();

	void	calcClosest(btScalar* m);

	void	setSimplexSolver(btSimplexSolverInterface* simplexSolver) {
		m_simplexSolver = simplexSolver;
	}

};

#endif //GL_SIMPLEX_1TO4_H
