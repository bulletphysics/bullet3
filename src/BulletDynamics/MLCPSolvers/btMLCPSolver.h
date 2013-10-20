/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///original version written by Erwin Coumans, October 2013

#ifndef BT_MLCP_SOLVER_H
#define BT_MLCP_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btMatrixX.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h"

class btMLCPSolver : public btSequentialImpulseConstraintSolver
{

protected:


	btMatrixXu m_A;
	btVectorXu m_b;
	btVectorXu m_x;
	btVectorXu m_lo;
	btVectorXu m_hi;
	
	btAlignedObjectArray<int> m_limitDependencies;
	btConstraintArray m_allConstraintArray;

	btMLCPSolverInterface* m_solver;

	virtual btScalar solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);
	virtual btScalar solveGroupCacheFriendlyIterations(btCollisionObject** bodies ,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);
	virtual void createMLCP(const btContactSolverInfo& infoGlobal);
	virtual void createMLCPFast(const btContactSolverInfo& infoGlobal);

	virtual void solveMLCP(const btContactSolverInfo& infoGlobal);

public:

	btMLCPSolver(	 btMLCPSolverInterface* solver);
	virtual ~btMLCPSolver();

	void setMLCPSolver(btMLCPSolverInterface* solver)
	{
		m_solver = solver;
	}

};


#endif //BT_MLCP_SOLVER_H
