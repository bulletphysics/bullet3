/*
Bullet Continuous Collision Detection and Physics Library - Parallel solver
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef SPU_PARALLELSOLVER_H
#define SPU_PARALLELSOLVER_H

#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "btThreadSupportInterface.h"
#include "LinearMath/btAlignedObjectArray.h"

class SolverTaskScheduler
{
protected:
	class	btThreadSupportInterface*	m_threadInterface;
	int						m_maxNumOutstandingTasks;

	unsigned int						m_currentTask;
	unsigned int						m_numBusyTasks;

	btAlignedObjectArray<struct SpuSolverTaskDesc>	m_taskDescriptors;
	btAlignedObjectArray<bool>						m_taskBusy;

public:
	SolverTaskScheduler (btThreadSupportInterface* threadIf, int maxOutstandingTasks);
	~SolverTaskScheduler ();

	struct SpuSolverTaskDesc* getTask ();

	void issueTask();
	void flushTasks();

	int getMaxOutstandingTasks()
	{
		return m_maxNumOutstandingTasks;
	}
};

class btParallelSequentialImpulseSolver : public btConstraintSolver
{
protected:

	struct SpuSolverHash*						m_solverHash;
	btAlignedObjectArray<struct ManifoldCellHolder>				m_sortedManifolds;
	btAlignedObjectArray<struct ConstraintCellHolder>				m_sortedConstraints;
	btAlignedObjectArray<class btRigidBody*>	m_allObjects;

	int											m_numberOfContacts;	

	SolverTaskScheduler							m_taskScheduler;

public:
	btParallelSequentialImpulseSolver (btThreadSupportInterface* threadIf, int maxOutstandingTasks);
	virtual ~btParallelSequentialImpulseSolver();

	virtual void prepareSolve (int numBodies, int numManifolds);
	virtual btScalar solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifold,int numManifolds,btTypedConstraint** constraints,int numConstraints, const btContactSolverInfo& info,class btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher);
	virtual void allSolved (const btContactSolverInfo& info,class btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc);
	virtual void reset ();
};

#endif