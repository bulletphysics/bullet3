/*
   Copyright (C) 2010 Sony Computer Entertainment Inc.
   All rights reserved.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#ifndef __BT_LOW_LEVEL_CONSTRAINT_SOLVER2_H
#define __BT_LOW_LEVEL_CONSTRAINT_SOLVER2_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"




#include "LinearMath/btScalar.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"
#include "physics_effects/low_level/pfx_low_level_include.h"
#include "../src/low_level/solver/pfx_parallel_group.h"

using namespace sce::PhysicsEffects;


class btPersistentManifold;
struct btLowLevelData;

class btLowLevelConstraintSolver2 : public btSequentialImpulseConstraintSolver
{
	
protected:

	btLowLevelData*		m_lowLevelData;

public:

	btLowLevelConstraintSolver2(btLowLevelData* lowLevelData);
	
	virtual ~btLowLevelConstraintSolver2();

	virtual btScalar solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifold,int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher);

};



#endif //__BT_LOW_LEVEL_CONSTRAINT_SOLVER2_H