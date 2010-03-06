/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#ifndef BT_HFFLUID_RIGID_DYNAMICS_WORLD_H
#define BT_HFFLUID_RIGID_DYNAMICS_WORLD_H

class btHfFluid;
typedef	btAlignedObjectArray<btHfFluid*> btHfFluidArray;

#define DRAWMODE_NORMAL 0
#define DRAWMODE_VELOCITY 1
#define DRAWMODE_MAX 2

#define BODY_DRAWMODE_NORMAL 0
#define BODY_DRAWMODE_VOXEL 1
#define BODY_DRAWMODE_MAX 2

class btHfFluidBuoyantConvexShape;

///experimental buyancy fluid demo
class btHfFluidRigidDynamicsWorld : public btDiscreteDynamicsWorld
{
	
	btHfFluidArray	m_hfFluids;
	int m_drawMode;
	int m_bodyDrawMode;
protected:
	
	virtual void	predictUnconstraintMotion(btScalar timeStep);
	
	virtual void	internalSingleStepSimulation( btScalar timeStep);

	void	updateFluids(btScalar timeStep);

	void	solveFluidConstraints(btScalar timeStep);

	virtual void	debugDrawWorld();

	void drawHfFluidGround (btIDebugDraw* debugDraw, btHfFluid* fluid);
	void drawHfFluidVelocity (btIDebugDraw* debugDraw, btHfFluid* fluid);
	void drawHfFluidBuoyantConvexShape (btIDebugDraw* debugDrawer, btCollisionObject* object, btHfFluidBuoyantConvexShape* buoyantShape, int voxelDraw);
	void drawHfFluidNormal (btIDebugDraw* debugDraw, btHfFluid* fluid);
public:
	
	btHfFluidRigidDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration);

	virtual ~btHfFluidRigidDynamicsWorld();
		
			
	void	addHfFluid(btHfFluid* fluid);

	void	removeHfFluid(btHfFluid* fluid);
	
	void setDrawMode (int drawMode)
	{
		m_drawMode = drawMode;
	}

	void setBodyDrawMode (int bodyDrawMode)
	{
		m_bodyDrawMode = bodyDrawMode;
	}

	btHfFluidArray& getHfFluidArray()
	{
		return m_hfFluids;
	}

	const btHfFluidArray& getHfFluidArray() const
	{
		return m_hfFluids;
	}
		
};

#endif //BT_HFFLUID_RIGID_DYNAMICS_WORLD_H
