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

#ifndef SIMULATION_ISLAND_MANAGER_H
#define SIMULATION_ISLAND_MANAGER_H

#include "BulletCollision/CollisionDispatch/btUnionFind.h"
#include "btCollisionCreateFunc.h"

class btCollisionWorld;
class btDispatcher;

///SimulationIslandManager creates and handles simulation islands, using btUnionFind
class btSimulationIslandManager
{
	btUnionFind m_unionFind;

public:
	btSimulationIslandManager();
	virtual ~btSimulationIslandManager();


	void InitUnionFind(int n);	
	
		
	btUnionFind& GetUnionFind() { return m_unionFind;}

	virtual	void	UpdateActivationState(btCollisionWorld* colWorld,btDispatcher* dispatcher);
	virtual	void	StoreIslandActivationState(btCollisionWorld* world);


	void	FindUnions(btDispatcher* dispatcher);

	

	struct	IslandCallback
	{
		virtual ~IslandCallback() {};

		virtual	void	ProcessIsland(class btPersistentManifold**	manifolds,int numManifolds) = 0;
	};

	void	BuildAndProcessIslands(btDispatcher* dispatcher,btCollisionObjectArray& collisionObjects, IslandCallback* callback);

};

#endif //SIMULATION_ISLAND_MANAGER_H

