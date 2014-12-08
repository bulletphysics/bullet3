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

#ifndef BT_SIMULATION_ISLAND_MANAGER_H
#define BT_SIMULATION_ISLAND_MANAGER_H

#include "BulletCollision/CollisionDispatch/btUnionFind.h"
#include "btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btCollisionObject.h"

class btCollisionObject;
class btCollisionWorld;
class btDispatcher;
class btPersistentManifold;
class btTypedConstraint;


///SimulationIslandManager creates and handles simulation islands, using btUnionFind
class btSimulationIslandManager
{
public:
    struct Island
    {
        // a simulation island consisting of bodies, manifolds and constraints,
        // to be passed into a constraint solver.
        btAlignedObjectArray<btCollisionObject*> bodyArray;
        btAlignedObjectArray<btPersistentManifold*> manifoldArray;
        btAlignedObjectArray<btTypedConstraint*> constraintArray;
        int id;  // island id
        bool isSleeping;

        void append( const Island& other );  // add bodies, manifolds, constraints to my own
    };

private:
    btUnionFind m_unionFind;
    btAlignedObjectArray<Island*> m_allocatedIslands;  // owner of all Islands
    btAlignedObjectArray<Island*> m_usedIslands;  // islands actively in use
    btAlignedObjectArray<Island*> m_freeIslands;  // islands ready to be reused
    btAlignedObjectArray<Island*> m_lookupIslandFromId;  // big lookup table to map islandId to Island pointer
    int m_minimumSolverBatchSize;
	bool m_splitIslands;

    Island* getIsland( int id );
    Island* allocateIsland( int id, int numBodies );
    void initIslandPools();
	
public:
	btSimulationIslandManager();
	virtual ~btSimulationIslandManager();


	void initUnionFind(int n);	
	
		
	btUnionFind& getUnionFind() { return m_unionFind;}

	virtual	void	updateActivationState(btCollisionWorld* colWorld,btDispatcher* dispatcher);
	virtual	void	storeIslandActivationState(btCollisionWorld* world);


	void	findUnions(btDispatcher* dispatcher,btCollisionWorld* colWorld);

	struct	IslandCallback
	{
		virtual ~IslandCallback() {};

        virtual	void processIsland( btCollisionObject** bodies,
                                    int numBodies,
                                    btPersistentManifold** manifolds,
                                    int numManifolds,
                                    btTypedConstraint** constraints,
                                    int numConstraints,
                                    int islandId
                                    ) = 0;
	};

    virtual void buildAndProcessIslands( btDispatcher* dispatcher, btCollisionWorld* collisionWorld, btAlignedObjectArray<btTypedConstraint*>& constraints, IslandCallback* callback );

	virtual void buildIslands(btDispatcher* dispatcher,btCollisionWorld* colWorld);

	bool getSplitIslands()
	{
		return m_splitIslands;
	}
	void setSplitIslands(bool doSplitIslands)
	{
		m_splitIslands = doSplitIslands;
	}
    int getMinimumSolverBatchSize() const
    {
        return m_minimumSolverBatchSize;
    }
    void setMinimumSolverBatchSize( int sz )
    {
        m_minimumSolverBatchSize = sz;
    }
};

#endif //BT_SIMULATION_ISLAND_MANAGER_H

