
#include "SimulationIslandManager.h"
#include "BroadphaseCollision/Dispatcher.h"
#include "NarrowPhaseCollision/PersistentManifold.h"
#include "CollisionDispatch/CollisionObject.h"
#include "CollisionDispatch/CollisionWorld.h"

#include <stdio.h>
#include <algorithm>


SimulationIslandManager::SimulationIslandManager()
{
}

void SimulationIslandManager::InitUnionFind(int n)
{
		m_unionFind.reset(n);
}
		

void SimulationIslandManager::FindUnions(Dispatcher* dispatcher)
{
	
	{
		for (int i=0;i<dispatcher->GetNumManifolds();i++)
		{
			const PersistentManifold* manifold = dispatcher->GetManifoldByIndexInternal(i);
			//static objects (invmass 0.f) don't merge !

			 const  CollisionObject* colObj0 = static_cast<const CollisionObject*>(manifold->GetBody0());
			 const  CollisionObject* colObj1 = static_cast<const CollisionObject*>(manifold->GetBody1());

			 if (colObj0 && colObj1 && dispatcher->NeedsResponse(*colObj0,*colObj1))
			 {
				if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
					((colObj1) && ((colObj1)->mergesSimulationIslands())))
				{

					m_unionFind.unite((colObj0)->m_islandTag1,
						(colObj1)->m_islandTag1);
				}
			 }
			
			
		}
	}
	
}


void	SimulationIslandManager::UpdateActivationState(CollisionWorld* colWorld,Dispatcher* dispatcher)
{
	
	InitUnionFind(colWorld->GetCollisionObjectArray().size());
	
	// put the index into m_controllers into m_tag	
	{
		std::vector<CollisionObject*>::iterator i;
		
		int index = 0;
		for (i=colWorld->GetCollisionObjectArray().begin();
		!(i==colWorld->GetCollisionObjectArray().end()); i++)
		{
			
			CollisionObject*	collisionObject= (*i);
			collisionObject->m_islandTag1 = index;
			collisionObject->m_hitFraction = 1.f;
			index++;
			
		}
	}
	// do the union find
	
	FindUnions(dispatcher);
	

	
}




void	SimulationIslandManager::StoreIslandActivationState(CollisionWorld* colWorld)
{
	// put the islandId ('find' value) into m_tag	
	{
		
		
		std::vector<CollisionObject*>::iterator i;
		
		int index = 0;
		for (i=colWorld->GetCollisionObjectArray().begin();
		!(i==colWorld->GetCollisionObjectArray().end()); i++)
		{
			CollisionObject* collisionObject= (*i);
			
			if (collisionObject->mergesSimulationIslands())
			{
				collisionObject->m_islandTag1 = m_unionFind.find(index);
			} else
			{
				collisionObject->m_islandTag1 = -1;
			}
			index++;
		}
	}
}

inline	int	getIslandId(const PersistentManifold* lhs)
{
	int islandId;
	const CollisionObject* rcolObj0 = static_cast<const CollisionObject*>(lhs->GetBody0());
	const CollisionObject* rcolObj1 = static_cast<const CollisionObject*>(lhs->GetBody1());
	islandId= rcolObj0->m_islandTag1>=0?rcolObj0->m_islandTag1:rcolObj1->m_islandTag1;
	return islandId;

}

bool PersistentManifoldSortPredicate(const PersistentManifold* lhs, const PersistentManifold* rhs)
{
	int rIslandId0,lIslandId0;
	rIslandId0 = getIslandId(rhs);
	lIslandId0 = getIslandId(lhs);
	return lIslandId0 < rIslandId0;
}


//
// todo: this is random access, it can be walked 'cache friendly'!
//
void SimulationIslandManager::BuildAndProcessIslands(Dispatcher* dispatcher,CollisionObjectArray& collisionObjects, IslandCallback* callback)
{
	

	int numBodies  = collisionObjects.size();

	//we are going to sort the unionfind array, and store the element id in the size
	//afterwards, we clean unionfind, to make sure no-one uses it anymore
	
	GetUnionFind().sortIslands();
	int numElem = GetUnionFind().getNumElements();

	int startIslandIndex=0,endIslandIndex=1;

	//update the sleeping state for bodies, if all are sleeping
	for (int startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
	{
		int islandId = GetUnionFind().getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (GetUnionFind().getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
		{
		}

		//int numSleeping = 0;

		bool allSleeping = true;

		int idx;
		for (idx=startIslandIndex;idx<endIslandIndex;idx++)
		{
			int i = GetUnionFind().getElement(idx).m_sz;

			CollisionObject* colObj0 = collisionObjects[i];
			if ((colObj0->m_islandTag1 != islandId) && (colObj0->m_islandTag1 != -1))
			{
				printf("error in island management\n");
			}

			assert((colObj0->m_islandTag1 == islandId) || (colObj0->m_islandTag1 == -1));
			if (colObj0->m_islandTag1 == islandId)
			{
				if (colObj0->GetActivationState()== ACTIVE_TAG)
				{
					allSleeping = false;
				}
				if (colObj0->GetActivationState()== DISABLE_DEACTIVATION)
				{
					allSleeping = false;
				}
			}
		}
		
		if (allSleeping)
		{
			int idx;
			for (idx=startIslandIndex;idx<endIslandIndex;idx++)
			{
				int i = GetUnionFind().getElement(idx).m_sz;
				CollisionObject* colObj0 = collisionObjects[i];
				if ((colObj0->m_islandTag1 != islandId) && (colObj0->m_islandTag1 != -1))
				{
					printf("error in island management\n");
				}

				assert((colObj0->m_islandTag1 == islandId) || (colObj0->m_islandTag1 == -1));

				if (colObj0->m_islandTag1 == islandId)
				{
					colObj0->SetActivationState( ISLAND_SLEEPING );
				}
			}
		} else
		{

			int idx;
			for (idx=startIslandIndex;idx<endIslandIndex;idx++)
			{
				int i = GetUnionFind().getElement(idx).m_sz;

				CollisionObject* colObj0 = collisionObjects[i];
				if ((colObj0->m_islandTag1 != islandId) && (colObj0->m_islandTag1 != -1))
				{
					printf("error in island management\n");
				}

				assert((colObj0->m_islandTag1 == islandId) || (colObj0->m_islandTag1 == -1));

				if (colObj0->m_islandTag1 == islandId)
				{
					if ( colObj0->GetActivationState() == ISLAND_SLEEPING)
					{
						colObj0->SetActivationState( WANTS_DEACTIVATION);
					}
				}
			}
		}
	}

	std::vector<PersistentManifold*>  islandmanifold;
	int i;
	int maxNumManifolds = dispatcher->GetNumManifolds();
	islandmanifold.reserve(maxNumManifolds);

	for (i=0;i<maxNumManifolds ;i++)
	{
		 PersistentManifold* manifold = dispatcher->GetManifoldByIndexInternal(i);
		 
		 CollisionObject* colObj0 = static_cast<CollisionObject*>(manifold->GetBody0());
		 CollisionObject* colObj1 = static_cast<CollisionObject*>(manifold->GetBody1());
		
		 //todo: check sleeping conditions!
		 if (((colObj0) && colObj0->GetActivationState() != ISLAND_SLEEPING) ||
			((colObj1) && colObj1->GetActivationState() != ISLAND_SLEEPING))
		{
			//filtering for response
			if (dispatcher->NeedsResponse(*colObj0,*colObj1))
				islandmanifold.push_back(manifold);
		}
	}

	int numManifolds = islandmanifold.size();

	// Sort manifolds, based on islands
	// Sort the vector using predicate and std::sort
	std::sort(islandmanifold.begin(), islandmanifold.end(), PersistentManifoldSortPredicate);

	//now process all active islands (sets of manifolds for now)

	int startManifoldIndex = 0;
	int endManifoldIndex = 1;

	for (startManifoldIndex=0;startManifoldIndex<numManifolds;startManifoldIndex = endManifoldIndex)
	{
		int islandId = getIslandId(islandmanifold[startManifoldIndex]);
		for (endManifoldIndex = startManifoldIndex+1;(endManifoldIndex<numManifolds) && (islandId == getIslandId(islandmanifold[endManifoldIndex]));endManifoldIndex++)
		{
		}
		/// Process the actual simulation, only if not sleeping/deactivated
		int numIslandManifolds = endManifoldIndex-startManifoldIndex;
		if (numIslandManifolds)
		{
			callback->ProcessIsland(&islandmanifold[startManifoldIndex],numIslandManifolds);
		}
	}
}
