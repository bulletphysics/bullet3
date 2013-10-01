/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btMultiBodyDynamicsWorld.h"
#include "btMultiBodyConstraintSolver.h"
#include "btMultiBody.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btQuickprof.h"

btMultiBodyDynamicsWorld::btMultiBodyDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btMultiBodyConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration)
	:btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
{
	//split impulse is not yet supported for Featherstone hierarchies
	getSolverInfo().m_splitImpulse = false;
	getSolverInfo().m_solverMode |=SOLVER_USE_2_FRICTION_DIRECTIONS;
}
	
btMultiBodyDynamicsWorld::~btMultiBodyDynamicsWorld ()
{

}


void	btMultiBodyDynamicsWorld::addMultiBody(btMultiBody* body, short group, short mask)
{
	m_multiBodies.push_back(body);

}

void	btMultiBodyDynamicsWorld::removeMultiBody(btMultiBody* body)
{
	m_multiBodies.remove(body);
}

void	btMultiBodyDynamicsWorld::calculateSimulationIslands()
{
	BT_PROFILE("calculateSimulationIslands");

	getSimulationIslandManager()->updateActivationState(getCollisionWorld(),getCollisionWorld()->getDispatcher());

    {
        //merge islands based on speculative contact manifolds too
        for (int i=0;i<this->m_predictiveManifolds.size();i++)
        {
            btPersistentManifold* manifold = m_predictiveManifolds[i];
            
            const btCollisionObject* colObj0 = manifold->getBody0();
            const btCollisionObject* colObj1 = manifold->getBody1();
            
            if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
                ((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
            {
				getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(),(colObj1)->getIslandTag());
            }
        }
    }
    
	{
		int i;
		int numConstraints = int(m_constraints.size());
		for (i=0;i< numConstraints ; i++ )
		{
			btTypedConstraint* constraint = m_constraints[i];
			if (constraint->isEnabled())
			{
				const btRigidBody* colObj0 = &constraint->getRigidBodyA();
				const btRigidBody* colObj1 = &constraint->getRigidBodyB();

				if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
					((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
				{
					getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(),(colObj1)->getIslandTag());
				}
			}
		}
	}

	//merge islands linked by Featherstone links
	for (int i=0;i<m_multiBodies.size();i++)
	{
		btMultiBody* body = m_multiBodies[i];
		if (body->getNumLinkColliders()>1)
		{
			btMultiBodyLinkCollider* prev = body->getLinkCollider(0);
			for (int b=1;b<body->getNumLinkColliders();b++)
			{
				btMultiBodyLinkCollider* cur = body->getLinkCollider(b);

				if (((cur) && (!(cur)->isStaticOrKinematicObject())) &&
					((prev) && (!(prev)->isStaticOrKinematicObject())))
				{
					int tagPrev = prev->getIslandTag();
					int tagCur = cur->getIslandTag();
					getSimulationIslandManager()->getUnionFind().unite(tagPrev, tagCur);
				}
				prev = cur;
				
			}
		}
	}

	//Store the island id in each body
	getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());

}


void	btMultiBodyDynamicsWorld::updateActivationState(btScalar timeStep)
{
	BT_PROFILE("btMultiBodyDynamicsWorld::updateActivationState");

	
	
	for ( int i=0;i<m_multiBodies.size();i++)
	{
		btMultiBody* body = m_multiBodies[i];
		if (body)
		{
			body->checkMotionAndSleepIfRequired(timeStep);
			if (!body->isAwake())
			{
				for (int b=0;b<body->getNumLinkColliders();b++)
				{
					btMultiBodyLinkCollider* col = body->getLinkCollider(b);
					if (col->getActivationState() == ACTIVE_TAG)
					{
						col->setActivationState( WANTS_DEACTIVATION);
						col->setDeactivationTime(0.f);
					}
				}
			} else
			{
				for (int b=0;b<body->getNumLinkColliders();b++)
				{
					btMultiBodyLinkCollider* col = body->getLinkCollider(b);
					if (col->getActivationState() != DISABLE_DEACTIVATION)
						col->setActivationState( ACTIVE_TAG );
				}
			}

		}
	}

	btDiscreteDynamicsWorld::updateActivationState(timeStep);
}
void	btMultiBodyDynamicsWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	btVector3 g = m_gravity;
	btAlignedObjectArray<btScalar> scratch_r;
	btAlignedObjectArray<btVector3> scratch_v;
	btAlignedObjectArray<btMatrix3x3> scratch_m;


	{
		BT_PROFILE("btMultiBody addForce and stepVelocities");
		for (int i=0;i<this->m_multiBodies.size();i++)
		{
			btMultiBody* bod = m_multiBodies[i];

			bool isSleeping = false;
			if (bod->getNumLinkColliders() && bod->getLinkCollider(0)->getActivationState()==ISLAND_SLEEPING)
			{
				isSleeping = true;
			} 
			if (!isSleeping)
			{
				scratch_r.resize(bod->getNumLinks()+1);
				scratch_v.resize(bod->getNumLinks()+1);
				scratch_m.resize(bod->getNumLinks()+1);

				bod->clearForcesAndTorques();
				bod->addBaseForce(g * bod->getBaseMass());

				for (int j = 0; j < bod->getNumLinks(); ++j) 
				{
					bod->addLinkForce(j, g * bod->getLinkMass(j));
				}

				bod->stepVelocities(solverInfo.m_timeStep, scratch_r, scratch_v, scratch_m);
			}
		}
	}
	btDiscreteDynamicsWorld::solveConstraints(solverInfo);
}

void	btMultiBodyDynamicsWorld::integrateTransforms(btScalar timeStep)
{
	btDiscreteDynamicsWorld::integrateTransforms(timeStep);

	{
		BT_PROFILE("btMultiBody stepPositions");
		//integrate and update the Featherstone hierarchies
		btAlignedObjectArray<btQuaternion> world_to_local;
		btAlignedObjectArray<btVector3> local_origin;

		for (int b=0;b<m_multiBodies.size();b++)
		{
			btMultiBody* bod = m_multiBodies[b];
			bool isSleeping = false;
			if (bod->getNumLinkColliders() && bod->getLinkCollider(0)->getActivationState()==ISLAND_SLEEPING)
			{
				isSleeping = true;
			} 
			if (!isSleeping)
			{
				int nLinks = bod->getNumLinks();

				///base + num links
				world_to_local.resize(nLinks+1);
				local_origin.resize(nLinks+1);

				bod->stepPositions(timeStep);

	 

				world_to_local[0] = bod->getWorldToBaseRot();
				local_origin[0] = bod->getBasePos();
      
				for (int k=0;k<bod->getNumLinks();k++)
				{
					const int parent = bod->getParent(k);
					world_to_local[k+1] = bod->getParentToLocalRot(k) * world_to_local[parent+1];
					local_origin[k+1] = local_origin[parent+1] + (quatRotate(world_to_local[k+1].inverse() , bod->getRVector(k)));
				}

				for (int m=0;m<bod->getNumLinkColliders();m++)
				{
					btMultiBodyLinkCollider* col = bod->getLinkCollider(m);
					int link = col->m_link;
					int index = link+1;

					float halfExtents[3]={7.5,0.05,4.5};
					btVector3 posr = local_origin[index];
					float pos[4]={posr.x(),posr.y(),posr.z(),1};
			
					float quat[4]={-world_to_local[index].x(),-world_to_local[index].y(),-world_to_local[index].z(),world_to_local[index].w()};
					//app->drawBox(halfExtents, pos,quat);

					btTransform tr;
					tr.setIdentity();
					tr.setOrigin(posr);
					tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));

					col->setWorldTransform(tr);
				}
			} else
			{
				bod->clearVelocities();
			}
		}
	}
}
