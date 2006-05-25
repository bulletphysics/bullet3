
#include "BulletOdeCollide.h"

//quick hack, we need internals, not just the public ode api
#include <../ode/src/collision_kernel.h>

#include "NarrowPhaseCollision/PersistentManifold.h"
#include "NarrowPhaseCollision/ManifoldPoint.h"

#include "BroadphaseCollision/CollisionAlgorithm.h"
#include "BulletOdeCollisionPair.h"
#include "../ode/src/collision_convex_internal.h"

#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + (skip)))


//Persistent collision pairs. Best is to store this in the broadphase/collision-pairs if the collision detection framework provides this
#define MAX_COLLISION_PAIRS 10000
static BulletOdeCollisionPair*	sCollisionPair[MAX_COLLISION_PAIRS];
static int numActiveCollisionPairs = 0;

void InitBulletOdeCollide()
{
	numActiveCollisionPairs  = 0;
}

void ExitBulletOdeCollide()
{
	//todo: cleanup memory
	numActiveCollisionPairs  = 0;
}



CollisionShape* GetCollisionShapeFromConvex(dGeomID geom)
{
	dUASSERT (geom && geom->type == dConvexClass,"argument not a convex");
	dxConvex* cnvx = (dxConvex*) geom;
	return cnvx->m_bulletCollisionShape;
}


BulletOdeCollisionPair*	FindCollisionPair(dGeomID o1,dGeomID o2)
{
	int i;
	for (i=0;i<numActiveCollisionPairs;i++)
	{
		if ( (sCollisionPair[i]->m_o1 == o1) && 
			(sCollisionPair[i]->m_o2 == o2))
		{
			return sCollisionPair[i];
		}
	}
	return 0;
}


void	RemoveOdeGeomFromCollisionCache(dGeomID geom)
{
	int i;
	for (i=numActiveCollisionPairs-1;i>=0;i--)
	{
		if ( (sCollisionPair[i]->m_o1 == geom) || 
			(sCollisionPair[i]->m_o2 == geom))
		{
			delete sCollisionPair[i];
			sCollisionPair[i] = sCollisionPair[numActiveCollisionPairs-1];
			numActiveCollisionPairs--;
		}
	}
}

int	BulletOdeCollide(dGeomID o1,dGeomID o2,dContactGeom *contact,int maxContact,int skip)
{

	//In order to have more then 1 point we use persistent manifold per overlapping pair
	BulletOdeCollisionPair* collisionPair = FindCollisionPair(o1,o2);
	if (!collisionPair)
	{
		if (numActiveCollisionPairs < MAX_COLLISION_PAIRS)
		{
			collisionPair = new BulletOdeCollisionPair(o1,o2);
			sCollisionPair[numActiveCollisionPairs++] = collisionPair ;
		} else
		{
			printf("overflow in collisionpair cache\n");
			return 0;
		}
	}

	//perform collision detection and recalculate the contact manifold
	collisionPair->CalculateContacts();

	//transfer contacts from PersistentManifold to gContactGeom
	int validContacts = 0;
	float maxDepth = -1e30f;

	for (int i=0; i<collisionPair->m_manifold->GetNumContacts(); i++) 
	{
		const ManifoldPoint& pt = collisionPair->m_manifold->GetContactPoint(i);

		if (pt.GetDistance() < 0.f)
		{
			float depth = -pt.GetDistance();

			//add contacts, and make sure to add the deepest contact in any case
			if ((validContacts < maxContact) || (depth > maxDepth))
			{
				maxDepth = depth;
				int contactIndex = validContacts;

				if (contactIndex >= maxContact)
					contactIndex = 0;

				if (contactIndex < maxContact)
				{
					SimdVector3 pos = (pt.m_positionWorldOnB + pt.m_positionWorldOnA)*0.5f;

					CONTACT(contact,contactIndex*skip)->depth = depth;
					CONTACT(contact,contactIndex*skip)->pos[0] = pos.getX();
					CONTACT(contact,contactIndex*skip)->pos[1] = pos.getY();
					CONTACT(contact,contactIndex*skip)->pos[2] = pos.getZ();
					CONTACT(contact,contactIndex*skip)->normal[0] = pt.m_normalWorldOnB.getX();
					CONTACT(contact,contactIndex*skip)->normal[1] = pt.m_normalWorldOnB.getY();
					CONTACT(contact,contactIndex*skip)->normal[2] = pt.m_normalWorldOnB.getZ();
					CONTACT(contact,contactIndex*skip)->g1 = o1;
					CONTACT(contact,contactIndex*skip)->g2 = o2;
				}
				if (validContacts < maxContact)
					validContacts++;
			}
			
		}
	}
	
	//printf("numcontacts: %d",validContacts);

	return validContacts;
}