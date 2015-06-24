
#include "btFractureBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"



void	btFractureBody::recomputeConnectivity(btCollisionWorld* world)
{
	m_connections.clear();
	//@todo use the AABB tree to avoid N^2 checks

	if (getCollisionShape()->isCompound())
	{
		btCompoundShape* compound = (btCompoundShape*)getCollisionShape();
		for (int i=0;i<compound->getNumChildShapes();i++)
		{
			for (int j=i+1;j<compound->getNumChildShapes();j++)
			{

				struct   MyContactResultCallback : public btCollisionWorld::ContactResultCallback
				{
					bool m_connected;
					btScalar m_margin;
					MyContactResultCallback() :m_connected(false),m_margin(0.05)
					{
					}
					virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
					{
						if (cp.getDistance()<=m_margin)
							m_connected = true;
						return 1.f;
					}
			   };

				MyContactResultCallback result;

				btCollisionObject obA;
				obA.setWorldTransform(compound->getChildTransform(i));
				obA.setCollisionShape(compound->getChildShape(i));
				btCollisionObject obB;
				obB.setWorldTransform(compound->getChildTransform(j));
				obB.setCollisionShape(compound->getChildShape(j));
				world->contactPairTest(&obA,&obB,result);
				if (result.m_connected)
				{
					btConnection tmp;
					tmp.m_childIndex0 = i;
					tmp.m_childIndex1 = j;
					tmp.m_childShape0 = compound->getChildShape(i);
					tmp.m_childShape1 = compound->getChildShape(j);
					tmp.m_strength = 1.f;//??
					m_connections.push_back(tmp);
				}
			}
		}
	}
	

}

btCompoundShape* btFractureBody::shiftTransformDistributeMass(btCompoundShape* boxCompound,btScalar mass,btTransform& shift)
{

	btVector3 principalInertia;

	btScalar* masses = new btScalar[boxCompound->getNumChildShapes()];
	for (int j=0;j<boxCompound->getNumChildShapes();j++)
	{
		//evenly distribute mass
		masses[j]=mass/boxCompound->getNumChildShapes();
	}

	return shiftTransform(boxCompound,masses,shift,principalInertia);

}


btCompoundShape* btFractureBody::shiftTransform(btCompoundShape* boxCompound,btScalar* masses,btTransform& shift, btVector3& principalInertia)
{
	btTransform principal;

	boxCompound->calculatePrincipalAxisTransform(masses,principal,principalInertia);


	///create a new compound with world transform/center of mass properly aligned with the principal axis

	///non-recursive compound shapes perform better
	
#ifdef USE_RECURSIVE_COMPOUND

	btCompoundShape* newCompound = new btCompoundShape();
	newCompound->addChildShape(principal.inverse(),boxCompound);
	newBoxCompound = newCompound;
	//m_collisionShapes.push_back(newCompound);

	//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,newCompound,principalInertia);

#else
#ifdef CHANGE_COMPOUND_INPLACE
	newBoxCompound = boxCompound;
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		boxCompound->updateChildTransform(i,newChildTransform);
	}
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		boxCompound->calculateLocalInertia(mass,localInertia);
	
#else
	///creation is faster using a new compound to store the shifted children
	btCompoundShape* newBoxCompound = new btCompoundShape();
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		newBoxCompound->addChildShape(newChildTransform,boxCompound->getChildShape(i));
	}



#endif

#endif//USE_RECURSIVE_COMPOUND

	shift = principal;
	return newBoxCompound;
}






