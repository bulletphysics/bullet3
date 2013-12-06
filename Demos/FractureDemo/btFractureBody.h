
#ifndef BT_FRACTURE_BODY
#define BT_FRACTURE_BODY

class btCollisionShape;
class btDynamicsWorld;
class btCollisionWorld;
class btCompoundShape;
class btManifoldPoint;

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#define CUSTOM_FRACTURE_TYPE (btRigidBody::CO_USER_TYPE+1)


struct btConnection
{
	
	btCollisionShape*	m_childShape0;
	btCollisionShape*	m_childShape1;
	int	m_childIndex0;
	int	m_childIndex1;
	btScalar	m_strength;
};

class btFractureBody : public btRigidBody
{
	//connections
public:

	btDynamicsWorld*	m_world;
	btAlignedObjectArray<btScalar>	m_masses;
	btAlignedObjectArray<btConnection>	m_connections;



	btFractureBody(	const btRigidBodyConstructionInfo& constructionInfo, btDynamicsWorld*	world)
		:btRigidBody(constructionInfo),
		m_world(world)
	{
		m_masses.push_back(constructionInfo.m_mass);
		m_internalType=CUSTOM_FRACTURE_TYPE+CO_RIGID_BODY;
	}



	///btRigidBody constructor for backwards compatibility. 
	///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
	btFractureBody(	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia, btScalar* masses, int numMasses, btDynamicsWorld*	world)
		:btRigidBody(mass,motionState,collisionShape,localInertia),
		m_world(world)
	{

		for (int i=0;i<numMasses;i++)
			m_masses.push_back(masses[i]);

		m_internalType=CUSTOM_FRACTURE_TYPE+CO_RIGID_BODY;
	}

	

	void	recomputeConnectivity(btCollisionWorld* world);
	

	static btCompoundShape* shiftTransform(btCompoundShape* boxCompound,btScalar* masses,btTransform& shift, btVector3& principalInertia);
	
	static btCompoundShape* shiftTransformDistributeMass(btCompoundShape* boxCompound,btScalar mass,btTransform& shift);
	
	static bool collisionCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1);

};


void fractureCallback(btDynamicsWorld* world, btScalar timeStep);
void glueCallback(btDynamicsWorld* world, btScalar timeStep);

#endif //BT_FRACTURE_BODY
