#include "b3CpuRigidBodyPipeline.h"

#include "Bullet3Dynamics/shared/b3IntegrateTransforms.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h"
#include "Bullet3Collision/BroadPhaseCollision/shared/b3Aabb.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3CollidableData.h"


struct b3CpuRigidBodyPipelineInternalData
{
	b3AlignedObjectArray<b3RigidBodyData> m_rigidBodies;
	b3AlignedObjectArray<b3Aabb> m_aabbWorldSpace;

	b3DynamicBvhBroadphase* m_bp;
	b3CpuNarrowPhase* m_np;
	b3Config m_config;
};
	

b3CpuRigidBodyPipeline::b3CpuRigidBodyPipeline(class b3CpuNarrowPhase* narrowphase, struct b3DynamicBvhBroadphase* broadphaseDbvt, const b3Config& config)
{
	m_data = new b3CpuRigidBodyPipelineInternalData;
	m_data->m_np = narrowphase;
	m_data->m_bp = broadphaseDbvt;
	m_data->m_config = config;
}

b3CpuRigidBodyPipeline::~b3CpuRigidBodyPipeline()
{
	delete m_data;
}

void b3CpuRigidBodyPipeline::updateAabbWorldSpace()
{

	for (int i=0;i<this->getNumBodies();i++)
	{
		b3RigidBodyData* body = &m_data->m_rigidBodies[i];
		b3Float4 position = body->m_pos;
		b3Quat	orientation = body->m_quat;

		int collidableIndex = body->m_collidableIdx;
		b3Collidable& collidable = m_data->m_np->getCollidableCpu(collidableIndex);
		int shapeIndex = collidable.m_shapeIndex;
		
		if (shapeIndex>=0)
		{
			

			b3Aabb localAabb = m_data->m_np->getLocalSpaceAabb(shapeIndex);
			b3Aabb& worldAabb = m_data->m_aabbWorldSpace[i];
			float margin=0.f;
			b3TransformAabb2(localAabb.m_minVec,localAabb.m_maxVec,margin,position,orientation,&worldAabb.m_minVec,&worldAabb.m_maxVec);
			m_data->m_bp->setAabb(i,worldAabb.m_minVec,worldAabb.m_maxVec,0);
		}
	}
}

void	b3CpuRigidBodyPipeline::computeOverlappingPairs()
{
	int numPairs = m_data->m_bp->getOverlappingPairCache()->getNumOverlappingPairs();
	m_data->m_bp->calculateOverlappingPairs();
	numPairs = m_data->m_bp->getOverlappingPairCache()->getNumOverlappingPairs();
	//printf("numPairs=%d\n",numPairs);
}

void b3CpuRigidBodyPipeline::computeContactPoints()
{
	
	b3AlignedObjectArray<b3Int4>& pairs = m_data->m_bp->getOverlappingPairCache()->getOverlappingPairArray();
	
	m_data->m_np->computeContacts(pairs,m_data->m_aabbWorldSpace, m_data->m_rigidBodies);

}
void	b3CpuRigidBodyPipeline::stepSimulation(float deltaTime)
{
	
	//update world space aabb's
	updateAabbWorldSpace();

	//compute overlapping pairs
	computeOverlappingPairs();

	//compute contacts
	computeContactPoints();

	//solve contacts
	
	//update transforms
	integrate(deltaTime);
	
	
}


void b3CpuRigidBodyPipeline::integrate(float deltaTime)
{
	float angDamping=0.f;
	b3Vector3 gravityAcceleration=b3MakeVector3(0,-9,0);

	//integrate transforms (external forces/gravity should be moved into constraint solver)
	for (int i=0;i<m_data->m_rigidBodies.size();i++)
	{
		b3IntegrateTransform(&m_data->m_rigidBodies[i],deltaTime,angDamping,gravityAcceleration);
	}

}

int		b3CpuRigidBodyPipeline::registerPhysicsInstance(float mass, const float* position, const float* orientation, int collidableIndex, int userData)
{
	b3RigidBodyData body;
	int bodyIndex = m_data->m_rigidBodies.size();
	body.m_invMass = mass ? 1.f/mass : 0.f;
	body.m_angVel.setValue(0,0,0);
	body.m_collidableIdx = collidableIndex;
	body.m_frictionCoeff = 0.3f;
	body.m_linVel.setValue(0,0,0);
	body.m_pos.setValue(position[0],position[1],position[2]);
	body.m_quat.setValue(orientation[0],orientation[1],orientation[2],orientation[3]);
	body.m_restituitionCoeff = 0.f;

	m_data->m_rigidBodies.push_back(body);

	
	if (collidableIndex>=0)
	{
		b3Aabb& worldAabb = m_data->m_aabbWorldSpace.expand();

		b3Aabb localAabb = m_data->m_np->getLocalSpaceAabb(collidableIndex);
		b3Vector3 localAabbMin=b3MakeVector3(localAabb.m_min[0],localAabb.m_min[1],localAabb.m_min[2]);
		b3Vector3 localAabbMax=b3MakeVector3(localAabb.m_max[0],localAabb.m_max[1],localAabb.m_max[2]);
		
		b3Scalar margin = 0.01f;
		b3Transform t;
		t.setIdentity();
		t.setOrigin(b3MakeVector3(position[0],position[1],position[2]));
		t.setRotation(b3Quaternion(orientation[0],orientation[1],orientation[2],orientation[3]));
		b3TransformAabb(localAabbMin,localAabbMax, margin,t,worldAabb.m_minVec,worldAabb.m_maxVec);

		m_data->m_bp->createProxy(worldAabb.m_minVec,worldAabb.m_maxVec,bodyIndex,0,1,1);
//		b3Vector3 aabbMin,aabbMax;
	//	m_data->m_bp->getAabb(bodyIndex,aabbMin,aabbMax);

	} else
	{
		b3Error("registerPhysicsInstance using invalid collidableIndex\n");
	}

	return bodyIndex;
}


const struct b3RigidBodyData* b3CpuRigidBodyPipeline::getBodyBuffer() const
{
	return m_data->m_rigidBodies.size() ? &m_data->m_rigidBodies[0] : 0;
}

int	b3CpuRigidBodyPipeline::getNumBodies() const
{
	return m_data->m_rigidBodies.size();
}
