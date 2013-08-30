#include "b3CpuRigidBodyPipeline.h"

#include "Bullet3Dynamics/shared/b3IntegrateTransforms.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h"


struct b3CpuRigidBodyPipelineInternalData
{
	b3AlignedObjectArray<b3RigidBodyData> m_rigidBodies;
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

}

void	b3CpuRigidBodyPipeline::computeOverlappingPairs()
{
	int numPairs = m_data->m_bp->getOverlappingPairCache()->getNumOverlappingPairs();
	m_data->m_bp->calculateOverlappingPairs();
	numPairs = m_data->m_bp->getOverlappingPairCache()->getNumOverlappingPairs();
}

void b3CpuRigidBodyPipeline::computeContactPoints()
{
	b3AlignedObjectArray<b3Aabb> aabbWorldSpace;
	b3AlignedObjectArray<b3Int4> pairs;
	
	
	m_data->m_np->computeContacts(&pairs,&aabbWorldSpace);

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
	int index = m_data->m_rigidBodies.size();
	body.m_invMass = mass ? 1.f/mass : 0.f;
	body.m_angVel.setValue(0,0,0);
	body.m_collidableIdx = collidableIndex;
	body.m_frictionCoeff = 0.3f;
	body.m_linVel.setValue(0,0,0);
	body.m_pos.setValue(position[0],position[1],position[2]);
	body.m_quat.setValue(orientation[0],orientation[1],orientation[2],orientation[3]);
	body.m_restituitionCoeff = 0.f;

	m_data->m_rigidBodies.push_back(body);









	return index;
}


const struct b3RigidBodyData* b3CpuRigidBodyPipeline::getBodyBuffer() const
{
	return m_data->m_rigidBodies.size() ? &m_data->m_rigidBodies[0] : 0;
}

int	b3CpuRigidBodyPipeline::getNumBodies() const
{
	return m_data->m_rigidBodies.size();
}
