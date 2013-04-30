#include "Bullet2GpuDemo.h"
#include "../b3GpuDynamicsWorld.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "BulletCollision/CollisionShapes/b3BoxShape.h"
#include "gpu_rigidbody/host/b3RigidBody.h"

void Bullet2GpuDemo::setupScene(const ConstructionInfo& ci)
{

//	m_data->m_np = np;
//	m_data->m_bp = bp;
//	m_data->m_rigidBodyPipeline
	m_gpuDynamicsWorld = new b3GpuDynamicsWorld(m_data->m_bp,m_data->m_np,m_data->m_rigidBodyPipeline);

	b3Vector3 halfExtents(100,1,100);
	b3BoxShape* boxShape = new b3BoxShape(halfExtents);
	b3Vector3 localInertia;
	b3Scalar mass=1.f;
	boxShape->calculateLocalInertia(mass,localInertia);
	b3RigidBody* body = new b3RigidBody(mass,0,boxShape,localInertia);
	m_gpuDynamicsWorld->addRigidBody(body);
}

void	Bullet2GpuDemo::destroyScene()
{
	delete m_gpuDynamicsWorld;
	m_gpuDynamicsWorld = 0;
}