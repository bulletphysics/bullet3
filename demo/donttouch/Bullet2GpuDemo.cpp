#include "Bullet2GpuDemo.h"
#include "../btGpuDynamicsWorld.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "gpu_rigidbody/host/btRigidBody.h"

void Bullet2GpuDemo::setupScene(const ConstructionInfo& ci)
{

//	m_data->m_np = np;
//	m_data->m_bp = bp;
//	m_data->m_rigidBodyPipeline
	m_gpuDynamicsWorld = new btGpuDynamicsWorld(m_data->m_bp,m_data->m_np,m_data->m_rigidBodyPipeline);

	btVector3 halfExtents(100,1,100);
	btBoxShape* boxShape = new btBoxShape(halfExtents);
	btVector3 localInertia;
	btScalar mass=1.f;
	boxShape->calculateLocalInertia(mass,localInertia);
	btRigidBody* body = new btRigidBody(mass,0,boxShape,localInertia);
	m_gpuDynamicsWorld->addRigidBody(body);
}

void	Bullet2GpuDemo::destroyScene()
{
	delete m_gpuDynamicsWorld;
	m_gpuDynamicsWorld = 0;
}