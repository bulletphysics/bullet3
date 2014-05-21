#ifndef BASIC_DEMO_H
#define BASIC_DEMO_H

#include "LinearMath/btVector3.h"
#include "Bullet2RigidBodyDemo.h"

#include "../../../Demos/BasicDemo/BasicDemoPhysicsSetup.h"


struct MyBasicDemoPhysicsSetup : public BasicDemoPhysicsSetup 
{
	SimpleOpenGL3App* m_glApp;

	virtual btRigidBody*	createRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, const btVector4& color);
	
	virtual btBoxShape* createBoxShape(const btVector3& halfExtents);
};

class BasicDemo : public Bullet2RigidBodyDemo
{
	
	MyBasicDemoPhysicsSetup m_physicsSetup;

public:

	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new BasicDemo(app);
	}

	BasicDemo(SimpleOpenGL3App* app);
	virtual ~BasicDemo();
	
	void	createGround(int cubeShapeId);

	virtual void	initPhysics();
	virtual void	exitPhysics();
	virtual void	renderScene();
	virtual void	stepSimulation(float dt);
};


#endif //BASIC_DEMO_H
