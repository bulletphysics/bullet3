#ifndef BASIC_DEMO_H
#define BASIC_DEMO_H

#include "LinearMath/btVector3.h"
#include "Bullet3AppSupport/Bullet2RigidBodyDemo.h"

#include "../../../Demos/BasicDemo/BasicDemoPhysicsSetup.h"




class BasicDemo : public Bullet2RigidBodyDemo
{
	
	

public:

	static BulletDemoInterface* MyCreateFunc(CommonGraphicsApp* app)
	{
		CommonPhysicsSetup* physicsSetup = new BasicDemoPhysicsSetup();
		return new BasicDemo(app, physicsSetup);
	}

	BasicDemo(CommonGraphicsApp* app, CommonPhysicsSetup* physicsSetup);
	virtual ~BasicDemo();
	
	void	createGround(int cubeShapeId);

};


#endif //BASIC_DEMO_H
