#ifndef RAGDOLL_DEMO_H
#define RAGDOLL_DEMO_H


#include "Bullet3AppSupport/CommonRigidBodySetup.h"
#include "../BasicDemo/BasicDemo.h"

struct BulletDemoInterface;
struct CommonGraphicsApp;

class RagDollSetup : public CommonRigidBodySetup
{
public:
	
	static BulletDemoInterface* MyCreateFunc(CommonGraphicsApp* app)
	{
		CommonPhysicsSetup* physicsSetup = new RagDollSetup();
		return new BasicDemo(app, physicsSetup);

	}
	
	void	initPhysics(GraphicsPhysicsBridge& gfxBridge);
	
};

#endif //RAGDOLL_DEMO_H

