#ifndef RAGDOLL_DEMO_H
#define RAGDOLL_DEMO_H


#include "../../../Demos/CommonRigidBodySetup.h"
#include "../BasicDemo/BasicDemo.h"

struct BulletDemoInterface;
struct SimpleOpenGL3App;

class RagDollSetup : public CommonRigidBodySetup
{
public:
	
	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		CommonPhysicsSetup* physicsSetup = new RagDollSetup();
		return new BasicDemo(app, physicsSetup);

	}
	
	void	initPhysics(GraphicsPhysicsBridge& gfxBridge);
	
};

#endif //RAGDOLL_DEMO_H

