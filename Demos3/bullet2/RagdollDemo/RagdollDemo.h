#ifndef RAGDOLL_DEMO_H
#define RAGDOLL_DEMO_H


#include "../BasicDemo/BasicDemo.h"

class RagDollDemo : public BasicDemo
{
public:
	
	RagDollDemo(SimpleOpenGL3App* app);
	virtual ~RagDollDemo();
	
	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new RagDollDemo(app);
	}
	
	void	initPhysics();
	
};

#endif //RAGDOLL_DEMO_H

