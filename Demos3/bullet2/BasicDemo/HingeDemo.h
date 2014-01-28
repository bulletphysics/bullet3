
#ifndef HINGE_DEMO_H
#define HINGE_DEMO_H

#include "BasicDemo.h"

class HingeDemo : public BasicDemo
{
public:
	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new HingeDemo(app);
	}

	HingeDemo(SimpleOpenGL3App* app);

	class btMultiBody* createFeatherstoneHinge(class btMultiBodyDynamicsWorld* world, const struct btMultiBodySettings2& settings);


	virtual void	initPhysics();
	virtual void	exitPhysics();

	virtual void	renderScene();

};

#endif //HINGE_DEMO_H

