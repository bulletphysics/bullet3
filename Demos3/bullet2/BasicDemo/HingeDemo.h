
#ifndef HINGE_DEMO_H
#define HINGE_DEMO_H

#include "BasicDemo.h"

enum HINGE_CREATION_METHOD
{
	FEATHERSTONE_HINGE=1,
	DANTZIG_HINGE,
	LEMKE_HINGE,
	PGS_HINGE,
	SI_HINGE,
	INERTIA_HINGE
};

class HingeDemo : public BasicDemo
{
	int m_hingeMethod;
	
public:
	static BulletDemoInterface* FeatherstoneCreateFunc(SimpleOpenGL3App* app)
	{
		return new HingeDemo(app, FEATHERSTONE_HINGE);
	}
	static BulletDemoInterface* DantzigCreateFunc(SimpleOpenGL3App* app)
	{
		return new HingeDemo(app, DANTZIG_HINGE);
	}
	static BulletDemoInterface* LemkeCreateFunc(SimpleOpenGL3App* app)
	{
		return new HingeDemo(app, LEMKE_HINGE);
	}
	static BulletDemoInterface* PGSCreateFunc(SimpleOpenGL3App* app)
	{
		return new HingeDemo(app, PGS_HINGE);
	}

	static BulletDemoInterface* SICreateFunc(SimpleOpenGL3App* app)
	{
		return new HingeDemo(app, SI_HINGE);
	}
	

	static BulletDemoInterface* InertiaCreateFunc(SimpleOpenGL3App* app)
	{
		return new HingeDemo(app, INERTIA_HINGE);
	}

	HingeDemo(SimpleOpenGL3App* app, HINGE_CREATION_METHOD hingeMethod);

	class btMultiBody* createFeatherstoneHinge(class btMultiBodyDynamicsWorld* world, const struct btMultiBodySettings2& settings);
	
	
	virtual void	initPhysics();
	virtual void	exitPhysics();

	virtual void	renderScene();

};

#endif //HINGE_DEMO_H

