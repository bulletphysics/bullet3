
#ifndef MULTI_DOF_DEMO_H
#define MULTI_DOF_DEMO_H

#include "BulletMultiBodyDemos.h"

class MultiDofDemo : public FeatherstoneDemo1
{
	
public:

	MultiDofDemo(SimpleOpenGL3App* app);
	virtual ~MultiDofDemo();


	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new MultiDofDemo(app);
	}
	
	virtual void	initPhysics();

	virtual void	stepSimulation(float deltaTime);


	btMultiBody* createFeatherstoneMultiBody_testMultiDof(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents, bool spherical = false, bool floating = false);
	void addColliders_testMultiDof(btMultiBody *pMultiBody, btMultiBodyDynamicsWorld *pWorld, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents);
	void addBoxes_testMultiDof();


};

#endif //MULTI_DOF_DEMO_H

