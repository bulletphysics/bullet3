#ifndef CHAIN_DEMO_H
#define CHAIN_DEMO_H

#include "LinearMath/btVector3.h"
#include "../BasicDemo/Bullet2RigidBodyDemo.h"



class ChainDemo : public Bullet2RigidBodyDemo
{

public:

	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new ChainDemo(app);
	}

	ChainDemo(SimpleOpenGL3App* app);
	virtual ~ChainDemo();
	
	void	createGround(int cubeShapeId);

	virtual void	initPhysics();
	virtual void	exitPhysics();
	virtual void	renderScene();
	virtual void	stepSimulation(float dt);
};


#endif //CHAIN_DEMO_H
