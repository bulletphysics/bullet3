#ifndef LUA_DEMO_H
#define LUA_DEMO_H

#include "LinearMath/btVector3.h"
#include "../BasicDemo/Bullet2RigidBodyDemo.h"


//We use a struct instead of class, to make it easier to interface with Lua
struct LuaDemo : public Bullet2RigidBodyDemo
{

public:

	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new LuaDemo(app);
	}

	LuaDemo(SimpleOpenGL3App* app);
	virtual ~LuaDemo();
	
	virtual void	initPhysics();
	virtual void	exitPhysics();
	virtual void	renderScene();
	virtual void	stepSimulation(float dt);
};


#endif //BASIC_DEMO_H
