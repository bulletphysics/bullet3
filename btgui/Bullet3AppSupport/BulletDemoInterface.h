#ifndef DEMO_INTERFACE_H
#define DEMO_INTERFACE_H

struct CommonGraphicsApp;


class BulletDemoInterface
{
public:

	typedef class BulletDemoInterface* (CreateFunc)(CommonGraphicsApp* app);

	virtual ~BulletDemoInterface()
	{
	}

	virtual void    initPhysics()=0;
	virtual void    exitPhysics()=0;
	virtual void	stepSimulation(float deltaTime)=0;
	virtual void	renderScene()=0;
	virtual void	physicsDebugDraw(int debugFlags)=0;//for now we reuse the flags in Bullet/src/LinearMath/btIDebugDraw.h
	virtual bool	mouseMoveCallback(float x,float y)=0;
	virtual bool	mouseButtonCallback(int button, int state, float x, float y)=0;
	virtual bool	keyboardCallback(int key, int state)=0;

};

class EmptyBulletDemo : public BulletDemoInterface
{
public:
	static BulletDemoInterface* MyCreateFunc(CommonGraphicsApp* app)
	{
		return new EmptyBulletDemo();
	}

	virtual void    initPhysics()
	{
	}
	virtual void    exitPhysics()
	{
	}
	virtual void	stepSimulation(float deltaTime)
	{
	}
	virtual void	renderScene()
	{
	}
	virtual void	physicsDebugDraw(int debugFlags)
	{
	}
	virtual bool	mouseMoveCallback(float x,float y)
	{
		return false;
	}
	virtual bool	mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}
	virtual bool	keyboardCallback(int key, int state)
	{
		return false;
	}
};


#endif //DEMO_INTERFACE_H

