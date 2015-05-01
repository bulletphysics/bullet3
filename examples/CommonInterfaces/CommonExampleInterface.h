

#ifndef COMMON_EXAMPLE_INTERFACE_H
#define COMMON_EXAMPLE_INTERFACE_H


class CommonExampleInterface
{
public:

	typedef class CommonExampleInterface* (CreateFunc)(struct PhysicsInterface* pint, struct GUIHelperInterface* helper, int option);

	virtual ~CommonExampleInterface()
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



#endif //COMMON_EXAMPLE_INTERFACE_H
