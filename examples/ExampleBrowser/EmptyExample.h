#ifndef EMPTY_EXAMPLE_H
#define EMPTY_EXAMPLE_H

#include "../CommonInterfaces/CommonExampleInterface.h"

class EmptyExample : public CommonExampleInterface
{
public:

	EmptyExample() {}
	virtual ~EmptyExample(){}

	static CommonExampleInterface* CreateFunc(struct CommonExampleOptions& /* unusedOptions*/)
	{
		return new EmptyExample;
	}

	virtual void    initPhysics(){}
	virtual void    exitPhysics(){}
	virtual void	stepSimulation(float deltaTime){}
	virtual void	renderScene(){}
	virtual void	physicsDebugDraw(int debugFlags){}
	virtual bool	mouseMoveCallback(float x,float y){ return false;}
	virtual bool	mouseButtonCallback(int button, int state, float x, float y){return false;}
	virtual bool	keyboardCallback(int key, int state){return false;}

};

 

#endif //EMPTY_EXAMPLE_H

