#ifndef GPU_RIGID_BODY_DEMO_H
#define GPU_RIGID_BODY_DEMO_H

#include "../GpuDemo.h"
#include "Bullet3Common/b3Vector3.h"

class GpuRigidBodyDemo : public GpuDemo
{
protected:
	class GLInstancingRenderer* m_instancingRenderer;
	class GLPrimitiveRenderer*	m_primRenderer;
	class b3gWindowInterface*	m_window;

	struct GpuRigidBodyDemoInternalData*	m_data;

public:
	
	GpuRigidBodyDemo();
	virtual ~GpuRigidBodyDemo();

	virtual void	initPhysics(const ConstructionInfo& ci);

	virtual void	setupScene(const ConstructionInfo& ci);

	virtual void	destroyScene(){};

	virtual void	exitPhysics();
	
	virtual const char* getName()
	{
		return "GRBD";
	}
	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuRigidBodyDemo;
		return demo;
	}
	
	virtual void renderScene();
	
	virtual void clientMoveAndDisplay();

	//for picking
	b3Vector3	getRayTo(int x,int y);
	virtual bool	mouseMoveCallback(float x,float y);
	virtual bool	mouseButtonCallback(int button, int state, float x, float y);
	virtual bool	keyboardCallback(int key, int state);
};

#endif //GPU_RIGID_BODY_DEMO_H

