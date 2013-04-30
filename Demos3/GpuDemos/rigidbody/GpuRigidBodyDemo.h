#ifndef GPU_RIGID_BODY_DEMO_H
#define GPU_RIGID_BODY_DEMO_H

#include "../GpuDemo.h"

class GpuRigidBodyDemo : public GpuDemo
{
protected:
	class GLInstancingRenderer* m_instancingRenderer;
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

	
};

#endif //GPU_RIGID_BODY_DEMO_H

