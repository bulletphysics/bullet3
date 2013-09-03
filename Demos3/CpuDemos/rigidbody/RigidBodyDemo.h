#ifndef B3_RIGID_BODY_DEMO_H
#define B3_RIGID_BODY_DEMO_H

#include "../CpuDemo.h"

struct RigidBodyDemo : public CpuDemo
{

	struct b3DynamicBvhBroadphase* m_bp;
	class b3CpuNarrowPhase* m_np;
	
	struct b3CpuRigidBodyPipeline* m_rb;

	GLInstancingRenderer*   m_instancingRenderer;
			
	virtual void    initPhysics(const ConstructionInfo& ci);
	
	virtual void    exitPhysics();
	
	virtual void renderScene();
	
	virtual void clientMoveAndDisplay();
	virtual const char* getName() {
		return "RigidBodyDemo";
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

	static CpuDemo* MyCreateFunc()
	{
		CpuDemo* demo = new RigidBodyDemo;
		return demo;
	}
};
#endif //B3_RIGID_BODY_DEMO_H

