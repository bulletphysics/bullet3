
#include "../CpuDemo.h"

struct RenderDemo : public CpuDemo
{
	GLInstancingRenderer*   m_instancingRenderer;
			
	virtual void    initPhysics(const ConstructionInfo& ci);
	
	virtual void    exitPhysics();
	
	virtual void renderScene();
	
	virtual void clientMoveAndDisplay();
	virtual const char* getName() {
		return "RenderDemo";
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
		CpuDemo* demo = new RenderDemo;
		return demo;
	}
};