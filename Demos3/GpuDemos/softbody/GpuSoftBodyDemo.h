#ifndef GPU_SOFT_BODY_DEMO_H
#define GPU_SOFT_BODY_DEMO_H

#include "../GpuDemo.h"

class GpuSoftBodyDemo : public GpuDemo
{
protected:
	class GLInstancingRenderer* m_instancingRenderer;
	class b3gWindowInterface*	m_window;

	struct GpuSoftBodyDemoInternalData*	m_data;

public:
	
	GpuSoftBodyDemo();
	virtual ~GpuSoftBodyDemo();

	virtual void	initPhysics(const ConstructionInfo& ci);

	virtual void	setupScene(const ConstructionInfo& ci);

	virtual void	destroyScene(){};

	virtual void	exitPhysics();
	
	virtual const char* getName()
	{
		return "GPUSOFT";
	}
	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuSoftBodyDemo;
		return demo;
	}
	
	virtual void renderScene();
	
	virtual void clientMoveAndDisplay();

	
};

class GpuSoftClothDemo : public GpuSoftBodyDemo
{

	public:
		GpuSoftClothDemo();
		virtual ~GpuSoftClothDemo();
		
	
	virtual void	setupScene(const ConstructionInfo& ci);
			
	virtual void	renderScene();

	virtual const char* getName()
	{
		return "GpuSoftCloth";
	}
	
	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuSoftClothDemo;
		return demo;
	}
	
};

#endif //GPU_SOFT_BODY_DEMO_H

