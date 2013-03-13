#ifndef PARTICLE_DEMO_H
#define PARTICLE_DEMO_H

#include "GpuDemo.h"

class ParticleDemo : public GpuDemo
{
	
protected:

	struct ParticleInternalData*	m_data;

	GLInstancingRenderer*	m_instancingRenderer;

	void initCL(int preferredDeviceIndex, int preferredPlatformIndex);
	void exitCL();

public:

	ParticleDemo();

	virtual ~ParticleDemo();

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void	initPhysics(const ConstructionInfo& ci);

	virtual void	exitPhysics();

	virtual const char* getName()
	{
		return "ParticleDemo";
	}
	static GpuDemo* CreateFunc()
	{
		GpuDemo* demo = new ParticleDemo;
		return demo;
	}

	virtual const btDynamicsWorld* getDynamicsWorld() const
	{
		return 0;
	}

	virtual void renderScene();

	virtual void clientMoveAndDisplay();
};

#endif //PARTICLE_DEMO_H
