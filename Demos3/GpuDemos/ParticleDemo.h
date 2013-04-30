#ifndef PARTICLE_DEMO_H
#define PARTICLE_DEMO_H

#include "GpuDemo.h"
struct GLInstancingRenderer;

class ParticleDemo : public GpuDemo
{
public:


	protected:
	struct ParticleInternalData*	m_data;

	GLInstancingRenderer*	m_instancingRenderer;

	virtual void initCL(int preferredDeviceIndex, int preferredPlatformIndex);
	virtual void exitCL();

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
	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new ParticleDemo;
		return demo;
	}



	virtual void renderScene();

	virtual void clientMoveAndDisplay();
};

#endif //PARTICLE_DEMO_H
