#ifndef SHADOW_MAP_DEMO_H
#define SHADOW_MAP_DEMO_H

#include "../GpuDemo.h"

class ShadowMapDemo : public GpuDemo
{
	struct ShadowMapDemoInternalData* m_shadowData;

public:

	ShadowMapDemo();
	virtual ~ShadowMapDemo();

	virtual const char* getName()
	{
		return "ShadowMapDemo";
	}
	
	virtual void    initPhysics(const ConstructionInfo& ci);
	
	virtual void    exitPhysics();
	
	virtual void renderScene();
	
	virtual void clientMoveAndDisplay();

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new ShadowMapDemo;
		return demo;
	}

};

#endif //SHADOW_MAP_DEMO_H
