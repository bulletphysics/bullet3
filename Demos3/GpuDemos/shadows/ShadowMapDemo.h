#ifndef SHADOW_MAP_DEMO_H
#define SHADOW_MAP_DEMO_H

#include "../GpuDemo.h"
#include "Bullet3Common/b3Vector3.h"

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
	
	void createConcaveMesh(const ConstructionInfo& ci, const char* fileName, const b3Vector3& shift, const b3Vector3& scaling);


	virtual void clientMoveAndDisplay();

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new ShadowMapDemo;
		return demo;
	}

};

#endif //SHADOW_MAP_DEMO_H
