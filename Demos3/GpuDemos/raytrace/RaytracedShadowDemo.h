#ifndef RAYTRACED_SHADOW_DEMO_H
#define RAYTRACED_SHADOW_DEMO_H

#include "../rigidbody/GpuConvexScene.h"

class GpuRaytraceScene : public GpuBoxPlaneScene
{
protected:
	b3AlignedObjectArray<b3RayInfo> primaryRays;

	struct GpuRaytraceInternalData* m_raytraceData;

public:
	GpuRaytraceScene();
	virtual ~GpuRaytraceScene();
	virtual const char* getName()
	{
		return "GPURaytrace";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuRaytraceScene;
		return demo;
	}
	
	virtual int	createDynamicsObjects(const ConstructionInfo& ci);

	void renderScene();
	void renderScene2();
};

#endif //RAYTRACED_SHADOW_DEMO_H
