#ifndef GPU_CONVEX_SCENE_H
#define GPU_CONVEX_SCENE_H

#include "GpuRigidBodyDemo.h"

class GpuConvexScene : public GpuRigidBodyDemo
{
public:

	GpuConvexScene(){}
	virtual ~GpuConvexScene(){}
	virtual const char* getName()
	{
		return "GRBConvex";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuConvexScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void createStaticEnvironment(const ConstructionInfo& ci);

};


class GpuConvexPlaneScene : public GpuConvexScene
{
public:

	GpuConvexPlaneScene(){}
	virtual ~GpuConvexPlaneScene(){}
	virtual const char* getName()
	{
		return "GRBConvexPlane";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuConvexPlaneScene;
		return demo;
	}

	virtual void createStaticEnvironment(const ConstructionInfo& ci);

};

#endif //GPU_CONVEX_SCENE_H
