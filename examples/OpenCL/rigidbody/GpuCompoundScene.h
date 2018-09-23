#ifndef GPU_COMPOUND_SCENE_H
#define GPU_COMPOUND_SCENE_H

#include "GpuRigidBodyDemo.h"

class GpuCompoundScene : public GpuRigidBodyDemo
{
public:
	GpuCompoundScene() {}
	virtual ~GpuCompoundScene() {}
	virtual const char* getName()
	{
		return "CompoundOnSphere";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuCompoundScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void createStaticEnvironment(const ConstructionInfo& ci);
};

class GpuCompoundPlaneScene : public GpuCompoundScene
{
public:
	GpuCompoundPlaneScene() {}
	virtual ~GpuCompoundPlaneScene() {}
	virtual const char* getName()
	{
		return "CompoundOnPlane";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuCompoundPlaneScene;
		return demo;
	}

	virtual void createStaticEnvironment(const ConstructionInfo& ci);
};
#endif  //GPU_COMPOUND_SCENE_H
