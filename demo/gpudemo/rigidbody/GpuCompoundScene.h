#ifndef GPU_COMPOUND_SCENE_H
#define GPU_COMPOUND_SCENE_H

#include "GpuRigidBodyDemo.h"

class GpuCompoundScene : public GpuRigidBodyDemo
{
public:

	GpuCompoundScene(){}
	virtual ~GpuCompoundScene(){}
	virtual const char* getName()
	{
		return "GpuCompound";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuCompoundScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

};

#endif //GPU_COMPOUND_SCENE_H
