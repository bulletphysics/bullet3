#ifndef GPU_SPHERE_SCENE_H
#define GPU_SPHERE_SCENE_H

#include "GpuRigidBodyDemo.h"

class GpuSphereScene : public GpuRigidBodyDemo
{
public:

	GpuSphereScene(){}
	virtual ~GpuSphereScene(){}
	virtual const char* getName()
	{
		return "BoxOnSphere";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuSphereScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

};

#endif //GPU_SPHERE_SCENE_H
