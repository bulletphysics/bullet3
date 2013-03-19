#ifndef CONCAVE_SCENE_H
#define CONCAVE_SCENE_H

#include "GpuRigidBodyDemo.h"

class ConcaveScene : public GpuRigidBodyDemo
{
public:

	ConcaveScene(){}
	virtual ~ConcaveScene(){}
	virtual const char* getName()
	{
		return "GRBConcave";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new ConcaveScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

};

#endif //CONCAVE_SCENE_H
