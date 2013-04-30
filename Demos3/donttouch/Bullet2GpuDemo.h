#ifndef BULLET2_GPU_DEMO_H
#define BULLET2_GPU_DEMO_H

#include "GpuRigidBodyDemo.h"

class Bullet2GpuDemo : public GpuRigidBodyDemo
{
protected:

	class b3GpuDynamicsWorld* m_gpuDynamicsWorld;

public:

	Bullet2GpuDemo(){}
	virtual ~Bullet2GpuDemo(){}
	virtual const char* getName()
	{
		return "Bullet2Gpu";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new Bullet2GpuDemo;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);
	virtual void	destroyScene();
};
#endif //BULLET2_GPU_DEMO_H

