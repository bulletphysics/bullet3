#ifndef B3_GPU_CONSTRAINTS_DEMO_H
#define B3_GPU_CONSTRAINTS_DEMO_H

#include "../rigidbody/GpuRigidBodyDemo.h"
#include "Bullet3Common/b3AlignedObjectArray.h"


class GpuConstraintsDemo : public GpuRigidBodyDemo
{
protected:
	class GLPrimitiveRenderer* m_primRenderer;

	class b3GpuRaycast*	m_raycaster;

public:

	GpuConstraintsDemo() :m_primRenderer(0), m_raycaster(0)
	{
	}
	virtual ~GpuConstraintsDemo(){}
	virtual const char* getName()
	{
		return "GpuConstraints";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuConstraintsDemo;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void	destroyScene();

	virtual int	createDynamicsObjects(const ConstructionInfo& ci);

	virtual int	createDynamicsObjects2(const ConstructionInfo& ci,const float* vertices, int numVertices, const int* indices,int numIndices);

	virtual void createStaticEnvironment(const ConstructionInfo& ci);

};



#endif //B3_GPU_CONSTRAINTS_DEMO_H
