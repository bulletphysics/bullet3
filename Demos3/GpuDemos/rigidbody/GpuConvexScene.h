#ifndef GPU_CONVEX_SCENE_H
#define GPU_CONVEX_SCENE_H

#include "GpuRigidBodyDemo.h"

class GpuConvexScene : public GpuRigidBodyDemo
{
protected:
	class GLPrimitiveRenderer* m_primRenderer;

public:

	GpuConvexScene() :m_primRenderer(0) {}
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

	virtual int	createDynamicsObjects(const ConstructionInfo& ci);

	virtual int	createDynamicsObjects2(const ConstructionInfo& ci,const float* vertices, int numVertices, const int* indices,int numIndices);

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


class GpuBoxPlaneScene : public GpuConvexPlaneScene
{
public:

	GpuBoxPlaneScene(){}
	virtual ~GpuBoxPlaneScene(){}
	virtual const char* getName()
	{
		return "GRBBoxPlane";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuBoxPlaneScene;
		return demo;
	}

	virtual int	createDynamicsObjects(const ConstructionInfo& ci);


};

class GpuRaytraceScene : public GpuBoxPlaneScene
{
protected:
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
};


#endif //GPU_CONVEX_SCENE_H
