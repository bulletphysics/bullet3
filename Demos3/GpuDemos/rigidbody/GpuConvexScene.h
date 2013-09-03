#ifndef GPU_CONVEX_SCENE_H
#define GPU_CONVEX_SCENE_H

#include "GpuRigidBodyDemo.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h"

class GpuConvexScene : public GpuRigidBodyDemo
{
protected:
	class GLPrimitiveRenderer* m_primRenderer;

	class b3GpuRaycast*	m_raycaster;

public:

	GpuConvexScene() :m_primRenderer(0), m_raycaster(0)
	{
	}
	virtual ~GpuConvexScene(){}
	virtual const char* getName()
	{
		return "Tetrahedra";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuConvexScene;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void	destroyScene();

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
		return "ConvexOnPlane";
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
		return "BoxBox";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuBoxPlaneScene;
		return demo;
	}

	virtual int	createDynamicsObjects(const ConstructionInfo& ci);


};

class GpuTetraScene : public GpuConvexScene
{

protected:
void createFromTetGenData(const char* ele,
	const char* node,
	const ConstructionInfo& ci);

public:
	virtual const char* getName()
	{
		return "TetraBreakable";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new GpuTetraScene;
		return demo;
	}

	virtual int	createDynamicsObjects(const ConstructionInfo& ci);

};


#endif //GPU_CONVEX_SCENE_H
