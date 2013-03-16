#ifndef BT_GPU_RIGIDBODY_PIPELINE_H
#define BT_GPU_RIGIDBODY_PIPELINE_H

#include "../../basic_initialize/btOpenCLInclude.h"

class btGpuRigidBodyPipeline
{
protected:
	struct btGpuRigidBodyPipelineInternalData*	m_data;

	int allocateCollidable();

public:

	btGpuRigidBodyPipeline(cl_context ctx,cl_device_id device, cl_command_queue  q , class btGpuNarrowPhase* narrowphase, class btGpuSapBroadphase* broadphaseSap);
	virtual ~btGpuRigidBodyPipeline();

	void	stepSimulation(float deltaTime);
	void	integrate(float timeStep);
	void	setupGpuAabbsFull();

	int		registerConvexPolyhedron(class btConvexUtility* convex);

	//int		registerConvexPolyhedron(const float* vertices, int strideInBytes, int numVertices, const float* scaling);
	//int		registerSphereShape(float radius);
	//int		registerPlaneShape(const btVector3& planeNormal, float planeConstant);
	
	//int		registerConcaveMesh(btAlignedObjectArray<btVector3>* vertices, btAlignedObjectArray<int>* indices, const float* scaling);
	//int		registerCompoundShape(btAlignedObjectArray<btGpuChildShape>* childShapes);

	
	int		registerPhysicsInstance(float mass, const float* position, const float* orientation, int collisionShapeIndex, int userData);

	cl_mem	getBodyBuffer();

	int	getNumBodies() const;

};

#endif //BT_GPU_RIGIDBODY_PIPELINE_H