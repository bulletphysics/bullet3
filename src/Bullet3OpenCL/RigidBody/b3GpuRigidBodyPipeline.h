#ifndef B3_GPU_RIGIDBODY_PIPELINE_H
#define B3_GPU_RIGIDBODY_PIPELINE_H

#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"

class b3GpuRigidBodyPipeline
{
protected:
	struct b3GpuRigidBodyPipelineInternalData*	m_data;

	int allocateCollidable();

public:


	b3GpuRigidBodyPipeline(cl_context ctx,cl_device_id device, cl_command_queue  q , class b3GpuNarrowPhase* narrowphase, class b3GpuSapBroadphase* broadphaseSap, struct b3DynamicBvhBroadphase* broadphaseDbvt);
	virtual ~b3GpuRigidBodyPipeline();

	void	stepSimulation(float deltaTime);
	void	integrate(float timeStep);
	void	setupGpuAabbsFull();

	int		registerConvexPolyhedron(class b3ConvexUtility* convex);

	//int		registerConvexPolyhedron(const float* vertices, int strideInBytes, int numVertices, const float* scaling);
	//int		registerSphereShape(float radius);
	//int		registerPlaneShape(const b3Vector3& planeNormal, float planeConstant);
	
	//int		registerConcaveMesh(b3AlignedObjectArray<b3Vector3>* vertices, b3AlignedObjectArray<int>* indices, const float* scaling);
	//int		registerCompoundShape(b3AlignedObjectArray<b3GpuChildShape>* childShapes);

	
	int		registerPhysicsInstance(float mass, const float* position, const float* orientation, int collisionShapeIndex, int userData, bool writeInstanceToGpu);
	//if you passed "writeInstanceToGpu" false in the registerPhysicsInstance method (for performance) you need to call writeAllInstancesToGpu after all instances are registered
	void	writeAllInstancesToGpu();

	void	addConstraint(class b3TypedConstraint* constraint);

	cl_mem	getBodyBuffer();

	int	getNumBodies() const;

};

#endif //B3_GPU_RIGIDBODY_PIPELINE_H