#ifndef BT_GPU_NARROWPHASE_H
#define BT_GPU_NARROWPHASE_H

#include "../../gpu_narrowphase/host/b3Collidable.h"
#include "basic_initialize/b3OpenCLInclude.h"
#include "BulletCommon/b3AlignedObjectArray.h"
#include "BulletCommon/b3Vector3.h"

class b3GpuNarrowPhase
{
protected:

	struct btGpuNarrowPhaseInternalData*	m_data;
	int m_acceleratedCompanionShapeIndex;
	int m_planeBodyIndex;
	int	m_static0Index;

	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;

	int registerConvexHullShape(class b3ConvexUtility* convexPtr, b3Collidable& col);
	int registerConcaveMeshShape(b3AlignedObjectArray<b3Vector3>* vertices, b3AlignedObjectArray<int>* indices, b3Collidable& col, const float* scaling);

public:

	


	b3GpuNarrowPhase(cl_context vtx, cl_device_id dev, cl_command_queue q, const struct b3Config& config);

	virtual ~b3GpuNarrowPhase(void);

	int		registerSphereShape(float radius);
	int		registerPlaneShape(const b3Vector3& planeNormal, float planeConstant);

	int registerCompoundShape(b3AlignedObjectArray<btGpuChildShape>* childShapes);
	int registerFace(const b3Vector3& faceNormal, float faceConstant);
	
	int	registerConcaveMesh(b3AlignedObjectArray<b3Vector3>* vertices, b3AlignedObjectArray<int>* indices,const float* scaling);
	
	//do they need to be merged?
	
	int	registerConvexHullShape(b3ConvexUtility* utilPtr);
	int	registerConvexHullShape(const float* vertices, int strideInBytes, int numVertices, const float* scaling);

	int registerRigidBody(int collidableIndex, float mass, const float* position, const float* orientation, const float* aabbMin, const float* aabbMax,bool writeToGpu);
	void setObjectTransform(const float* position, const float* orientation , int bodyIndex);

	void	writeAllBodiesToGpu();

	void	readbackAllBodiesToCpu();
	void	getObjectTransformFromCpu(float* position, float* orientation , int bodyIndex) const;

	virtual void computeContacts(cl_mem broadphasePairs, int numBroadphasePairs, cl_mem aabbs, int numObjects);
	

	cl_mem	getBodiesGpu();
	int	getNumBodiesGpu() const;

	cl_mem	getBodyInertiasGpu();
	int	getNumBodyInertiasGpu() const;

	cl_mem	getCollidablesGpu();
	int		getNumCollidablesGpu() const;

	cl_mem	getContactsGpu();
	int	getNumContactsGpu() const;

	cl_mem	getAabbBufferGpu();
	

	int allocateCollidable();

	b3Collidable& getCollidableCpu(int collidableIndex);
	const b3Collidable& getCollidableCpu(int collidableIndex) const;

	const struct b3SapAabb& getLocalSpaceAabb(int collidableIndex) const;
};

#endif //BT_GPU_NARROWPHASE_H

