#ifndef BT_GPU_NARROWPHASE_H
#define BT_GPU_NARROWPHASE_H

#include "../../gpu_sat/host/btCollidable.h"
#include "basic_initialize/btOpenCLInclude.h"
#include "BulletCommon/btAlignedObjectArray.h"
#include "BulletCommon/btVector3.h"

class btGpuNarrowPhase
{
protected:

	struct btGpuNarrowPhaseInternalData*	m_data;
	int m_acceleratedCompanionShapeIndex;
	int m_planeBodyIndex;
	int	m_static0Index;

	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;

	int registerConvexHullShape(class btConvexUtility* convexPtr, btCollidable& col);
	int registerConcaveMeshShape(btAlignedObjectArray<btVector3>* vertices, btAlignedObjectArray<int>* indices, btCollidable& col, const float* scaling);

public:

	


	btGpuNarrowPhase(cl_context vtx, cl_device_id dev, cl_command_queue q, const struct btConfig& config);

	virtual ~btGpuNarrowPhase(void);

	
	int registerCompoundShape(btAlignedObjectArray<btGpuChildShape>* childShapes);
	int registerFace(const btVector3& faceNormal, float faceConstant);
	
	int	registerConcaveMesh(btAlignedObjectArray<btVector3>* vertices, btAlignedObjectArray<int>* indices,const float* scaling);
	
	//do they need to be merged?
	
	int	registerConvexHullShape(btConvexUtility* utilPtr);
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

	btCollidable& getCollidableCpu(int collidableIndex);
	const btCollidable& getCollidableCpu(int collidableIndex) const;

	const struct btSapAabb& getLocalSpaceAabb(int collidableIndex) const;
};

#endif //BT_GPU_NARROWPHASE_H

