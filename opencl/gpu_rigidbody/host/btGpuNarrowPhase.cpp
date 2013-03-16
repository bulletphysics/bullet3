#include "btGpuNarrowPhase.h"


#include "parallel_primitives/host/btOpenCLArray.h"
#include "../../gpu_sat/host/btConvexPolyhedronCL.h"
#include "../../gpu_sat/host/ConvexHullContact.h"
#include "../../gpu_broadphase/host/btSapAabb.h"
#include <string.h>
#include "btConfig.h"

struct btGpuNarrowPhaseInternalData
{
	btAlignedObjectArray<btConvexUtility*>* m_convexData;
    
	btAlignedObjectArray<btConvexPolyhedronCL> m_convexPolyhedra;
	btAlignedObjectArray<btVector3> m_uniqueEdges;
	btAlignedObjectArray<btVector3> m_convexVertices;
	btAlignedObjectArray<int> m_convexIndices;
    
	btOpenCLArray<btConvexPolyhedronCL>* m_convexPolyhedraGPU;
	btOpenCLArray<btVector3>* m_uniqueEdgesGPU;
	btOpenCLArray<btVector3>* m_convexVerticesGPU;
	btOpenCLArray<int>* m_convexIndicesGPU;
    
    btOpenCLArray<btVector3>* m_worldVertsB1GPU;
    btOpenCLArray<btInt4>* m_clippingFacesOutGPU;
    btOpenCLArray<btVector3>* m_worldNormalsAGPU;
    btOpenCLArray<btVector3>* m_worldVertsA1GPU;
    btOpenCLArray<btVector3>* m_worldVertsB2GPU;
    
	btAlignedObjectArray<btGpuChildShape> m_cpuChildShapes;
	btOpenCLArray<btGpuChildShape>*	m_gpuChildShapes;
    
	btAlignedObjectArray<btGpuFace> m_convexFaces;
	btOpenCLArray<btGpuFace>* m_convexFacesGPU;
    
	GpuSatCollision*	m_gpuSatCollision;
	    
	btAlignedObjectArray<btInt2>* m_pBufPairsCPU;
    
	btOpenCLArray<btInt2>* m_convexPairsOutGPU;
	btOpenCLArray<btInt2>* m_planePairs;
    
	btOpenCLArray<btContact4>* m_pBufContactOutGPU;
	btAlignedObjectArray<btContact4>* m_pBufContactOutCPU;
	
    
	btAlignedObjectArray<btRigidBodyCL>* m_bodyBufferCPU;
	btOpenCLArray<btRigidBodyCL>* m_bodyBufferGPU;
    
	btAlignedObjectArray<btInertiaCL>*	m_inertiaBufferCPU;
	btOpenCLArray<btInertiaCL>*	m_inertiaBufferGPU;
    
	int m_numAcceleratedShapes;
	int m_numAcceleratedRigidBodies;
    
	btAlignedObjectArray<btCollidable>	m_collidablesCPU;
	btOpenCLArray<btCollidable>*	m_collidablesGPU;

	btOpenCLArray<btSapAabb>* m_localShapeAABBGPU;
	btAlignedObjectArray<btSapAabb>* m_localShapeAABBCPU;

	btConfig	m_config;
    
};





btGpuNarrowPhase::btGpuNarrowPhase(cl_context ctx, cl_device_id device, cl_command_queue queue, const btConfig& config)
:m_data(0) ,m_planeBodyIndex(-1),m_static0Index(-1),
m_context(ctx),
m_device(device),
m_queue(queue)
{
    
	m_data = new btGpuNarrowPhaseInternalData();
	memset(m_data,0,sizeof(btGpuNarrowPhaseInternalData));
    
	m_data->m_config = config;
	
	m_data->m_gpuSatCollision = new GpuSatCollision(ctx,device,queue);
	m_data->m_pBufPairsCPU = new btAlignedObjectArray<btInt2>;
	m_data->m_pBufPairsCPU->resize(config.m_maxBroadphasePairs);
	
	m_data->m_convexPairsOutGPU = new btOpenCLArray<btInt2>(ctx,queue,config.m_maxBroadphasePairs,false);
	m_data->m_planePairs = new btOpenCLArray<btInt2>(ctx,queue,config.m_maxBroadphasePairs,false);
    
	m_data->m_pBufContactOutCPU = new btAlignedObjectArray<btContact4>();
	m_data->m_pBufContactOutCPU->resize(config.m_maxBroadphasePairs);
	m_data->m_bodyBufferCPU = new btAlignedObjectArray<btRigidBodyCL>();
	m_data->m_bodyBufferCPU->resize(config.m_maxConvexBodies);
    
	m_data->m_inertiaBufferCPU = new btAlignedObjectArray<btInertiaCL>();
	m_data->m_inertiaBufferCPU->resize(config.m_maxConvexBodies);
	
	m_data->m_pBufContactOutGPU = new btOpenCLArray<btContact4>(ctx,queue, config.m_maxBroadphasePairs,true);
	btContact4 test = m_data->m_pBufContactOutGPU->forcedAt(0);

	m_data->m_inertiaBufferGPU = new btOpenCLArray<btInertiaCL>(ctx,queue,config.m_maxConvexBodies,false);
	m_data->m_collidablesGPU = new btOpenCLArray<btCollidable>(ctx,queue,config.m_maxConvexShapes);

	m_data->m_localShapeAABBCPU = new btAlignedObjectArray<btSapAabb>;
	m_data->m_localShapeAABBGPU = new btOpenCLArray<btSapAabb>(ctx,queue,config.m_maxConvexShapes);
    
    
	//m_data->m_solverDataGPU = adl::Solver<adl::TYPE_CL>::allocate(ctx,queue, config.m_maxBroadphasePairs,false);
	m_data->m_bodyBufferGPU = new btOpenCLArray<btRigidBodyCL>(ctx,queue, config.m_maxConvexBodies,false);

	m_data->m_convexFacesGPU = new btOpenCLArray<btGpuFace>(ctx,queue,config.m_maxConvexShapes*config.m_maxFacesPerShape,false);
	m_data->m_gpuChildShapes = new btOpenCLArray<btGpuChildShape>(ctx,queue,config.m_maxCompoundChildShapes,false);
	
	m_data->m_convexPolyhedraGPU = new btOpenCLArray<btConvexPolyhedronCL>(ctx,queue,config.m_maxConvexShapes,false);
	m_data->m_uniqueEdgesGPU = new btOpenCLArray<btVector3>(ctx,queue,config.m_maxConvexUniqueEdges,true);
	m_data->m_convexVerticesGPU = new btOpenCLArray<btVector3>(ctx,queue,config.m_maxConvexVertices,true);
	m_data->m_convexIndicesGPU = new btOpenCLArray<int>(ctx,queue,config.m_maxConvexIndices,true);
    
    
	m_data->m_worldVertsB1GPU = new btOpenCLArray<btVector3>(ctx,queue,config.m_maxConvexBodies*config.m_maxVerticesPerFace);
    m_data->m_clippingFacesOutGPU = new  btOpenCLArray<btInt4>(ctx,queue,config.m_maxConvexBodies);
    m_data->m_worldNormalsAGPU = new  btOpenCLArray<btVector3>(ctx,queue,config.m_maxConvexBodies);
	m_data->m_worldVertsA1GPU = new btOpenCLArray<btVector3>(ctx,queue,config.m_maxConvexBodies*config.m_maxVerticesPerFace);
    m_data->m_worldVertsB2GPU = new  btOpenCLArray<btVector3>(ctx,queue,config.m_maxConvexBodies*config.m_maxVerticesPerFace);
    
    

	m_data->m_convexData = new btAlignedObjectArray<btConvexUtility* >();
    

	m_data->m_convexData->resize(config.m_maxConvexShapes);
	m_data->m_convexPolyhedra.resize(config.m_maxConvexShapes);
    
	m_data->m_numAcceleratedShapes = 0;
	m_data->m_numAcceleratedRigidBodies = 0;
    
	//m_data->m_contactCGPU = new btOpenCLArray<Constraint4>(ctx,queue,config.m_maxBroadphasePairs,false);
	//m_data->m_frictionCGPU = new btOpenCLArray<adl::Solver<adl::TYPE_CL>::allocateFrictionConstraint( m_data->m_deviceCL, config.m_maxBroadphasePairs);
    
}


btGpuNarrowPhase::~btGpuNarrowPhase()
{
	delete m_data->m_gpuSatCollision;
	delete m_data->m_pBufPairsCPU;
	delete m_data->m_convexPairsOutGPU;
	delete m_data->m_planePairs;
	delete m_data->m_pBufContactOutCPU;
	delete m_data->m_bodyBufferCPU;
	delete m_data->m_inertiaBufferCPU;
	delete m_data->m_pBufContactOutGPU;
	delete m_data->m_inertiaBufferGPU;
	delete m_data->m_collidablesGPU;
	delete m_data->m_localShapeAABBCPU;
	delete m_data->m_localShapeAABBGPU;
	delete m_data->m_bodyBufferGPU;
	delete m_data->m_convexFacesGPU;
	delete m_data->m_gpuChildShapes;
	delete m_data->m_convexPolyhedraGPU;
	delete m_data->m_uniqueEdgesGPU;
	delete m_data->m_convexVerticesGPU;
	delete m_data->m_convexIndicesGPU;
	delete m_data->m_worldVertsB1GPU;
    delete m_data->m_clippingFacesOutGPU;
    delete m_data->m_worldNormalsAGPU;
	delete m_data->m_worldVertsA1GPU;
    delete m_data->m_worldVertsB2GPU;
    
    
    delete m_data->m_convexData;
	delete m_data;
}


int	btGpuNarrowPhase::allocateCollidable()
{
	int curSize = m_data->m_collidablesCPU.size();
	m_data->m_collidablesCPU.expand();
	return curSize;
}



int btGpuNarrowPhase::registerConvexHullShape(btConvexUtility* convexPtr,btCollidable& col)
{
	m_data->m_convexData->resize(m_data->m_numAcceleratedShapes+1);
	m_data->m_convexPolyhedra.resize(m_data->m_numAcceleratedShapes+1);
	
    
	btConvexPolyhedronCL& convex = m_data->m_convexPolyhedra.at(m_data->m_convexPolyhedra.size()-1);
	convex.mC = convexPtr->mC;
	convex.mE = convexPtr->mE;
	convex.m_extents= convexPtr->m_extents;
	convex.m_localCenter = convexPtr->m_localCenter;
	convex.m_radius = convexPtr->m_radius;
	
	convex.m_numUniqueEdges = convexPtr->m_uniqueEdges.size();
	int edgeOffset = m_data->m_uniqueEdges.size();
	convex.m_uniqueEdgesOffset = edgeOffset;
	
	m_data->m_uniqueEdges.resize(edgeOffset+convex.m_numUniqueEdges);
    
	//convex data here
	int i;
	for ( i=0;i<convexPtr->m_uniqueEdges.size();i++)
	{
		m_data->m_uniqueEdges[edgeOffset+i] = convexPtr->m_uniqueEdges[i];
	}
    
	int faceOffset = m_data->m_convexFaces.size();
	convex.m_faceOffset = faceOffset;
	convex.m_numFaces = convexPtr->m_faces.size();
	m_data->m_convexFaces.resize(faceOffset+convex.m_numFaces);
	for (i=0;i<convexPtr->m_faces.size();i++)
	{
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[0] = convexPtr->m_faces[i].m_plane[0];
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[1] = convexPtr->m_faces[i].m_plane[1];
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[2] = convexPtr->m_faces[i].m_plane[2];
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[3] = convexPtr->m_faces[i].m_plane[3];
		int indexOffset = m_data->m_convexIndices.size();
		int numIndices = convexPtr->m_faces[i].m_indices.size();
		m_data->m_convexFaces[convex.m_faceOffset+i].m_numIndices = numIndices;
		m_data->m_convexFaces[convex.m_faceOffset+i].m_indexOffset = indexOffset;
		m_data->m_convexIndices.resize(indexOffset+numIndices);
		for (int p=0;p<numIndices;p++)
		{
			m_data->m_convexIndices[indexOffset+p] = convexPtr->m_faces[i].m_indices[p];
		}
	}
    
	convex.m_numVertices = convexPtr->m_vertices.size();
	int vertexOffset = m_data->m_convexVertices.size();
	convex.m_vertexOffset =vertexOffset;
	m_data->m_convexVertices.resize(vertexOffset+convex.m_numVertices);
	for (int i=0;i<convexPtr->m_vertices.size();i++)
	{
		m_data->m_convexVertices[vertexOffset+i] = convexPtr->m_vertices[i];
	}

	(*m_data->m_convexData)[m_data->m_numAcceleratedShapes] = convexPtr;
	
	m_data->m_convexFacesGPU->copyFromHost(m_data->m_convexFaces);
    
	m_data->m_convexPolyhedraGPU->copyFromHost(m_data->m_convexPolyhedra);
	m_data->m_uniqueEdgesGPU->copyFromHost(m_data->m_uniqueEdges);
	m_data->m_convexVerticesGPU->copyFromHost(m_data->m_convexVertices);
	m_data->m_convexIndicesGPU->copyFromHost(m_data->m_convexIndices);
    
    
	return m_data->m_numAcceleratedShapes++;
}


int		btGpuNarrowPhase::registerConvexHullShape(const float* vertices, int strideInBytes, int numVertices, const float* scaling)
{
	btAlignedObjectArray<btVector3> verts;
	
	unsigned char* vts = (unsigned char*) vertices;
	for (int i=0;i<numVertices;i++)
	{
		float* vertex = (float*) &vts[i*strideInBytes];
		verts.push_back(btVector3(vertex[0]*scaling[0],vertex[1]*scaling[1],vertex[2]*scaling[2]));
	}

	btConvexUtility* utilPtr = new btConvexUtility();
	bool merge = true;
	if (numVertices)
	{
		utilPtr->initializePolyhedralFeatures(&verts[0],verts.size(),merge);
	}

	int collidableIndex = registerConvexHullShape(utilPtr);
	return collidableIndex;
}

int		btGpuNarrowPhase::registerConvexHullShape(btConvexUtility* utilPtr)
{
	int collidableIndex = allocateCollidable();
	btCollidable& col = getCollidableCpu(collidableIndex);
	col.m_shapeType = SHAPE_CONVEX_HULL;
	col.m_shapeIndex = -1;
	
	
	{
		btVector3 localCenter(0,0,0);
		for (int i=0;i<utilPtr->m_vertices.size();i++)
			localCenter+=utilPtr->m_vertices[i];
		localCenter*= (1.f/utilPtr->m_vertices.size());
		utilPtr->m_localCenter = localCenter;

		col.m_shapeIndex = registerConvexHullShape(utilPtr,col);
	}

	if (col.m_shapeIndex>=0)
	{
		btSapAabb aabb;
		
		btVector3 myAabbMin(1e30f,1e30f,1e30f);
		btVector3 myAabbMax(-1e30f,-1e30f,-1e30f);

		for (int i=0;i<utilPtr->m_vertices.size();i++)
		{
			myAabbMin.setMin(utilPtr->m_vertices[i]);
			myAabbMax.setMax(utilPtr->m_vertices[i]);
		}
		aabb.m_min[0] = myAabbMin[0];
		aabb.m_min[1] = myAabbMin[1];
		aabb.m_min[2] = myAabbMin[2];
		aabb.m_minIndices[3] = 0;

		aabb.m_max[0] = myAabbMax[0];
		aabb.m_max[1] = myAabbMax[1];
		aabb.m_max[2] = myAabbMax[2];
		aabb.m_signedMaxIndices[3] = 0;

		m_data->m_localShapeAABBCPU->push_back(aabb);
		m_data->m_localShapeAABBGPU->push_back(aabb);
	}
	
	return collidableIndex;

}







cl_mem	btGpuNarrowPhase::getBodiesGpu()
{
	return (cl_mem)m_data->m_bodyBufferGPU->getBufferCL();
}


int	btGpuNarrowPhase::getNumBodiesGpu() const
{
	return m_data->m_bodyBufferGPU->size();
}

cl_mem	btGpuNarrowPhase::getBodyInertiasGpu()
{
	return (cl_mem)m_data->m_inertiaBufferGPU->getBufferCL();
}

int	btGpuNarrowPhase::getNumBodyInertiasGpu() const
{
	return m_data->m_inertiaBufferGPU->size();
}


btCollidable& btGpuNarrowPhase::getCollidableCpu(int collidableIndex)
{
	return m_data->m_collidablesCPU[collidableIndex];
}

const btCollidable& btGpuNarrowPhase::getCollidableCpu(int collidableIndex) const
{
	return m_data->m_collidablesCPU[collidableIndex];
}

cl_mem btGpuNarrowPhase::getCollidablesGpu()
{
	return m_data->m_collidablesGPU->getBufferCL();
}

cl_mem	btGpuNarrowPhase::getAabbBufferGpu()
{
	return m_data->m_localShapeAABBGPU->getBufferCL();
}
int	btGpuNarrowPhase::getNumCollidablesGpu() const
{
	return m_data->m_collidablesGPU->size();
}





int	btGpuNarrowPhase::getNumContactsGpu() const
{
	return m_data->m_pBufContactOutGPU->size();
}
cl_mem btGpuNarrowPhase::getContactsGpu()
{
	return m_data->m_pBufContactOutGPU->getBufferCL();
}


void btGpuNarrowPhase::computeContacts(cl_mem broadphasePairs, int numBroadphasePairs, cl_mem aabbs, int numObjects)
{
	int nContactOut = 0;

	int maxTriConvexPairCapacity = 8192;
	btOpenCLArray<btInt4> triangleConvexPairs(m_context,m_queue, maxTriConvexPairCapacity);
	int numTriConvexPairsOut=0;
	
	btOpenCLArray<btInt2> broadphasePairsGPU(m_context,m_queue);
	broadphasePairsGPU.setFromOpenCLBuffer(broadphasePairs,numBroadphasePairs);
	btOpenCLArray<btYetAnotherAabb> clAabbArray(this->m_context,this->m_queue);
	clAabbArray.setFromOpenCLBuffer(aabbs,numObjects);

	m_data->m_gpuSatCollision->computeConvexConvexContactsGPUSAT(
		&broadphasePairsGPU, numBroadphasePairs,
		m_data->m_bodyBufferGPU,
		m_data->m_pBufContactOutGPU,
		nContactOut,
		*m_data->m_convexPolyhedraGPU,
		*m_data->m_convexVerticesGPU,
		*m_data->m_uniqueEdgesGPU,
		*m_data->m_convexFacesGPU,
		*m_data->m_convexIndicesGPU,
		*m_data->m_collidablesGPU,
		*m_data->m_gpuChildShapes,
		clAabbArray,
		*m_data->m_worldVertsB1GPU,
		*m_data->m_clippingFacesOutGPU,
		*m_data->m_worldNormalsAGPU,
		*m_data->m_worldVertsA1GPU,
		*m_data->m_worldVertsB2GPU,

		numObjects,
		maxTriConvexPairCapacity,
		triangleConvexPairs,
		numTriConvexPairsOut
		);

}

const btSapAabb& btGpuNarrowPhase::getLocalSpaceAabb(int collidableIndex) const
{
	return m_data->m_localShapeAABBCPU->at(collidableIndex);
}





int btGpuNarrowPhase::registerRigidBody(int collidableIndex, float mass, const float* position, const float* orientation , const float* aabbMinPtr, const float* aabbMaxPtr,bool writeToGpu)
{
	btVector3 aabbMin(aabbMinPtr[0],aabbMinPtr[1],aabbMinPtr[2]);
	btVector3 aabbMax (aabbMaxPtr[0],aabbMaxPtr[1],aabbMaxPtr[2]);
	
	btAssert(m_data->m_numAcceleratedRigidBodies< (m_data->m_config.m_maxConvexBodies-1));
    
	m_data->m_bodyBufferGPU->resize(m_data->m_numAcceleratedRigidBodies+1);
    
	btRigidBodyCL& body = m_data->m_bodyBufferCPU->at(m_data->m_numAcceleratedRigidBodies);
    
	float friction = 1.f;
	float restitution = 0.f;
    
	body.m_frictionCoeff = friction;
	body.m_restituitionCoeff = restitution;
	body.m_angVel.setZero();
	body.m_linVel.setValue(0,0,0);//.setZero();
	body.m_pos.setValue(position[0],position[1],position[2]);
	body.m_quat.setValue(orientation[0],orientation[1],orientation[2],orientation[3]);
	body.m_collidableIdx = collidableIndex;
	if (collidableIndex>=0)
	{
//		body.m_shapeType = m_data->m_collidablesCPU.at(collidableIndex).m_shapeType;
	} else
	{
	//	body.m_shapeType = CollisionShape::SHAPE_PLANE;
		m_planeBodyIndex = m_data->m_numAcceleratedRigidBodies;
	}
	//body.m_shapeType = shapeType;
	
	
	body.m_invMass = mass? 1.f/mass : 0.f;
    
	if (writeToGpu)
	{
		m_data->m_bodyBufferGPU->copyFromHostPointer(&body,1,m_data->m_numAcceleratedRigidBodies);
	}
    
	btInertiaCL& shapeInfo = m_data->m_inertiaBufferCPU->at(m_data->m_numAcceleratedRigidBodies);
    
	if (mass==0.f)
	{
		if (m_data->m_numAcceleratedRigidBodies==0)
			m_static0Index = 0;
        
		shapeInfo.m_initInvInertia.setValue(0,0,0,0,0,0,0,0,0);
		shapeInfo.m_invInertiaWorld.setValue(0,0,0,0,0,0,0,0,0);
	} else
	{
        
		assert(body.m_collidableIdx>=0);
        
		//approximate using the aabb of the shape
        
		//Aabb aabb = (*m_data->m_shapePointers)[shapeIndex]->m_aabb;
		btVector3 halfExtents = (aabbMax-aabbMin);//*0.5f;//fake larger inertia makes demos more stable ;-)
        
		btVector3 localInertia;
        
		float lx=2.f*halfExtents[0];
		float ly=2.f*halfExtents[1];
		float lz=2.f*halfExtents[2];
        
		localInertia.setValue( (mass/12.0f) * (ly*ly + lz*lz),
                                   (mass/12.0f) * (lx*lx + lz*lz),
                                   (mass/12.0f) * (lx*lx + ly*ly));
        
		btVector3 invLocalInertia;
		invLocalInertia[0] = 1.f/localInertia[0];
		invLocalInertia[1] = 1.f/localInertia[1];
		invLocalInertia[2] = 1.f/localInertia[2];
		invLocalInertia[3] = 0.f;
        
		shapeInfo.m_initInvInertia.setValue(
			invLocalInertia[0],		0,						0,
			0,						invLocalInertia[1],		0,
			0,						0,						invLocalInertia[2]);

		btMatrix3x3 m (body.m_quat);

		shapeInfo.m_invInertiaWorld = m.scaled(invLocalInertia) * m.transpose();
        
	}
    
	if (writeToGpu)
		m_data->m_inertiaBufferGPU->copyFromHostPointer(&shapeInfo,1,m_data->m_numAcceleratedRigidBodies);
    
    
    
	return m_data->m_numAcceleratedRigidBodies++;
}

void	btGpuNarrowPhase::writeAllBodiesToGpu()
{
	m_data->m_bodyBufferGPU->resize(m_data->m_numAcceleratedRigidBodies);
	m_data->m_inertiaBufferGPU->resize(m_data->m_numAcceleratedRigidBodies);
    
	m_data->m_bodyBufferGPU->copyFromHostPointer(&m_data->m_bodyBufferCPU->at(0),m_data->m_numAcceleratedRigidBodies);
	m_data->m_inertiaBufferGPU->copyFromHostPointer(&m_data->m_inertiaBufferCPU->at(0),m_data->m_numAcceleratedRigidBodies);
    
	m_data->m_collidablesGPU->copyFromHost(m_data->m_collidablesCPU);
	
    
}
