/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2009 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT3DGRIDBROADPHASEOCL_H
#define BT3DGRIDBROADPHASEOCL_H

#ifdef __APPLE__
#ifdef USE_MINICL
	#include <MiniCL/cl.h>
#else
	#include <MiniCL/cl.h>
#endif
//CL_PLATFORM_MINI_CL could be defined in build system
#else
//#include <GL/glew.h>
// standard utility and system includes
#ifdef USE_MINICL
	#include <MiniCL/cl.h>
#else
	#include <CL/cl.h>
#endif
// Extra CL/GL include
//#include <CL/cl_gl.h>
#endif //__APPLE__

namespace adl
{
	struct Device;
	struct DeviceCL;
};

#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "btGpu3DGridBroadphaseSharedTypes.h"
#include "btGpu3DGridBroadphase.h"


#define GRID3DOCL_CHECKERROR(a, b) if((a)!=(b)) { printf("3D GRID OCL Error : %d\n", (a)); btAssert((a) == (b)); }

enum
{
	GRID3DOCL_KERNEL_CALC_HASH_AABB = 0,
	GRID3DOCL_KERNEL_CLEAR_CELL_START,
	GRID3DOCL_KERNEL_FIND_CELL_START,
	GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS,
	GRID3DOCL_KERNEL_FIND_PAIRS_LARGE,
	GRID3DOCL_KERNEL_COMPUTE_CACHE_CHANGES,
	GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF,
	GRID3DOCL_KERNEL_TOTAL
};

struct bt3dGridOCLKernelInfo
{
	int			m_Id;
	cl_kernel	m_kernel;
	char*		m_name;
	int			m_workgroupSize;
};


///The bt3dGridBroadphaseOCL uses OpenCL-capable GPU to compute overlapping pairs

class bt3dGridBroadphaseOCL : public btGpu3DGridBroadphase
{
protected:
	int						m_hashSize;
	cl_context				m_cxMainContext;
	cl_device_id			m_cdDevice;
	cl_command_queue		m_cqCommandQue;
	cl_program				m_cpProgram;
	bt3dGridOCLKernelInfo	m_kernels[GRID3DOCL_KERNEL_TOTAL];
	// data buffers
	cl_mem					m_dBodiesHash;
	cl_mem					m_dCellStart;
	cl_mem					m_dPairBuff; 
	cl_mem					m_dPairBuffStartCurr;
public:
	cl_mem					m_dAABB;
protected:
	cl_mem					m_dPairScanChanged;
	cl_mem					m_dPairsChanged;
	cl_mem					m_dBpParams;

	adl::Device*			m_deviceHost;
	adl::DeviceCL*			m_deviceCL;
	bool					m_ownsDevice;


public:
	unsigned int			m_numPrefixSum;

	bt3dGridBroadphaseOCL(	btOverlappingPairCache* overlappingPairCache,
							const btVector3& cellSize, 
							int gridSizeX, int gridSizeY, int gridSizeZ, 
							int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxy,
							btScalar maxSmallProxySize,
							int maxSmallProxiesPerCell = 8,
							cl_context context = NULL,
							cl_device_id device = NULL,
							cl_command_queue queue = NULL,
							adl::DeviceCL* deviceCL = 0
							);
	virtual ~bt3dGridBroadphaseOCL();

protected:
	void initCL(cl_context context, cl_device_id device, cl_command_queue queue);
	void initKernels();
	void allocateBuffers();
	void prefillBuffers();
	void initKernel(int kernelId, char* pName);
	void allocateArray(void** devPtr, unsigned int size);
	void freeArray(void* devPtr);
	void runKernelWithWorkgroupSize(int kernelId, int globalSize);
	void setKernelArg(int kernelId, int argNum, int argSize, void* argPtr);
	void copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs = 0, int hostOffs = 0);
	void copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs = 0, int devOffs = 0);

// overrides
	virtual void setParameters(bt3DGridBroadphaseParams* hostParams);
	virtual void prepareAABB();
	virtual void calcHashAABB();
	virtual void sortHash();	
	virtual void findCellStart();
	virtual void findOverlappingPairs();
	virtual void findPairsLarge();
	virtual void computePairCacheChanges();
	virtual void scanOverlappingPairBuff(bool copyToCpu=true);
	virtual void squeezeOverlappingPairBuff();
	virtual void resetPool(btDispatcher* dispatcher);
};

#endif //BT3DGRIDBROADPHASEOCL_H