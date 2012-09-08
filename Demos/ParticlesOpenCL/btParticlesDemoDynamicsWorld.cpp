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

#include <stdio.h>
#ifdef __APPLE__
//CL_PLATFORM_MINI_CL could be defined in build system
#else
#include <GL/glew.h>
#ifdef USE_MINICL

#include <MiniCL/cl_platform.h> //for CL_PLATFORM_MINI_CL definition
#else
#include <CL/cl_platform.h> //for CL_PLATFORM_MINI_CL definition
#endif
#endif //__APPLE__


#include "btOpenCLUtils.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "LinearMath/btQuickprof.h"
#include "GlutStuff.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"

#include "btParticlesDynamicsWorld.h"
#include "GL_DialogWindow.h"

//when loading from disk, you need to remove the 'MSTRINGIFY' line at the start, and ); at the end of the .cl file

#define LOAD_FROM_MEMORY
#ifdef LOAD_FROM_MEMORY
#define MSTRINGIFY(A) #A
static const char* source= 
#include "ParticlesOCL.cl"
#endif //LOAD_FROM_MEMORY

btParticlesDynamicsWorld::~btParticlesDynamicsWorld()
{
}

static int gStepNum = 0;

int	btParticlesDynamicsWorld::stepSimulation( btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{
	startProfiling(timeStep);
	m_timeStep = timeStep;
	BT_PROFILE("stepSimulation");
//	printf("Step : %d\n", gStepNum);
	{
		BT_PROFILE("IntegrateMotion");
		runIntegrateMotionKernel();
	}
	{
		runComputeCellIdKernel();
	}
	{
		BT_PROFILE("SortHash");
		runSortHashKernel();
	}
	{
		BT_PROFILE("FindCellStart");
		runFindCellStartKernel();
	}
	{
		BT_PROFILE("CollideParticles");
//		printf("\ncollide particles\n\n");
		runCollideParticlesKernel();
	}
	gStepNum++;

#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE
	return 1;
}

static unsigned int getMaxPowOf2(unsigned int num)
{
	unsigned int maxPowOf2 = 1;
	for(int bit = 1; bit < 32; bit++)
	{
		if(maxPowOf2 >= num)
		{
			break;
		}
		maxPowOf2 <<= 1;
	}
	return maxPowOf2;
}


void btParticlesDynamicsWorld::initDeviceData()
{
	getShapeData();
}



void btParticlesDynamicsWorld::postInitDeviceData()
{
	m_hashSize = getMaxPowOf2(m_numParticles);
	createVBO();
	allocateBuffers();
	adjustGrid();
	grabSimulationData();
}


void btParticlesDynamicsWorld::getShapeData()
{
	int numObjects = getNumCollisionObjects();
	btCollisionObjectArray& collisionObjects = getCollisionObjectArray();
	for(int i = 0; i < numObjects; i++)
	{
		btCollisionObject* colObj = collisionObjects[i];
		btCollisionShape* pShape = colObj->getCollisionShape();
		int shapeType = pShape->getShapeType();
		if(shapeType == SPHERE_SHAPE_PROXYTYPE)
		{
			btSphereShape* pSph = (btSphereShape*)pShape;
			btScalar sphRad = pSph->getRadius();
			if(!i)
			{
				m_particleRad = sphRad;
			}
			else
			{
				btAssert(m_particleRad == sphRad);
			}
		}
		else
		{
			btAssert(0);
		}
	}
	printf("Total number of particles : %d\n", m_numParticles);
}

void btParticlesDynamicsWorld::allocateBuffers()
{
    cl_int ciErrNum;
	// positions of spheres
	m_hPos.resize(m_numParticles);
	m_hVel.resize(m_numParticles);
	m_hSortedPos.resize(m_numParticles);
	m_hSortedVel.resize(m_numParticles);
	m_hPosHash.resize(m_hashSize); 
	for(int i = 0; i < m_hashSize; i++) { m_hPosHash[i].x = 0x7FFFFFFF; m_hPosHash[i].y = 0; }
    unsigned int memSize = sizeof(btVector3) *  m_numParticles;
    m_dPos = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    m_dVel = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    m_dSortedPos = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    m_dSortedVel = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_hashSize * sizeof(btInt2);
	m_dPosHash = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	// global simulation parameters
	memSize = sizeof(btSimParams);
	m_dSimParams = clCreateBuffer(m_cxMainContext, CL_MEM_READ_ONLY, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void btParticlesDynamicsWorld::adjustGrid()
{
	//btVector3 wmin( BT_LARGE_FLOAT,  BT_LARGE_FLOAT,  BT_LARGE_FLOAT);
	//btVector3 wmax(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);

	btVector3 wmin( BT_LARGE_FLOAT,  BT_LARGE_FLOAT, BT_LARGE_FLOAT);
	btVector3 wmax(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
	btVector3 boxDiag(m_particleRad, m_particleRad, m_particleRad);
	for(int i = 0; i < m_numParticles; i++)
	{
		btVector3 pos = m_hPos[i];
		btVector3 boxMin = pos - boxDiag;
		btVector3 boxMax = pos + boxDiag;
		wmin.setMin(boxMin);
		wmax.setMax(boxMax);
	}
	m_worldMin = wmin;
	m_worldMax = wmax;
	btVector3 wsize = m_worldMax - m_worldMin;
	wsize[3] = 1.0f;

	glBindBufferARB(GL_ARRAY_BUFFER, m_colVbo);
    btVector3* color = (btVector3*)glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for(int i = 0; i < m_numParticles; i++, color++)
	{
		*color = (m_hPos[i] - m_worldMin) / wsize;
		(*color)[3] = 1.f;
	}
    glUnmapBufferARB(GL_ARRAY_BUFFER);

/*
	wsize[0] *= 0.5f;
	wsize[1] *= 0.1f;
	wsize[2] *= 0.5f;
	m_worldMin -= wsize;
	m_worldMax += wsize;
*/
	m_worldMin.setValue(-WORLD_SIZE, -WORLD_SIZE, -WORLD_SIZE);
	m_worldMax.setValue( WORLD_SIZE,  WORLD_SIZE,  WORLD_SIZE);
	wsize = m_worldMax - m_worldMin;

	m_cellSize[0] = m_cellSize[1] = m_cellSize[2] = m_particleRad * btScalar(2.f);

	m_simParams.m_worldMin[0] = m_worldMin[0];
	m_simParams.m_worldMin[1] = m_worldMin[1];
	m_simParams.m_worldMin[2] = m_worldMin[2];

	m_simParams.m_worldMax[0] = m_worldMax[0];
	m_simParams.m_worldMax[1] = m_worldMax[1];
	m_simParams.m_worldMax[2] = m_worldMax[2];

	m_simParams.m_cellSize[0] = m_cellSize[0];
	m_simParams.m_cellSize[1] = m_cellSize[1];
	m_simParams.m_cellSize[2] = m_cellSize[2];

	m_simParams.m_gridSize[0] = (int)(wsize[0] / m_cellSize[0] + 0.999999f);
	m_simParams.m_gridSize[1] = (int)(wsize[1] / m_cellSize[1] + 0.999999f);
	m_simParams.m_gridSize[2] = (int)(wsize[2] / m_cellSize[2] + 0.999999f);

	m_numGridCells = m_simParams.m_gridSize[0] * m_simParams.m_gridSize[1] * m_simParams.m_gridSize[2];
	m_hCellStart.resize(m_numGridCells);
    unsigned int memSize = sizeof(int) *  m_numGridCells;
    cl_int ciErrNum;
	m_dCellStart = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

}


void btParticlesDynamicsWorld::grabSimulationData()
{
//	const btVector3& gravity = getGravity();
	//btVector3 gravity(0., -0.06, 0.);
	//btVector3 gravity(0., -0.0003f, 0.);
	btVector3 gravity(0,-0.0003,0);

	

	m_simParams.m_gravity[0] = gravity[0];
	m_simParams.m_gravity[1] = gravity[1];
	m_simParams.m_gravity[2] = gravity[2];
	m_simParams.m_particleRad = m_particleRad;
	m_simParams.m_globalDamping = 1.0f;
	m_simParams.m_boundaryDamping = -0.5f;

//	m_simParams.m_collisionDamping = 0.02f;
//	m_simParams.m_spring = 0.5f;
//	m_simParams.m_shear = 0.1f;
//	m_simParams.m_attraction = 0.0f;
	m_simParams.m_collisionDamping = 0.025f;//0.02f;
	m_simParams.m_spring = 0.5f;
	m_simParams.m_shear = 0.1f;
	m_simParams.m_attraction = 0.001f;



	// copy data to GPU
    cl_int ciErrNum;
	unsigned int memSize = sizeof(btVector3) * m_numParticles;
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dVel, CL_TRUE, 0, memSize, &(m_hVel[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = sizeof(btSimParams);
    ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dSimParams, CL_TRUE, 0, memSize, &m_simParams, 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
	memSize = m_hashSize * sizeof(btInt2);
	ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}


void btParticlesDynamicsWorld::createVBO()
{
    // create buffer object
    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	// positions of spheres
    unsigned int memSize = sizeof(btVector3) *  m_numParticles;
    glBufferData(GL_ARRAY_BUFFER, memSize, 0, GL_DYNAMIC_DRAW);
	// colors
	GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, memSize, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m_colVbo = vbo;
    // fill color buffer
    glBindBufferARB(GL_ARRAY_BUFFER, m_colVbo);
    float *data = (float*)glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    float *ptr = data;
    for(int i = 0; i < m_numParticles; i++) 
	{
        float t = i / (float)m_numParticles;
		ptr[0] = 0.f;
		ptr[1] = 1.f;
		ptr[2] = 0.f;
        ptr+=3;
        *ptr++ = 1.0f;
    }
    glUnmapBufferARB(GL_ARRAY_BUFFER);
	glBindBufferARB(GL_ARRAY_BUFFER, 0);
}



void btParticlesDynamicsWorld::initCLKernels(int argc, char** argv)
{
    cl_int ciErrNum;

	if (!m_cxMainContext)
	{
		
		cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
		m_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0, 0);
	
		int numDev = btOpenCLUtils::getNumDevices(m_cxMainContext);
		if (!numDev)
		{
			btAssert(0);
			exit(0);//this is just a demo, exit now
		}

		m_cdDevice =  btOpenCLUtils::getDevice(m_cxMainContext,0);
    	oclCHECKERROR(ciErrNum, CL_SUCCESS);

		btOpenCLDeviceInfo clInfo;
		btOpenCLUtils::getDeviceInfo(m_cdDevice,clInfo);
		btOpenCLUtils::printDeviceInfo(m_cdDevice);

		// create a command-queue
		m_cqCommandQue = clCreateCommandQueue(m_cxMainContext, m_cdDevice, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
	// Program Setup
	size_t program_length;


#ifdef LOAD_FROM_MEMORY
	program_length = strlen(source);
	printf("OpenCL compiles ParticlesOCL.cl ... ");
#else

	const char* fileName = "ParticlesOCL.cl";
	FILE * fp = fopen(fileName, "rb");
	char newFileName[512];
	
	if (fp == NULL)
	{
		sprintf(newFileName,"..//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
	}
	
	if (fp == NULL)
	{
		sprintf(newFileName,"Demos//ParticlesOpenCL//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
	}

	if (fp == NULL)
	{
		sprintf(newFileName,"..//..//..//..//..//Demos//ParticlesOpenCL//%s",fileName);
		fp = fopen(newFileName, "rb");
		if (fp)
			fileName = newFileName;
		else
		{
			printf("cannot find %s\n",newFileName);
			exit(0);
		}
	}

//	char *source = oclLoadProgSource(".//Demos//SpheresGrid//SpheresGrid.cl", "", &program_length);
	//char *source = btOclLoadProgSource(".//Demos//SpheresOpenCL//Shared//SpheresGrid.cl", "", &program_length);

	char *source = btOclLoadProgSource(fileName, "", &program_length);
	if(source == NULL)
	{
		printf("ERROR : OpenCL can't load file %s\n", fileName);
	}
//	oclCHECKERROR (source == NULL, oclFALSE);   
	btAssert(source != NULL);

	// create the program
	printf("OpenCL compiles %s ...", fileName);

#endif //LOAD_FROM_MEMORY


	//printf("%s\n", source);

	m_cpProgram = clCreateProgramWithSource(m_cxMainContext, 1, (const char**)&source, &program_length, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
#ifndef LOAD_FROM_MEMORY
	free(source);
#endif //LOAD_FROM_MEMORY

	//#define LOCAL_SIZE_LIMIT 1024U
#define LOCAL_SIZE_MAX 1024U

		    // Build the program with 'mad' Optimization option
#ifdef MAC
	const char* flags = "-I. -DLOCAL_SIZE_MAX=1024U -cl-mad-enable -DMAC -DGUID_ARG";
#else
	const char* flags = "-I. -DLOCAL_SIZE_MAX=1024U -DGUID_ARG= ";
#endif
	// build the program
	ciErrNum = clBuildProgram(m_cpProgram, 0, NULL, flags, NULL, NULL);
	if(ciErrNum != CL_SUCCESS)
	{
		// write out standard error
//		oclLog(LOGBOTH | ERRORMSG, (double)ciErrNum, STDERROR);
		// write out the build log and ptx, then exit
		char cBuildLog[10240];
//		char* cPtx;
//		size_t szPtxLength;
		clGetProgramBuildInfo(m_cpProgram, m_cdDevice, CL_PROGRAM_BUILD_LOG, 
							  sizeof(cBuildLog), cBuildLog, NULL );
//		oclGetProgBinary(m_cpProgram, oclGetFirstDev(m_cxMainContext), &cPtx, &szPtxLength);
//		oclLog(LOGBOTH | CLOSELOG, 0.0, "\n\nLog:\n%s\n\n\n\n\nPtx:\n%s\n\n\n", cBuildLog, cPtx);
		printf("\n\n%s\n\n\n", cBuildLog);
		printf("Press ENTER key to terminate the program\n");
		getchar();
		exit(-1); 
	}
	printf("OK\n");

	// create the kernels

	postInitDeviceData();

	initKernel(PARTICLES_KERNEL_COMPUTE_CELL_ID, "kComputeCellId");
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_COMPUTE_CELL_ID].m_kernel, 1, sizeof(cl_mem), (void*) &m_dPos);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_COMPUTE_CELL_ID].m_kernel, 2, sizeof(cl_mem), (void*) &m_dPosHash);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_COMPUTE_CELL_ID].m_kernel, 3, sizeof(cl_mem), (void*) &m_dSimParams);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	initKernel(PARTICLES_KERNEL_INTEGRATE_MOTION, "kIntegrateMotion");
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_INTEGRATE_MOTION].m_kernel, 1, sizeof(cl_mem), (void *) &m_dPos);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_INTEGRATE_MOTION].m_kernel, 2, sizeof(cl_mem), (void *) &m_dVel);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_INTEGRATE_MOTION].m_kernel, 3, sizeof(cl_mem), (void *) &m_dSimParams);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);


	initKernel(PARTICLES_KERNEL_CLEAR_CELL_START, "kClearCellStart");
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_CLEAR_CELL_START].m_kernel, 0, sizeof(int),		(void *) &m_numGridCells);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_CLEAR_CELL_START].m_kernel, 1, sizeof(cl_mem),	(void*) &m_dCellStart);

	initKernel(PARTICLES_KERNEL_FIND_CELL_START, "kFindCellStart");
//	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_FIND_CELL_START].m_kernel, 0, sizeof(int),	(void*) &m_numParticles);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_FIND_CELL_START].m_kernel, 1, sizeof(cl_mem),	(void*) &m_dPosHash);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_FIND_CELL_START].m_kernel, 2, sizeof(cl_mem),	(void*) &m_dCellStart);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_FIND_CELL_START].m_kernel, 3, sizeof(cl_mem),	(void*) &m_dPos);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_FIND_CELL_START].m_kernel, 4, sizeof(cl_mem),	(void*) &m_dVel);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_FIND_CELL_START].m_kernel, 5, sizeof(cl_mem),	(void*) &m_dSortedPos);
	ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_FIND_CELL_START].m_kernel, 6, sizeof(cl_mem),	(void*) &m_dSortedVel);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	initKernel(PARTICLES_KERNEL_COLLIDE_PARTICLES, "kCollideParticles");
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_COLLIDE_PARTICLES].m_kernel, 1, sizeof(cl_mem),	(void*) &m_dVel);
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_COLLIDE_PARTICLES].m_kernel, 2, sizeof(cl_mem),	(void*) &m_dSortedPos);
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_COLLIDE_PARTICLES].m_kernel, 3, sizeof(cl_mem),	(void*) &m_dSortedVel);
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_COLLIDE_PARTICLES].m_kernel, 4, sizeof(cl_mem),	(void*) &m_dPosHash);
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_COLLIDE_PARTICLES].m_kernel, 5, sizeof(cl_mem),	(void*) &m_dCellStart);
	ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_COLLIDE_PARTICLES].m_kernel, 6, sizeof(cl_mem),	(void*) &m_dSimParams);

	initKernel(PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL, "kBitonicSortCellIdLocal");
	initKernel(PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1, "kBitonicSortCellIdLocal1");
	initKernel(PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL, "kBitonicSortCellIdMergeGlobal");
	initKernel(PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL, "kBitonicSortCellIdMergeLocal");
}

static btInt4 cpu_getGridPos(btVector3& worldPos, btSimParams* pParams)
{
    btInt4 gridPos;
	gridPos.x = (int)floor((worldPos[0] - pParams->m_worldMin[0]) / pParams->m_cellSize[0]);
	gridPos.y = (int)floor((worldPos[1] - pParams->m_worldMin[1]) / pParams->m_cellSize[1]);
	gridPos.z = (int)floor((worldPos[2] - pParams->m_worldMin[2]) / pParams->m_cellSize[2]);
    return gridPos;
}

static unsigned int cpu_getPosHash(btInt4& gridPos, btSimParams* pParams)
{
	btInt4 gridDim = *((btInt4*)(pParams->m_gridSize));
	if(gridPos.x < 0) gridPos.x = 0;
	if(gridPos.x >= gridDim.x) gridPos.x = gridDim.x - 1;
	if(gridPos.y < 0) gridPos.y = 0;
	if(gridPos.y >= gridDim.y) gridPos.y = gridDim.y - 1;
	if(gridPos.z < 0) gridPos.z = 0;
	if(gridPos.z >= gridDim.z) gridPos.z = gridDim.z - 1;
	unsigned int hash = gridPos.z * gridDim.y * gridDim.x + gridPos.y * gridDim.x + gridPos.x;
	return hash;
} 




void btParticlesDynamicsWorld::runComputeCellIdKernel()
{
    cl_int ciErrNum;
#if 0
	if(m_useCpuControls[SIMSTAGE_COMPUTE_CELL_ID]->m_active)
	{	// CPU version
		unsigned int memSize = sizeof(btVector3) * m_numParticles;
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		for(int index = 0; index < m_numParticles; index++)
		{
			btVector3 pos = m_hPos[index];
			btInt4 gridPos = cpu_getGridPos(pos, &m_simParams);
			unsigned int hash = cpu_getPosHash(gridPos, &m_simParams);
			m_hPosHash[index].x = hash;
			m_hPosHash[index].y = index;
		}
		memSize = sizeof(btInt2) * m_numParticles;
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
	else
#endif
	{
		BT_PROFILE("ComputeCellId");
		runKernelWithWorkgroupSize(PARTICLES_KERNEL_COMPUTE_CELL_ID, m_numParticles);
		ciErrNum = clFinish(m_cqCommandQue);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
/*
	// check
	int memSize = sizeof(btInt2) * m_hashSize;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = sizeof(float) * 4 * m_numParticles;
    ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);
*/

	{
		BT_PROFILE("Copy VBO");
		// Explicit Copy (until OpenGL interop will work)
		// map the PBO to copy data from the CL buffer via host
		glBindBufferARB(GL_ARRAY_BUFFER, m_vbo);    
		// map the buffer object into client's memory
		void* ptr = glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY_ARB);
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, sizeof(float) * 4 * m_numParticles, ptr, 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		glUnmapBufferARB(GL_ARRAY_BUFFER); 
		glBindBufferARB(GL_ARRAY_BUFFER,0);
	}
}



static btVector3 cpu_collideTwoParticles(
    btVector3& posA,
    btVector3& posB,
    btVector3& velA,
    btVector3& velB,
    float radiusA,
    float radiusB,
    float spring,
    float damping,
    float shear,
    float attraction
)
{
    //Calculate relative position
    btVector3  relPos = posB - posA; relPos[3] = 0.f;
    float        dist = sqrt(relPos[0] * relPos[0] + relPos[1] * relPos[1] + relPos[2] * relPos[2]);
    float collideDist = radiusA + radiusB;

    btVector3 force = btVector3(0, 0, 0);
    if(dist < collideDist)
	{
        btVector3 norm = relPos / dist;

        //Relative velocity
        btVector3 relVel = velB - velA; relVel[3] = 0.f;;

        //Relative tangential velocity
        float relVelDotNorm = relVel.dot(norm);
		btVector3 tanVel = relVel - relVelDotNorm * norm; 
        //Spring force (potential)
        //float springFactor = -spring * (collideDist - dist);
		float springFactor = -0.4 * (collideDist - dist);
		force = springFactor * norm + damping * relVel;// + shear * tanVel + attraction * relPos;
    }
    return force;
}

struct btPair
{
	union
	{
		int value;
		short v0[2];
	};
};

void btParticlesDynamicsWorld::runCollideParticlesKernel()
{
	btAlignedObjectArray<int>	pairs;

//	float particleRad = m_simParams.m_particleRad;
//	float collideDist2 = (particleRad + particleRad)*(particleRad + particleRad);
	cl_int ciErrNum;
	if(m_useCpuControls[SIMSTAGE_COLLIDE_PARTICLES]->m_active)
	{	// CPU version
		int memSize = sizeof(btVector3) * m_numParticles;
		{
			BT_PROFILE("Copy from GPU");
			ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dSortedPos, CL_TRUE, 0, memSize, &(m_hSortedPos[0]), 0, NULL, NULL);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dSortedVel, CL_TRUE, 0, memSize, &(m_hSortedVel[0]), 0, NULL, NULL);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			memSize = sizeof(btInt2) * m_numParticles;
			ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
			memSize = m_numGridCells * sizeof(int);
			ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dCellStart, CL_TRUE, 0, memSize, &(m_hCellStart[0]), 0, NULL, NULL);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}

		for(int index = 0; index < m_numParticles; index++)
		{
			btVector3 posA = m_hSortedPos[index];
			btVector3 velA = m_hSortedVel[index];
			btVector3 force = btVector3(0, 0, 0);
			float particleRad = m_simParams.m_particleRad;
			float collisionDamping = m_simParams.m_collisionDamping;
			float spring = m_simParams.m_spring;
			float shear = m_simParams.m_shear;
			float attraction = m_simParams.m_attraction;
			int unsortedIndex = m_hPosHash[index].y;
			//Get address in grid
			btInt4 gridPosA = cpu_getGridPos(posA, &m_simParams);
			//Accumulate surrounding cells
			btInt4 gridPosB; 
			for(int z = -1; z <= 1; z++)
			{
				gridPosB.z = gridPosA.z + z;
				for(int y = -1; y <= 1; y++)
				{
					gridPosB.y = gridPosA.y + y;
					for(int x = -1; x <= 1; x++)
					{
						gridPosB.x = gridPosA.x + x;
						//Get start particle index for this cell
						unsigned int hashB = cpu_getPosHash(gridPosB, &m_simParams);
						int startI = m_hCellStart[hashB];
						//Skip empty cell
						if(startI < 0)
						{
							continue;
						}
						//Iterate over particles in this cell
						int endI = startI + 32;
						if(endI > m_numParticles) 
							endI = m_numParticles;

						for(int j = startI; j < endI; j++)
						{
							unsigned int hashC = m_hPosHash[j].x;
							if(hashC != hashB)
							{
								break;
							}
							if(j == index)
							{
								continue;
							}

							btPair pair;
							pair.v0[0] = index;
							pair.v0[1] = j;
							pairs.push_back(pair.value);

//							printf("index=%d, j=%d\n",index,j);
//							printf("(index=%d, j=%d) ",index,j);
							btVector3 posB = m_hSortedPos[j];
							btVector3 velB = m_hSortedVel[j];
							//Collide two spheres
							force += cpu_collideTwoParticles(	posA, posB, velA, velB, particleRad, particleRad, 
																spring, collisionDamping, shear, attraction);
						}
					}
				}
			}     
			//Write new velocity back to original unsorted location
			m_hVel[unsortedIndex] = velA + force;
		}	

//#define BRUTE_FORCE_CHECK 1
#ifdef BRUTE_FORCE_CHECK
		for(int index = 0; index < m_numParticles; index++)
		{
			btVector3 posA = m_hSortedPos[index];
			btVector3 velA = m_hSortedVel[index];
			btVector3 force = btVector3(0, 0, 0);
			int unsortedIndex = m_hPosHash[index].y;
			
			float collisionDamping = m_simParams.m_collisionDamping;
			float spring = m_simParams.m_spring;
			float shear = m_simParams.m_shear;
			float attraction = m_simParams.m_attraction;
			for(int j = 0 ; j < m_numParticles; j++)
			{
				if (index!=j)
				{
					btVector3 posB = m_hSortedPos[j];
					btVector3 velB = m_hSortedVel[j];


					btVector3  relPos = posB - posA; relPos[3] = 0.f;
					float        dist2 = (relPos[0] * relPos[0] + relPos[1] * relPos[1] + relPos[2] * relPos[2]);
					

					
					if(dist2 < collideDist2)
					{
										//Collide two spheres
						//				force += cpu_collideTwoParticles(	posA, posB, velA, velB, particleRad, particleRad, 
						//													spring, collisionDamping, shear, attraction);

						btPair pair;
						pair.v0[0] = index;
						pair.v0[1] = j;
						if (pairs.findLinearSearch(pair.value)==pairs.size())
						{
							printf("not found index=%d, j=%d\n",index,j);
						} 

										
					}
				}
			}
			//Write new velocity back to original unsorted location
			//m_hVel[unsortedIndex] = velA + force;
		}
#endif //BRUTE_FORCE_CHECK

		memSize = sizeof(btVector3) * m_numParticles;
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dVel, CL_TRUE, 0, memSize, &(m_hVel[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
	else
	{
		runKernelWithWorkgroupSize(PARTICLES_KERNEL_COLLIDE_PARTICLES, m_numParticles);
		cl_int ciErrNum = clFinish(m_cqCommandQue);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
}


void btParticlesDynamicsWorld::runIntegrateMotionKernel()
{
    cl_int ciErrNum;
	if(m_useCpuControls[SIMSTAGE_INTEGRATE_MOTION]->m_active)
	{
		// CPU version
#if 1
		// read from GPU
		unsigned int memSize = sizeof(btVector3) * m_numParticles;
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dVel, CL_TRUE, 0, memSize, &(m_hVel[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		for(int index = 0; index < m_numParticles; index++)
		{
			btVector3 pos = m_hPos[index];
			btVector3 vel = m_hVel[index];
			pos[3] = 1.0f;
			vel[3] = 0.0f;
			// apply gravity
			btVector3 gravity;
			gravity[0] = m_simParams.m_gravity[0];
			gravity[1] = m_simParams.m_gravity[1];
			gravity[2] = m_simParams.m_gravity[2];

			float particleRad = m_simParams.m_particleRad;
			float globalDamping = m_simParams.m_globalDamping;
			float boundaryDamping = m_simParams.m_boundaryDamping;
			vel += gravity * m_timeStep;
			vel *= globalDamping;
			// integrate position
			pos += vel * m_timeStep;
			// collide with world boundaries
			btVector3 worldMin;
			worldMin[0] = m_simParams.m_worldMin[0];
			worldMin[1] = m_simParams.m_worldMin[1];
			worldMin[2] = m_simParams.m_worldMin[2];

			btVector3 worldMax;
			worldMax[0] = m_simParams.m_worldMax[0];
			worldMax[1] = m_simParams.m_worldMax[1];
			worldMax[2] = m_simParams.m_worldMax[2];

			for(int j = 0; j < 3; j++)
			{
				if(pos[j] < (worldMin[j] + particleRad))
				{
					pos[j] = worldMin[j] + particleRad;
					vel[j] *= boundaryDamping;
				}
				if(pos[j] > (worldMax[j] - particleRad))
				{
					pos[j] = worldMax[j] - particleRad;
					vel[j] *= boundaryDamping;
				}
			}
			// write back position and velocity
			m_hPos[index] = pos;
			m_hVel[index] = vel;
		}
#endif
		// write back to GPU
		memSize = sizeof(btVector3) * m_numParticles;
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dVel, CL_TRUE, 0, memSize, &(m_hVel[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
	else
	{
		// Set work size and execute the kernel
		ciErrNum = clSetKernelArg(m_kernels[PARTICLES_KERNEL_INTEGRATE_MOTION].m_kernel, 4, sizeof(float), &m_timeStep);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		runKernelWithWorkgroupSize(PARTICLES_KERNEL_INTEGRATE_MOTION, m_numParticles);
		ciErrNum = clFinish(m_cqCommandQue);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
}

void btParticlesDynamicsWorld::runSortHashKernel()
{
	cl_int ciErrNum;
	int memSize = m_numParticles * sizeof(btInt2);
	if(m_useCpuControls[SIMSTAGE_SORT_CELL_ID]->m_active)
	{
		// CPU version
		// get hash from GPU
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		// sort
		class btHashPosKey
		{
		public:
			unsigned int hash;
			unsigned int index;
			void quickSort(btHashPosKey* pData, int lo, int hi)
			{
				int i=lo, j=hi;
				btHashPosKey x = pData[(lo+hi)/2];
				do
				{    
					while(pData[i].hash < x.hash) i++; 
					while(x.hash < pData[j].hash) j--;
					if(i <= j)
					{
						btHashPosKey t = pData[i];
						pData[i] = pData[j];
						pData[j] = t;
						i++; j--;
					}
				} while(i <= j);
				if(lo < j) pData->quickSort(pData, lo, j);
				if(i < hi) pData->quickSort(pData, i, hi);
			}
			void bitonicSort(btHashPosKey* pData, int lo, int n, bool dir)
			{
				if(n > 1)
				{
					int m = n / 2;
					bitonicSort(pData, lo, m, !dir);
					bitonicSort(pData, lo + m, n - m, dir);
					bitonicMerge(pData, lo, n, dir);
				}
			}
			void bitonicMerge(btHashPosKey* pData, int lo, int n, bool dir)
			{
				if(n > 1)
				{
					int m = greatestPowerOfTwoLessThan(n);
					for(int i = lo; i < (lo + n - m); i++)
					{
						compare(pData, i, i + m, dir);
					}
					bitonicMerge(pData, lo, m, dir);
					bitonicMerge(pData, lo + m, n - m, dir);
				}
			}
			void compare(btHashPosKey* pData, int i, int j, bool dir)
			{
				if(dir == (pData[i].hash > pData[j].hash))
				{
					btHashPosKey t = pData[i];
					pData[i] = pData[j];
					pData[j] = t;
				}
			}
			int greatestPowerOfTwoLessThan(int n)
			{
				int k = 1;
				while(k < n)
				{
					k = k << 1;
				}
				return k>>1;
			}
		};
		btHashPosKey* pHash = (btHashPosKey*)(&m_hPosHash[0]);
		pHash->quickSort(pHash, 0, m_numParticles-1 );
	//	pHash->bitonicSort(pHash, 0, m_hashSize, true);
		// write back to GPU
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
	else
	{
		 // bitonic sort on GPU (shared memory)	
		int dir = 1;
		bitonicSortNv(m_dPosHash, 1, m_hashSize, dir);
		ciErrNum = clFinish(m_cqCommandQue);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
#if 0
	// check order
	memSize = m_numParticles * sizeof(btInt2);
	ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	for(int i = 1; i < m_hashSize; i++)
	{
		if(m_hPosHash[i-1].x > m_hPosHash[i].x)
		{
			printf("Hash sort error at %d\n", i);
		}
	}
#endif
}


void btParticlesDynamicsWorld::runFindCellStartKernel()
{
    cl_int ciErrNum;
	if(m_useCpuControls[SIMSTAGE_FIND_CELL_START]->m_active)
	{
		// CPU version
		// get hash from GPU
		int memSize = m_numParticles * sizeof(btInt2);
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPosHash, CL_TRUE, 0, memSize, &(m_hPosHash[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		memSize = sizeof(btVector3) * m_numParticles;
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dPos, CL_TRUE, 0, memSize, &(m_hPos[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, m_dVel, CL_TRUE, 0, memSize, &(m_hVel[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		// clear cells
		for(int i = 0; i < m_numGridCells; i++)
		{
			m_hCellStart[i] = -1;
		}
		// find start of each cell in sorted hash
		btInt2 hash = m_hPosHash[0];
		m_hCellStart[hash.x] = 0;
		int unsortedIndex = hash.y;
		btVector3 pos = m_hPos[unsortedIndex];
		btVector3 vel = m_hVel[unsortedIndex];
		m_hSortedPos[0] = pos;
		m_hSortedVel[0] = vel;
		for(int i = 1; i < m_numParticles; i++)
		{
			if(m_hPosHash[i-1].x != m_hPosHash[i].x)
			{
				m_hCellStart[m_hPosHash[i].x] = i;
			}
			unsortedIndex = m_hPosHash[i].y;
			pos = m_hPos[unsortedIndex];
			vel = m_hVel[unsortedIndex];
			m_hSortedPos[i] = pos;
			m_hSortedVel[i] = vel;
		}
		// write back to GPU
		memSize = m_numGridCells * sizeof(int);
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dCellStart, CL_TRUE, 0, memSize, &(m_hCellStart[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		memSize = sizeof(btVector3) * m_numParticles;
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dSortedPos, CL_TRUE, 0, memSize, &(m_hSortedPos[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, m_dSortedVel, CL_TRUE, 0, memSize, &(m_hSortedVel[0]), 0, NULL, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
	else
	{	// GPU
		runKernelWithWorkgroupSize(PARTICLES_KERNEL_CLEAR_CELL_START, m_numGridCells);
		runKernelWithWorkgroupSize(PARTICLES_KERNEL_FIND_CELL_START, m_numParticles);
		ciErrNum = clFinish(m_cqCommandQue);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
}


void btParticlesDynamicsWorld::initKernel(int kernelId, const char* pName)
{
	
	cl_int ciErrNum;
	cl_kernel kernel = clCreateKernel(m_cpProgram, pName, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	size_t wgSize;
	ciErrNum = clGetKernelWorkGroupInfo(kernel, m_cdDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	

	

//	if (wgSize > 256)
//		wgSize = 256;

	if (wgSize > 512)
		wgSize = 512;

//	if (wgSize > 1024)
//		wgSize = 1024;

	m_kernels[kernelId].m_Id = kernelId;
	m_kernels[kernelId].m_kernel = kernel;
	m_kernels[kernelId].m_name = pName;
	m_kernels[kernelId].m_workgroupSize = wgSize;

	return;
}

void btParticlesDynamicsWorld::runKernelWithWorkgroupSize(int kernelId, int globalSize)
{
	if(globalSize <= 0)
	{
		return;
	}
	cl_kernel kernelFunc = m_kernels[kernelId].m_kernel;
	cl_int ciErrNum = clSetKernelArg(kernelFunc, 0, sizeof(int), (void*)&globalSize);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	int workgroupSize = m_kernels[kernelId].m_workgroupSize;
	if(workgroupSize <= 0)
	{ // let OpenCL library calculate workgroup size
		size_t globalWorkSize[2];
		globalWorkSize[0] = globalSize;
		globalWorkSize[1] = 1;
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, kernelFunc, 1, NULL, globalWorkSize, NULL, 0,0,0 );
	}
	else
	{
		size_t localWorkSize[2], globalWorkSize[2];
		workgroupSize = btMin(workgroupSize, globalSize);
		int num_t = globalSize / workgroupSize;
		int num_g = num_t * workgroupSize;
		if(num_g < globalSize)
		{
			num_t++;
		}
		localWorkSize[0]  = workgroupSize;
		globalWorkSize[0] = num_t * workgroupSize;
		localWorkSize[1] = 1;
		globalWorkSize[1] = 1;
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, kernelFunc, 1, NULL, globalWorkSize, localWorkSize, 0,0,0 );
	}
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}


//Note: logically shared with BitonicSort OpenCL code!
// TODO : get parameter from OpenCL and pass it to kernel (needed for platforms other than NVIDIA)
//static const unsigned int LOCAL_SIZE_LIMIT = 1024U;

void btParticlesDynamicsWorld::bitonicSortNv(cl_mem pKey, unsigned int batch, unsigned int arrayLength, unsigned int dir)
{
	unsigned int localSizeLimit = m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_workgroupSize * 2;
    if(arrayLength < 2)
        return;
    //Only power-of-two array lengths are supported so far
    dir = (dir != 0);
    cl_int ciErrNum;
    size_t localWorkSize, globalWorkSize;
    if(arrayLength <= localSizeLimit)
    {
        btAssert( (batch * arrayLength) % localSizeLimit == 0);
        //Launch bitonicSortLocal
		ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 0,   sizeof(cl_mem), (void *)&pKey);
        ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 1,  sizeof(cl_uint), (void *)&arrayLength);
        ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 2,  sizeof(cl_uint), (void *)&dir);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = localSizeLimit / 2;
        globalWorkSize = batch * arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);
    }
    else
    {
        //Launch bitonicSortLocal1
        ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = localSizeLimit / 2;
        globalWorkSize = batch * arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        for(unsigned int size = 2 * localSizeLimit; size <= arrayLength; size <<= 1)
        {
            for(unsigned stride = size / 2; stride > 0; stride >>= 1)
            {
                if(stride >= localSizeLimit)
                {
                    //Launch bitonicMergeGlobal
                    ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 2, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 3, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 4, sizeof(cl_uint), (void *)&dir);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = localSizeLimit / 4;
                    globalWorkSize = batch * arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);
                }
                else
                {
                    //Launch bitonicMergeLocal
					ciErrNum  = clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 2, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 3, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 4, sizeof(cl_uint), (void *)&dir);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = localSizeLimit / 2;
                    globalWorkSize = batch * arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, m_kernels[PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL].m_kernel, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);
                    break;
                }
            }
        }
    }
}

