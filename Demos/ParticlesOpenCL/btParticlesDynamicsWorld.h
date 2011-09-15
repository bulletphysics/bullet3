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


#ifndef BT_PARTICLES_DYNAMICS_WORLD_H
#define BT_PARTICLES_DYNAMICS_WORLD_H


#ifdef USE_MINICL
#include <MiniCL/cl.h>
#include <MiniCL/cl_gl.h>
#else
#ifdef __APPLE__
	#include <OpenCL/cl.h>
#else
	#include <CL/cl.h>
	#include <CL/cl_gl.h>
#endif __APPLE__
#endif






#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"

#include "btParticlesSharedDefs.h"
#include "btParticlesSharedTypes.h"


#define PARTICLES_MAX_NEIGHBORS (32)
#define DEF_PARTICLE_RADIUS (0.023f)
//#define WORLD_SIZE 1.9f
#define WORLD_SIZE 1.f

enum
{
	PARTICLES_KERNEL_INTEGRATE_MOTION = 0,
	PARTICLES_KERNEL_COMPUTE_CELL_ID,
	PARTICLES_KERNEL_CLEAR_CELL_START,
	PARTICLES_KERNEL_FIND_CELL_START,
	PARTICLES_KERNEL_COLLIDE_PARTICLES,
	PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL,
	PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_LOCAL_1,
	PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_GLOBAL,
	PARTICLES_KERNEL_BITONIC_SORT_CELL_ID_MERGE_LOCAL,
	PARTICLES_KERNEL_TOTAL
};


enum 
{
	SIMSTAGE_NONE = 0,
	SIMSTAGE_INTEGRATE_MOTION,
	SIMSTAGE_COMPUTE_CELL_ID,
	SIMSTAGE_SORT_CELL_ID,
	SIMSTAGE_FIND_CELL_START,
	SIMSTAGE_COLLIDE_PARTICLES,
	SIMSTAGE_TOTAL
};

struct btKernelInfo
{
	int			m_Id;
	cl_kernel	m_kernel;
	const char* m_name;
	int			m_workgroupSize;
};

class btParticlesDynamicsWorld : public btDiscreteDynamicsWorld
{
public:
	int			m_numParticles;
	int			m_usedDevice;
	btScalar	m_particleRad;
	struct GL_ToggleControl* m_useCpuControls[SIMSTAGE_TOTAL];
	
protected:
	int			m_hashSize; // power of 2 >= m_numSpheres;
	int			m_numGridCells; 
	int			m_maxNeighbors;
	int			m_numSolverIterations;
	// CPU side data
public:
	btAlignedObjectArray<btVector3>	m_hPos;
	btAlignedObjectArray<btVector3>	m_hVel;
	btAlignedObjectArray<btVector3>	m_hSortedPos;
	btAlignedObjectArray<btVector3>	m_hSortedVel;
protected:
	btAlignedObjectArray<btInt2>	m_hPosHash;
	btAlignedObjectArray<int>		m_hCellStart;
	// GPU side data
	cl_mem		m_dPos;
	cl_mem		m_dVel;
	cl_mem		m_dPosHash;
	cl_mem		m_dCellStart;
	cl_mem		m_dSimParams; // copy of m_simParams : global simulation paramerers such as gravity, etc. 
	cl_mem		m_dSortedPos;
	cl_mem		m_dSortedVel;
	// OpenCL 
public:
	cl_context			m_cxMainContext;
	cl_device_id		m_cdDevice;
	cl_command_queue	m_cqCommandQue;
	cl_program			m_cpProgram;
protected:
	btKernelInfo		m_kernels[PARTICLES_KERNEL_TOTAL];

	btVector3			m_cellSize;

public:
	btVector3			m_worldMin;
	btVector3			m_worldMax;
	// vbo variables
	GLuint			m_vbo;
	unsigned int	m_posVbo;
	unsigned int	m_colVbo;
	btSimParams		m_simParams;
	float			m_timeStep;

	int getNumParticles() { return m_numParticles; }
	float* getPosBuffer() { return (float*)&(m_hPos[0]); }


	btParticlesDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration,
			int maxObjs , int maxNeighbors = PARTICLES_MAX_NEIGHBORS)
		: btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
	{ 
		m_cxMainContext = 0;
		m_usedDevice = 1;
//		m_particleRad = btScalar(0.5f);
		m_particleRad = DEF_PARTICLE_RADIUS;
		m_simParams.m_gravity[0] = 0.f;
		m_simParams.m_gravity[1] = -10.f;
		m_simParams.m_gravity[2] = 0.f;
		m_simParams.m_gravity[3] = 0.f;
		m_numSolverIterations = 4;
	}
	virtual ~btParticlesDynamicsWorld();
	virtual int	stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

	void initDeviceData();
	void initCLKernels(int argc, char** argv);
	void createVBO();
	void postInitDeviceData();
	void getShapeData();
	void allocateBuffers();
	void grabSimulationData();
	void adjustGrid();
	void runIntegrateMotionKernel();
	void runComputeCellIdKernel();
	void runSortHashKernel();
	void runFindCellStartKernel();
	void runCollideParticlesKernel();

	void initKernel(int kernelId, const char* pName);
	void runKernelWithWorkgroupSize(int kernelId, int globalSize);
	void bitonicSortNv(cl_mem pKey, unsigned int batch, unsigned int arrayLength, unsigned int dir);

	void scanExclusiveLocal1(cl_mem d_Dst, cl_mem d_Src, unsigned int n, unsigned int size);
	void scanExclusiveLocal2(cl_mem d_Buffer, cl_mem d_Dst, cl_mem d_Src, unsigned int n, unsigned int size);
	void uniformUpdate(cl_mem d_Dst, cl_mem d_Buffer, unsigned int n);
	void scanExclusive(cl_mem d_Dst, cl_mem d_Src, unsigned int arrayLength);

};


#endif //BT_PARTICLES_DYNAMICS_WORLD_H
