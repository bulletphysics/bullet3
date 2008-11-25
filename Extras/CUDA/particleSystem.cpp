/*
 * Copyright 1993-2006 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO USER:   
 *
 * This source code is subject to NVIDIA ownership rights under U.S. and 
 * international Copyright laws.  
 *
 * NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE 
 * CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR 
 * IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH 
 * REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF 
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.   
 * IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL, 
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS 
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE 
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE 
 * OR PERFORMANCE OF THIS SOURCE CODE.  
 *
 * U.S. Government End Users.  This source code is a "commercial item" as 
 * that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of 
 * "commercial computer software" and "commercial computer software 
 * documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995) 
 * and is provided to the U.S. Government only as a commercial end item.  
 * Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through 
 * 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the 
 * source code with only those rights set forth herein.
 */

#include "particleSystem.h"
#include "particleSystem.cuh"
#include "radixsort.cuh"
#include "particles_kernel.cuh"

//#include <cutil.h>

#include <assert.h>
#include <math.h>
#include <memory.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <GL/glew.h>

#include <btBulletDynamicsCommon.h>
#include "../../Demos/OpenGL/GLDebugDrawer.h"

#include "btCudaBroadphase.h"


#ifndef CUDART_PI_F
#define CUDART_PI_F         3.141592654f
#endif

#define USE_BULLET 1

#define VEL_DIR_FACT (30.0F)
#define ACC_DIR_FACT (VEL_DIR_FACT*VEL_DIR_FACT)
#define VEL_INV_FACT (1.0F/VEL_DIR_FACT)
#define ACC_INV_FACT (1.0F/ACC_DIR_FACT)

GLDebugDrawer debugDrawer;


ParticleSystem::ParticleSystem(uint numParticles, uint3 gridSize) :
    m_bInitialized(false),
    m_numParticles(numParticles),
    m_hPos(0),
    m_hVel(0),
    m_currentPosRead(0),
    m_currentVelRead(0),
    m_currentPosWrite(1),
    m_currentVelWrite(1),
    m_gridSize(gridSize),
    m_maxParticlesPerCell(4),
    m_timer(0),
    m_solverIterations(1),
//	m_simulationMode(SIMULATION_CUDA)
	m_simulationMode(SIMULATION_BULLET_CPU)
{
    m_dPos[0] = m_dPos[1] = 0;
    m_dVel[0] = m_dVel[1] = 0;

    m_numGridCells = m_gridSize.x*m_gridSize.y*m_gridSize.z;
    float3 worldSize = make_float3(2.0f, 2.0f, 2.0f);

    // set simulation parameters
    m_params.gridSize = m_gridSize;
    m_params.numCells = m_numGridCells;
    m_params.numBodies = m_numParticles;
    m_params.maxParticlesPerCell = m_maxParticlesPerCell;

    m_params.worldOrigin = make_float3(-1.0f, -1.0f, -1.0f);
    m_params.cellSize = make_float3(worldSize.x / m_gridSize.x, worldSize.y / m_gridSize.y, worldSize.z / m_gridSize.z);

    m_params.particleRadius = m_params.cellSize.x * 0.5f;
    m_params.colliderPos = make_float4(0.0f, -0.7f, 0.0f, 1.0f);
    m_params.colliderRadius = 0.2f;

    m_params.spring = 0.5f;
    m_params.damping = 0.02f;
    m_params.shear = 0.1f;
    m_params.attraction = 0.0f;
    m_params.boundaryDamping = -0.5f;

    m_params.gravity = make_float3(0.0f, -0.0003f, 0.0f);
    m_params.globalDamping = 1.0f;

    _initialize(numParticles);

#if USE_BULLET
	initializeBullet();
#endif
}

ParticleSystem::~ParticleSystem()
{
#if USE_BULLET
	finalizeBullet();
#endif
    _finalize();
    m_numParticles = 0;
}

uint
ParticleSystem::createVBO(uint size)
{
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    registerGLBufferObject(vbo);
    return vbo;
}

inline float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}

void colorRamp(float t, float *r)
{
    const int ncolors = 7;
    float c[ncolors][3] = {
        { 1.0, 0.0, 0.0, },
        {  1.0, 0.5, 0.0, },
	{  1.0, 1.0, 0.0, },
	{  0.0, 1.0, 0.0, },
	{  0.0, 1.0, 1.0, },
	{  0.0, 0.0, 1.0, },
	{  1.0, 0.0, 1.0, },
    };
    t = t * (ncolors-1);
    int i = (int) t;
    float u = t - floor(t);
    r[0] = lerp(c[i][0], c[i+1][0], u);
    r[1] = lerp(c[i][1], c[i+1][1], u);
    r[2] = lerp(c[i][2], c[i+1][2], u);
}

void
ParticleSystem::_initialize(int numParticles)
{
    assert(!m_bInitialized);

    m_numParticles = numParticles;

    // allocate host storage
    m_hPos = new float[m_numParticles*4];
    m_hVel = new float[m_numParticles*4];
    memset(m_hPos, 0, m_numParticles*4*sizeof(float));
    memset(m_hVel, 0, m_numParticles*4*sizeof(float));

    m_hGridCounters = new uint[m_numGridCells];
    m_hGridCells = new uint[m_numGridCells*m_maxParticlesPerCell];
    memset(m_hGridCounters, 0, m_numGridCells*sizeof(uint));
    memset(m_hGridCells, 0, m_numGridCells*m_maxParticlesPerCell*sizeof(uint));

    m_hParticleHash = new uint[m_numParticles*2];
    memset(m_hParticleHash, 0, m_numParticles*2*sizeof(uint));

    m_hCellStart = new uint[m_numGridCells];
    memset(m_hCellStart, 0, m_numGridCells*sizeof(uint));

    // allocate GPU data
    unsigned int memSize = sizeof(float) * 4 * m_numParticles;

    m_posVbo[0] = createVBO(memSize);
    m_posVbo[1] = createVBO(memSize);
    
    allocateArray((void**)&m_dVel[0], memSize);
    allocateArray((void**)&m_dVel[1], memSize);

    allocateArray((void**)&m_dSortedPos, memSize);
    allocateArray((void**)&m_dSortedVel, memSize);

#if USE_SORT
    allocateArray((void**)&m_dParticleHash[0], m_numParticles*2*sizeof(uint));
    allocateArray((void**)&m_dParticleHash[1], m_numParticles*2*sizeof(uint));
    allocateArray((void**)&m_dCellStart, m_numGridCells*sizeof(uint));
#else
    allocateArray((void**)&m_dGridCounters, m_numGridCells*sizeof(uint));
    allocateArray((void**)&m_dGridCells, m_numGridCells*m_maxParticlesPerCell*sizeof(uint));
#endif

    m_colorVBO = createVBO(m_numParticles*4*sizeof(float));

#if 1
    // fill color buffer
    glBindBufferARB(GL_ARRAY_BUFFER, m_colorVBO);
    float *data = (float *) glMapBufferARB(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    float *ptr = data;
    for(uint i=0; i<m_numParticles; i++) {
        float t = i / (float) m_numParticles;
#if 0
        *ptr++ = rand() / (float) RAND_MAX;
        *ptr++ = rand() / (float) RAND_MAX;
        *ptr++ = rand() / (float) RAND_MAX;
#else
        colorRamp(t, ptr);
        ptr+=3;
#endif
        *ptr++ = 1.0f;
    }
    glUnmapBufferARB(GL_ARRAY_BUFFER);
#endif

//    CUT_SAFE_CALL(cutCreateTimer(&m_timer));

    setParameters(&m_params);

    m_bInitialized = true;
}

void
ParticleSystem::_finalize()
{
    assert(m_bInitialized);

    delete [] m_hPos;
    delete [] m_hVel;

    delete [] m_hGridCounters;
    delete [] m_hGridCells;

    freeArray(m_dVel[0]);
    freeArray(m_dVel[1]);

    freeArray(m_dSortedPos);
    freeArray(m_dSortedVel);

#if USE_SORT
    freeArray(m_dParticleHash[0]);
    freeArray(m_dParticleHash[1]);
    freeArray(m_dCellStart);
#else
    freeArray(m_dGridCounters);
    freeArray(m_dGridCells);
#endif

    unregisterGLBufferObject(m_posVbo[0]);
    unregisterGLBufferObject(m_posVbo[1]);
    glDeleteBuffers(2, (const GLuint*)m_posVbo);

    glDeleteBuffers(1, (const GLuint*)&m_colorVBO);
}

void 
ParticleSystem::update(float deltaTime)
{
    assert(m_bInitialized);
#if USE_BULLET
	switch (m_simulationMode)
	{
		case SIMULATION_CUDA:
		{
			updateCuda(deltaTime);
			break;
		}
		case SIMULATION_BULLET_CPU:
		{
			updateBullet(deltaTime);
			break;
		}
		default:
		{
			printf("unknown simulation method\n");
		}
	}
#else
	updateCuda(deltaTime);
#endif
}

void 
ParticleSystem::updateBullet(float deltaTime)
{
	float* hPos = copyBuffersFromDeviceToHost();
	float* hVel = m_hVel;

	for (uint i=0;i<m_params.numBodies;i++)
	{
		float3 pos;
		pos.x = hPos[i*4];
		pos.y = hPos[i*4+1];
		pos.z = hPos[i*4+2];
		float3 vel;
		vel.x = hVel[i*4];
		vel.y = hVel[i*4+1];
		vel.z = hVel[i*4+2];
//		if (pos.x > 1.0f - m_params.particleRadius) { pos.x = 1.0f - m_params.particleRadius; vel.x *= m_params.boundaryDamping; }
//		if (pos.x < -1.0f + m_params.particleRadius) { pos.x = -1.0f + m_params.particleRadius; vel.x *= m_params.boundaryDamping;}
//		if (pos.y > 1.0f - m_params.particleRadius) { pos.y = 1.0f - m_params.particleRadius; vel.y *= m_params.boundaryDamping; }
//		if (pos.y < -1.0f + m_params.particleRadius) { pos.y = -1.0f + m_params.particleRadius; vel.y *= m_params.boundaryDamping;}
//		if (pos.z > 1.0f - m_params.particleRadius) { pos.z = 1.0f - m_params.particleRadius; vel.z *= m_params.boundaryDamping; }
//		if (pos.z < -1.0f + m_params.particleRadius) { pos.z = -1.0f + m_params.particleRadius; vel.z *= m_params.boundaryDamping;}
		btTransform& trans = m_bulletParticles[i]->getWorldTransform();
		trans.setOrigin(btVector3(pos.x, pos.y, pos.z));
		m_bulletParticles[i]->setLinearVelocity(btVector3(vel.x, vel.y, vel.z)*btScalar(VEL_DIR_FACT));
		m_bulletParticles[i]->setAngularVelocity(btVector3(0,0,0));
	}

	glUnmapBufferARB(GL_ARRAY_BUFFER);

	std::swap(m_currentPosRead, m_currentPosWrite);
    std::swap(m_currentVelRead, m_currentVelWrite);

	btTransform& collTrans = m_bulletCollider->getWorldTransform();
	collTrans.setOrigin(btVector3(m_params.colliderPos.x, m_params.colliderPos.y, m_params.colliderPos.z));

	m_dynamicsWorld->stepSimulation(deltaTime);

	glBindBufferARB(GL_ARRAY_BUFFER, m_posVbo[m_currentPosRead]);
	hPos = (float *) glMapBufferARB(GL_ARRAY_BUFFER, GL_READ_WRITE);//GL_WRITE_ONLY);

	//sync transform and velocity from Bullet to particle system
	for (uint i=0;i<m_params.numBodies;i++)
	{
		btTransform& trans = m_bulletParticles[i]->getWorldTransform();
		hPos[i*4] = trans.getOrigin().getX();
		hPos[i*4+1] = trans.getOrigin().getY();
		hPos[i*4+2] = trans.getOrigin().getZ();
		hVel[i*4] = m_bulletParticles[i]->getLinearVelocity().getX() * VEL_INV_FACT;
		hVel[i*4+1] = m_bulletParticles[i]->getLinearVelocity().getY() * VEL_INV_FACT;
		hVel[i*4+2] = m_bulletParticles[i]->getLinearVelocity().getZ() * VEL_INV_FACT;
	}
	copyBuffersFromHostToDevice();

	collTrans = m_bulletCollider->getWorldTransform();
	m_params.colliderPos.x = collTrans.getOrigin().getX();
	m_params.colliderPos.y = collTrans.getOrigin().getY();
	m_params.colliderPos.z = collTrans.getOrigin().getZ();
	
}



void 
ParticleSystem::updateCuda(float deltaTime)
{
#ifndef BT_NO_PROFILE
	CProfileManager::Reset();
#endif //BT_NO_PROFILE
	BT_PROFILE("update CUDA");
    // update constants
    setParameters(&m_params);

    // integrate
	{
		BT_PROFILE("integrate");
		integrateSystem(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
						m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite], 
						deltaTime,
						m_numParticles);
	}
    std::swap(m_currentPosRead, m_currentPosWrite);
    std::swap(m_currentVelRead, m_currentVelWrite);

#if USE_SORT
    // sort and search method

    // calculate hash
	{
		BT_PROFILE("calcHash");
		calcHash(m_posVbo[m_currentPosRead],
				 m_dParticleHash[0],
				 m_numParticles);
	}

#if DEBUG_GRID
    copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
    printf("particle hash:\n");
    for(uint i=0; i<m_numParticles; i++) {
        printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
    }
#endif

    // sort particles based on hash
	{
		BT_PROFILE("RadixSort");
		RadixSort((KeyValuePair *) m_dParticleHash[0], (KeyValuePair *) m_dParticleHash[1], m_numParticles, 32);
	}

#if DEBUG_GRID
    copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
    printf("particle hash sorted:\n");
    for(uint i=0; i<m_numParticles; i++) {
        printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
    }
#endif

	// reorder particle arrays into sorted order and
	// find start of each cell
	{
		BT_PROFILE("reorder");
		reorderDataAndFindCellStart(m_dParticleHash[0],
									m_posVbo[m_currentPosRead],
									m_dVel[m_currentVelRead],
									m_dSortedPos,
									m_dSortedVel,
									m_dCellStart,
									m_numParticles,
									m_numGridCells);
	}
#if DEBUG_GRID
    copyArrayFromDevice((void *) m_hCellStart, (void *) m_dCellStart, 0, sizeof(uint)*m_numGridCells);
    printf("cell start:\n");
    for(uint i=0; i<m_numGridCells; i++) {
        printf("%d: %d\n", i, m_hCellStart[i]);
    }
#endif

#else
    // update grid using atomics
    updateGrid(m_posVbo[m_currentPosRead],
               m_dGridCounters,
               m_dGridCells,
               m_numParticles,
               m_numGridCells);
#endif

    // process collisions
	{
		BT_PROFILE("collide");
		for(uint i=0; i<m_solverIterations; i++) {
			collide(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
					m_dSortedPos, m_dSortedVel,
					m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite],
					m_dGridCounters,
					m_dGridCells,
					m_dParticleHash[0],
					m_dCellStart,
					m_numParticles,
					m_numGridCells,
					m_maxParticlesPerCell
					);
			std::swap(m_currentVelRead, m_currentVelWrite);
		}
    }
#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE
}

void
ParticleSystem::dumpGrid()
{
    // debug
    copyArrayFromDevice(m_hGridCounters, m_dGridCounters, 0, sizeof(uint)*m_numGridCells);
    copyArrayFromDevice(m_hGridCells, m_dGridCells, 0, sizeof(uint)*m_numGridCells*m_maxParticlesPerCell);
    uint total = 0;
    uint maxPerCell = 0;
    for(uint i=0; i<m_numGridCells; i++) {
        if (m_hGridCounters[i] > maxPerCell)
            maxPerCell = m_hGridCounters[i];
        if (m_hGridCounters[i] > 0) {
            printf("%d (%d): ", i, m_hGridCounters[i]);
            for(uint j=0; j<m_hGridCounters[i]; j++) {
                printf("%d ", m_hGridCells[i*m_maxParticlesPerCell + j]);
            }
            total += m_hGridCounters[i];
            printf("\n");
        }
    }
    printf("max per cell = %d\n", maxPerCell);
    printf("total = %d\n", total);
}

void
ParticleSystem::dumpParticles(uint start, uint count)
{
    // debug
    copyArrayFromDevice(m_hPos, 0, m_posVbo[m_currentPosRead], sizeof(float)*4*count);
    copyArrayFromDevice(m_hVel, m_dVel[m_currentVelRead], 0, sizeof(float)*4*count);

    for(uint i=start; i<start+count; i++) {
//        printf("%d: ", i);
        printf("pos: (%.4f, %.4f, %.4f, %.4f)\n", m_hPos[i*4+0], m_hPos[i*4+1], m_hPos[i*4+2], m_hPos[i*4+3]);
        printf("vel: (%.4f, %.4f, %.4f, %.4f)\n", m_hVel[i*4+0], m_hVel[i*4+1], m_hVel[i*4+2], m_hVel[i*4+3]);
    }
}

float* 
ParticleSystem::getArray(ParticleArray array)
{
    assert(m_bInitialized);
 
    float* hdata = 0;
    float* ddata = 0;

    unsigned int vbo = 0;

    switch (array)
    {
    default:
    case POSITION:
        hdata = m_hPos;
        ddata = m_dPos[m_currentPosRead];
        vbo = m_posVbo[m_currentPosRead];
        break;
    case VELOCITY:
        hdata = m_hVel;
        ddata = m_dVel[m_currentVelRead];
        break;
    }

    copyArrayFromDevice(hdata, ddata, vbo, m_numParticles*4*sizeof(float));
    return hdata;
}

void
ParticleSystem::setArray(ParticleArray array, const float* data, int start, int count)
{
    assert(m_bInitialized);
 
    switch (array)
    {
    default:
    case POSITION:
        {
            unregisterGLBufferObject(m_posVbo[m_currentPosRead]);
            glBindBuffer(GL_ARRAY_BUFFER, m_posVbo[m_currentPosRead]);
            glBufferSubData(GL_ARRAY_BUFFER, start*4*sizeof(float), count*4*sizeof(float), data);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            registerGLBufferObject(m_posVbo[m_currentPosRead]);
        }
        break;
    case VELOCITY:
        copyArrayToDevice(m_dVel[m_currentVelRead], data, start*4*sizeof(float), count*4*sizeof(float));
        break;
    }       
}

inline float frand()
{
    return rand() / (float) RAND_MAX;
}

void
ParticleSystem::initGrid(uint *size, float spacing, float jitter, uint numParticles)
{
    srand(1973);
    for(uint z=0; z<size[2]; z++) {
        for(uint y=0; y<size[1]; y++) {
            for(uint x=0; x<size[0]; x++) {
                uint i = (z*size[1]*size[0]) + (y*size[0]) + x;
                if (i < numParticles) {
                    m_hPos[i*4] = (spacing * x) + m_params.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+1] = (spacing * y) + m_params.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+2] = (spacing * z) + m_params.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+3] = 1.0f;

				    m_hVel[i*4] = 0.0f;
				    m_hVel[i*4+1] = 0.0f;
				    m_hVel[i*4+2] = 0.0f;
				    m_hVel[i*4+3] = 0.0f;
                }
            }
        }
    }
}

void
ParticleSystem::reset(ParticleConfig config)
{
	switch(config)
	{
	default:
	case CONFIG_RANDOM:
		{
			int p = 0, v = 0;
			for(uint i=0; i < m_numParticles; i++) 
			{
				float point[3];
				point[0] = frand();
				point[1] = frand();
				point[2] = frand();
				m_hPos[p++] = 2 * (point[0] - 0.5f);
				m_hPos[p++] = 2 * (point[1] - 0.5f);
				m_hPos[p++] = 2 * (point[2] - 0.5f);
				m_hPos[p++] = 1.0f; // radius
				m_hVel[v++] = 0.0f;
				m_hVel[v++] = 0.0f;
				m_hVel[v++] = 0.0f;
				m_hVel[v++] = 0.0f;
			}
		}
		break;

    case CONFIG_GRID:
        {
            float jitter = m_params.particleRadius*0.01f;
            uint s = (int) ceilf(powf((float) m_numParticles, 1.0f / 3.0f));
            uint gridSize[3];
            gridSize[0] = gridSize[1] = gridSize[2] = s;
            initGrid(gridSize, m_params.particleRadius*2.0f, jitter, m_numParticles);
        }
        break;
	}

    setArray(POSITION, m_hPos, 0, m_numParticles);
    setArray(VELOCITY, m_hVel, 0, m_numParticles);
}

void
ParticleSystem::addSphere(int start, float *pos, float *vel, int r, float spacing)
{
    uint index = start;
    for(int z=-r; z<=r; z++) {
        for(int y=-r; y<=r; y++) {
            for(int x=-r; x<=r; x++) {
                float dx = x*spacing;
                float dy = y*spacing;
                float dz = z*spacing;
                float l = sqrtf(dx*dx + dy*dy + dz*dz);
                if ((l <= m_params.particleRadius*2.0f*r) && (index < m_numParticles)) {
                    m_hPos[index*4]   = pos[0] + dx;
                    m_hPos[index*4+1] = pos[1] + dy; 
                    m_hPos[index*4+2] = pos[2] + dz;
                    m_hPos[index*4+3] = pos[3];

                    m_hVel[index*4]   = vel[0];
                    m_hVel[index*4+1] = vel[1];
                    m_hVel[index*4+2] = vel[2];
                    m_hVel[index*4+3] = vel[3];
                    index++;
                }
            }
        }
    }

    setArray(POSITION, m_hPos, start, index);
    setArray(VELOCITY, m_hVel, start, index);

}


void	ParticleSystem::initializeBullet()
{
	
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
//	m_broadphase = new btDbvtBroadphase();
//	m_broadphase = new btAxisSweep3(btVector3(-3,-3,-3),btVector3(3,3,3));
	m_broadphase = new btCudaBroadphase(btVector3(-1, -1, -1), btVector3(1, 1, 1), 64, 64, 64, m_params.numBodies, 16, 64, 8, btScalar(1.0f/1.733f));
//	m_broadphase = new bt3DGridBroadphase(btVector3(-1, -1, -1), btVector3(1, 1, 1), 64, 64, 64, m_params.numBodies, 16, 64, 8, btScalar(1.0f/1.733f));


	m_constraintSolver=new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver,m_collisionConfiguration);
	m_dynamicsWorld->setDebugDrawer(&debugDrawer);
	//debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawPairs);


//	m_dynamicsWorld->setGravity(100*btVector3(m_params.gravity.x,m_params.gravity.y,m_params.gravity.z));
	m_dynamicsWorld->setGravity(btScalar(ACC_DIR_FACT) * btVector3(m_params.gravity.x,m_params.gravity.y,m_params.gravity.z));
	m_dynamicsWorld->getSolverInfo().m_numIterations=1;

	btRigidBody* body;

	btCollisionShape* boxShape = new btBoxShape(btVector3(btScalar(1.2),btScalar(0.05),btScalar(1.2)));
//	boxShape->setMargin(0.03f);

	btScalar mass(0.);
	btVector3 localInertia(0,0,0);
	btRigidBody::btRigidBodyConstructionInfo boxRbcInfo(mass, 0, boxShape, localInertia);
	boxRbcInfo.m_startWorldTransform.setIdentity();
	boxRbcInfo.m_startWorldTransform.setOrigin(btVector3(0, -1.05f,0));
	boxRbcInfo.m_friction = 0.0f;
	body = new btRigidBody(boxRbcInfo);
	m_dynamicsWorld->addRigidBody(body);

	boxRbcInfo.m_startWorldTransform.setIdentity();
	boxRbcInfo.m_startWorldTransform.setOrigin(btVector3(0, 1.05f,0));
	boxRbcInfo.m_friction = 0.0f;
	body = new btRigidBody(boxRbcInfo);
	m_dynamicsWorld->addRigidBody(body);

	boxRbcInfo.m_startWorldTransform.setIdentity();
	boxRbcInfo.m_startWorldTransform.getBasis().setEulerZYX(0, 0, SIMD_HALF_PI);
	boxRbcInfo.m_startWorldTransform.setOrigin(btVector3(-1.05f, 0, 0));
	boxRbcInfo.m_friction = 0.0f;
	body = new btRigidBody(boxRbcInfo);
	m_dynamicsWorld->addRigidBody(body);

	boxRbcInfo.m_startWorldTransform.setIdentity();
	boxRbcInfo.m_startWorldTransform.getBasis().setEulerZYX(0, 0, SIMD_HALF_PI);
	boxRbcInfo.m_startWorldTransform.setOrigin(btVector3(1.05f, 0, 0));
	boxRbcInfo.m_friction = 0.0f;
	body = new btRigidBody(boxRbcInfo);
	m_dynamicsWorld->addRigidBody(body);

	boxRbcInfo.m_startWorldTransform.setIdentity();
	boxRbcInfo.m_startWorldTransform.getBasis().setEulerZYX(SIMD_HALF_PI, 0, 0);
	boxRbcInfo.m_startWorldTransform.setOrigin(btVector3(0, 0, -1.05f));
	boxRbcInfo.m_friction = 0.0f;
	body = new btRigidBody(boxRbcInfo);
	m_dynamicsWorld->addRigidBody(body);

	boxRbcInfo.m_startWorldTransform.setIdentity();
	boxRbcInfo.m_startWorldTransform.getBasis().setEulerZYX(SIMD_HALF_PI, 0, 0);
	boxRbcInfo.m_startWorldTransform.setOrigin(btVector3(0, 0, 1.05f));
	boxRbcInfo.m_friction = 0.0f;
	body = new btRigidBody(boxRbcInfo);
	m_dynamicsWorld->addRigidBody(body);



	unsigned int i;

	btSphereShape* particleSphere = new btSphereShape(m_params.particleRadius);
	particleSphere->setMargin(0.0);
	particleSphere->calculateLocalInertia(1,localInertia);

	reset(CONFIG_GRID);

	for (i=0;i<m_params.numBodies;i++)
	{
		btRigidBody::btRigidBodyConstructionInfo rbci(1.,0,particleSphere,localInertia);
		rbci.m_startWorldTransform.setOrigin(btVector3(m_hPos[i*4],m_hPos[i*4+1],m_hPos[i*4+2]));
		body = new btRigidBody(rbci);
		body->setActivationState(DISABLE_DEACTIVATION);
		m_bulletParticles.push_back(body);
		m_dynamicsWorld->addRigidBody(body);
	}

	btSphereShape* colliderSphere = new btSphereShape(m_params.colliderRadius);
	colliderSphere->setMargin(0.0);
	colliderSphere->calculateLocalInertia(10., localInertia);
	btRigidBody::btRigidBodyConstructionInfo rbci(5., 0, colliderSphere,localInertia);
	rbci.m_startWorldTransform.setOrigin(btVector3(m_params.colliderPos.x, m_params.colliderPos.y, m_params.colliderPos.z));
	body = new btRigidBody(rbci);
	body->setActivationState(DISABLE_DEACTIVATION);
	m_bulletCollider = body;
	m_dynamicsWorld->addRigidBody(body);

/*	for (i=0;i<6;i++)
	{
		btVector4	planeEq;
		worldBox->getPlaneEquation(planeEq,i);

		planeShape = new btStaticPlaneShape(-planeEq,planeEq.getW());
		planeShape->setMargin(0.f);
		btRigidBody::btRigidBodyConstructionInfo rbci(0.f,0,planeShape);
		body = new btRigidBody(rbci);
		m_dynamicsWorld->addRigidBody(body);
	}
*/
}

void	ParticleSystem::finalizeBullet()
{
	delete m_dynamicsWorld;
	delete m_constraintSolver;
	delete m_broadphase;
	delete m_dispatcher ;
	delete m_collisionConfiguration;
}

float*	ParticleSystem::copyBuffersFromDeviceToHost()
{
	copyArrayFromDevice(m_hVel, m_dVel[m_currentVelRead], 0, sizeof(float)*4*m_numParticles);
	// fill color buffer
	glBindBufferARB(GL_ARRAY_BUFFER, m_posVbo[m_currentPosRead]);
	float* hPosData = (float *) glMapBufferARB(GL_ARRAY_BUFFER, GL_READ_WRITE);//GL_WRITE_ONLY);
	return hPosData;
}

void	ParticleSystem::copyBuffersFromHostToDevice()
{
		glUnmapBufferARB(GL_ARRAY_BUFFER);
		copyArrayToDevice(m_dVel[m_currentVelRead],m_hVel, 0, sizeof(float)*4*m_numParticles);
}


void	ParticleSystem::debugDraw()
{
#if USE_BULLET
	glDisable(GL_DEPTH_TEST);
	m_dynamicsWorld->debugDrawWorld();
	glEnable(GL_DEPTH_TEST);
#endif
}


