
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "particles_kernel.cuh"
#include "particleSystem.cuh"
#include "radixsort.cuh"
#include "vector_functions.h"
#include <stdio.h>

#ifdef WIN32//for glut.h
#include <windows.h>
#endif

#include <GL/glew.h>
//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif



#define USE_SORT 1

#include "btCudaBroadphase.h"
#include "LinearMath/btAlignedAllocator.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

btCudaBroadphase::btCudaBroadphase(SimParams& simParams,int maxProxies) :
btSimpleBroadphase(maxProxies,
				     new (btAlignedAlloc(sizeof(btSortedOverlappingPairCache),16)) btSortedOverlappingPairCache),
	 m_bInitialized(false),
	m_numParticles(simParams.numBodies),
    m_hPos(0),
    m_hVel(0),
    m_currentPosRead(0),
    m_currentVelRead(0),
    m_currentPosWrite(1),
    m_currentVelWrite(1),
    m_maxParticlesPerCell(4),
    m_simParams(simParams)
{
	m_ownsPairCache = true;

	m_dPos[0] = m_dPos[1] = 0;
    m_dVel[0] = m_dVel[1] = 0;

	m_simParams.gridSize.x = 64;
	m_simParams.gridSize.y = 64;
	m_simParams.gridSize.z = 64;


    m_simParams.numCells = m_simParams.gridSize.x*m_simParams.gridSize.y*m_simParams.gridSize.z;
	m_simParams.worldSize = make_float3(2.0f, 2.0f, 2.0f);

    // set simulation parameters
    
    m_simParams.numBodies = m_numParticles;
    m_simParams.maxParticlesPerCell = m_maxParticlesPerCell;

    m_simParams.worldOrigin = make_float3(-1.0f, -1.0f, -1.0f);
    m_simParams.cellSize = make_float3(m_simParams.worldSize.x / m_simParams.gridSize.x, m_simParams.worldSize.y / m_simParams.gridSize.y, m_simParams.worldSize.z / m_simParams.gridSize.z);

    m_simParams.particleRadius = m_simParams.cellSize.x * 0.5f;
    m_simParams.colliderPos = make_float4(-1.2f, -0.8f, 0.8f, 1.0f);
    m_simParams.colliderRadius = 0.2f;

    m_simParams.spring = 0.5f;
    m_simParams.damping = 0.02f;
    m_simParams.shear = 0.1f;
    m_simParams.attraction = 0.0f;
    m_simParams.boundaryDamping = -0.5f;

    m_simParams.gravity = make_float3(0.0f, -0.0003f, 0.0f);
    m_simParams.globalDamping = 1.0f;

    _initialize(m_numParticles);

}

static inline float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}

static void colorRamp(float t, float *r)
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


unsigned int btCudaBroadphase::createVBO(unsigned int size)
{
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    registerGLBufferObject(vbo);
    return vbo;
}


void btCudaBroadphase::_initialize(int numParticles)
{
    assert(!m_bInitialized);

    // allocate host storage
    m_hPos = new float[numParticles*4];
    m_hVel = new float[numParticles*4];
	m_hSortedPos = new float[numParticles*4];
    memset(m_hPos, 0, numParticles*4*sizeof(float));
    memset(m_hVel, 0, numParticles*4*sizeof(float));
	memset(m_hSortedPos, 0, numParticles*4*sizeof(float));

    m_hGridCounters = new uint[m_simParams.numCells];
    m_hGridCells = new uint[m_simParams.numCells*m_maxParticlesPerCell];
    memset(m_hGridCounters, 0, m_simParams.numCells*sizeof(uint));
    memset(m_hGridCells, 0, m_simParams.numCells*m_maxParticlesPerCell*sizeof(uint));

    m_hParticleHash = new uint[numParticles*2];
    memset(m_hParticleHash, 0, numParticles*2*sizeof(uint));

    m_hCellStart = new uint[m_simParams.numCells];
    memset(m_hCellStart, 0, m_simParams.numCells*sizeof(uint));

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
    allocateArray((void**)&m_dCellStart, m_simParams.numCells*sizeof(uint));
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


    setParameters(&m_simParams);

    m_bInitialized = true;
}



void btCudaBroadphase::_finalize()
{
    assert(m_bInitialized);

    delete [] m_hPos;
    delete [] m_hVel;
	delete [] m_hSortedPos;

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

btCudaBroadphase::~btCudaBroadphase()
{
	//btSimpleBroadphase will free memory of btSortedOverlappingPairCache, because m_ownsPairCache
	assert(m_bInitialized);

  _finalize();

}

/*
int btCudaBroadphase::myCollideCell2(int3   gridPos,
                   uint    index,
                   unsigned int* particleHash,
                   unsigned int* cellStart)
{
    int numOverlap = 0;
	

    if ((gridPos.x < 0) || (gridPos.x > params.gridSize.x-1) ||
        (gridPos.y < 0) || (gridPos.y > params.gridSize.y-1) ||
        (gridPos.z < 0) || (gridPos.z > params.gridSize.z-1)) {
        return force;
    }

    uint gridHash = calcGridHash(gridPos);

    // get start of bucket for this cell
    uint bucketStart = FETCH(cellStart, gridHash);
    if (bucketStart == 0xffffffff)
        return force;   // cell empty
 
    // iterate over particles in this cell
    for(uint i=0; i<params.maxParticlesPerCell; i++) {
        uint index2 = bucketStart + i;
        uint2 cellData = FETCH(particleHash, index2);
        if (cellData.x != gridHash) break;   // no longer in same bucket

        if (index2 != index) {              // check not colliding with self
	        float4 pos2 = FETCH(oldPos, index2);
            float4 vel2 = FETCH(oldVel, index2);

            // collide two spheres
            float3 projVec = collideSpheres(pos, pos2, vel, vel2, params.particleRadius, params.particleRadius, params.attraction);
            force += projVec;
        }
    }

    return force;
}
*/




void	btCudaBroadphase::calculateOverlappingPairs(btDispatcher* dispatcher)
{
	//first check for new overlapping pairs
	int j;
	static int frameCount = 0;
	//printf("framecount=%d\n",frameCount++);


	int numRejected=0;

	if (m_numHandles >= 0)
	{

//#define _USE_BRUTEFORCE_N 1
#ifdef _USE_BRUTEFORCE_N

		int i;
		for (i=0;i<m_numHandles;i++)
		{
			btSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];

			for (j=i+1;j<m_numHandles;j++)
			{
				btSimpleBroadphaseProxy* proxy1 = &m_pHandles[i];
				
				if (proxy0 != proxy1)
				{
					btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
					btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);

					if (aabbOverlap(p0,p1))
					{
						if ( !m_pairCache->findPair(proxy0,proxy1))
						{
							m_pairCache->addOverlappingPair(proxy0,proxy1);
						}
					} else
					{
					if (!m_pairCache->hasDeferredRemoval())
					{
						if ( m_pairCache->findPair(proxy0,proxy1))
						{
							m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
						}
					}

					}
				}
				proxy1 = &m_pHandles[proxy1->GetNextAllocated()];

			}
			proxy0 = &m_pHandles[proxy0->GetNextAllocated()];

		}
#else //_USE_BRUTEFORCE_N

			// update constants
		setParameters(&m_simParams);

		float deltaTime = 1./60.f;

		/*
		
				// integrate
				integrateSystem(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
								m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite], 
								deltaTime,
								m_numParticles);




				btSwap(m_currentPosRead, m_currentPosWrite);
				btSwap(m_currentVelRead, m_currentVelWrite);
*/

#if USE_SORT
				// sort and search method

				// calculate hash
				calcHash(m_posVbo[m_currentPosRead],
						 m_dParticleHash[0],
						 m_numParticles);

#if DEBUG_GRID
				copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
				printf("particle hash:\n");
				for(uint i=0; i<m_numParticles; i++) {
					printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
				}
#endif

			

				// sort particles based on hash
				RadixSort((KeyValuePair *) m_dParticleHash[0], (KeyValuePair *) m_dParticleHash[1], m_numParticles, 32);

#if DEBUG_GRID
				copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
				printf("particle hash sorted:\n");
				for(uint i=0; i<m_numParticles; i++) {
					printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
				}
#endif

			
				// reorder particle arrays into sorted order and
				// find start of each cell
				reorderDataAndFindCellStart(m_dParticleHash[0],
											m_posVbo[m_currentPosRead],
											m_dVel[m_currentVelRead],
											m_dSortedPos,
											m_dSortedVel,
											m_dCellStart,
											m_numParticles,
											m_simParams.numCells);

//#define DEBUG_GRID2
#ifdef DEBUG_GRID2
				copyArrayFromDevice((void *) m_hCellStart, (void *) m_dCellStart, 0, sizeof(uint)*m_simParams.numCells);
				printf("cell start:\n");
				for(uint i=0; i<16; i++) {
					printf("%d: %d//", i, m_hCellStart[i]);
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

				/*
				dsadsa
*/

				
/*
				int m_solverIterations = 1;

				// process collisions
				for(uint i=0; i<m_solverIterations; i++) {
					collide(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
							m_dSortedPos, m_dSortedVel,
							m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite],
							m_dGridCounters,
							m_dGridCells,
							m_dParticleHash[0],
							m_dCellStart,
							m_numParticles,
							m_simParams.numCells,
							m_maxParticlesPerCell
							);

					btSwap(m_currentVelRead, m_currentVelWrite);
					
				}
*/

				copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
				copyArrayFromDevice((void *) m_hCellStart, (void *) m_dCellStart, 0, sizeof(uint)*m_simParams.numCells);
				copyArrayFromDevice((void *) m_hSortedPos, (void*) m_dSortedPos,0 , sizeof(float)*4*m_numParticles);
	
//#define DEBUG_INDICES 1
#ifdef DEBUG_INDICES
				{
					printf("cell start:\n");
					for(uint i=0; i<16; i++) {
						printf("%d: %d\n", i, m_hCellStart[i]);
					}
				}
				{
					printf("particle hash sorted:\n");
					for(uint i=0; i<m_numParticles; i++) {
						printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
					}
				}
#endif //DEBUG_INDICES

				{
//					printf("cell start:\n");
//					for(uint i=0; i<m_simParams.numCells; i++) {
//						printf("%d: %d\n", i, m_hCellStart[i]);
//					}
				}

				//printf("particle hash sorted:\n");
				for(uint pi=0; pi<m_numParticles; pi++) 
				{
					int index = m_hParticleHash[pi*2+1];
		
					//printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
					//perform an AABB check?
					   // examine only neighbouring cells

					

					btSimpleBroadphaseProxy* proxy0 = &m_pHandles[index];
					btVector3 mypos = (proxy0->m_min+proxy0->m_max)*0.5f;

//					float4* p = (float4*)&m_hSortedPos[index*4];
					

					int3 particleGridPos;
					particleGridPos.x = floor((mypos.x() - m_simParams.worldOrigin.x) / m_simParams.cellSize.x);
					particleGridPos.y = floor((mypos.y() - m_simParams.worldOrigin.y) / m_simParams.cellSize.y);
					particleGridPos.z = floor((mypos.z() - m_simParams.worldOrigin.z) / m_simParams.cellSize.z);

					
					//for(int z=0; z<1; z++) 
					for(int z=-1; z<=1; z++) 
					{
					//	for(int y=0; y<1; y++) 
						for(int y=-1; y<=1; y++) 
						{
					//		for(int x=0; x<1; x++) 
							for(int x=-1; x<=1; x++) 
							{
								int3 gridPos;
								gridPos.x = particleGridPos.x + x;
								gridPos.y = particleGridPos.y + y;
								gridPos.z = particleGridPos.z + z;

								 if ((gridPos.x < 0) || (gridPos.x > m_simParams.gridSize.x-1) ||
									(gridPos.y < 0) || (gridPos.y > m_simParams.gridSize.y-1) ||
									(gridPos.z < 0) || (gridPos.z > m_simParams.gridSize.z-1)) 
								 {
									continue;
								 }


								gridPos.x = max(0, min(gridPos.x, m_simParams.gridSize.x-1));
								gridPos.y = max(0, min(gridPos.y, m_simParams.gridSize.y-1));
								gridPos.z = max(0, min(gridPos.z, m_simParams.gridSize.z-1));
								uint gridHash = ((gridPos.z*m_simParams.gridSize.y)* m_simParams.gridSize.x) + (gridPos.y* m_simParams.gridSize.x) + gridPos.x;

								// get start of bucket for this cell
								unsigned int bucketStart = m_hCellStart[gridHash];
								if (bucketStart == 0xffffffff)
									continue;
								 
								// iterate over particles in this cell
								for(uint q=0; q<m_simParams.maxParticlesPerCell; q++) 
								{
									///add overlap with planes


									uint cellIndex2 = bucketStart + q;
									int cellData = m_hParticleHash[cellIndex2*2];
									if (cellData != gridHash) 
										break;   // no longer in same bucket

									int particleIndex2 = m_hParticleHash[cellIndex2*2+1];
									if (particleIndex2!= index && particleIndex2<index) 
									{              // check not colliding with self
										//add an overlapping pair
										//printf("add pair (%d,%d)\n",particleIndex2,index);
										btSimpleBroadphaseProxy* proxy1 = &m_pHandles[particleIndex2];
										
										//do a more exact AABB overlap test before adding the pair
										bool hasOverlap = testAabbOverlap(proxy0,proxy1);
										if (hasOverlap)
											m_pairCache->addOverlappingPair(proxy0,proxy1);
										else
										{
											numRejected++;
										}

									}
								}
								


								//int numOverlap += myCollideCell2(gridPos + make_int3(x, y, z), index, pos, vel, oldPos, oldVel, particleHash, cellStart);
							}
						}
					}
				}




#endif //_USE_BRUTEFORCE_N

		///if this broadphase is used in a btMultiSapBroadphase, we shouldn't sort the overlapping paircache
		if (m_ownsPairCache && m_pairCache->hasDeferredRemoval())
		{
			
			btBroadphasePairArray&	overlappingPairArray = m_pairCache->getOverlappingPairArray();

			//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
			//overlappingPairArray.quickSort(btBroadphasePairSortPredicate());
			overlappingPairArray.heapSort(btBroadphasePairSortPredicate());
			//printf("A) overlappingPairArray.size()=%d\n",overlappingPairArray.size());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;


			btBroadphasePair previousPair;
			previousPair.m_pProxy0 = 0;
			previousPair.m_pProxy1 = 0;
			previousPair.m_algorithm = 0;
			
			
			int i;
			for (i=0;i<overlappingPairArray.size();i++)
			{
			
				btBroadphasePair& pair = overlappingPairArray[i];

				bool isDuplicate = (pair == previousPair);

				previousPair = pair;

				bool needsRemoval = false;

				if (!isDuplicate)
				{
					bool hasOverlap = testAabbOverlap(pair.m_pProxy0,pair.m_pProxy1);

					if (hasOverlap)
					{
						needsRemoval = false;//callback->processOverlap(pair);
					} else
					{
						needsRemoval = true;
					}
				} else
				{
					//remove duplicate
					needsRemoval = true;
					//should have no algorithm
					btAssert(!pair.m_algorithm);
				}
				
				if (needsRemoval)
				{
					m_pairCache->cleanOverlappingPair(pair,dispatcher);

			//		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
			//		m_overlappingPairArray.pop_back();
					pair.m_pProxy0 = 0;
					pair.m_pProxy1 = 0;
					m_invalidPair++;

				} 
				
			}

		///if you don't like to skip the invalid pairs in the array, execute following code:
		#define CLEAN_INVALID_PAIRS 1
		#ifdef CLEAN_INVALID_PAIRS

			//perform a sort, to sort 'invalid' pairs to the end
			//overlappingPairArray.quickSort(btBroadphasePairSortPredicate());
			overlappingPairArray.heapSort(btBroadphasePairSortPredicate());
			//printf("B) overlappingPairArray.size()=%d\n",overlappingPairArray.size());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
//			printf("C) overlappingPairArray.size()=%d\n",overlappingPairArray.size());
			m_invalidPair = 0;
		#endif//CLEAN_INVALID_PAIRS

		}
	}

	//printf("numRejected=%d\n",numRejected);
}

static inline float frand()
{
    return rand() / (float) RAND_MAX;
}


void btCudaBroadphase::initGrid(unsigned int* size, float spacing, float jitter, unsigned int numParticles)
{
    srand(1973);
#ifdef CONTROLLED_START
	float extra=0.01f;
    for(uint z=0; z<size[2]; z++) {
        for(uint y=0; y<size[1]; y++) {
            for(uint x=0; x<size[0]; x++) {
                uint i = (z*size[1]*size[0]) + (y*size[0]) + x;
                if (i < numParticles) {
                    m_hPos[i*4] = (spacing * x) + m_simParams.particleRadius - 1.0f+extra;//+ (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+1] = (spacing * y) + m_simParams.particleRadius - 1.0f;//+ (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+2] = (spacing * z) + m_simParams.particleRadius - 1.0f;//+ (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+3] = 1.0f;
					extra=0.f;

				    m_hVel[i*4] = 0.0f;
				    m_hVel[i*4+1] = 0.0f;
				    m_hVel[i*4+2] = 0.0f;
				    m_hVel[i*4+3] = 0.0f;
                }
            }
			extra=0.f;
        }
    }
#else
	for(uint z=0; z<size[2]; z++) {
        for(uint y=0; y<size[1]; y++) {
            for(uint x=0; x<size[0]; x++) {
                uint i = (z*size[1]*size[0]) + (y*size[0]) + x;
                if (i < numParticles) {
                    m_hPos[i*4] = (spacing * x) + m_simParams.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+1] = (spacing * y) + m_simParams.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+2] = (spacing * z) + m_simParams.particleRadius - 1.0f + (frand()*2.0f-1.0f)*jitter;
                    m_hPos[i*4+3] = 1.0f;

				    m_hVel[i*4] = 0.0f;
				    m_hVel[i*4+1] = 0.0f;
				    m_hVel[i*4+2] = 0.0f;
				    m_hVel[i*4+3] = 0.0f;
                }
            }
        }
    }
#endif

}



void btCudaBroadphase::reset(ParticleConfig config)
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
            float jitter = m_simParams.particleRadius*0.01f;
            uint s = (int) ceilf(powf((float) m_numParticles, 1.0f / 3.0f));
            uint gridSize[3];
            gridSize[0] = gridSize[1] = gridSize[2] = s;
            initGrid(gridSize, m_simParams.particleRadius*2.0f, jitter, m_numParticles);
        }
        break;
	}

    setArray(POSITION, m_hPos, 0, m_numParticles);
    setArray(VELOCITY, m_hVel, 0, m_numParticles);

}



void btCudaBroadphase::addSphere(int start, float *pos, float *vel, int r, float spacing)
{
    uint index = start;
    for(int z=-r; z<=r; z++) {
        for(int y=-r; y<=r; y++) {
            for(int x=-r; x<=r; x++) {
                float dx = x*spacing;
                float dy = y*spacing;
                float dz = z*spacing;
                float l = sqrtf(dx*dx + dy*dy + dz*dz);
                if ((l <= m_simParams.particleRadius*2.0f*r) && (index < m_numParticles)) {
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


void btCudaBroadphase::setArray(ParticleArray array, const float* data, int start, int count)
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


float*  btCudaBroadphase::getArray(ParticleArray array)
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

void btCudaBroadphase::dumpGrid()
{
    // debug
    copyArrayFromDevice(m_hGridCounters, m_dGridCounters, 0, sizeof(uint)*m_simParams.numCells);
    copyArrayFromDevice(m_hGridCells, m_dGridCells, 0, sizeof(uint)*m_simParams.numCells*m_maxParticlesPerCell);
    uint total = 0;
    uint maxPerCell = 0;
    for(uint i=0; i<m_simParams.numCells; i++) {
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

void btCudaBroadphase::dumpParticles(unsigned int  start, unsigned int count)
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

float*	btCudaBroadphase::copyBuffersFromDeviceToHost()
{
	//	copyArrayFromDevice(m_hPos, 0, m_posVbo[m_currentPosRead], sizeof(float)*4*m_numParticles);
		copyArrayFromDevice(m_hVel, m_dVel[m_currentVelRead], 0, sizeof(float)*4*m_numParticles);
		// fill color buffer
		glBindBufferARB(GL_ARRAY_BUFFER, m_posVbo[m_currentPosRead]);
		float* hPosData = (float *) glMapBufferARB(GL_ARRAY_BUFFER, GL_READ_WRITE);//GL_WRITE_ONLY);
		return hPosData;
}

void	btCudaBroadphase::copyBuffersFromHostToDevice()
{
		glUnmapBufferARB(GL_ARRAY_BUFFER);
		copyArrayToDevice(m_dVel[m_currentVelRead],m_hVel, 0, sizeof(float)*4*m_numParticles);
}

float* btCudaBroadphase::getHvelPtr()
{
	return m_hVel;
}

float*	btCudaBroadphase::getHposPtr()
{
	return m_hPos;
}

void	btCudaBroadphase::quickHack(float deltaTime)
{
		// update constants
		setParameters(&m_simParams);

	


				// integrate
				integrateSystem(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
								m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite], 
								deltaTime,
								m_numParticles);




				btSwap(m_currentPosRead, m_currentPosWrite);
				btSwap(m_currentVelRead, m_currentVelWrite);

#if USE_SORT
				// sort and search method

				// calculate hash
				calcHash(m_posVbo[m_currentPosRead],
						 m_dParticleHash[0],
						 m_numParticles);

#if DEBUG_GRID
				copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
				printf("particle hash:\n");
				for(uint i=0; i<m_numParticles; i++) {
					printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
				}
#endif

				// sort particles based on hash
				RadixSort((KeyValuePair *) m_dParticleHash[0], (KeyValuePair *) m_dParticleHash[1], m_numParticles, 32);

#if DEBUG_GRID
				copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
				printf("particle hash sorted:\n");
				for(uint i=0; i<m_numParticles; i++) {
					printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
				}
#endif

				// reorder particle arrays into sorted order and
				// find start of each cell
				reorderDataAndFindCellStart(m_dParticleHash[0],
											m_posVbo[m_currentPosRead],
											m_dVel[m_currentVelRead],
											m_dSortedPos,
											m_dSortedVel,
											m_dCellStart,
											m_numParticles,
											m_simParams.numCells);

//#define DEBUG_GRID2
#ifdef DEBUG_GRID2
				copyArrayFromDevice((void *) m_hCellStart, (void *) m_dCellStart, 0, sizeof(uint)*m_simParams.numCells);
				printf("cell start:\n");
				for(uint i=0; i<m_simParams.numCells; i++) {
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

				/*
				dsadsa
*/

				

				int m_solverIterations = 1;

				// process collisions
				for(uint i=0; i<m_solverIterations; i++) {
					collide(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
							m_dSortedPos, m_dSortedVel,
							m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite],
							m_dGridCounters,
							m_dGridCells,
							m_dParticleHash[0],
							m_dCellStart,
							m_numParticles,
							m_simParams.numCells,
							m_maxParticlesPerCell
							);

					btSwap(m_currentVelRead, m_currentVelWrite);
					
				}
				

}

void	btCudaBroadphase::integrate()
{
			// update constants
		setParameters(&m_simParams);

		float deltaTime = 1./60.f;


				// integrate
				integrateSystem(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
								m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite], 
								deltaTime,
								m_numParticles);

				btSwap(m_currentPosRead, m_currentPosWrite);
				btSwap(m_currentVelRead, m_currentVelWrite);
}

void	btCudaBroadphase::quickHack2()
{
		// update constants
		setParameters(&m_simParams);

	
				// integrate
				integrateSystem(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
								m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite], 
								0.f,
								m_numParticles);





				btSwap(m_currentPosRead, m_currentPosWrite);
				btSwap(m_currentVelRead, m_currentVelWrite);

#if USE_SORT
				// sort and search method

				// calculate hash
				calcHash(m_posVbo[m_currentPosRead],
						 m_dParticleHash[0],
						 m_numParticles);

#if DEBUG_GRID
				copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
				printf("particle hash:\n");
				for(uint i=0; i<m_numParticles; i++) {
					printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
				}
#endif

				// sort particles based on hash
				RadixSort((KeyValuePair *) m_dParticleHash[0], (KeyValuePair *) m_dParticleHash[1], m_numParticles, 32);

#if DEBUG_GRID
				copyArrayFromDevice((void *) m_hParticleHash, (void *) m_dParticleHash[0], 0, sizeof(uint)*2*m_numParticles);
				printf("particle hash sorted:\n");
				for(uint i=0; i<m_numParticles; i++) {
					printf("%d: %d, %d\n", i, m_hParticleHash[i*2], m_hParticleHash[i*2+1]);
				}
#endif

				// reorder particle arrays into sorted order and
				// find start of each cell
				reorderDataAndFindCellStart(m_dParticleHash[0],
											m_posVbo[m_currentPosRead],
											m_dVel[m_currentVelRead],
											m_dSortedPos,
											m_dSortedVel,
											m_dCellStart,
											m_numParticles,
											m_simParams.numCells);

//#define DEBUG_GRID2
#ifdef DEBUG_GRID2
				copyArrayFromDevice((void *) m_hCellStart, (void *) m_dCellStart, 0, sizeof(uint)*m_simParams.numCells);
				printf("cell start:\n");
				for(uint i=0; i<m_simParams.numCells; i++) {
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

				/*
				dsadsa
*/

				
/*
				int m_solverIterations = 1;

				// process collisions
				for(uint i=0; i<m_solverIterations; i++) {
					collide(m_posVbo[m_currentPosRead], m_posVbo[m_currentPosWrite],
							m_dSortedPos, m_dSortedVel,
							m_dVel[m_currentVelRead], m_dVel[m_currentVelWrite],
							m_dGridCounters,
							m_dGridCells,
							m_dParticleHash[0],
							m_dCellStart,
							m_numParticles,
							m_simParams.numCells,
							m_maxParticlesPerCell
							);

					btSwap(m_currentVelRead, m_currentVelWrite);
					
				}
				*/

				

}
