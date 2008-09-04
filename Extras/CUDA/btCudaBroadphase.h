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

#ifndef CUDA_BROADPHASE_H
#define CUDA_BROADPHASE_H

#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"

///The btCudaBroadphase uses CUDA to compute overlapping pairs using a GPU.
class btCudaBroadphase : public btSimpleBroadphase
{

	

	bool m_bInitialized;
    int	m_numParticles;

    // CPU data
    float* m_hPos;
    float* m_hVel;
	float*	m_hSortedPos;

    unsigned int*  m_hGridCounters;
    unsigned int*  m_hGridCells;

    unsigned int*  m_hParticleHash;
    unsigned int*  m_hCellStart;
	

    // GPU data
    float* m_dPos[2];
    float* m_dVel[2];

    float* m_dSortedPos;
    float* m_dSortedVel;

    // uniform grid data
    unsigned int*  m_dGridCounters; // counts number of entries per grid cell
    unsigned int*  m_dGridCells;    // contains indices of up to "m_maxParticlesPerCell" particles per cell

    unsigned int*  m_dParticleHash[2];
    unsigned int*  m_dCellStart;

    unsigned int	m_posVbo[2];
    unsigned int	m_colorVBO;

    unsigned int	m_currentPosRead, m_currentVelRead;
    unsigned int	m_currentPosWrite, m_currentVelWrite;

    // params
	struct SimParams&	m_simParams;
    

    
    unsigned int	m_maxParticlesPerCell;

protected:
	
	unsigned int createVBO(unsigned int size);

	void _initialize(int numParticles);

	void _finalize();



public:

	enum ParticleArray
    {
        POSITION,
        VELOCITY,
    };

	enum ParticleConfig
    {
	    CONFIG_RANDOM,
	    CONFIG_GRID,
	    _NUM_CONFIGS
    };

	btCudaBroadphase(SimParams& simParams,int maxProxies);

	virtual ~btCudaBroadphase();

	void initGrid(unsigned int* size, float spacing, float jitter, unsigned int numParticles);

	void reset(ParticleConfig config);

	void   setArray(ParticleArray array, const float* data, int start, int count);

	float*  getArray(ParticleArray array);

	void addSphere(int start, float *pos, float *vel, int r, float spacing);

	virtual void	calculateOverlappingPairs(btDispatcher* dispatcher);

	unsigned int getCurrentReadBuffer() const { return m_posVbo[m_currentPosRead]; }
    unsigned int getColorBuffer() const { return m_colorVBO; }
	void dumpParticles(unsigned int  start, unsigned int count);
	void	dumpGrid();
	
	float*	copyBuffersFromDeviceToHost();
	void	copyBuffersFromHostToDevice();
	float* getHvelPtr();
	float*	getHposPtr();
	void	quickHack(float deltaTime);
	void	quickHack2();
	void	integrate();

};
#endif //CUDA_BROADPHASE_H