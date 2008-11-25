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

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  Keep this file free from Bullet headers
//  it is included into CUDA program
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#ifndef CUDA_BROADPHASE_KERNEL_H
#define CUDA_BROADPHASE_KERNEL_H

#define CUDA_BROADPHASE_USE_CUDA 1

#define BT_CUDA_PAIR_FOUND_FLG (0x40000000)
#define BT_CUDA_PAIR_NEW_FLG   (0x20000000)
#define BT_CUDA_PAIR_ANY_FLG   (BT_CUDA_PAIR_FOUND_FLG | BT_CUDA_PAIR_NEW_FLG)

struct btCudaBroadphaseParams 
{
	unsigned int	m_gridSizeX;
	unsigned int	m_gridSizeY;
	unsigned int	m_gridSizeZ;
	unsigned int	m_numCells;
	float			m_worldOriginX;
	float			m_worldOriginY;
	float			m_worldOriginZ;
	float			m_cellSizeX;
	float			m_cellSizeY;
	float			m_cellSizeZ;
	unsigned int	m_numBodies;
	unsigned int	m_maxBodiesPerCell;
};

struct btCuda3F1U
{
	float			fx;
	float			fy;
	float			fz;
	unsigned int	uw;
};


extern "C"
{
// CPU functions
	void bt3DGrid_setParameters(btCudaBroadphaseParams* hostParams);
	void bt3DGrid_calcHashAABB(btCuda3F1U* pAABB, unsigned int* hash,	unsigned int numBodies);
	void bt3DGrid_findCellStart(unsigned int* hash, unsigned int* cellStart, unsigned int numBodies, unsigned int numCells);
	void bt3DGrid_findOverlappingPairs(	btCuda3F1U*	pAABB, unsigned int* pHash,
										unsigned int*	pCellStart,
										unsigned int*	pPairBuff,
										unsigned int*	pPairBuffStartCurr,
										unsigned int	numBodies);
	void bt3DGrid_findPairsLarge(	btCuda3F1U*	pAABB, unsigned int* pHash,
									unsigned int*	pCellStart,
									unsigned int*	pPairBuff,
									unsigned int*	pPairBuffStartCurr,
									unsigned int	numBodies,
									unsigned int	numLarge);

	void bt3DGrid_computePairCacheChanges(	unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, 
											unsigned int* pPairScan, btCuda3F1U* pAABB, unsigned int numBodies);
	void bt3DGrid_squeezeOverlappingPairBuff(	unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, 
												unsigned int* pPairOut, btCuda3F1U* pAABB, unsigned int numBodies);

// CUDA functions
	void btCuda_allocateArray(void** devPtr, unsigned int size);
	void btCuda_freeArray(void* devPtr);
	void btCuda_copyArrayFromDevice(void* host, const void* device, unsigned int size);
	void btCuda_copyArrayToDevice(void* device, const void* host, unsigned int size);
	void btCuda_setParameters(btCudaBroadphaseParams* hostParams);
	void btCuda_calcHashAABB(btCuda3F1U* pAABB, unsigned int* hash,	unsigned int numBodies);
	void btCuda_findCellStart(unsigned int* hash, unsigned int* cellStart, unsigned int numBodies, unsigned int numCells);
	void btCuda_findOverlappingPairs(	btCuda3F1U*	pAABB, unsigned int* pHash,
										unsigned int*	pCellStart,
										unsigned int*	pPairBuff,
										unsigned int*	pPairBuffStartCurr,
										unsigned int	numBodies);
	void btCuda_findPairsLarge(	btCuda3F1U*	pAABB, unsigned int* pHash,
								unsigned int*	pCellStart,
								unsigned int*	pPairBuff,
								unsigned int*	pPairBuffStartCurr,
								unsigned int	numBodies,
								unsigned int	numLarge);

	void btCuda_computePairCacheChanges(unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, 
										unsigned int* pPairScan, btCuda3F1U* pAABB, unsigned int numBodies);
	void btCuda_squeezeOverlappingPairBuff(	unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, 
											unsigned int* pPairOut, btCuda3F1U* pAABB, unsigned int numBodies);
}


#endif // CUDA_BROADPHASE_KERNEL_H