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

#include "btCudaBroadphaseKernel.h"

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//               K E R N E L    F U N C T I O N S 
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// calculate position in uniform grid
BT3DGRID__device__ int3 btCuda_calcGridPos(float4 p)
{
    int3 gridPos;
    gridPos.x = (int)floor((p.x - BT3DGRIDparams.m_worldOriginX) / BT3DGRIDparams.m_cellSizeX);
    gridPos.y = (int)floor((p.y - BT3DGRIDparams.m_worldOriginY) / BT3DGRIDparams.m_cellSizeY);
    gridPos.z = (int)floor((p.z - BT3DGRIDparams.m_worldOriginZ) / BT3DGRIDparams.m_cellSizeZ);
    return gridPos;
}

//----------------------------------------------------------------------------------------

// calculate address in grid from position (clamping to edges)
BT3DGRID__device__ uint btCuda_calcGridHash(int3 gridPos)
{
    gridPos.x = BT3DGRIDmax(0, BT3DGRIDmin(gridPos.x, (int)BT3DGRIDparams.m_gridSizeX - 1));
    gridPos.y = BT3DGRIDmax(0, BT3DGRIDmin(gridPos.y, (int)BT3DGRIDparams.m_gridSizeY - 1));
    gridPos.z = BT3DGRIDmax(0, BT3DGRIDmin(gridPos.z, (int)BT3DGRIDparams.m_gridSizeZ - 1));
    return BT3DGRID__mul24(BT3DGRID__mul24(gridPos.z, BT3DGRIDparams.m_gridSizeY), BT3DGRIDparams.m_gridSizeX) + BT3DGRID__mul24(gridPos.y, BT3DGRIDparams.m_gridSizeX) + gridPos.x;
}

//----------------------------------------------------------------------------------------

// calculate grid hash value for each body using its AABB
BT3DGRID__global__ void calcHashAABBD(btCuda3F1U* pAABB, uint2* pHash, uint numBodies)
{
    int index = BT3DGRID__mul24(BT3DGRIDblockIdx.x, BT3DGRIDblockDim.x) + BT3DGRIDthreadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	btCuda3F1U bbMin = pAABB[index*2];
	btCuda3F1U bbMax = pAABB[index*2 + 1];
	float4 pos;
	pos.x = (bbMin.fx + bbMax.fx) * 0.5f;
	pos.y = (bbMin.fy + bbMax.fy) * 0.5f;
	pos.z = (bbMin.fz + bbMax.fz) * 0.5f;
    // get address in grid
    int3 gridPos = btCuda_calcGridPos(pos);
    uint gridHash = btCuda_calcGridHash(gridPos);
    // store grid hash and body index
    pHash[index] = BT3DGRIDmake_uint2(gridHash, index);
}

//----------------------------------------------------------------------------------------

BT3DGRID__global__ void findCellStartD(uint2* pHash, uint* cellStart, uint numBodies)
{
    int index = BT3DGRID__mul24(BT3DGRIDblockIdx.x, BT3DGRIDblockDim.x) + BT3DGRIDthreadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
    uint2 sortedData = pHash[index];
	// Load hash data into shared memory so that we can look 
	// at neighboring body's hash value without loading
	// two hash values per thread
	BT3DGRID__shared__ uint sharedHash[257];
	sharedHash[BT3DGRIDthreadIdx.x+1] = sortedData.x;
	if((index > 0) && (BT3DGRIDthreadIdx.x == 0))
	{
		// first thread in block must load neighbor body hash
		volatile uint2 prevData = pHash[index-1];
		sharedHash[0] = prevData.x;
	}
	BT3DGRID__syncthreads();
	if((index == 0) || (sortedData.x != sharedHash[BT3DGRIDthreadIdx.x]))
	{
		cellStart[sortedData.x] = index;
	}
}

//----------------------------------------------------------------------------------------

BT3DGRID__device__ uint cudaTestAABBOverlap(btCuda3F1U min0, btCuda3F1U max0, btCuda3F1U min1, btCuda3F1U max1)
{
	return	(min0.fx <= max1.fx)&& (min1.fx <= max0.fx) && 
			(min0.fy <= max1.fy)&& (min1.fy <= max0.fy) && 
			(min0.fz <= max1.fz)&& (min1.fz <= max0.fz); 
}

//----------------------------------------------------------------------------------------

BT3DGRID__device__ void findPairsInCell(int3	gridPos,
								uint    index,
								uint2*  pHash,
								uint*   pCellStart,
								btCuda3F1U* pAABB, 
								uint*   pPairBuff,
								uint2*	pPairBuffStartCurr,
								uint	numBodies)
{
    if (	(gridPos.x < 0) || (gridPos.x > (int)BT3DGRIDparams.m_gridSizeX - 1)
		||	(gridPos.y < 0) || (gridPos.y > (int)BT3DGRIDparams.m_gridSizeY - 1)
		||  (gridPos.z < 0) || (gridPos.z > (int)BT3DGRIDparams.m_gridSizeZ - 1)) 
    {
		return;
	}
    uint gridHash = btCuda_calcGridHash(gridPos);
    // get start of bucket for this cell
    uint bucketStart = pCellStart[gridHash];
    if (bucketStart == 0xffffffff)
	{
        return;   // cell empty
	}
	// iterate over bodies in this cell
    uint2 sortedData = pHash[index];
	uint unsorted_indx = sortedData.y;
    btCuda3F1U min0 = BT3DGRIDFETCH(pAABB, unsorted_indx*2); 
	btCuda3F1U max0 = BT3DGRIDFETCH(pAABB, unsorted_indx*2 + 1);
	uint handleIndex =  min0.uw;
	uint2 start_curr = pPairBuffStartCurr[handleIndex];
	uint start = start_curr.x;
	uint curr = start_curr.y;
	uint2 start_curr_next = pPairBuffStartCurr[handleIndex+1];
	uint curr_max = start_curr_next.x - start - 1;
	uint bucketEnd = bucketStart + BT3DGRIDparams.m_maxBodiesPerCell;
	bucketEnd = (bucketEnd > numBodies) ? numBodies : bucketEnd;
	for(uint index2 = bucketStart; index2 < bucketEnd; index2++) 
	{
        uint2 cellData = pHash[index2];
        if (cellData.x != gridHash)
        {
			break;   // no longer in same bucket
		}
		uint unsorted_indx2 = cellData.y;
        if (unsorted_indx2 < unsorted_indx) // check not colliding with self
        {   
			btCuda3F1U min1 = BT3DGRIDFETCH(pAABB, unsorted_indx2*2);
			btCuda3F1U max1 = BT3DGRIDFETCH(pAABB, unsorted_indx2*2 + 1);
			if(cudaTestAABBOverlap(min0, max0, min1, max1))
			{
				uint handleIndex2 = min1.uw;
				uint k;
				for(k = 0; k < curr; k++)
				{
					uint old_pair = pPairBuff[start+k] & (~BT_CUDA_PAIR_ANY_FLG);
					if(old_pair == handleIndex2)
					{
						pPairBuff[start+k] |= BT_CUDA_PAIR_FOUND_FLG;
						break;
					}
				}
				if(k == curr)
				{
					pPairBuff[start+curr] = handleIndex2 | BT_CUDA_PAIR_NEW_FLG;
					if(curr >= curr_max) 
					{ // not a good solution, but let's avoid crash
						break;
					}
					curr++;
				}
			}
		}
	}
	pPairBuffStartCurr[handleIndex] = BT3DGRIDmake_uint2(start, curr);
    return;
}

//----------------------------------------------------------------------------------------

BT3DGRID__global__ void
findOverlappingPairsD(	btCuda3F1U*	pAABB, uint2* pHash, uint* pCellStart, uint* pPairBuff, 
						uint2* pPairBuffStartCurr, uint numBodies)
{
    int index = BT3DGRID__mul24(BT3DGRIDblockIdx.x, BT3DGRIDblockDim.x) + BT3DGRIDthreadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
    uint2 sortedData = pHash[index];
	uint unsorted_indx = sortedData.y;
	btCuda3F1U bbMin = BT3DGRIDFETCH(pAABB, unsorted_indx*2);
	btCuda3F1U bbMax = BT3DGRIDFETCH(pAABB, unsorted_indx*2 + 1);
	float4 pos;
	pos.x = (bbMin.fx + bbMax.fx) * 0.5f;
	pos.y = (bbMin.fy + bbMax.fy) * 0.5f;
	pos.z = (bbMin.fz + bbMax.fz) * 0.5f;
    // get address in grid
    int3 gridPos = btCuda_calcGridPos(pos);
    // examine only neighbouring cells
    for(int z=-1; z<=1; z++) {
        for(int y=-1; y<=1; y++) {
            for(int x=-1; x<=1; x++) {
                findPairsInCell(gridPos + BT3DGRIDmake_int3(x, y, z), index, pHash, pCellStart, pAABB, pPairBuff, pPairBuffStartCurr, numBodies);
            }
        }
    }
}

//----------------------------------------------------------------------------------------

BT3DGRID__global__ void
findPairsLargeD(	btCuda3F1U* pAABB, uint2* pHash, uint* pCellStart, uint* pPairBuff, 
						uint2* pPairBuffStartCurr, uint numBodies, uint numLarge)
{
    int index = BT3DGRID__mul24(BT3DGRIDblockIdx.x, BT3DGRIDblockDim.x) + BT3DGRIDthreadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
    uint2 sortedData = pHash[index];
	uint unsorted_indx = sortedData.y;
	btCuda3F1U min0 = BT3DGRIDFETCH(pAABB, unsorted_indx*2);
	btCuda3F1U max0 = BT3DGRIDFETCH(pAABB, unsorted_indx*2 + 1);
	uint handleIndex =  min0.uw;
	uint2 start_curr = pPairBuffStartCurr[handleIndex];
	uint start = start_curr.x;
	uint curr = start_curr.y;
	uint2 start_curr_next = pPairBuffStartCurr[handleIndex+1];
	uint curr_max = start_curr_next.x - start - 1;
    for(uint i = 0; i < numLarge; i++)
    {
		uint indx2 = numBodies + i;
		btCuda3F1U min1 = BT3DGRIDFETCH(pAABB, indx2*2);
		btCuda3F1U max1 = BT3DGRIDFETCH(pAABB, indx2*2 + 1);
		if(cudaTestAABBOverlap(min0, max0, min1, max1))
		{
			uint k;
			uint handleIndex2 =  min1.uw;
			for(k = 0; k < curr; k++)
			{
				uint old_pair = pPairBuff[start+k] & (~BT_CUDA_PAIR_ANY_FLG);
				if(old_pair == handleIndex2)
				{
					pPairBuff[start+k] |= BT_CUDA_PAIR_FOUND_FLG;
					break;
				}
			}
			if(k == curr)
			{
				pPairBuff[start+curr] = handleIndex2 | BT_CUDA_PAIR_NEW_FLG;
				if(curr >= curr_max) 
				{ // not a good solution, but let's avoid crash
					break;
				}
				curr++;
			}
		}
    }
	pPairBuffStartCurr[handleIndex] = BT3DGRIDmake_uint2(start, curr);
    return;
}

//----------------------------------------------------------------------------------------

BT3DGRID__global__ void computePairCacheChangesD(uint* pPairBuff, uint2* pPairBuffStartCurr, uint* pPairScan, btCuda3F1U* pAABB, uint numBodies)
{
    int index = BT3DGRID__mul24(BT3DGRIDblockIdx.x, BT3DGRIDblockDim.x) + BT3DGRIDthreadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	btCuda3F1U bbMin = pAABB[index * 2];
	uint handleIndex = bbMin.uw;
	uint2 start_curr = pPairBuffStartCurr[handleIndex];
	uint start = start_curr.x;
	uint curr = start_curr.y;
	uint *pInp = pPairBuff + start;
	uint num_changes = 0;
	for(uint k = 0; k < curr; k++, pInp++)
	{
		if(!((*pInp) & BT_CUDA_PAIR_FOUND_FLG))
		{
			num_changes++;
		}
	}
	pPairScan[index+1] = num_changes;
}

//----------------------------------------------------------------------------------------

BT3DGRID__global__ void squeezeOverlappingPairBuffD(uint* pPairBuff, uint2* pPairBuffStartCurr, uint* pPairScan, uint* pPairOut, btCuda3F1U* pAABB, uint numBodies)
{
    int index = BT3DGRID__mul24(BT3DGRIDblockIdx.x, BT3DGRIDblockDim.x) + BT3DGRIDthreadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	btCuda3F1U bbMin = pAABB[index * 2];
	uint handleIndex = bbMin.uw;
	uint2 start_curr = pPairBuffStartCurr[handleIndex];
	uint start = start_curr.x;
	uint curr = start_curr.y;
	uint* pInp = pPairBuff + start;
	uint* pOut = pPairOut + pPairScan[index];
	uint* pOut2 = pInp;
	uint num = 0; 
	for(uint k = 0; k < curr; k++, pInp++)
	{
		if(!((*pInp) & BT_CUDA_PAIR_FOUND_FLG))
		{
			*pOut = *pInp;
			pOut++;
		}
		if((*pInp) & BT_CUDA_PAIR_ANY_FLG)
		{
			*pOut2 = (*pInp) & (~BT_CUDA_PAIR_ANY_FLG);
			pOut2++;
			num++;
		}
	}
	pPairBuffStartCurr[handleIndex] = BT3DGRIDmake_uint2(start, num);
} // squeezeOverlappingPairBuffD()


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//               E N D   O F    K E R N E L    F U N C T I O N S 
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

extern "C"
{

//Round a / b to nearest higher integer value
int BT3DGRIDPREF(iDivUp)(int a, int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

// compute grid and thread block size for a given number of elements
void BT3DGRIDPREF(computeGridSize)(int n, int blockSize, int &numBlocks, int &numThreads)
{
    numThreads = BT3DGRIDmin(blockSize, n);
    numBlocks = BT3DGRIDPREF(iDivUp)(n, numThreads);
}

void BT3DGRIDPREF(calcHashAABB)(btCuda3F1U* pAABB, unsigned int* hash,	unsigned int numBodies)
{
    int numThreads, numBlocks;
    BT3DGRIDPREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    // execute the kernel
    BT3DGRIDEXECKERNEL(numBlocks, numThreads, calcHashAABBD, (pAABB, (uint2*)hash, numBodies));
    // check if kernel invocation generated an error
    CUT_CHECK_ERROR("calcHashAABBD kernel execution failed");
}

void BT3DGRIDPREF(findCellStart(unsigned int* hash, unsigned int* cellStart, unsigned int numBodies, unsigned int numCells))
{
    int numThreads, numBlocks;
    BT3DGRIDPREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
	MY_CUDA_SAFE_CALL(BT3DGPRDMemset(cellStart, 0xffffffff, numCells*sizeof(uint)));
	BT3DGRIDEXECKERNEL(numBlocks, numThreads, findCellStartD, ((uint2*)hash, (uint*)cellStart, numBodies));
    CUT_CHECK_ERROR("Kernel execution failed: findCellStartD");
}

void BT3DGRIDPREF(findOverlappingPairs(btCuda3F1U* pAABB, unsigned int* pHash,	unsigned int* pCellStart, unsigned int*	pPairBuff, unsigned int*	pPairBuffStartCurr, unsigned int	numBodies))
{
#if B_CUDA_USE_TEX
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, pAABBTex, pAABB, numBodies * 2 * sizeof(btCuda3F1U)));
#endif
    int numThreads, numBlocks;
    BT3DGRIDPREF(computeGridSize)(numBodies, 64, numBlocks, numThreads);
    BT3DGRIDEXECKERNEL(numBlocks, numThreads, findOverlappingPairsD, (pAABB,(uint2*)pHash,(uint*)pCellStart,(uint*)pPairBuff,(uint2*)pPairBuffStartCurr,numBodies));
    CUT_CHECK_ERROR("Kernel execution failed: bt_CudaFindOverlappingPairsD");
#if B_CUDA_USE_TEX
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(pAABBTex));
#endif
 }



void BT3DGRIDPREF(findPairsLarge(btCuda3F1U* pAABB, unsigned int* pHash, unsigned int* pCellStart, unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int numBodies, unsigned int numLarge))
{
#if B_CUDA_USE_TEX
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, pAABBTex, pAABB, (numBodies+numLarge) * 2 * sizeof(btCuda3F1U)));
#endif
    int numThreads, numBlocks;
    BT3DGRIDPREF(computeGridSize)(numBodies, 64, numBlocks, numThreads);
    BT3DGRIDEXECKERNEL(numBlocks, numThreads, findPairsLargeD, (pAABB,(uint2*)pHash,(uint*)pCellStart,(uint*)pPairBuff,(uint2*)pPairBuffStartCurr,numBodies,numLarge));
    CUT_CHECK_ERROR("Kernel execution failed: btCuda_findPairsLargeD");
#if B_CUDA_USE_TEX
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(pAABBTex));
#endif
 } 


void BT3DGRIDPREF(computePairCacheChanges(unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, btCuda3F1U* pAABB, unsigned int numBodies))
{
    int numThreads, numBlocks;
    BT3DGRIDPREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    BT3DGRIDEXECKERNEL(numBlocks, numThreads, computePairCacheChangesD, ((uint*)pPairBuff,(uint2*)pPairBuffStartCurr,(uint*)pPairScan,pAABB,numBodies));
    CUT_CHECK_ERROR("Kernel execution failed: btCudaComputePairCacheChangesD");
 }


void BT3DGRIDPREF(squeezeOverlappingPairBuff(unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, unsigned int* pPairOut, btCuda3F1U* pAABB, unsigned int numBodies))
{
    int numThreads, numBlocks;
    BT3DGRIDPREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    BT3DGRIDEXECKERNEL(numBlocks, numThreads, squeezeOverlappingPairBuffD, ((uint*)pPairBuff,(uint2*)pPairBuffStartCurr,(uint*)pPairScan,(uint*)pPairOut,pAABB,numBodies));
    CUT_CHECK_ERROR("Kernel execution failed: btCudaSqueezeOverlappingPairBuffD");
} // btCuda_squeezeOverlappingPairBuff()


}   // extern "C"
