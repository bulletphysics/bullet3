MSTRINGIFY(

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



int4 getGridPos(float4 worldPos, __global float4* pParams)
{
    int4 gridPos;
    gridPos.x = (int)floor((worldPos.x - pParams[1].x) / pParams[3].x);
    gridPos.y = (int)floor((worldPos.y - pParams[1].y) / pParams[3].y);
    gridPos.z = (int)floor((worldPos.z - pParams[1].z) / pParams[3].z);
    return gridPos;
}

unsigned int getPosHash(int4 gridPos, __global float4* pParams)
{
	int4 gridDim = *((__global int4*)(pParams + 4));
	if(gridPos.x < 0) gridPos.x = 0;
	if(gridPos.x >= gridDim.x) gridPos.x = gridDim.x - 1;
	if(gridPos.y < 0) gridPos.y = 0;
	if(gridPos.y >= gridDim.y) gridPos.y = gridDim.y - 1;
	if(gridPos.z < 0) gridPos.z = 0;
	if(gridPos.z >= gridDim.z) gridPos.z = gridDim.z - 1;
	unsigned int hash = gridPos.z * gridDim.y * gridDim.x + gridPos.y * gridDim.x + gridPos.x;
	return hash;
} 


__kernel void kComputeCellId(	int numParticles, 
								__global float4* pPos, 
								__global int2* pPosHash,
								__global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numParticles)
    {
		return;
    }
	float4 pos = pPos[index];
	int4 gridPos = getGridPos(pos, pParams);
	unsigned int hash = getPosHash(gridPos, pParams);
	pPosHash[index].x = hash;
	pPosHash[index].y = index;
}

__kernel void kClearCellStart(	int numCells, 
								__global int* pCellStart GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numCells)
	{
		return;
	}
	pCellStart[index] = -1;
}

__kernel void kFindCellStart(	int numParticles, 
								__global int2* pHash, 
								__global int* cellStart,
								__global float4* pPos,
								__global float4* pVel,
								__global float4* pSortedPos,
								__global float4* pSortedVel GUID_ARG)
{
    int index = get_global_id(0);
	__local int sharedHash[1025];//maximum workgroup size 1024
	int2 sortedData;
	
    if(index < numParticles)
	{

		sortedData = pHash[index];
		// Load hash data into shared memory so that we can look 
		// at neighboring body's hash value without loading
		// two hash values per thread
		sharedHash[get_local_id(0) + 1] = sortedData.x;
		if((index > 0) && (get_local_id(0) == 0))
		{
			// first thread in block must load neighbor body hash
			sharedHash[0] = pHash[index-1].x;
		}
		
	}
    barrier(CLK_LOCAL_MEM_FENCE);
	
	if(index < numParticles)
	{
		if((index == 0) || (sortedData.x != sharedHash[get_local_id(0)]))
		{
			cellStart[sortedData.x] = index;
		}
		int unsortedIndex = sortedData.y;
		float4 pos = pPos[unsortedIndex];
		float4 vel = pVel[unsortedIndex];
		pSortedPos[index] = pos;
		pSortedVel[index] = vel;
	}
}

__kernel void kIntegrateMotion(	int numParticles,
								__global float4* pPos, 
								__global float4* pVel, 
								__global float4* pParams, 
								float timeStep GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numParticles)
    {
		return;
	}
    float4 pos = pPos[index];
    float4 vel = pVel[index];
    pos.w = 1.0f;
    vel.w = 0.0f;
    // apply gravity
    float4 gravity = *((__global float4*)(pParams + 0));
    float particleRad = pParams[5].x;
    float globalDamping = pParams[5].y;
    float boundaryDamping = pParams[5].z;
    vel += gravity * timeStep;
    vel *= globalDamping;
    // integrate position
    pos += vel * timeStep;
    // collide with world boundaries
    float4 worldMin = *((__global float4*)(pParams + 1));
    float4 worldMax = *((__global float4*)(pParams + 2));
    
    
    if(pos.x < (worldMin.x + 2*particleRad))
    {
        pos.x = worldMin.x + 2*particleRad;
        vel.x *= boundaryDamping;
    }
    if(pos.x > (worldMax.x - 2*particleRad))
    {
        pos.x = worldMax.x - 2*particleRad;
        vel.x *= boundaryDamping;
    }
    if(pos.y < (worldMin.y + 2*particleRad))
    {
        pos.y = worldMin.y + 2*particleRad;
        vel.y *= boundaryDamping;
    }
    if(pos.y > (worldMax.y - 2*particleRad))
    {
        pos.y = worldMax.y - 2*particleRad;
        vel.y *= boundaryDamping;
    }
    if(pos.z < (worldMin.z + 2*particleRad))
    {
        pos.z = worldMin.z + 2*particleRad;
        vel.z *= boundaryDamping;
    }
    if(pos.z > (worldMax.z - 2*particleRad))
    {
        pos.z = worldMax.z - 2*particleRad;
        vel.z *= boundaryDamping;
    }
    // write back position and velocity
    pPos[index] = pos;
    pVel[index] = vel;
}


float4 collideTwoParticles(
    float4 posA,
    float4 posB,
    float4 velA,
    float4 velB,
    float radiusA,
    float radiusB,
    float spring,
    float damping,
    float shear,
    float attraction
)
{
    //Calculate relative position
    float4     relPos = posB - posA; relPos.w = 0.f;
    float        dist = sqrt(relPos.x * relPos.x + relPos.y * relPos.y + relPos.z * relPos.z);
    float collideDist = radiusA + radiusB;

    float4 force = (float4)0.f;
    if(dist < collideDist){
        float4 norm = relPos * (1.f / dist); norm.w = 0.f;

        //Relative velocity
        float4 relVel = velB - velA; relVel.w = 0.f;

        //Relative tangential velocity
        float relVelDotNorm = relVel.x * norm.x + relVel.y * norm.y + relVel.z * norm.z;
        float4 tanVel = relVel - norm * relVelDotNorm;  tanVel.w = 0.f;

        //Spring force (potential)
        float springFactor = -spring * (collideDist - dist);
        force = springFactor * norm + damping * relVel + shear * tanVel + attraction * relPos;
        force.w = 0.f;
    }
    return force;
}


__kernel void kCollideParticles(int numParticles,
								__global float4*		pVel,          //output: new velocity
								__global const float4* pSortedPos, //input: reordered positions
								__global const float4* pSortedVel, //input: reordered velocities
								__global const int2   *pPosHash,        //input: reordered particle indices
								__global const int   *pCellStart,    //input: cell boundaries
								__global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numParticles)
	{
        return;
	}

    float4   posA = pSortedPos[index];
    float4   velA = pSortedVel[index];
    float4 force = (float4)0.f;
    float particleRad = pParams[5].x;
    float collisionDamping = pParams[5].w;
    float spring = pParams[6].x;
    float shear = pParams[6].y;
    float attraction = pParams[6].z;
    int unsortedIndex = pPosHash[index].y;

    //Get address in grid
    int4 gridPosA = getGridPos(posA, pParams);

    //Accumulate surrounding cells
    int4 gridPosB; 
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
                uint hashB = getPosHash(gridPosB, pParams);
                int startI = pCellStart[hashB];
                //Skip empty cell
                if(startI < 0)
                {
                    continue;
                }
               //Iterate over particles in this cell
                int endI = startI + 32;
                if(endI >= numParticles) 
					endI = numParticles ;
					
                for(int j = startI; j < endI; j++)
                {
					uint hashC = pPosHash[j].x;
					if(hashC != hashB)
					{
						break;
					}
					if(j == index)
					{
						continue;
					}
                    float4 posB = pSortedPos[j];
                    float4 velB = pSortedVel[j];
                    //Collide two spheres
                    force += collideTwoParticles(	posA, posB, velA, velB, particleRad, particleRad, 
													spring, collisionDamping, shear, attraction);
                }
            }
		}
	}     
    //Write new velocity back to original unsorted location
    pVel[unsortedIndex] = velA + force;
}






/*
 * Copyright 1993-2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and 
 * proprietary rights in and to this software and related documentation. 
 * Any use, reproduction, disclosure, or distribution of this software 
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA) 
 * associated with this source code for terms and conditions that govern 
 * your use of this NVIDIA software.
 * 
 */



inline void ComparatorPrivate(int2* keyA, int2* keyB, uint dir)
{
    if((keyA[0].x > keyB[0].x) == dir)
    {
		int2 tmp = *keyA;
		*keyA = *keyB;
		*keyB = tmp;
    }
}

inline void ComparatorLocal(__local int2* keyA, __local int2* keyB, uint dir)
{
    if((keyA[0].x > keyB[0].x) == dir)
    {
		int2 tmp = *keyA;
		*keyA = *keyB;
		*keyB = tmp;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Monolithic bitonic sort kernel for short arrays fitting into local memory
////////////////////////////////////////////////////////////////////////////////
__kernel void kBitonicSortCellIdLocal(__global int2* pKey, uint arrayLength, uint dir GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_MAX];
    int localSizeLimit = get_local_size(0) * 2;

    //Offset to the beginning of subbatch and load data
    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

    for(uint size = 2; size < arrayLength; size <<= 1)
    {
        //Bitonic merge
        uint ddd = dir ^ ( (get_local_id(0) & (size / 2)) != 0 );
        for(uint stride = size / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos +      0], &l_key[pos + stride], ddd);
        }
    }

    //ddd == dir for the last bitonic merge step
    {
        for(uint stride = arrayLength / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], dir);
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}

////////////////////////////////////////////////////////////////////////////////
// Bitonic sort kernel for large arrays (not fitting into local memory)
////////////////////////////////////////////////////////////////////////////////
//Bottom-level bitonic sort
//Almost the same as bitonicSortLocal with the only exception
//of even / odd subarrays (of LOCAL_SIZE_LIMIT points) being
//sorted in opposite directions
__kernel void kBitonicSortCellIdLocal1(__global int2* pKey GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_MAX];
    uint localSizeLimit = get_local_size(0) * 2;

    //Offset to the beginning of subarray and load data
    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

    uint comparatorI = get_global_id(0) & ((localSizeLimit / 2) - 1);

    for(uint size = 2; size < localSizeLimit; size <<= 1)
    {
        //Bitonic merge
        uint ddd = (comparatorI & (size / 2)) != 0;
        for(uint stride = size / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
        }
    }

    //Odd / even arrays of localSizeLimit elements
    //sorted in opposite directions
    {
        uint ddd = (get_group_id(0) & 1);
        for(uint stride = localSizeLimit / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}

//Bitonic merge iteration for 'stride' >= LOCAL_SIZE_LIMIT
__kernel void kBitonicSortCellIdMergeGlobal(__global int2* pKey, uint arrayLength, uint size, uint stride, uint dir GUID_ARG)
{
    uint global_comparatorI = get_global_id(0);
    uint        comparatorI = global_comparatorI & (arrayLength / 2 - 1);

    //Bitonic merge
    uint ddd = dir ^ ( (comparatorI & (size / 2)) != 0 );
    uint pos = 2 * global_comparatorI - (global_comparatorI & (stride - 1));

    int2 keyA = pKey[pos +      0];
    int2 keyB = pKey[pos + stride];

    ComparatorPrivate(&keyA, &keyB, ddd);

    pKey[pos +      0] = keyA;
    pKey[pos + stride] = keyB;
}

//Combined bitonic merge steps for
//'size' > LOCAL_SIZE_LIMIT and 'stride' = [1 .. LOCAL_SIZE_LIMIT / 2]
__kernel void kBitonicSortCellIdMergeLocal(__global int2* pKey, uint arrayLength, uint stride, uint size, uint dir GUID_ARG)
{
    __local int2 l_key[LOCAL_SIZE_MAX];
    int localSizeLimit = get_local_size(0) * 2;

    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

    //Bitonic merge
    uint comparatorI = get_global_id(0) & ((arrayLength / 2) - 1);
    uint         ddd = dir ^ ( (comparatorI & (size / 2)) != 0 );
    for(; stride > 0; stride >>= 1)
    {
        barrier(CLK_LOCAL_MEM_FENCE);
        uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
        ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}

);
