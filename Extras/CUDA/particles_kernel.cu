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

/* 
 * Device code.
 */

#ifndef _PARTICLES_KERNEL_H_
#define _PARTICLES_KERNEL_H_

#include <stdio.h>
#include <math.h>
#include "cutil_math.h"
#include "math_constants.h"
#include "particles_kernel.cuh"

#if USE_TEX
// textures for particle position and velocity
texture<float4, 1, cudaReadModeElementType> oldPosTex;
texture<float4, 1, cudaReadModeElementType> oldVelTex;

texture<uint2, 1, cudaReadModeElementType> particleHashTex;
texture<uint, 1, cudaReadModeElementType> cellStartTex;

texture<uint, 1, cudaReadModeElementType> gridCountersTex;
texture<uint, 1, cudaReadModeElementType> gridCellsTex;
#endif

__constant__ SimParams params;

// integrate particle attributes
__global__ void
integrate(float4* newPos, float4* newVel, 
          float4* oldPos, float4* oldVel, 
          float deltaTime)
{
    int index = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

	float4 pos4 = oldPos[index];
    float4 vel4 = oldVel[index];
    float3 pos = make_float3(pos4);
    float3 vel = make_float3(vel4);

    vel += params.gravity * deltaTime;
    vel *= params.globalDamping;

    // new position = old position + velocity * deltaTime
    pos += vel * deltaTime;

    // bounce off cube sides
    if (pos.x > 1.0f - params.particleRadius) { pos.x = 1.0f - params.particleRadius; vel.x *= params.boundaryDamping; }
    if (pos.x < -1.0f + params.particleRadius) { pos.x = -1.0f + params.particleRadius; vel.x *= params.boundaryDamping;}
    if (pos.y > 1.0f - params.particleRadius) { pos.y = 1.0f - params.particleRadius; vel.y *= params.boundaryDamping; }
    if (pos.y < -1.0f + params.particleRadius) { pos.y = -1.0f + params.particleRadius; vel.y *= params.boundaryDamping;}
    if (pos.z > 1.0f - params.particleRadius) { pos.z = 1.0f - params.particleRadius; vel.z *= params.boundaryDamping; }
    if (pos.z < -1.0f + params.particleRadius) { pos.z = -1.0f + params.particleRadius; vel.z *= params.boundaryDamping;}

    // store new position and velocity
    newPos[index] = make_float4(pos, pos4.w);
    newVel[index] = make_float4(vel, vel4.w);
}

// calculate position in uniform grid
__device__ int3 calcGridPos(float4 p)
{
    int3 gridPos;
    gridPos.x = floor((p.x - params.worldOrigin.x) / params.cellSize.x);
    gridPos.y = floor((p.y - params.worldOrigin.y) / params.cellSize.y);
    gridPos.z = floor((p.z - params.worldOrigin.z) / params.cellSize.z);
    return gridPos;
}

// calculate address in grid from position (clamping to edges)
__device__ uint calcGridHash(int3 gridPos)
{
    gridPos.x = max(0, min(gridPos.x, params.gridSize.x-1));
    gridPos.y = max(0, min(gridPos.y, params.gridSize.y-1));
    gridPos.z = max(0, min(gridPos.z, params.gridSize.z-1));
    return __mul24(__mul24(gridPos.z, params.gridSize.y), params.gridSize.x) + __mul24(gridPos.y, params.gridSize.x) + gridPos.x;
}

// add particle to cell using atomics
__device__ void addParticleToCell(int3 gridPos,
                                  uint index,
                                  uint* gridCounters,
                                  uint* gridCells)
{
    // calculate grid hash
    uint gridHash = calcGridHash(gridPos);

    // increment cell counter using atomics
#if defined CUDA_NO_SM_11_ATOMIC_INTRINSICS
    int counter = 0;
#else
    int counter = atomicAdd(&gridCounters[gridHash], 1); // returns previous value
    counter = min(counter, params.maxParticlesPerCell-1);
#endif

    // write particle index into this cell (very uncoalesced!)
    gridCells[gridHash*params.maxParticlesPerCell + counter] = index;
}


// update uniform grid
__global__ void
updateGridD(float4* pos,
            uint*   gridCounters,
            uint*   gridCells)
{
    int index = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
    float4 p = pos[index];

    // get address in grid
    int3 gridPos = calcGridPos(p);

    addParticleToCell(gridPos, index, gridCounters, gridCells);
}

// calculate grid hash value for each particle
__global__ void
calcHashD(float4* pos,
          uint2*  particleHash)
{
    int index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
    float4 p = pos[index];

    // get address in grid
    int3 gridPos = calcGridPos(p);
    uint gridHash = calcGridHash(gridPos);

    // store grid hash and particle index
    particleHash[index] = make_uint2(gridHash, index);
}

// rearrange particle data into sorted order, and find the start of each cell in the
// sorted hash array
__global__ void
reorderDataAndFindCellStartD(uint2*  particleHash,  // particle id sorted by hash
				             float4* oldPos,
							 float4* oldVel,
							 float4* sortedPos, 
							 float4* sortedVel,
							 uint*   cellStart)
{
    int index = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

    uint2 sortedData = particleHash[index];

	// Load hash data into shared memory so that we can look 
	// at neighboring particle's hash value without loading
	// two hash values per thread
	__shared__ uint sharedHash[257];
	sharedHash[threadIdx.x+1] = sortedData.x;
	if (index > 0 && threadIdx.x == 0)
	{
		// first thread in block must load neighbor particle hash
		volatile uint2 prevData = particleHash[index-1];
		sharedHash[0] = prevData.x;
	}

	__syncthreads();
	if (index == 0 || sortedData.x != sharedHash[threadIdx.x])
	{
		cellStart[sortedData.x] = index;
	}

	// Now use the sorted index to reorder the pos and vel data
	float4 pos = FETCH(oldPos, sortedData.y);       // macro does either global read or texture fetch
    float4 vel = FETCH(oldVel, sortedData.y);       // see particles_kernel.cuh

    sortedPos[index] = pos;
    sortedVel[index] = vel;

}

// collide two spheres using DEM method
__device__ float3 collideSpheres(float4 posA, float4 posB,
                                 float4 velA, float4 velB,
                                 float radiusA, float radiusB,
                                 float attraction)
{
	// calculate relative position
    float3 relPos;
    relPos.x = posB.x - posA.x;
    relPos.y = posB.y - posA.y;
    relPos.z = posB.z - posA.z;

    float dist = length(relPos);
    float collideDist = radiusA + radiusB;

    float3 force = make_float3(0.0f);
    if (dist < collideDist) {
        float3 norm = relPos / dist;

		// relative velocity
        float3 relVel;
        relVel.x = velB.x - velA.x;
        relVel.y = velB.y - velA.y;
        relVel.z = velB.z - velA.z;

        // relative tangential velocity
        float3 tanVel = relVel - (dot(relVel, norm) * norm);

        // spring force
        force = -params.spring*(collideDist - dist) * norm;
        // dashpot (damping) force
        force += params.damping*relVel;
        // tangential shear force
        force += params.shear*tanVel;
		// attraction
        force += attraction*relPos;
    }

    return force;
}


// collide particle with all particles in a given cell
// version using grid built with atomics
__device__
float3 collideCell(int3 gridPos,
                   uint index,
                   float4 pos,
                   float4 vel,
                   float4* oldPos, 
                   float4* oldVel,
                   uint*   gridCounters,
                   uint*   gridCells)
{
    float3 force = make_float3(0.0f);

    if ((gridPos.x < 0) || (gridPos.x > params.gridSize.x-1) ||
        (gridPos.y < 0) || (gridPos.y > params.gridSize.y-1) ||
        (gridPos.z < 0) || (gridPos.z > params.gridSize.z-1)) {
        return force;
    }

    uint gridHash = calcGridHash(gridPos);
    
    // iterate over particles in this cell
    uint particlesInCell = FETCH(gridCounters, gridHash);
    particlesInCell = min(particlesInCell, params.maxParticlesPerCell-1);

    for(uint i=0; i<particlesInCell; i++) {
        uint index2 = FETCH(gridCells, gridHash*params.maxParticlesPerCell + i);

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


// version using sorted grid
__device__
float3 collideCell2(int3   gridPos,
                   uint    index,
                   float4  pos,
                   float4  vel,
                   float4* oldPos, 
                   float4* oldVel,
                   uint2*  particleHash,
                   uint*   cellStart)
{
    float3 force = make_float3(0.0f);

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


__global__ void
collideD(float4* newPos, float4* newVel, 
         float4* oldPos, float4* oldVel, 
#if USE_SORT
         uint2*  particleHash,
         uint*   cellStart
#else
         uint*   gridCounters,
         uint*   gridCells
#endif
         )
{
    int index = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;

    // read particle data from sorted arrays
	float4 pos = FETCH(oldPos, index);
    float4 vel = FETCH(oldVel, index);

    // get address in grid
    int3 gridPos = calcGridPos(pos);

    float3 force = make_float3(0.0f);

    // examine only neighbouring cells
    for(int z=-1; z<=1; z++) {
        for(int y=-1; y<=1; y++) {
            for(int x=-1; x<=1; x++) {
#if USE_SORT
                force += collideCell2(gridPos + make_int3(x, y, z), index, pos, vel, oldPos, oldVel, particleHash, cellStart);
#else
                force += collideCell(gridPos + make_int3(x, y, z), index, pos, vel, oldPos, oldVel, gridCounters, gridCells);
#endif
            }
        }
    }

    float3 projVec = collideSpheres(pos, params.colliderPos, vel, make_float4(0.0f, 0.0f, 0.0f, 0.0f), params.particleRadius, params.colliderRadius, 0.0f);
    force += projVec;

#if USE_SORT
    // write new velocity back to original unsorted location
    volatile uint2 sortedData = particleHash[index];
    newVel[sortedData.y] = vel + make_float4(force, 0.0f);
#else
    newVel[index] = vel + make_float4(force, 0.0f);
#endif
}

#endif
