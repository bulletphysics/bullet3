#ifndef PARTICLES_KERNEL_H
#define PARTICLES_KERNEL_H

#define BLOCKDIM 64
#define USE_SORT 1

#ifndef __DEVICE_EMULATION__
#define USE_TEX 1
#endif

#ifdef USE_TEX
#define FETCH(t, i) tex1Dfetch(t##Tex, i)
#else
#define FETCH(t, i) t[i]
#endif

#include "vector_types.h"
typedef unsigned int uint;

struct SimParams {
    float4 colliderPos;
    float  colliderRadius;    

    float3 gravity;
    float globalDamping;
    float particleRadius;

    uint3 gridSize;
    uint numCells;
    float3 worldOrigin;
    
    float3 cellSize;
    float3 worldSize; 
    uint3 m_gridSize;

    uint numBodies;
    uint maxParticlesPerCell;

    float spring;
    float damping;
    float shear;
    float attraction;
    float boundaryDamping;
};

#endif
