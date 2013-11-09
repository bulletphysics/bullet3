MSTRINGIFY(


typedef struct
{
	float4	m_gravity;
	float4	m_worldMin;
	float4	m_worldMax;
	
	float m_particleRad;
	float m_globalDamping;
	float m_boundaryDamping;
	float m_collisionDamping;

	float m_spring;
	float m_shear;
	float m_attraction;
	float m_dummy;
} btSimParams;


__kernel void  updatePositionsKernel( __global float4* linearVelocities, __global float4* positions,const int numNodes)
{
	int nodeID = get_global_id(0);
	float timeStep = 0.0166666;
	
	float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254);
	
	if( nodeID < numNodes )
	{
		positions[nodeID] += linearVelocities[nodeID]*timeStep;
	}
}



__kernel void integrateMotionKernel(	int numParticles,
								__global float4* pPos, 
								__global float4* pVel, 
								__global const btSimParams* simParams,
								float timeStep )
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
    float4 gravity = simParams[0].m_gravity;
    float particleRad = simParams[0].m_particleRad;
    float globalDamping = simParams[0].m_globalDamping;
    float boundaryDamping = simParams[0].m_boundaryDamping;

    vel += gravity * timeStep;
    vel *= globalDamping;
    // integrate position
    pos += vel * timeStep;
    // collide with world boundaries
    float4 worldMin = simParams[0].m_worldMin;
    float4 worldMax = simParams[0].m_worldMax;
    
    /*
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
	*/
    if(pos.y < (worldMin.y + 2*particleRad))
    {
        pos.y = worldMin.y + 2*particleRad;
        vel.y *= boundaryDamping;
    }
	/*
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
	*/
    // write back position and velocity
    pPos[index] = pos;
    pVel[index] = vel;
}

typedef struct 
{
	float			fx;
	float			fy;
	float			fz;
	int	uw;
} btAABBCL;

__kernel void collideParticlesKernel(  __global float4* pPos, __global float4* pVel, __global int2* pairs, const int numPairs)
{
	int index = get_global_id(0);
	if (index<numPairs)
	{
		

	}
	
}


__kernel void updateAabbsKernel(  __global float4* pPos, __global btAABBCL* pAABB, float particleRadius, const int numNodes)
{
	int nodeID = get_global_id(0);
		
	if( nodeID < numNodes )
	{
		float4 position = pPos[nodeID];
		float4 extent = (float4) (	particleRadius,particleRadius,particleRadius,0.f);
			
	
		pAABB[nodeID*2].fx = position.x-extent.x;
		pAABB[nodeID*2].fy = position.y-extent.y;
		pAABB[nodeID*2].fz = position.z-extent.z;
		pAABB[nodeID*2].uw = nodeID;
	
		pAABB[nodeID*2+1].fx = position.x+extent.x;
		pAABB[nodeID*2+1].fy = position.y+extent.y;
		pAABB[nodeID*2+1].fz = position.z+extent.z;
		pAABB[nodeID*2+1].uw = nodeID;
	} 
}


);