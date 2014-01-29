__kernel void moveObjectsKernel(__global float4* posOrnColors, int numObjects)
{
	int iGID = get_global_id(0);
	if (iGID>=numObjects)
		return;
	__global float4* positions = &posOrnColors[0];
	if (iGID<0.5*numObjects)
	{
		positions[iGID].y +=0.01f;
	}
	__global float4* colors = &posOrnColors[numObjects*2];
	colors[iGID] = (float4)(0,0,1,1);
}

__kernel void colorPairsKernel2(__global float4* posOrnColors, int numObjects, __global const int4* pairs, int indexOffset, int numPairs)
{
	int iPairId = get_global_id(0);
	if (iPairId>=numPairs)
		return;
	__global float4* colors = &posOrnColors[numObjects*2];

	int iObjectA = pairs[iPairId].x-indexOffset;
	int iObjectB = pairs[iPairId].y-indexOffset;
	colors[iObjectA] = (float4)(1,0,0,1);
	colors[iObjectB] = (float4)(1,0,0,1);
}

__kernel void 
  sineWaveKernel( __global float4* posOrnColors, __global float* pBodyTimes,float timeStepPos, float mAmplitude,const int numNodes)
{
	int nodeID = get_global_id(0);
	if( nodeID < numNodes )
	{
		pBodyTimes[nodeID] += timeStepPos;
		float4 position = posOrnColors[nodeID];
		position.x = native_cos(pBodyTimes[nodeID]*2.17f)*mAmplitude + native_sin(pBodyTimes[nodeID])*mAmplitude*0.5f;
		position.y = native_cos(pBodyTimes[nodeID]*1.38f)*mAmplitude + native_sin(pBodyTimes[nodeID]*mAmplitude);
		position.z = native_cos(pBodyTimes[nodeID]*2.17f)*mAmplitude + native_sin(pBodyTimes[nodeID]*0.777f)*mAmplitude;
		
		posOrnColors[nodeID] = position;
		__global float4* colors = &posOrnColors[numNodes*2];
		colors[nodeID] = (float4)(0,0,1,1);
	}
}

typedef struct 
{
	float			fx;
	float			fy;
	float			fz;
	int	uw;
} b3AABBCL;

__kernel void updateAabbSimple( __global float4* posOrnColors, const int numNodes, __global b3AABBCL* pAABB)
{
	int nodeId = get_global_id(0);
	if( nodeId < numNodes )
	{
	
		b3AABBCL orgAabbMin = pAABB[nodeId*2];
		b3AABBCL orgAabbMax = pAABB[nodeId*2+1];
		int orgNodeId = orgAabbMin.uw;
		int orgBroadphaseIndex = orgAabbMax.uw;
		
		float4 position = posOrnColors[nodeId];
		float4 argAabbMinVec = (float4)(orgAabbMin.fx,orgAabbMin.fy,orgAabbMin.fz,0.f);
		float4 argAabbMaxVec = (float4)(orgAabbMax.fx,orgAabbMax.fy,orgAabbMax.fz,0.f);
		float4 halfExtents = 0.5f*(argAabbMaxVec-argAabbMinVec);
		
		pAABB[nodeId*2].fx = position.x-halfExtents.x;
		pAABB[nodeId*2].fy = position.y-halfExtents.y;
		pAABB[nodeId*2].fz = position.z-halfExtents.z;
		pAABB[nodeId*2].uw = orgNodeId;
		pAABB[nodeId*2+1].fx = position.x+halfExtents.x;
		pAABB[nodeId*2+1].fy = position.y+halfExtents.y;
		pAABB[nodeId*2+1].fz = position.z+halfExtents.z;
		pAABB[nodeId*2+1].uw = orgBroadphaseIndex;		
	}
}
