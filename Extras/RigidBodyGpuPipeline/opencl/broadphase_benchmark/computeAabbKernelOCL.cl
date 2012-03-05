MSTRINGIFY(

typedef struct 
{
	int bla;
	int numElem;
} MyAabbConstDataCL ;

typedef struct 
{
	float			minfx;
	float			minfy;
	float			minfz;
	unsigned int	index0;
	float			maxfx;
	float			maxfy;
	float			maxfz;
	unsigned int	index1;	
} btAabbCL;


__kernel void   computeAabb( __global btAabbCL* aabbs,__global float4* positions, MyAabbConstDataCL cb)
{
	int nodeID = get_global_id(0);
		
	if( nodeID < cb.numElem )
	{
		aabbs[nodeID].minfx = positions[nodeID].x -1.f;
		aabbs[nodeID].minfy = positions[nodeID].y -1.f;
		aabbs[nodeID].minfz = positions[nodeID].z -1.f;	
		aabbs[nodeID].index0 = nodeID;	
		aabbs[nodeID].maxfx = positions[nodeID].x +1.f;
		aabbs[nodeID].maxfy = positions[nodeID].y +1.f;
		aabbs[nodeID].maxfz = positions[nodeID].z +1.f;		
		aabbs[nodeID].index1 = nodeID;
	}
}


__kernel void countOverlappingpairs(	int numObjects,
										__global int* pPairBuff, 
										__global int2* pPairBuffStartCurr, 
										__global int* pPairScan, 
										__global float4* pAABB )
{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
	float4 bbMin = pAABB[index * 2];
	int handleIndex = as_int(bbMin.w);
	int2 start_curr = pPairBuffStartCurr[handleIndex];
	int start = start_curr.x;
	int curr = start_curr.y;
	__global int *pInp = pPairBuff + start;
	int num_changes = 0;
	for(int k = 0; k < curr; k++, pInp++)
	{
		if(((*pInp) & 0x60000000))//either new or existing pairs (ignore old non-overlapping pairs)
		{
			num_changes++;
		}
	}
	pPairScan[index+1] = num_changes;
} 


__kernel void squeezePairCaches(	int numObjects,
											__global int* pPairBuff, 
											__global int2* pPairBuffStartCurr, 
											__global int* pPairScan,
											__global int2* pPairOut, 
											__global float4* pAABB )
{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
	float4 bbMin = pAABB[index * 2];
	int handleIndex = as_int(bbMin.w);
	int2 start_curr = pPairBuffStartCurr[handleIndex];
	int start = start_curr.x;
	int curr = start_curr.y;
	__global int* pInp = pPairBuff + start;
	__global int2* pOut = pPairOut + pPairScan[index+1];
	__global int* pOut2 = pInp;
	int num = 0; 
	for(int k = 0; k < curr; k++, pInp++)
	{
		if(((*pInp) & 0x60000000))
		{
			int2	newpair;
			newpair.x = handleIndex;
			newpair.y = (*pInp) & (~0x60000000);
			*pOut = newpair;
			pOut++;
		}
		if((*pInp) & 0x60000000)
		{
			*pOut2 = (*pInp) & (~0x60000000);
			pOut2++;
			num++;
		}
	}
	int2 newStartCurr;
	newStartCurr.x = start;
	newStartCurr.y = num;
	pPairBuffStartCurr[handleIndex] = newStartCurr;
}
);