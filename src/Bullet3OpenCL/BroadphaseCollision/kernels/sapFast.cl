/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#define NEW_PAIR_MARKER -1
#define REMOVED_PAIR_MARKER -2

typedef struct 
{
	union
	{
		float4	m_min;
		float   m_minElems[4];
		int			m_minIndices[4];
	};
	union
	{
		float4	m_max;
		float   m_maxElems[4];
		int			m_maxIndices[4];
	};
} btAabbCL;

typedef struct 
{
	union
	{
		unsigned int m_key;
		unsigned int x;
	};

	union
	{
		unsigned int m_value;
		unsigned int y;
		
	};
}b3SortData;


/// conservative test for overlap between two aabbs
bool TestAabbAgainstAabb2(const btAabbCL* aabb1, __local const btAabbCL* aabb2);
bool TestAabbAgainstAabb2(const btAabbCL* aabb1, __local const btAabbCL* aabb2)
{
//skip pairs between static (mass=0) objects
	if ((aabb1->m_maxIndices[3]==0) && (aabb2->m_maxIndices[3] == 0))
		return false;
		
	bool overlap = true;
	overlap = (aabb1->m_min.x > aabb2->m_max.x || aabb1->m_max.x < aabb2->m_min.x) ? false : overlap;
	overlap = (aabb1->m_min.z > aabb2->m_max.z || aabb1->m_max.z < aabb2->m_min.z) ? false : overlap;
	overlap = (aabb1->m_min.y > aabb2->m_max.y || aabb1->m_max.y < aabb2->m_min.y) ? false : overlap;
	return overlap;
}

__kernel void   computePairsIncremental3dSapKernel( __global const uint2* objectMinMaxIndexGPUaxis0,
													__global const uint2* objectMinMaxIndexGPUaxis1,
													__global const uint2* objectMinMaxIndexGPUaxis2,
													__global const uint2* objectMinMaxIndexGPUaxis0prev,
													__global const uint2* objectMinMaxIndexGPUaxis1prev,
													__global const uint2* objectMinMaxIndexGPUaxis2prev,
													__global const b3SortData*	   sortedAxisGPU0,
													__global const b3SortData*	   sortedAxisGPU1,
													__global const b3SortData*	   sortedAxisGPU2,
													__global const b3SortData*	   sortedAxisGPU0prev,
													__global const b3SortData*	   sortedAxisGPU1prev,
													__global const b3SortData*	   sortedAxisGPU2prev,
													__global int4*			addedHostPairsGPU,
													__global int4*			removedHostPairsGPU,
													volatile __global int*				addedHostPairsCount,
													volatile __global int*				removedHostPairsCount,
													int maxCapacity,
													int numObjects)
{
	int i = get_global_id(0);
	if (i>=numObjects)
		return;

	__global const uint2* objectMinMaxIndexGPU[3][2];
	objectMinMaxIndexGPU[0][0]=objectMinMaxIndexGPUaxis0;
	objectMinMaxIndexGPU[1][0]=objectMinMaxIndexGPUaxis1;
	objectMinMaxIndexGPU[2][0]=objectMinMaxIndexGPUaxis2;
	objectMinMaxIndexGPU[0][1]=objectMinMaxIndexGPUaxis0prev;
	objectMinMaxIndexGPU[1][1]=objectMinMaxIndexGPUaxis1prev;
	objectMinMaxIndexGPU[2][1]=objectMinMaxIndexGPUaxis2prev;

	__global const b3SortData* sortedAxisGPU[3][2];
	sortedAxisGPU[0][0] = sortedAxisGPU0;
	sortedAxisGPU[1][0] = sortedAxisGPU1;
	sortedAxisGPU[2][0] = sortedAxisGPU2;
	sortedAxisGPU[0][1] = sortedAxisGPU0prev;
	sortedAxisGPU[1][1] = sortedAxisGPU1prev;
	sortedAxisGPU[2][1] = sortedAxisGPU2prev;

	int m_currentBuffer = 0;

	for (int axis=0;axis<3;axis++)
	{
		//int i = checkObjects[a];

		unsigned int curMinIndex = objectMinMaxIndexGPU[axis][m_currentBuffer][i].x;
		unsigned int curMaxIndex = objectMinMaxIndexGPU[axis][m_currentBuffer][i].y;
		unsigned int prevMinIndex = objectMinMaxIndexGPU[axis][1-m_currentBuffer][i].x;
		int dmin = curMinIndex - prevMinIndex;
				
		unsigned int prevMaxIndex = objectMinMaxIndexGPU[axis][1-m_currentBuffer][i].y;

		int dmax = curMaxIndex - prevMaxIndex;
	
		for (int otherbuffer = 0;otherbuffer<2;otherbuffer++)
		{
			if (dmin!=0)
			{
				int stepMin = dmin<0 ? -1 : 1;
				for (int j=prevMinIndex;j!=curMinIndex;j+=stepMin)
				{
					int otherIndex2 = sortedAxisGPU[axis][otherbuffer][j].y;
					int otherIndex = otherIndex2/2;
					if (otherIndex!=i)
					{
						bool otherIsMax = ((otherIndex2&1)!=0);

						if (otherIsMax)
						{
									
							bool overlap = true;

							for (int ax=0;ax<3;ax++)
							{
								if ((objectMinMaxIndexGPU[ax][m_currentBuffer][i].x > objectMinMaxIndexGPU[ax][m_currentBuffer][otherIndex].y) ||
									(objectMinMaxIndexGPU[ax][m_currentBuffer][i].y < objectMinMaxIndexGPU[ax][m_currentBuffer][otherIndex].x))
									overlap=false;
							}

						//	b3Assert(overlap2==overlap);

							bool prevOverlap = true;

							for (int ax=0;ax<3;ax++)
							{
								if ((objectMinMaxIndexGPU[ax][1-m_currentBuffer][i].x > objectMinMaxIndexGPU[ax][1-m_currentBuffer][otherIndex].y) ||
									(objectMinMaxIndexGPU[ax][1-m_currentBuffer][i].y < objectMinMaxIndexGPU[ax][1-m_currentBuffer][otherIndex].x))
									prevOverlap=false;
							}
									

							//b3Assert(overlap==overlap2);
								


							if (dmin<0)
							{
								if (overlap && !prevOverlap)
								{
									//add a pair
									int4 newPair;
									if (i<=otherIndex)
									{
										newPair.x = i;
										newPair.y = otherIndex;
									} else
									{
										newPair.x = otherIndex;
										newPair.y = i;
									}
									
									{
										int curPair = atomic_inc(addedHostPairsCount);
										if (curPair<maxCapacity)
										{
											addedHostPairsGPU[curPair].x = newPair.x;
											addedHostPairsGPU[curPair].y = newPair.y;
											addedHostPairsGPU[curPair].z = NEW_PAIR_MARKER;
											addedHostPairsGPU[curPair].w = NEW_PAIR_MARKER;

										}
									}

								}
							} 
							else
							{
								if (!overlap && prevOverlap)
								{
									
									//remove a pair
									int4 removedPair;
									if (i<=otherIndex)
									{
										removedPair.x = i;
										removedPair.y = otherIndex;
									} else
									{
										removedPair.x = otherIndex;
										removedPair.y = i;
									}
									{
										int curPair = atomic_inc(removedHostPairsCount);
										if (curPair<maxCapacity)
										{
											
											removedHostPairsGPU[curPair].x = removedPair.x;
											removedHostPairsGPU[curPair].y = removedPair.y;
											removedHostPairsGPU[curPair].z = REMOVED_PAIR_MARKER;
											removedHostPairsGPU[curPair].w = REMOVED_PAIR_MARKER;

										}
									}
								}
							}//otherisMax
						}//if (dmin<0)
					}//if (otherIndex!=i)
				}//for (int j=
			}
				
			if (dmax!=0)
			{
				int stepMax = dmax<0 ? -1 : 1;
				for (int j=prevMaxIndex;j!=curMaxIndex;j+=stepMax)
				{
					int otherIndex2 = sortedAxisGPU[axis][otherbuffer][j].y;
					int otherIndex = otherIndex2/2;
					if (otherIndex!=i)
					{
						bool otherIsMin = ((otherIndex2&1)==0);
						if (otherIsMin)
						{
									
							bool overlap = true;

							for (int ax=0;ax<3;ax++)
							{
								if ((objectMinMaxIndexGPU[ax][m_currentBuffer][i].x > objectMinMaxIndexGPU[ax][m_currentBuffer][otherIndex].y) ||
									(objectMinMaxIndexGPU[ax][m_currentBuffer][i].y < objectMinMaxIndexGPU[ax][m_currentBuffer][otherIndex].x))
									overlap=false;
							}
							//b3Assert(overlap2==overlap);

							bool prevOverlap = true;

							for (int ax=0;ax<3;ax++)
							{
								if ((objectMinMaxIndexGPU[ax][1-m_currentBuffer][i].x > objectMinMaxIndexGPU[ax][1-m_currentBuffer][otherIndex].y) ||
									(objectMinMaxIndexGPU[ax][1-m_currentBuffer][i].y < objectMinMaxIndexGPU[ax][1-m_currentBuffer][otherIndex].x))
									prevOverlap=false;
							}
									

							if (dmax>0)
							{
								if (overlap && !prevOverlap)
								{
									//add a pair
									int4 newPair;
									if (i<=otherIndex)
									{
										newPair.x = i;
										newPair.y = otherIndex;
									} else
									{
										newPair.x = otherIndex;
										newPair.y = i;
									}
									{
										int curPair = atomic_inc(addedHostPairsCount);
										if (curPair<maxCapacity)
										{
											
											addedHostPairsGPU[curPair].x = newPair.x;
											addedHostPairsGPU[curPair].y = newPair.y;
											addedHostPairsGPU[curPair].z = NEW_PAIR_MARKER;
											addedHostPairsGPU[curPair].w = NEW_PAIR_MARKER;

										}
									}
							
								}
							} 
							else
							{
								if (!overlap && prevOverlap)
								{
									//if (otherIndex2&1==0) -> min?
									//remove a pair
									int4 removedPair;
									if (i<=otherIndex)
									{
										removedPair.x = i;
										removedPair.y = otherIndex;
									} else
									{
										removedPair.x = otherIndex;
										removedPair.y = i;
									}
									{
										int curPair = atomic_inc(removedHostPairsCount);
										if (curPair<maxCapacity)
										{
											
											removedHostPairsGPU[curPair].x = removedPair.x;
											removedHostPairsGPU[curPair].y = removedPair.y;
											removedHostPairsGPU[curPair].z = REMOVED_PAIR_MARKER;
											removedHostPairsGPU[curPair].w = REMOVED_PAIR_MARKER;
										}
									}
								
								}
							}
							
						}//if (dmin<0)
					}//if (otherIndex!=i)
				}//for (int j=
			}
		}//for (int otherbuffer
	}//for (int axis=0;


}


__kernel void   computePairsKernelLocalSharedMemoryBatchWrite( __global const btAabbCL* aabbs, volatile __global int4* pairsOut,volatile  __global int* pairCount, int numObjects, int axis, int maxPairs)
{
	int i = get_global_id(0);
	int localId = get_local_id(0);

	__local int numActiveWgItems[1];
	__local int breakRequest[1];
	__local btAabbCL localAabbs[128];// = aabbs[i];
	
	int2 myPairs[64];
	
	btAabbCL myAabb;
	
	myAabb = (i<numObjects)? aabbs[i]:aabbs[0];
	float testValue = 	myAabb.m_maxElems[axis];
	
	if (localId==0)
	{
		numActiveWgItems[0] = 0;
		breakRequest[0] = 0;
	}
	int localCount=0;
	int block=0;
	localAabbs[localId] = (i+block)<numObjects? aabbs[i+block] : aabbs[0];
	localAabbs[localId+64] = (i+block+64)<numObjects? aabbs[i+block+64]: aabbs[0];
	
	barrier(CLK_LOCAL_MEM_FENCE);
	atomic_inc(numActiveWgItems);
	barrier(CLK_LOCAL_MEM_FENCE);
	int localBreak = 0;
	int curNumPairs = 0;
	
	int j=i+1;
	do
	{
		barrier(CLK_LOCAL_MEM_FENCE);
	
		if (j<numObjects)
		{
	  	if(testValue < (localAabbs[localCount+localId+1].m_minElems[axis])) 
			{
				if (!localBreak)
				{
					atomic_inc(breakRequest);
					localBreak = 1;
				}
			}
		}
		
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if (j>=numObjects && !localBreak)
		{
			atomic_inc(breakRequest);
			localBreak = 1;
		}
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if (!localBreak)
		{
			if (TestAabbAgainstAabb2(&myAabb,&localAabbs[localCount+localId+1]))
			{
				int2 myPair;
				myPair.x = myAabb.m_minIndices[3];
				myPair.y = localAabbs[localCount+localId+1].m_minIndices[3];
				myPairs[curNumPairs] = myPair;
				curNumPairs++;
				if (curNumPairs==64)
				{
					int curPair = atomic_add(pairCount,curNumPairs);
					for (int p=0;p<curNumPairs;p++)
					{
						if ((curPair+p)<maxPairs)
						{
							int4 tmpPair;
							tmpPair.x = myPairs[p].x;
							tmpPair.y = myPairs[p].y;
							tmpPair.z = NEW_PAIR_MARKER;
							tmpPair.w = NEW_PAIR_MARKER;
							

							pairsOut[curPair+p] = tmpPair; //flush to main memory
						}
					}
					curNumPairs = 0;
				}
			}
		}
		barrier(CLK_LOCAL_MEM_FENCE);
		
		localCount++;
		if (localCount==64)
		{
			localCount = 0;
			block+=64;			
			localAabbs[localId] = ((i+block)<numObjects) ? aabbs[i+block] : aabbs[0];
			localAabbs[localId+64] = ((i+64+block)<numObjects) ? aabbs[i+block+64] : aabbs[0];
		}
		j++;
		
	} while (breakRequest[0]<numActiveWgItems[0]);
	
	
	if (curNumPairs>0)
	{
		//avoid a buffer overrun
		int curPair = atomic_add(pairCount,curNumPairs);
		for (int p=0;p<curNumPairs;p++)
		{
				if ((curPair+p)<maxPairs)
				{
					int4 tmpPair;
					tmpPair.x = myPairs[p].x;
					tmpPair.y = myPairs[p].y;
					tmpPair.z = NEW_PAIR_MARKER;
					tmpPair.w = NEW_PAIR_MARKER;
					pairsOut[curPair+p] = tmpPair; //flush to main memory
				}
		}
		curNumPairs = 0;
	}
}