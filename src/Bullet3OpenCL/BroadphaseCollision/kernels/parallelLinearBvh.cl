/*
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Initial Author Jackson Lee, 2014

typedef float b3Scalar;
typedef float4 b3Vector3;
#define b3Max max
#define b3Min min
#define b3Sqrt sqrt

typedef struct
{
	unsigned int m_key;
	unsigned int m_value;
} SortDataCL;

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
} b3AabbCL;


unsigned int interleaveBits(unsigned int x)
{
	//........ ........ ......12 3456789A	//x
	//....1..2 ..3..4.. 5..6..7. .8..9..A	//x after interleaving bits
	
	//........ ....1234 56789A12 3456789A	//x |= (x << 10)
	//........ ....1111 1....... ...11111	//0x 00 0F 80 1F
	//........ ....1234 5....... ...6789A	//x = ( x | (x << 10) ) & 0x000F801F; 
	
	//.......1 23451234 5.....67 89A6789A	//x |= (x <<  5)
	//.......1 1.....11 1.....11 .....111	//0x 01 83 83 07
	//.......1 2.....34 5.....67 .....89A	//x = ( x | (x <<  5) ) & 0x01838307;
	
	//....12.1 2..34534 5..67.67 ..89A89A	//x |= (x <<  3)
	//....1... 1..1...1 1..1...1 ..1...11	//0x 08 91 91 23
	//....1... 2..3...4 5..6...7 ..8...9A	//x = ( x | (x <<  3) ) & 0x08919123;
	
	//...11..2 2.33..4N 5.66..77 .88..9NA	//x |= (x <<  1)	( N indicates overlapping bits, first overlap is bit {4,5} second is {9,A} )
	//....1..1 ..1...1. 1..1..1. .1...1.1	//0x 09 22 92 45
	//....1..2 ..3...4. 5..6..7. .8...9.A	//x = ( x | (x <<  1) ) & 0x09229245;
	
	//...11.22 .33..445 5.66.77. 88..99AA	//x |= (x <<  1)
	//....1..1 ..1..1.. 1..1..1. .1..1..1	//0x 09 34 92 29
	//....1..2 ..3..4.. 5..6..7. .8..9..A	//x = ( x | (x <<  1) ) & 0x09349229;
	
	//........ ........ ......11 11111111	//0x000003FF
	x &= 0x000003FF;		//Clear all bits above bit 10
	
	x = ( x | (x << 10) ) & 0x000F801F;
	x = ( x | (x <<  5) ) & 0x01838307;
	x = ( x | (x <<  3) ) & 0x08919123;
	x = ( x | (x <<  1) ) & 0x09229245;
	x = ( x | (x <<  1) ) & 0x09349229;
	
	return x;
}
unsigned int getMortonCode(unsigned int x, unsigned int y, unsigned int z)
{
	return interleaveBits(x) << 0 | interleaveBits(y) << 1 | interleaveBits(z) << 2;
}

__kernel void separateAabbs(__global b3AabbCL* unseparatedAabbs, __global int* aabbIndices, __global b3AabbCL* out_aabbs, int numAabbsToSeparate)
{
	int separatedAabbIndex = get_global_id(0);
	if(separatedAabbIndex >= numAabbsToSeparate) return;

	int unseparatedAabbIndex = aabbIndices[separatedAabbIndex];
	out_aabbs[separatedAabbIndex] = unseparatedAabbs[unseparatedAabbIndex];
}

//Should replace with an optimized parallel reduction
__kernel void findAllNodesMergedAabb(__global b3AabbCL* out_mergedAabb, int numAabbsNeedingMerge)
{
	//Each time this kernel is added to the command queue, 
	//the number of AABBs needing to be merged is halved
	//
	//Example with 159 AABBs:
	//	numRemainingAabbs == 159 / 2 + 159 % 2 == 80
	//	numMergedAabbs == 159 - 80 == 79
	//So, indices [0, 78] are merged with [0 + 80, 78 + 80]
	
	int numRemainingAabbs = numAabbsNeedingMerge / 2 + numAabbsNeedingMerge % 2;
	int numMergedAabbs = numAabbsNeedingMerge - numRemainingAabbs;
	
	int aabbIndex = get_global_id(0);
	if(aabbIndex >= numMergedAabbs) return;
	
	int otherAabbIndex = aabbIndex + numRemainingAabbs;
	
	b3AabbCL aabb = out_mergedAabb[aabbIndex];
	b3AabbCL otherAabb = out_mergedAabb[otherAabbIndex];
		
	b3AabbCL mergedAabb;
	mergedAabb.m_min = b3Min(aabb.m_min, otherAabb.m_min);
	mergedAabb.m_max = b3Max(aabb.m_max, otherAabb.m_max);
	out_mergedAabb[aabbIndex] = mergedAabb;
}

__kernel void assignMortonCodesAndAabbIndicies(__global b3AabbCL* worldSpaceAabbs, __global b3AabbCL* mergedAabbOfAllNodes, 
												__global SortDataCL* out_mortonCodesAndAabbIndices, int numAabbs)
{
	int leafNodeIndex = get_global_id(0);	//Leaf node index == AABB index
	if(leafNodeIndex >= numAabbs) return;
	
	b3AabbCL mergedAabb = mergedAabbOfAllNodes[0];
	b3Vector3 gridCenter = (mergedAabb.m_min + mergedAabb.m_max) * 0.5f;
	b3Vector3 gridCellSize = (mergedAabb.m_max - mergedAabb.m_min) / (float)1024;
	
	b3AabbCL aabb = worldSpaceAabbs[leafNodeIndex];
	b3Vector3 aabbCenter = (aabb.m_min + aabb.m_max) * 0.5f;
	b3Vector3 aabbCenterRelativeToGrid = aabbCenter - gridCenter;
	
	//Quantize into integer coordinates
	//floor() is needed to prevent the center cell, at (0,0,0) from being twice the size
	b3Vector3 gridPosition = aabbCenterRelativeToGrid / gridCellSize;
	
	int4 discretePosition;
	discretePosition.x = (int)( (gridPosition.x >= 0.0f) ? gridPosition.x : floor(gridPosition.x) );
	discretePosition.y = (int)( (gridPosition.y >= 0.0f) ? gridPosition.y : floor(gridPosition.y) );
	discretePosition.z = (int)( (gridPosition.z >= 0.0f) ? gridPosition.z : floor(gridPosition.z) );
	
	//Clamp coordinates into [-512, 511], then convert range from [-512, 511] to [0, 1023]
	discretePosition = b3Max( -512, b3Min(discretePosition, 511) );
	discretePosition += 512;
	
	//Interleave bits(assign a morton code, also known as a z-curve)
	unsigned int mortonCode = getMortonCode(discretePosition.x, discretePosition.y, discretePosition.z);
	
	//
	SortDataCL mortonCodeIndexPair;
	mortonCodeIndexPair.m_key = mortonCode;
	mortonCodeIndexPair.m_value = leafNodeIndex;
	
	out_mortonCodesAndAabbIndices[leafNodeIndex] = mortonCodeIndexPair;
}

#define B3_PLVBH_TRAVERSE_MAX_STACK_SIZE 128

//The most significant bit(0x80000000) of a int32 is used to distinguish between leaf and internal nodes.
//If it is set, then the index is for an internal node; otherwise, it is a leaf node. 
//In both cases, the bit should be cleared to access the index.
int isLeafNode(int index) { return (index >> 31 == 0); }
int getIndexWithInternalNodeMarkerRemoved(int index) { return index & (~0x80000000); }
int getIndexWithInternalNodeMarkerSet(int isLeaf, int index) { return (isLeaf) ? index : (index | 0x80000000); }

__kernel void constructBinaryTree(__global int* firstIndexOffsetPerLevel,
									__global int* numNodesPerLevel,
									__global int2* out_internalNodeChildIndices, 
									__global int* out_internalNodeParentNodes, 
									__global int* out_leafNodeParentNodes, 
									int numLevels, int numInternalNodes)
{
	int internalNodeIndex = get_global_id(0);
	if(internalNodeIndex >= numInternalNodes) return;
	
	//Find the level that this node is in, using linear search(could replace with binary search)
	int level = 0;
	int numInternalLevels = numLevels - 1;		//All levels except the last are internal nodes
	for(; level < numInternalLevels; ++level)
	{
		if( firstIndexOffsetPerLevel[level] <= internalNodeIndex && internalNodeIndex < firstIndexOffsetPerLevel[level + 1]) break;
	}
	
	//Check lower levels to find child nodes
	//Left child is always in the next level, but the same does not apply to the right child
	int indexInLevel = internalNodeIndex - firstIndexOffsetPerLevel[level];
	int firstIndexInNextLevel = firstIndexOffsetPerLevel[level + 1];	//Should never be out of bounds(see for loop above)
	
	int leftChildLevel = level + 1;
	int leftChildIndex = firstIndexInNextLevel + indexInLevel * 2;
	
	int rightChildLevel = level + 1;
	int rightChildIndex = leftChildIndex + 1;
	
	//Under certain conditions, the right child index as calculated above is invalid; need to find the correct index
	//
	//First condition: must be at least 2 levels apart from the leaf node level;
	//if the current level is right next to the leaf node level, then the right child
	//will never be invalid due to the way the nodes are allocated (also avoid a out-of-bounds memory access)
	//
	//Second condition: not enough nodes in the next level for each parent to have 2 children, so the right child is invalid
	//
	//Third condition: must be the last node in its level
	if( level < numLevels - 2 
		&& numNodesPerLevel[level] * 2 > numNodesPerLevel[level + 1] 
		&& indexInLevel == numNodesPerLevel[level] - 1 )
	{
		//Check lower levels until we find a node without a parent
		for(; rightChildLevel < numLevels - 1; ++rightChildLevel)
		{
			int rightChildNextLevel = rightChildLevel + 1;
		
			//If this branch is taken, it means that the last node in rightChildNextLevel has no parent
			if( numNodesPerLevel[rightChildLevel] * 2 < numNodesPerLevel[rightChildNextLevel] )
			{
				//Set the node to the last node in rightChildNextLevel
				rightChildLevel = rightChildNextLevel;
				rightChildIndex = firstIndexOffsetPerLevel[rightChildNextLevel] + numNodesPerLevel[rightChildNextLevel] - 1;
				break;
			}
		}
	}
	
	int isLeftChildLeaf = (leftChildLevel >= numLevels - 1);
	int isRightChildLeaf = (rightChildLevel >= numLevels - 1);
	
	//If left/right child is a leaf node, the index needs to be corrected
	//the way the index is calculated assumes that the leaf and internal nodes are in a contiguous array,
	//with leaf nodes at the end of the array; in actuality, the leaf and internal nodes are in separate arrays
	{
		int leafNodeLevel = numLevels - 1;
		leftChildIndex = (isLeftChildLeaf) ? leftChildIndex - firstIndexOffsetPerLevel[leafNodeLevel] : leftChildIndex;
		rightChildIndex = (isRightChildLeaf) ? rightChildIndex - firstIndexOffsetPerLevel[leafNodeLevel] : rightChildIndex;
	}
	
	//Set the negative sign bit if the node is internal
	int2 childIndices;
	childIndices.x = getIndexWithInternalNodeMarkerSet(isLeftChildLeaf, leftChildIndex);
	childIndices.y = getIndexWithInternalNodeMarkerSet(isRightChildLeaf, rightChildIndex);
	out_internalNodeChildIndices[internalNodeIndex] = childIndices;
	
	//Assign parent node index to children
	__global int* out_leftChildParentNodeIndices = (isLeftChildLeaf) ? out_leafNodeParentNodes : out_internalNodeParentNodes;
	out_leftChildParentNodeIndices[leftChildIndex] = internalNodeIndex;
	
	__global int* out_rightChildParentNodeIndices = (isRightChildLeaf) ? out_leafNodeParentNodes : out_internalNodeParentNodes;
	out_rightChildParentNodeIndices[rightChildIndex] = internalNodeIndex;
}

__kernel void determineInternalNodeAabbs(__global int* firstIndexOffsetPerLevel,
										__global int* numNodesPerLevel, 
										__global int2* internalNodeChildIndices,
										__global SortDataCL* mortonCodesAndAabbIndices,
										__global b3AabbCL* leafNodeAabbs, 
										__global int2* out_internalNodeLeafIndexRanges,
										__global b3AabbCL* out_internalNodeAabbs, 
										int numLevels, int numInternalNodes, int level)
{
	int i = get_global_id(0);
	if(i >= numInternalNodes) return;
	
	//For each node in a level, check its child nodes to determine its AABB
	{
		int indexInLevel = i;	//Index relative to firstIndexOffsetPerLevel[level]
		
		int numNodesInLevel = numNodesPerLevel[level];
		if(indexInLevel < numNodesInLevel)
		{
			int internalNodeIndexGlobal = indexInLevel + firstIndexOffsetPerLevel[level];
			int2 childIndicies = internalNodeChildIndices[internalNodeIndexGlobal];
			
			int leftChildIndex = getIndexWithInternalNodeMarkerRemoved(childIndicies.x);
			int rightChildIndex = getIndexWithInternalNodeMarkerRemoved(childIndicies.y);
		
			int isLeftChildLeaf = isLeafNode(childIndicies.x);
			int isRightChildLeaf = isLeafNode(childIndicies.y);
			
			//left/RightChildLeafIndex == Rigid body indicies
			int leftChildLeafIndex = (isLeftChildLeaf) ? mortonCodesAndAabbIndices[leftChildIndex].m_value : -1;
			int rightChildLeafIndex = (isRightChildLeaf) ? mortonCodesAndAabbIndices[rightChildIndex].m_value : -1;
			
			b3AabbCL leftChildAabb = (isLeftChildLeaf) ? leafNodeAabbs[leftChildLeafIndex] : out_internalNodeAabbs[leftChildIndex];
			b3AabbCL rightChildAabb = (isRightChildLeaf) ? leafNodeAabbs[rightChildLeafIndex] : out_internalNodeAabbs[rightChildIndex];
			
			//
			b3AabbCL internalNodeAabb;
			internalNodeAabb.m_min = b3Min(leftChildAabb.m_min, rightChildAabb.m_min);
			internalNodeAabb.m_max = b3Max(leftChildAabb.m_max, rightChildAabb.m_max);
			out_internalNodeAabbs[internalNodeIndexGlobal] = internalNodeAabb;
			
			//For index range, x == min and y == max; left child always has lower index
			int2 leafIndexRange;
			leafIndexRange.x = (isLeftChildLeaf) ? leftChildIndex : out_internalNodeLeafIndexRanges[leftChildIndex].x;
			leafIndexRange.y = (isRightChildLeaf) ? rightChildIndex : out_internalNodeLeafIndexRanges[rightChildIndex].y;
			
			out_internalNodeLeafIndexRanges[internalNodeIndexGlobal] = leafIndexRange;
		}
	}
}


//From sap.cl
#define NEW_PAIR_MARKER -1

bool TestAabbAgainstAabb2(const b3AabbCL* aabb1, const b3AabbCL* aabb2)
{
	bool overlap = true;
	overlap = (aabb1->m_min.x > aabb2->m_max.x || aabb1->m_max.x < aabb2->m_min.x) ? false : overlap;
	overlap = (aabb1->m_min.z > aabb2->m_max.z || aabb1->m_max.z < aabb2->m_min.z) ? false : overlap;
	overlap = (aabb1->m_min.y > aabb2->m_max.y || aabb1->m_max.y < aabb2->m_min.y) ? false : overlap;
	return overlap;
}
//From sap.cl

__kernel void plbvhCalculateOverlappingPairs(__global b3AabbCL* rigidAabbs, 

											__global int* rootNodeIndex, 
											__global int2* internalNodeChildIndices, 
											__global b3AabbCL* internalNodeAabbs,
											__global int2* internalNodeLeafIndexRanges,
											
											__global SortDataCL* mortonCodesAndAabbIndices,
											__global int* out_numPairs, __global int4* out_overlappingPairs, 
											int maxPairs, int numQueryAabbs)
{
#define USE_SPATIALLY_COHERENT_INDICIES		//mortonCodesAndAabbIndices[] contains rigid body indices sorted along the z-curve
#ifdef USE_SPATIALLY_COHERENT_INDICIES
	int queryRigidIndex = get_group_id(0) * get_local_size(0) + get_local_id(0);
	if(queryRigidIndex >= numQueryAabbs) return;
	
	int queryBvhNodeIndex = queryRigidIndex;
	queryRigidIndex = mortonCodesAndAabbIndices[queryRigidIndex].m_value;		//	fix queryRigidIndex naming for this branch
#else
	int queryRigidIndex = get_global_id(0);
	if(queryRigidIndex >= numQueryAabbs) return;
#endif

	b3AabbCL queryAabb = rigidAabbs[queryRigidIndex];
	
	int stack[B3_PLVBH_TRAVERSE_MAX_STACK_SIZE];
	
	int stackSize = 1;
	stack[0] = *rootNodeIndex;
	
	while(stackSize)
	{
		int internalOrLeafNodeIndex = stack[ stackSize - 1 ];
		--stackSize;
		
		int isLeaf = isLeafNode(internalOrLeafNodeIndex);	//Internal node if false
		int bvhNodeIndex = getIndexWithInternalNodeMarkerRemoved(internalOrLeafNodeIndex);
		
		//Optimization - if the node is not a leaf, check whether the highest leaf index of that node
		//is less than the queried node's index to avoid testing each pair twice.
		{
			//	fix: produces duplicate pairs
		//	int highestLeafIndex = (isLeaf) ? numQueryAabbs : internalNodeLeafIndexRanges[bvhNodeIndex].y;
		//	if(highestLeafIndex < queryBvhNodeIndex) continue;
		}
		
		//bvhRigidIndex is not used if internal node
		int bvhRigidIndex = (isLeaf) ? mortonCodesAndAabbIndices[bvhNodeIndex].m_value : -1;
	
		b3AabbCL bvhNodeAabb = (isLeaf) ? rigidAabbs[bvhRigidIndex] : internalNodeAabbs[bvhNodeIndex];
		if( queryRigidIndex != bvhRigidIndex && TestAabbAgainstAabb2(&queryAabb, &bvhNodeAabb) )
		{
			if(isLeaf && rigidAabbs[queryRigidIndex].m_minIndices[3] < rigidAabbs[bvhRigidIndex].m_minIndices[3])
			{
				int4 pair;
				pair.x = rigidAabbs[queryRigidIndex].m_minIndices[3];
				pair.y = rigidAabbs[bvhRigidIndex].m_minIndices[3];
				pair.z = NEW_PAIR_MARKER;
				pair.w = NEW_PAIR_MARKER;
				
				int pairIndex = atomic_inc(out_numPairs);
				if(pairIndex < maxPairs) out_overlappingPairs[pairIndex] = pair;
			}
			
			if(!isLeaf)	//Internal node
			{
				if(stackSize + 2 > B3_PLVBH_TRAVERSE_MAX_STACK_SIZE)
				{
					//Error
				}
				else
				{
					stack[ stackSize++ ] = internalNodeChildIndices[bvhNodeIndex].x;
					stack[ stackSize++ ] = internalNodeChildIndices[bvhNodeIndex].y;
				}
			}
		}
		
	}
}


//From rayCastKernels.cl
typedef struct
{
	float4 m_from;
	float4 m_to;
} b3RayInfo;
//From rayCastKernels.cl

b3Vector3 b3Vector3_normalize(b3Vector3 v)
{
	b3Vector3 normal = (b3Vector3){v.x, v.y, v.z, 0.f};
	return normalize(normal);	//OpenCL normalize == vector4 normalize
}
b3Scalar b3Vector3_length2(b3Vector3 v) { return v.x*v.x + v.y*v.y + v.z*v.z; }
b3Scalar b3Vector3_dot(b3Vector3 a, b3Vector3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

int rayIntersectsAabb(b3Vector3 rayOrigin, b3Scalar rayLength, b3Vector3 rayNormalizedDirection, b3AabbCL aabb)
{
	//AABB is considered as 3 pairs of 2 planes( {x_min, x_max}, {y_min, y_max}, {z_min, z_max} ).
	//t_min is the point of intersection with the closer plane, t_max is the point of intersection with the farther plane.
	//
	//if (rayNormalizedDirection.x < 0.0f), then max.x will be the near plane 
	//and min.x will be the far plane; otherwise, it is reversed.
	//
	//In order for there to be a collision, the t_min and t_max of each pair must overlap.
	//This can be tested for by selecting the highest t_min and lowest t_max and comparing them.
	
	int4 isNegative = isless( rayNormalizedDirection, (b3Vector3){0.0f, 0.0f, 0.0f, 0.0f} );	//isless(x,y) returns (x < y)
	
	//When using vector types, the select() function checks the most signficant bit, 
	//but isless() sets the least significant bit.
	isNegative <<= 31;

	//select(b, a, condition) == condition ? a : b
	//When using select() with vector types, (condition[i]) is true if its most significant bit is 1
	b3Vector3 t_min = ( select(aabb.m_min, aabb.m_max, isNegative) - rayOrigin ) / rayNormalizedDirection;
	b3Vector3 t_max = ( select(aabb.m_max, aabb.m_min, isNegative) - rayOrigin ) / rayNormalizedDirection;
	
	b3Scalar t_min_final = 0.0f;
	b3Scalar t_max_final = rayLength;
	
	//Must use fmin()/fmax(); if one of the parameters is NaN, then the parameter that is not NaN is returned. 
	//Behavior of min()/max() with NaNs is undefined. (See OpenCL Specification 1.2 [6.12.2] and [6.12.4])
	//Since the innermost fmin()/fmax() is always not NaN, this should never return NaN.
	t_min_final = fmax( t_min.z, fmax(t_min.y, fmax(t_min.x, t_min_final)) );
	t_max_final = fmin( t_max.z, fmin(t_max.y, fmin(t_max.x, t_max_final)) );
	
	return (t_min_final <= t_max_final);
}

__kernel void plbvhRayTraverse(__global b3AabbCL* rigidAabbs,

								__global int* rootNodeIndex, 
								__global int2* internalNodeChildIndices, 
								__global b3AabbCL* internalNodeAabbs,
								__global int2* internalNodeLeafIndexRanges,
								__global SortDataCL* mortonCodesAndAabbIndices,
								
								__global b3RayInfo* rays,
								
								__global int* out_numRayRigidPairs, 
								__global int2* out_rayRigidPairs,
								int maxRayRigidPairs, int numRays)
{
	int rayIndex = get_global_id(0);
	if(rayIndex >= numRays) return;
	
	//
	b3Vector3 rayFrom = rays[rayIndex].m_from;
	b3Vector3 rayTo = rays[rayIndex].m_to;
	b3Vector3 rayNormalizedDirection = b3Vector3_normalize(rayTo - rayFrom);
	b3Scalar rayLength = b3Sqrt( b3Vector3_length2(rayTo - rayFrom) );
	
	//
	int stack[B3_PLVBH_TRAVERSE_MAX_STACK_SIZE];
	
	int stackSize = 1;
	stack[0] = *rootNodeIndex;
	
	while(stackSize)
	{
		int internalOrLeafNodeIndex = stack[ stackSize - 1 ];
		--stackSize;
		
		int isLeaf = isLeafNode(internalOrLeafNodeIndex);	//Internal node if false
		int bvhNodeIndex = getIndexWithInternalNodeMarkerRemoved(internalOrLeafNodeIndex);
		
		//bvhRigidIndex is not used if internal node
		int bvhRigidIndex = (isLeaf) ? mortonCodesAndAabbIndices[bvhNodeIndex].m_value : -1;
	
		b3AabbCL bvhNodeAabb = (isLeaf) ? rigidAabbs[bvhRigidIndex] : internalNodeAabbs[bvhNodeIndex];
		if( rayIntersectsAabb(rayFrom, rayLength, rayNormalizedDirection, bvhNodeAabb)  )
		{
			if(isLeaf)
			{
				int2 rayRigidPair;
				rayRigidPair.x = rayIndex;
				rayRigidPair.y = rigidAabbs[bvhRigidIndex].m_minIndices[3];
				
				int pairIndex = atomic_inc(out_numRayRigidPairs);
				if(pairIndex < maxRayRigidPairs) out_rayRigidPairs[pairIndex] = rayRigidPair;
			}
			
			if(!isLeaf)	//Internal node
			{
				if(stackSize + 2 > B3_PLVBH_TRAVERSE_MAX_STACK_SIZE)
				{
					//Error
				}
				else
				{
					stack[ stackSize++ ] = internalNodeChildIndices[bvhNodeIndex].x;
					stack[ stackSize++ ] = internalNodeChildIndices[bvhNodeIndex].y;
				}
			}
		}
	}
}

__kernel void plbvhLargeAabbAabbTest(__global b3AabbCL* smallAabbs, __global b3AabbCL* largeAabbs, 
									__global int* out_numPairs, __global int4* out_overlappingPairs, 
									int maxPairs, int numLargeAabbRigids, int numSmallAabbRigids)
{
	int smallAabbIndex = get_global_id(0);
	if(smallAabbIndex >= numSmallAabbRigids) return;
	
	b3AabbCL smallAabb = smallAabbs[smallAabbIndex];
	for(int i = 0; i < numLargeAabbRigids; ++i)
	{
		b3AabbCL largeAabb = largeAabbs[i];
		if( TestAabbAgainstAabb2(&smallAabb, &largeAabb) )
		{
			int4 pair;
			pair.x = smallAabb.m_minIndices[3];
			pair.y = largeAabb.m_minIndices[3];
			pair.z = NEW_PAIR_MARKER;
			pair.w = NEW_PAIR_MARKER;
			
			int pairIndex = atomic_inc(out_numPairs);
			if(pairIndex < maxPairs) out_overlappingPairs[pairIndex] = pair;
		}
	}
}
__kernel void plbvhLargeAabbRayTest(__global b3AabbCL* largeRigidAabbs, __global b3RayInfo* rays,
									__global int* out_numRayRigidPairs,  __global int2* out_rayRigidPairs,
									int numLargeAabbRigids, int maxRayRigidPairs, int numRays)
{
	int rayIndex = get_global_id(0);
	if(rayIndex >= numRays) return;
	
	b3Vector3 rayFrom = rays[rayIndex].m_from;
	b3Vector3 rayTo = rays[rayIndex].m_to;
	b3Vector3 rayNormalizedDirection = b3Vector3_normalize(rayTo - rayFrom);
	b3Scalar rayLength = b3Sqrt( b3Vector3_length2(rayTo - rayFrom) );
	
	for(int i = 0; i < numLargeAabbRigids; ++i)
	{
		b3AabbCL rigidAabb = largeRigidAabbs[i];
		if( rayIntersectsAabb(rayFrom, rayLength, rayNormalizedDirection, rigidAabb) )
		{
			int2 rayRigidPair;
			rayRigidPair.x = rayIndex;
			rayRigidPair.y = rigidAabb.m_minIndices[3];
			
			int pairIndex = atomic_inc(out_numRayRigidPairs);
			if(pairIndex < maxRayRigidPairs) out_rayRigidPairs[pairIndex] = rayRigidPair;
		}
	}
}



#define B3_PLBVH_LINKED_LIST_INVALID_NODE -1

int longestCommonPrefix(int i, int j) { return clz(i ^ j); }

__kernel void computePrefixAndInitPointers(__global SortDataCL* mortonCodesAndAabbIndices,
											__global int* out_commonPrefixes,
											__global int* out_leftInternalNodePointers, 
											__global int* out_rightInternalNodePointers,
											int numInternalNodes)
{
	int internalNodeIndex = get_global_id(0);
	if (internalNodeIndex >= numInternalNodes) return;
	
	//Compute common prefix
	{
		//Here, (internalNodeIndex + 1) is never out of bounds since it is a leaf node index,
		//and the number of internal nodes is always numLeafNodes - 1
		int leftLeafMortonCode = mortonCodesAndAabbIndices[internalNodeIndex].m_key;
		int rightLeafMortonCode = mortonCodesAndAabbIndices[internalNodeIndex + 1].m_key;
	
		out_commonPrefixes[internalNodeIndex] = longestCommonPrefix(leftLeafMortonCode, rightLeafMortonCode);
	}
	
	//Assign neighbor pointers of this node
	{
		int leftInternalIndex = internalNodeIndex - 1;
		int rightInternalIndex = internalNodeIndex + 1;
		
		out_leftInternalNodePointers[internalNodeIndex] = (leftInternalIndex >= 0) ? leftInternalIndex : B3_PLBVH_LINKED_LIST_INVALID_NODE;
		out_rightInternalNodePointers[internalNodeIndex] = (rightInternalIndex < numInternalNodes) ? rightInternalIndex :  B3_PLBVH_LINKED_LIST_INVALID_NODE;
	}
}

__kernel void correctDuplicatePrefixes(__global int* commonPrefixes, __global int* out_maxCommonPrefix, int numInternalNodes)
{
	int internalNodeIndex = get_global_id(0);
	if (internalNodeIndex >= numInternalNodes) return;
	
	int commonPrefix = commonPrefixes[internalNodeIndex];
	
	//Linear search to find the size of the subtree
	int firstSubTreeIndex = internalNodeIndex;
	int lastSubTreeIndex = internalNodeIndex;
	
	while(firstSubTreeIndex - 1 >= 0 && commonPrefix == commonPrefixes[firstSubTreeIndex - 1]) --firstSubTreeIndex;
	while(lastSubTreeIndex + 1 < numInternalNodes && commonPrefix == commonPrefixes[lastSubTreeIndex + 1]) ++lastSubTreeIndex;
	
	//Fix duplicate common prefixes by incrementing them so that a subtree is formed.
	//Recursively divide the tree until the position of the split is this node's index.
	//Every time this node is not the split node, increment the common prefix.
	int isCurrentSplitNode = false;
	int correctedCommonPrefix = commonPrefix;
	
	while(!isCurrentSplitNode)
	{
		int numInternalNodesInSubTree = lastSubTreeIndex - firstSubTreeIndex + 1;
		int splitNodeIndex = firstSubTreeIndex + numInternalNodesInSubTree / 2;
	
		if(internalNodeIndex > splitNodeIndex) firstSubTreeIndex = splitNodeIndex + 1;
		else if(internalNodeIndex < splitNodeIndex) lastSubTreeIndex = splitNodeIndex - 1;
		//else if(internalNodeIndex == splitNodeIndex) break;
		
		isCurrentSplitNode = (internalNodeIndex == splitNodeIndex);
		if(!isCurrentSplitNode) correctedCommonPrefix++;
	}
	
	commonPrefixes[internalNodeIndex] = correctedCommonPrefix;
	atomic_max(out_maxCommonPrefix, correctedCommonPrefix);
}

//Set so that it is always greater than the actual common prefixes, and never selected as a parent node.
//If there are no duplicates, then the highest common prefix is 32 or 64, depending on the number of bits used for the z-curve.
//Duplicates common prefixes increase the highest common prefix by N, where 2^N is the number of duplicate nodes.
#define B3_PLBVH_INVALID_COMMON_PREFIX 128

__kernel void buildBinaryRadixTreeLeafNodes(__global int* commonPrefixes, __global int* out_leftChildNodes, 
											__global int* out_rightChildNodes, int numLeafNodes)
{
	int leafNodeIndex = get_global_id(0);
	if (leafNodeIndex >= numLeafNodes) return;
	
	int numInternalNodes = numLeafNodes - 1;
	
	int leftSplitIndex = leafNodeIndex - 1;
	int rightSplitIndex = leafNodeIndex;
	
	int leftCommonPrefix = (leftSplitIndex >= 0) ? commonPrefixes[leftSplitIndex] : B3_PLBVH_INVALID_COMMON_PREFIX;
	int rightCommonPrefix = (rightSplitIndex < numInternalNodes) ? commonPrefixes[rightSplitIndex] : B3_PLBVH_INVALID_COMMON_PREFIX;
	
	//Parent node is the highest adjacent common prefix that is lower than the node's common prefix
	//Leaf nodes are considered as having the highest common prefix
	int isLeftHigherCommonPrefix = (leftCommonPrefix > rightCommonPrefix);
	
	//Handle cases for the edge nodes; the first and last node
	//For leaf nodes, leftCommonPrefix and rightCommonPrefix should never both be B3_PLBVH_INVALID_COMMON_PREFIX
	if(leftCommonPrefix == B3_PLBVH_INVALID_COMMON_PREFIX) isLeftHigherCommonPrefix = false;
	if(rightCommonPrefix == B3_PLBVH_INVALID_COMMON_PREFIX) isLeftHigherCommonPrefix = true;
	
	int parentNodeIndex = (isLeftHigherCommonPrefix) ? leftSplitIndex : rightSplitIndex;
	
	//If the left node is the parent, then this node is its right child and vice versa
	__global int* out_childNode = (isLeftHigherCommonPrefix) ? out_rightChildNodes : out_leftChildNodes;
	
	int isLeaf = 1;
	out_childNode[parentNodeIndex] = getIndexWithInternalNodeMarkerSet(isLeaf, leafNodeIndex);
}

__kernel void buildBinaryRadixTreeInternalNodes(__global int* commonPrefixes, __global SortDataCL* mortonCodesAndAabbIndices,
												__global int* leftChildNodes, __global int* rightChildNodes,
												__global int* leftNeighborPointers, __global int* rightNeighborPointers,
												__global b3AabbCL* leafNodeAabbs, __global b3AabbCL* internalNodeAabbs,
												__global int* out_rootNodeIndex,
												int processedCommonPrefix, int numInternalNodes)
{
	int internalNodeIndex = get_global_id(0);
	if (internalNodeIndex >= numInternalNodes) return;
	
	int commonPrefix = commonPrefixes[internalNodeIndex];
	if (commonPrefix == processedCommonPrefix)
	{
		//Check neighbors and compare the common prefix to select the parent node
		int leftNodeIndex = leftNeighborPointers[internalNodeIndex];
		int rightNodeIndex = rightNeighborPointers[internalNodeIndex];
		
		int leftCommonPrefix = (leftNodeIndex != B3_PLBVH_LINKED_LIST_INVALID_NODE) ? commonPrefixes[leftNodeIndex] : B3_PLBVH_INVALID_COMMON_PREFIX;
		int rightCommonPrefix = (rightNodeIndex != B3_PLBVH_LINKED_LIST_INVALID_NODE) ? commonPrefixes[rightNodeIndex] : B3_PLBVH_INVALID_COMMON_PREFIX;
		
		//Parent node is the highest common prefix that is lower than the node's common prefix
		//Since the nodes with lower common prefixes are removed, that condition does not have to be tested for,
		//and we only need to pick the node with the higher prefix.
		int isLeftHigherCommonPrefix = (leftCommonPrefix > rightCommonPrefix);
		
		//
		if(leftCommonPrefix == B3_PLBVH_INVALID_COMMON_PREFIX) isLeftHigherCommonPrefix = false;
		else if(rightCommonPrefix == B3_PLBVH_INVALID_COMMON_PREFIX) isLeftHigherCommonPrefix = true;
		
		int isRootNode = false;
		if(leftCommonPrefix == B3_PLBVH_INVALID_COMMON_PREFIX && rightCommonPrefix == B3_PLBVH_INVALID_COMMON_PREFIX) isRootNode = true;
		
		int parentNodeIndex = (isLeftHigherCommonPrefix) ? leftNodeIndex : rightNodeIndex;
		
		//If the left node is the parent, then this node is its right child and vice versa
		__global int* out_childNode = (isLeftHigherCommonPrefix) ? rightChildNodes : leftChildNodes;
		
		int isLeaf = 0;
		if(!isRootNode) out_childNode[parentNodeIndex] = getIndexWithInternalNodeMarkerSet(isLeaf, internalNodeIndex);
		
		if(isRootNode) *out_rootNodeIndex = getIndexWithInternalNodeMarkerSet(isLeaf, internalNodeIndex);
		
		//Remove this node from the linked list, 
		//so that the left and right nodes point at each other instead of this node
		if(leftNodeIndex != B3_PLBVH_LINKED_LIST_INVALID_NODE) rightNeighborPointers[leftNodeIndex] = rightNodeIndex;
		if(rightNodeIndex != B3_PLBVH_LINKED_LIST_INVALID_NODE) leftNeighborPointers[rightNodeIndex] = leftNodeIndex;
		
		//For debug
		leftNeighborPointers[internalNodeIndex] = -2;
		rightNeighborPointers[internalNodeIndex] = -2;
	}
	
	//Processing occurs from highest common prefix to lowest common prefix
	//Nodes in the previously processed level have their children set, so we merge their child AABBs here
	if (commonPrefix == processedCommonPrefix + 1)
	{
		int leftChildIndex = leftChildNodes[internalNodeIndex];
		int rightChildIndex = rightChildNodes[internalNodeIndex];
		
		int isLeftChildLeaf = isLeafNode(leftChildIndex);
		int isRightChildLeaf = isLeafNode(rightChildIndex);
		
		leftChildIndex = getIndexWithInternalNodeMarkerRemoved(leftChildIndex);
		rightChildIndex = getIndexWithInternalNodeMarkerRemoved(rightChildIndex);
		
		//leftRigidIndex/rightRigidIndex is not used if internal node
		int leftRigidIndex = (isLeftChildLeaf) ? mortonCodesAndAabbIndices[leftChildIndex].m_value : -1;
		int rightRigidIndex = (isRightChildLeaf) ? mortonCodesAndAabbIndices[rightChildIndex].m_value : -1;
		
		b3AabbCL leftChildAabb = (isLeftChildLeaf) ? leafNodeAabbs[leftRigidIndex] : internalNodeAabbs[leftChildIndex];
		b3AabbCL rightChildAabb = (isRightChildLeaf) ? leafNodeAabbs[rightRigidIndex] : internalNodeAabbs[rightChildIndex];
		
		b3AabbCL mergedAabb;
		mergedAabb.m_min = b3Min(leftChildAabb.m_min, rightChildAabb.m_min);
		mergedAabb.m_max = b3Max(leftChildAabb.m_max, rightChildAabb.m_max);
		internalNodeAabbs[internalNodeIndex] = mergedAabb;
	}
}

__kernel void convertChildNodeFormat(__global int* leftChildNodes, __global int* rightChildNodes, 
									__global int2* out_childNodes, int numInternalNodes)
{
	int internalNodeIndex = get_global_id(0);
	if (internalNodeIndex >= numInternalNodes) return;
	
	int2 childNodesIndices;
	childNodesIndices.x = leftChildNodes[internalNodeIndex];
	childNodesIndices.y = rightChildNodes[internalNodeIndex];
	
	out_childNodes[internalNodeIndex] = childNodesIndices;
}


