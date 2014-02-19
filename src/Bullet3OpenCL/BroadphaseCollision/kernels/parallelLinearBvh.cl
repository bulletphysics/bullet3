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

/*
__kernel void findAllNodesMergedAabb(__global b3AabbCL* worldSpaceAabbs, __global b3AabbCL* out_mergedAabb, int numAabbs)
{
	int aabbIndex = get_global_id(0);
	if(aabbIndex >= numAabbs) return;
	
	//Find the most significant bit(msb)
	int mostSignificantBit = 0;
	{
		int temp = numLeaves;
		while(temp >>= 1) mostSignificantBit++;		//Start counting from 0 (0 and 1 have msb 0, 2 has msb 1)
	}
	
	int numberOfAabbsAboveMsbSplit = numAabbs & ~( ~(0) << mostSignificantBit );	//	verify
	int numRemainingAabbs = (1 << mostSignificantBit);
	
	//Merge AABBs above most significant bit so that the number of remaining AABBs is a power of 2
	//For example, if there are 159 AABBs = 128 + 31, then merge indices [0, 30] and 128 + [0, 30]
	if(aabbIndex < numberOfAabbsAboveMsbSplit)
	{
		int otherAabbIndex = numRemainingAabbs + aabbIndex;
		
		b3AabbCL aabb = worldSpaceAabbs[aabbIndex];
		b3AabbCL otherAabb = worldSpaceAabbs[otherAabbIndex];
		
		b3AabbCL mergedAabb;
		mergedAabb.m_min = b3Min(aabb.m_min, otherAabb.m_min);
		mergedAabb.m_max = b3Max(aabb.m_max, otherAabb.m_max);
		out_mergedAabb[aabbIndex] = mergedAabb;
	}
	
	barrier(CLK_GLOBAL_MEM_FENCE);
	
	//
	int offset = numRemainingAabbs / 2;
	while(offset >= 1)
	{
		if(aabbIndex < offset)
		{
			int otherAabbIndex = aabbIndex + offset;
		
			b3AabbCL aabb = worldSpaceAabbs[aabbIndex];
			b3AabbCL otherAabb = worldSpaceAabbs[otherAabbIndex];
		
			b3AabbCL mergedAabb;
			mergedAabb.m_min = b3Min(aabb.m_min, otherAabb.m_min);
			mergedAabb.m_max = b3Max(aabb.m_max, otherAabb.m_max);
			out_mergedAabb[aabbIndex] = mergedAabb;
		}
		
		offset = offset / 2;
		
		barrier(CLK_GLOBAL_MEM_FENCE);
	}
}
*/

__kernel void assignMortonCodesAndAabbIndicies(__global b3AabbCL* worldSpaceAabbs, 
												__global SortDataCL* out_mortonCodesAndAabbIndices, 
												b3Scalar cellSize, int numAabbs)
{
	int leafNodeIndex = get_global_id(0);	//Leaf node index == AABB index
	if(leafNodeIndex >= numAabbs) return;
	
	b3AabbCL aabb = worldSpaceAabbs[leafNodeIndex];
	
	b3Vector3 center = (aabb.m_min + aabb.m_max) * 0.5f;
	
	//Quantize into integer coordinates
	//floor() is needed to prevent the center cell, at (0,0,0) from being twice the size
	b3Vector3 gridPosition = center / cellSize;
	
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
#define B3_PLBVH_ROOT_NODE_MARKER -1	//Used to indicate that the (root) node has no parent 
#define B3_PLBVH_ROOT_NODE_INDEX 0

//For elements of internalNodeChildIndices(int2), the negative bit determines whether it is a leaf or internal node.
//Positive index == leaf node, while negative index == internal node (remove negative sign to get index).
//
//Since the root internal node is at index 0, no internal nodes should reference it as a child,
//and so index 0 is always used to indicate a leaf node.
int isLeafNode(int index) { return (index >= 0); }
int getIndexWithInternalNodeMarkerRemoved(int index) { return (index >= 0) ? index : -index; }
int getIndexWithInternalNodeMarkerSet(int isLeaf, int index) { return (isLeaf) ? index : -index; }

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
		rightChildIndex = (isLeftChildLeaf) ? rightChildIndex - firstIndexOffsetPerLevel[leafNodeLevel] : rightChildIndex;
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
										__global b3AabbCL* out_internalNodeAabbs, int numLevels, int numInternalNodes)
{
	int i = get_global_id(0);
	if(i >= numInternalNodes) return;
	
	int numInternalLevels = numLevels - 1;
	
	//Starting from the level next to the leaf nodes, move towards the root(level 0)
	for(int level = numInternalLevels - 1; level >= 0; --level)
	{
		int indexInLevel = i;	//Index relative to firstIndexOffsetPerLevel[level]
		
		int numNodesInLevel = numNodesPerLevel[level];
		if(i < numNodesInLevel)
		{
			int internalNodeIndexGlobal = indexInLevel + firstIndexOffsetPerLevel[level];
			int2 childIndicies = internalNodeChildIndices[internalNodeIndexGlobal];
			
			int leftChildIndex = getIndexWithInternalNodeMarkerRemoved(childIndicies.x);
			int rightChildIndex = getIndexWithInternalNodeMarkerRemoved(childIndicies.y);
		
			int isLeftChildLeaf = isLeafNode(childIndicies.x);
			int isRightChildLeaf = isLeafNode(childIndicies.y);
			
			int leftChildLeafIndex = (isLeftChildLeaf) ? mortonCodesAndAabbIndices[leftChildIndex].m_value : -1;
			int rightChildLeafIndex = (isRightChildLeaf) ? mortonCodesAndAabbIndices[rightChildIndex].m_value : -1;
			
			b3AabbCL leftChildAabb = (isLeftChildLeaf) ? leafNodeAabbs[leftChildLeafIndex] : out_internalNodeAabbs[leftChildIndex];
			b3AabbCL rightChildAabb = (isRightChildLeaf) ? leafNodeAabbs[rightChildLeafIndex] : out_internalNodeAabbs[rightChildIndex];
			
			b3AabbCL internalNodeAabb;
			internalNodeAabb.m_min = b3Min(leftChildAabb.m_min, rightChildAabb.m_min);
			internalNodeAabb.m_max = b3Max(leftChildAabb.m_max, rightChildAabb.m_max);
			out_internalNodeAabbs[internalNodeIndexGlobal] = internalNodeAabb;
		}
	
		barrier(CLK_GLOBAL_MEM_FENCE);
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
											__global int2* internalNodeChildIndices, __global b3AabbCL* internalNodeAabbs,
											__global SortDataCL* mortonCodesAndAabbIndices,
											__global int* out_numPairs, __global int4* out_overlappingPairs, 
											int maxPairs, int numQueryAabbs)
{
#define USE_SPATIALLY_COHERENT_INDICIES		//mortonCodesAndAabbIndices[] contains rigid body indices sorted along the z-curve
#ifdef USE_SPATIALLY_COHERENT_INDICIES
	int queryRigidIndex = get_group_id(0) * get_local_size(0) + get_local_id(0);
	if(queryRigidIndex >= numQueryAabbs) return;
	
	queryRigidIndex = mortonCodesAndAabbIndices[queryRigidIndex].m_value;
#else
	int queryRigidIndex = get_global_id(0);
	if(queryRigidIndex >= numQueryAabbs) return;
#endif

	b3AabbCL queryAabb = rigidAabbs[queryRigidIndex];
	
	int stack[B3_PLVBH_TRAVERSE_MAX_STACK_SIZE];
	
	//Starting by placing only the root node index, 0, in the stack causes it to be detected as a leaf node(see isLeafNode() in loop)
	int stackSize = 2;
	stack[0] = internalNodeChildIndices[B3_PLBVH_ROOT_NODE_INDEX].x;
	stack[1] = internalNodeChildIndices[B3_PLBVH_ROOT_NODE_INDEX].y;
	
	while(stackSize)
	{
		int internalOrLeafNodeIndex = stack[ stackSize - 1 ];
		--stackSize;
		
		int isLeaf = isLeafNode(internalOrLeafNodeIndex);	//Internal node if false
		int bvhNodeIndex = getIndexWithInternalNodeMarkerRemoved(internalOrLeafNodeIndex);
	
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

