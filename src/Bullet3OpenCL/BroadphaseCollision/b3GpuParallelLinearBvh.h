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

#ifndef B3_GPU_PARALLEL_LINEAR_BVH_H
#define B3_GPU_PARALLEL_LINEAR_BVH_H

//#include "Bullet3Collision/BroadPhaseCollision/shared/b3Aabb.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3SapAabb.h"
#include "Bullet3Common/shared/b3Int2.h"
#include "Bullet3Common/shared/b3Int4.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h"

#include "Bullet3OpenCL/ParallelPrimitives/b3FillCL.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.h"

#include "Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvhKernels.h"

#define B3_PLBVH_ROOT_NODE_MARKER -1	//Syncronize with parallelLinearBvh.cl

///@brief GPU Parallel Linearized Bounding Volume Heirarchy(LBVH) that is reconstructed every frame
///@remarks
///Main references: \n
///"Fast BVH Construction on GPUs" [Lauterbach et al. 2009] \n
///"Maximizing Parallelism in the Construction of BVHs, Octrees, and k-d trees" [Karras 2012] \n
///@par
///The basic algorithm for building the BVH as presented in [Lauterbach et al. 2009] consists of 4 stages:
/// - [fully parallel] Assign morton codes for each AABB using its center (after quantizing the AABB centers into a virtual grid) 
/// - [fully parallel] Sort morton codes
/// - [somewhat parallel] Build radix binary tree (assign parent/child pointers for internal nodes of the BVH) 
/// - [somewhat parallel] Set internal node AABBs 
///@par
///[Karras 2012] improves on the algorithm by introducing fully parallel methods for the last 2 stages.
///The BVH implementation here is almost the same as [Karras 2012], but a different method is used for constructing the tree.
/// - Instead of building a binary radix tree, we simply pair each node with its nearest sibling.
/// This has the effect of further worsening the quality of the BVH, but the main spatial partitioning is done by the
/// Z-curve anyways, and this method should be simpler and faster during construction.
/// - Rather than traveling upwards towards the root from the leaf nodes, as in the paper,
/// each internal node checks its child nodes to get its AABB.
class b3GpuParallelLinearBvh
{
	cl_command_queue m_queue;
	
	cl_program m_parallelLinearBvhProgram;
	
	cl_kernel m_separateAabbsKernel;
	cl_kernel m_findAllNodesMergedAabbKernel;
	cl_kernel m_assignMortonCodesAndAabbIndiciesKernel;
	
	//Simple binary tree construction kernels
	cl_kernel m_constructBinaryTreeKernel;
	cl_kernel m_determineInternalNodeAabbsKernel;
	
	//Radix binary tree construction kernels
	cl_kernel m_computePrefixAndInitPointersKernel;
	cl_kernel m_correctDuplicatePrefixesKernel;
	cl_kernel m_buildBinaryRadixTreeLeafNodesKernel;
	cl_kernel m_buildBinaryRadixTreeInternalNodesKernel;
	cl_kernel m_convertChildNodeFormatKernel;
	
	//Traversal kernels
	cl_kernel m_plbvhCalculateOverlappingPairsKernel;
	cl_kernel m_plbvhRayTraverseKernel;
	
	cl_kernel m_plbvhLargeAabbAabbTestKernel;
	cl_kernel m_plbvhLargeAabbRayTestKernel;

	b3FillCL m_fill;
	b3RadixSort32CL m_radixSorter;
	
	//
	b3OpenCLArray<int> m_rootNodeIndex;
	
	//1 element per level in the tree
	b3AlignedObjectArray<int> m_numNodesPerLevelCpu;			//Level 0(m_numNodesPerLevelCpu[0]) is the root, last level contains the leaf nodes
	b3AlignedObjectArray<int> m_firstIndexOffsetPerLevelCpu;	//Contains the index/offset of the first node in that level
	b3OpenCLArray<int> m_numNodesPerLevelGpu;
	b3OpenCLArray<int> m_firstIndexOffsetPerLevelGpu;
	
	//1 element per internal node (number_of_internal_nodes = number_of_leaves - 1)
	b3OpenCLArray<b3SapAabb> m_internalNodeAabbs;
	b3OpenCLArray<b3Int2> m_internalNodeLeafIndexRanges;		//x == min leaf index, y == max leaf index
	b3OpenCLArray<b3Int2> m_internalNodeChildNodes;				//x == left child, y == right child
	b3OpenCLArray<int> m_internalNodeParentNodes;
	
	//1 element per internal node; for radix binary tree construction
	b3OpenCLArray<int> m_maxCommonPrefix;
	b3OpenCLArray<int> m_commonPrefixes;
	b3OpenCLArray<int> m_leftInternalNodePointers;				//Linked list
	b3OpenCLArray<int> m_rightInternalNodePointers;				//Linked list
	b3OpenCLArray<int> m_internalNodeLeftChildNodes;
	b3OpenCLArray<int> m_internalNodeRightChildNodes;
	
	//1 element per leaf node (leaf nodes only include small AABBs)
	b3OpenCLArray<int> m_leafNodeParentNodes;
	b3OpenCLArray<b3SortData> m_mortonCodesAndAabbIndicies;		//m_key = morton code, m_value == aabb index
	b3OpenCLArray<b3SapAabb> m_mergedAabb;
	b3OpenCLArray<b3SapAabb> m_leafNodeAabbs;					//Contains only small AABBs
	
	//1 element per large AABB
	b3OpenCLArray<b3SapAabb> m_largeAabbs;	//Not stored in the BVH
	
public:
	b3GpuParallelLinearBvh(cl_context context, cl_device_id device, cl_command_queue queue);
	virtual ~b3GpuParallelLinearBvh();
	
	void build(const b3OpenCLArray<b3SapAabb>& worldSpaceAabbs, const b3OpenCLArray<int>& smallAabbIndices, 
				const b3OpenCLArray<int>& largeAabbIndices);
	
	///b3GpuParallelLinearBvh::build() must be called before this function. calculateOverlappingPairs() uses
	///the worldSpaceAabbs parameter of b3GpuParallelLinearBvh::build() as the query AABBs.
	///@param out_numPairs If number of pairs exceeds the max number of pairs, this is clamped to the max number.
	///@param out_overlappingPairs The size() of this array is used to determine the max number of pairs.
	///If the number of overlapping pairs is < out_overlappingPairs.size(), out_overlappingPairs is resized.
	void calculateOverlappingPairs(b3OpenCLArray<int>& out_numPairs, b3OpenCLArray<b3Int4>& out_overlappingPairs);
	
	///@param out_numRigidRayPairs Array of length 1; contains the number of detected ray-rigid AABB intersections;
	///this value may be greater than out_rayRigidPairs.size() if out_rayRigidPairs is not large enough.
	///@param out_rayRigidPairs Contains an array of rays intersecting rigid AABBs; x == ray index, y == rigid body index.
	///If the size of this array is insufficient to hold all ray-rigid AABB intersections, additional intersections are discarded.
	void testRaysAgainstBvhAabbs(const b3OpenCLArray<b3RayInfo>& rays, 
								b3OpenCLArray<int>& out_numRayRigidPairs, b3OpenCLArray<b3Int2>& out_rayRigidPairs);
								
private:
	void constructSimpleBinaryTree();
	
	void constructRadixBinaryTree();
};

#endif
