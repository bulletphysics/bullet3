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

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
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
/// Z-curve anyways, and this method should be simpler and faster during construction. Additionally, it is still possible
/// to improve the quality of the BVH by rearranging the connections between nodes.
/// - Due to the way the tree is constructed, it becomes unnecessary to use atomic_inc to get
/// the AABB for each internal node. Rather than traveling upwards from the leaf nodes, as in the paper,
/// each internal node checks its child nodes to get its AABB.
class b3GpuParallelLinearBvh
{
	cl_command_queue m_queue;
	
	cl_program m_parallelLinearBvhProgram;
	
	cl_kernel m_assignMortonCodesAndAabbIndiciesKernel;
	cl_kernel m_constructBinaryTreeKernel;
	cl_kernel m_determineInternalNodeAabbsKernel;
	
	cl_kernel m_plbvhCalculateOverlappingPairsKernel;

	b3FillCL m_fill;
	b3RadixSort32CL m_radixSorter;
	
	//1 element per level in the tree
	b3AlignedObjectArray<int> m_numNodesPerLevelCpu;			//Level 0(m_numNodesPerLevelCpu[0]) is the root, last level contains the leaf nodes
	b3AlignedObjectArray<int> m_firstIndexOffsetPerLevelCpu;	//Contains the index/offset of the first node in that level
	b3OpenCLArray<int> m_numNodesPerLevelGpu;
	b3OpenCLArray<int> m_firstIndexOffsetPerLevelGpu;
	
	//1 element per internal node (number_of_internal_nodes = number_of_leaves - 1); index 0 is the root node
	b3OpenCLArray<b3SapAabb> m_internalNodeAabbs;
	b3OpenCLArray<b3Int2> m_internalNodeChildNodes;				//x == left child, y == right child
	b3OpenCLArray<int> m_internalNodeParentNodes;
	
	//1 element per leaf node
	b3OpenCLArray<int> m_leafNodeParentNodes;
	b3OpenCLArray<b3SortData> m_mortonCodesAndAabbIndicies;		//m_key = morton code, m_value == aabb index
	
public:
	b3GpuParallelLinearBvh(cl_context context, cl_device_id device, cl_command_queue queue) :
		m_queue(queue),
		m_fill(context, device, queue),
		m_radixSorter(context, device, queue),
		
		m_numNodesPerLevelGpu(context, queue),
		m_firstIndexOffsetPerLevelGpu(context, queue),
		m_internalNodeAabbs(context, queue),
		m_internalNodeChildNodes(context, queue),
		m_internalNodeParentNodes(context, queue),
		m_leafNodeParentNodes(context, queue),
		m_mortonCodesAndAabbIndicies(context, queue)
	{
		const char CL_PROGRAM_PATH[] = "src/Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvh.cl";
		
		const char* kernelSource = parallelLinearBvhCL;	//parallelLinearBvhCL.h
		cl_int error;
		char* additionalMacros = 0;
		m_parallelLinearBvhProgram = b3OpenCLUtils::compileCLProgramFromString(context, device, kernelSource, &error, additionalMacros, CL_PROGRAM_PATH);
		b3Assert(m_parallelLinearBvhProgram);
		
		m_assignMortonCodesAndAabbIndiciesKernel = b3OpenCLUtils::compileCLKernelFromString( context, device, kernelSource, "assignMortonCodesAndAabbIndicies", &error, m_parallelLinearBvhProgram, additionalMacros );
		b3Assert(m_assignMortonCodesAndAabbIndiciesKernel);
		m_constructBinaryTreeKernel = b3OpenCLUtils::compileCLKernelFromString( context, device, kernelSource, "constructBinaryTree", &error, m_parallelLinearBvhProgram, additionalMacros );
		b3Assert(m_constructBinaryTreeKernel);
		m_determineInternalNodeAabbsKernel = b3OpenCLUtils::compileCLKernelFromString( context, device, kernelSource, "determineInternalNodeAabbs", &error, m_parallelLinearBvhProgram, additionalMacros );
		b3Assert(m_determineInternalNodeAabbsKernel);
		
		m_plbvhCalculateOverlappingPairsKernel = b3OpenCLUtils::compileCLKernelFromString( context, device, kernelSource, "plbvhCalculateOverlappingPairs", &error, m_parallelLinearBvhProgram, additionalMacros );
		b3Assert(m_plbvhCalculateOverlappingPairsKernel);
	}
	
	virtual ~b3GpuParallelLinearBvh() 
	{
		clReleaseKernel(m_assignMortonCodesAndAabbIndiciesKernel);
		clReleaseKernel(m_constructBinaryTreeKernel);
		clReleaseKernel(m_determineInternalNodeAabbsKernel);
		
		clReleaseKernel(m_plbvhCalculateOverlappingPairsKernel);
		
		clReleaseProgram(m_parallelLinearBvhProgram);
	}

	//	fix: need to handle/test case with 2 nodes
	
	///@param cellsize A virtual grid of size 2^10^3 is used in the process of creating the BVH
	void build(const b3OpenCLArray<b3SapAabb>& worldSpaceAabbs, b3Scalar cellSize)
	{
		B3_PROFILE("b3ParallelLinearBvh::build()");
	
		int numLeaves = worldSpaceAabbs.size();	//Number of leaves in the BVH == Number of rigid body AABBs
		int numInternalNodes = numLeaves - 1;
	
		if(numLeaves < 2) return;
	
		//
		{
			m_internalNodeAabbs.resize(numInternalNodes);
			m_internalNodeChildNodes.resize(numInternalNodes);
			m_internalNodeParentNodes.resize(numInternalNodes);
	
			m_leafNodeParentNodes.resize(numLeaves);
			m_mortonCodesAndAabbIndicies.resize(numLeaves);
		}
		
		//Determine number of levels in the binary tree( numLevels = ceil( log2(numLeaves) ) )
		//The number of levels is equivalent to the number of bits needed to uniquely identify each node(including both internal and leaf nodes)
		int numLevels = 0;
		{
			//Find the most significant bit(msb)
			int mostSignificantBit = 0;
			{
				int temp = numLeaves;
				while(temp >>= 1) mostSignificantBit++;		//Start counting from 0 (0 and 1 have msb 0, 2 has msb 1)
			}
			numLevels = mostSignificantBit + 1;
			
			//If the number of nodes is not a power of 2(as in, can be expressed as 2^N where N is an integer), then there is 1 additional level
			if( ~(1 << mostSignificantBit) & numLeaves ) numLevels++;
			
			if(1) printf("numLeaves, numLevels, mostSignificantBit: %d, %d, %d \n", numLeaves, numLevels, mostSignificantBit);
		}
		
		//Determine number of nodes per level, use prefix sum to get offsets of each level, and send to GPU
		{
			B3_PROFILE("Determine number of nodes per level");
			
			m_numNodesPerLevelCpu.resize(numLevels);
			
			//The last level contains the leaf nodes; number of leaves is already known
			if(numLevels - 1 >= 0) m_numNodesPerLevelCpu[numLevels - 1] = numLeaves;
			
			//Calculate number of nodes in each level; 
			//start from the second to last level(level right next to leaf nodes) and move towards the root(level 0)
			int hasRemainder = 0;
			for(int levelIndex = numLevels - 2; levelIndex >= 0; --levelIndex)
			{
				int numNodesPreviousLevel = m_numNodesPerLevelCpu[levelIndex + 1];		//For first iteration this == numLeaves
			
				bool allNodesAllocated = ( (numNodesPreviousLevel + hasRemainder) % 2 == 0 );
			
				int numNodesCurrentLevel = (allNodesAllocated) ? (numNodesPreviousLevel + hasRemainder) / 2 : numNodesPreviousLevel / 2;
				m_numNodesPerLevelCpu[levelIndex] = numNodesCurrentLevel;
				
				hasRemainder = static_cast<int>(!allNodesAllocated);
			}
			
			//Prefix sum to calculate the first index offset of each level
			{
				m_firstIndexOffsetPerLevelCpu = m_numNodesPerLevelCpu;
				
				//Perform inclusive scan
				for(int i = 1; i < m_firstIndexOffsetPerLevelCpu.size(); ++i) 
					m_firstIndexOffsetPerLevelCpu[i] += m_firstIndexOffsetPerLevelCpu[i - 1];
				
				//Convert inclusive scan to exclusive scan to get the offsets
				//This is equivalent to shifting each element in m_firstIndexOffsetPerLevelCpu[] by 1 to the right, 
				//and setting the first element to 0
				for(int i = 0; i < m_firstIndexOffsetPerLevelCpu.size(); ++i) 
					m_firstIndexOffsetPerLevelCpu[i] -= m_numNodesPerLevelCpu[i];
			}
			
			if(1)
			{
				int numInternalNodes = 0;
				for(int i = 0; i < numLevels; ++i) 
					if(i < numLevels - 1) numInternalNodes += m_numNodesPerLevelCpu[i];
				printf("numInternalNodes: %d\n", numInternalNodes);
				
				for(int i = 0; i < numLevels; ++i) 
					printf("numNodes, offset[%d]: %d, %d \n", i, m_numNodesPerLevelCpu[i], m_firstIndexOffsetPerLevelCpu[i]);
				printf("\n");
			}
			
			//Copy to GPU
			m_numNodesPerLevelGpu.copyFromHost(m_numNodesPerLevelCpu, false);
			m_firstIndexOffsetPerLevelGpu.copyFromHost(m_firstIndexOffsetPerLevelCpu, false);
			clFinish(m_queue);
		}
		
		//Find the AABB of all input AABBs; this is used to define the size of 
		//each cell in the virtual grid(2^10 cells in each dimension).
		{
			B3_PROFILE("Find AABB of merged nodes");
		
			/*b3BufferInfoCL bufferInfo[] = 
			{
				b3BufferInfoCL( worldSpaceAabbs.getBufferCL() ),
				b3BufferInfoCL( m_allNodesMergedAabb.getBufferCL() ),
			};
			
			b3LauncherCL launcher(m_queue, m_findAllNodesMergedAabb, "m_findAllNodesMergedAabb");
			launcher.setBuffers( bufferInfo, sizeof(bufferInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst(numLeaves);
			
			launcher.launch1D(numLeaves);
			clFinish(m_queue);*/
		}
		
		//Insert the center of the AABBs into a virtual grid,
		//then convert the discrete grid coordinates into a morton code
		//For each element in m_mortonCodesAndAabbIndicies, set
		//	m_key == morton code (value to sort by)
		//	m_value = AABB index
		{
			B3_PROFILE("Assign morton codes");
		
			b3BufferInfoCL bufferInfo[] = 
			{
				b3BufferInfoCL( worldSpaceAabbs.getBufferCL() ),
				b3BufferInfoCL( m_mortonCodesAndAabbIndicies.getBufferCL() )
			};
			
			b3LauncherCL launcher(m_queue, m_assignMortonCodesAndAabbIndiciesKernel, "m_assignMortonCodesAndAabbIndiciesKernel");
			launcher.setBuffers( bufferInfo, sizeof(bufferInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst(cellSize);
			launcher.setConst(numLeaves);
			
			launcher.launch1D(numLeaves);
			clFinish(m_queue);
		}
		
		//
		{
			B3_PROFILE("Sort leaves by morton codes");
		
			m_radixSorter.execute(m_mortonCodesAndAabbIndicies);
			clFinish(m_queue);
		}
		
		//Optional; only element at m_internalNodeParentNodes[0], the root node, needs to be set here
		//as the parent indices of other nodes are overwritten during m_constructBinaryTreeKernel
		{
			B3_PROFILE("Reset parent node indices");
			
			m_fill.execute( m_internalNodeParentNodes, B3_PLBVH_ROOT_NODE_MARKER, m_internalNodeParentNodes.size() );
			m_fill.execute( m_leafNodeParentNodes, B3_PLBVH_ROOT_NODE_MARKER, m_leafNodeParentNodes.size() );
			clFinish(m_queue);
		}
		
		//Construct binary tree; find the children of each internal node, and assign parent nodes
		{
			B3_PROFILE("Construct binary tree");
		
			b3BufferInfoCL bufferInfo[] = 
			{
				b3BufferInfoCL( m_firstIndexOffsetPerLevelGpu.getBufferCL() ),
				b3BufferInfoCL( m_numNodesPerLevelGpu.getBufferCL() ),
				b3BufferInfoCL( m_internalNodeChildNodes.getBufferCL() ),
				b3BufferInfoCL( m_internalNodeParentNodes.getBufferCL() ),
				b3BufferInfoCL( m_leafNodeParentNodes.getBufferCL() )
			};
			
			b3LauncherCL launcher(m_queue, m_constructBinaryTreeKernel, "m_constructBinaryTreeKernel");
			launcher.setBuffers( bufferInfo, sizeof(bufferInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst(numLevels);
			launcher.setConst(numInternalNodes);
			
			launcher.launch1D(numInternalNodes);
			clFinish(m_queue);
			
			if(1)
			{
				static b3AlignedObjectArray<b3Int2> internalNodeChildNodes;	
				m_internalNodeChildNodes.copyToHost(internalNodeChildNodes, false);
				clFinish(m_queue);
				
				for(int i = 0; i < 256; ++i) printf("ch[%d]: %d, %d\n", i, internalNodeChildNodes[i].x, internalNodeChildNodes[i].y);
				printf("\n");
			}
		}
		
		//For each internal node, check children to get its AABB; start from the 
		//last level and move towards the root
		{
			B3_PROFILE("Set AABBs");
		
			b3BufferInfoCL bufferInfo[] = 
			{
				b3BufferInfoCL( m_firstIndexOffsetPerLevelGpu.getBufferCL() ),
				b3BufferInfoCL( m_numNodesPerLevelGpu.getBufferCL() ),
				b3BufferInfoCL( m_internalNodeChildNodes.getBufferCL() ),
				b3BufferInfoCL( m_mortonCodesAndAabbIndicies.getBufferCL() ),
				b3BufferInfoCL( worldSpaceAabbs.getBufferCL() ),
				b3BufferInfoCL( m_internalNodeAabbs.getBufferCL() )
			};
			
			b3LauncherCL launcher(m_queue, m_determineInternalNodeAabbsKernel, "m_determineInternalNodeAabbsKernel");
			launcher.setBuffers( bufferInfo, sizeof(bufferInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst(numLevels);
			launcher.setConst(numInternalNodes);
			
			launcher.launch1D(numLeaves);
			clFinish(m_queue);
			
			if(0)
			{
				static b3AlignedObjectArray<b3SapAabb> rigidAabbs;
				worldSpaceAabbs.copyToHost(rigidAabbs, false);
				clFinish(m_queue);
			
				b3SapAabb actualRootAabb;
				actualRootAabb.m_minVec = b3MakeVector3(B3_LARGE_FLOAT, B3_LARGE_FLOAT, B3_LARGE_FLOAT);
				actualRootAabb.m_maxVec = b3MakeVector3(-B3_LARGE_FLOAT, -B3_LARGE_FLOAT, -B3_LARGE_FLOAT);
				for(int i = 0; i < rigidAabbs.size(); ++i)
				{
					actualRootAabb.m_minVec.setMin(rigidAabbs[i].m_minVec);
					actualRootAabb.m_maxVec.setMax(rigidAabbs[i].m_maxVec);
				}
				printf("actualRootMin: %f, %f, %f \n", actualRootAabb.m_minVec.x, actualRootAabb.m_minVec.y, actualRootAabb.m_minVec.z);
				printf("actualRootMax: %f, %f, %f \n", actualRootAabb.m_maxVec.x, actualRootAabb.m_maxVec.y, actualRootAabb.m_maxVec.z);
			
				b3SapAabb rootAabb = m_internalNodeAabbs.at(0);
				printf("rootMin: %f, %f, %f \n", rootAabb.m_minVec.x, rootAabb.m_minVec.y, rootAabb.m_minVec.z);
				printf("rootMax: %f, %f, %f \n", rootAabb.m_maxVec.x, rootAabb.m_maxVec.y, rootAabb.m_maxVec.z);
				printf("\n");
			}
		}
	}
	
	//Max number of pairs is out_overlappingPairs.size()
	//If the number of overlapping pairs is < out_overlappingPairs.size(), the array is resized
	void calculateOverlappingPairs(const b3OpenCLArray<b3SapAabb>& worldSpaceAabbs, 
									b3OpenCLArray<int>& out_numPairs, b3OpenCLArray<b3Int4>& out_overlappingPairs)
	{
		b3Assert( out_numPairs.size() == 1 );
		
		int maxPairs = out_overlappingPairs.size();
			
		int reset = 0;
		out_numPairs.copyFromHostPointer(&reset, 1);
		
		{
			B3_PROFILE("PLBVH calculateOverlappingPairs");
		
			int numQueryAabbs = worldSpaceAabbs.size();
			
			b3BufferInfoCL bufferInfo[] = 
			{
				b3BufferInfoCL( worldSpaceAabbs.getBufferCL() ),
				
				b3BufferInfoCL( m_internalNodeChildNodes.getBufferCL() ),
				b3BufferInfoCL( m_internalNodeAabbs.getBufferCL() ),
				b3BufferInfoCL( m_mortonCodesAndAabbIndicies.getBufferCL() ),
				
				b3BufferInfoCL( out_numPairs.getBufferCL() ),
				b3BufferInfoCL( out_overlappingPairs.getBufferCL() )
			};
			
			b3LauncherCL launcher(m_queue, m_plbvhCalculateOverlappingPairsKernel, "m_plbvhCalculateOverlappingPairsKernel");
			launcher.setBuffers( bufferInfo, sizeof(bufferInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst(maxPairs);
			launcher.setConst(numQueryAabbs);
			
			launcher.launch1D(numQueryAabbs);
			clFinish(m_queue);
		}
		
		
		//
		int numPairs = -1;
		out_numPairs.copyToHostPointer(&numPairs, 1);
		if(numPairs > maxPairs)
		{
			b3Error("Error running out of pairs: numPairs = %d, maxPairs = %d.\n", numPairs, maxPairs);
			numPairs = maxPairs;
			out_numPairs.copyFromHostPointer(&maxPairs, 1);
		}
		
		out_overlappingPairs.resize(numPairs);
	}
};

#endif
