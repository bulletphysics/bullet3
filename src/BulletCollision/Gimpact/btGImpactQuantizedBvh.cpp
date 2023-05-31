/*! \file gim_box_set.h
\author Francisco Leon Najera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGImpactQuantizedBvh.h"
#include "LinearMath/btQuickprof.h"

#include <stack>
#include <tuple>

//#include "cvmarkersobj.h"


//using namespace Concurrency;

#ifdef TRI_COLLISION_PROFILING
btClock g_q_tree_clock;

float g_q_accum_tree_collision_time = 0;
int g_q_count_traversing = 0;

void bt_begin_gim02_q_tree_time()
{
	g_q_tree_clock.reset();
}

void bt_end_gim02_q_tree_time()
{
	g_q_accum_tree_collision_time += g_q_tree_clock.getTimeMicroseconds();
	g_q_count_traversing++;
}

//! Gets the average time in miliseconds of tree collisions
float btGImpactQuantizedBvh::getAverageTreeCollisionTime()
{
	if (g_q_count_traversing == 0) return 0;

	float avgtime = g_q_accum_tree_collision_time;
	avgtime /= (float)g_q_count_traversing;

	g_q_accum_tree_collision_time = 0;
	g_q_count_traversing = 0;
	return avgtime;

	//	float avgtime = g_q_count_traversing;
	//	g_q_count_traversing = 0;
	//	return avgtime;
}

#endif  //TRI_COLLISION_PROFILING

/////////////////////// btQuantizedBvhTree /////////////////////////////////

void btQuantizedBvhTree::calc_quantization(
	GIM_BVH_DATA_ARRAY& primitive_boxes, btScalar boundMargin)
{
	//calc globa box
	btAABB global_bound;
	global_bound.invalidate();

	for (int i = 0; i < primitive_boxes.size(); i++)
	{
		global_bound.merge(primitive_boxes[i].m_bound);
	}

	bt_calc_quantization_parameters(
		m_global_bound.m_min, m_global_bound.m_max, m_bvhQuantization, global_bound.m_min, global_bound.m_max, boundMargin);
}

int btQuantizedBvhTree::_calc_splitting_axis(
	GIM_BVH_DATA_ARRAY& primitive_boxes, int startIndex, int endIndex)
{
	int i;

	btVector3 means(btScalar(0.), btScalar(0.), btScalar(0.));
	btVector3 variance(btScalar(0.), btScalar(0.), btScalar(0.));
	int numIndices = endIndex - startIndex;

	for (i = startIndex; i < endIndex; i++)
	{
		btVector3 center = btScalar(0.5) * (primitive_boxes[i].m_bound.m_max +
											primitive_boxes[i].m_bound.m_min);
		means += center;
	}
	means *= (btScalar(1.) / (btScalar)numIndices);

	for (i = startIndex; i < endIndex; i++)
	{
		btVector3 center = btScalar(0.5) * (primitive_boxes[i].m_bound.m_max +
											primitive_boxes[i].m_bound.m_min);
		btVector3 diff2 = center - means;
		diff2 = diff2 * diff2;
		variance += diff2;
	}
	variance *= (btScalar(1.) / ((btScalar)numIndices - 1));

	return variance.maxAxis();
}

int btQuantizedBvhTree::_sort_and_calc_splitting_index(
	GIM_BVH_DATA_ARRAY& primitive_boxes, int startIndex,
	int endIndex, int splitAxis)
{
	int i;
	int splitIndex = startIndex;
	int numIndices = endIndex - startIndex;

	// average of centers
	btScalar splitValue = 0.0f;

	btVector3 means(btScalar(0.), btScalar(0.), btScalar(0.));
	for (i = startIndex; i < endIndex; i++)
	{
		btVector3 center = btScalar(0.5) * (primitive_boxes[i].m_bound.m_max +
											primitive_boxes[i].m_bound.m_min);
		means += center;
	}
	means *= (btScalar(1.) / (btScalar)numIndices);

	splitValue = means[splitAxis];

	//sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
	for (i = startIndex; i < endIndex; i++)
	{
		btVector3 center = btScalar(0.5) * (primitive_boxes[i].m_bound.m_max +
											primitive_boxes[i].m_bound.m_min);
		if (center[splitAxis] > splitValue)
		{
			//swap
			primitive_boxes.swap(i, splitIndex);
			//swapLeafNodes(i,splitIndex);
			splitIndex++;
		}
	}

	//if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
	//otherwise the tree-building might fail due to stack-overflows in certain cases.
	//unbalanced1 is unsafe: it can cause stack overflows
	//bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

	//unbalanced2 should work too: always use center (perfect balanced trees)
	//bool unbalanced2 = true;

	//this should be safe too:
	int rangeBalancedIndices = numIndices / 3;
	bool unbalanced = ((splitIndex <= (startIndex + rangeBalancedIndices)) || (splitIndex >= (endIndex - 1 - rangeBalancedIndices)));

	if (unbalanced)
	{
		splitIndex = startIndex + (numIndices >> 1);
	}

	btAssert(!((splitIndex == startIndex) || (splitIndex == (endIndex))));

	return splitIndex;
}

void btQuantizedBvhTree::_build_sub_tree(GIM_BVH_DATA_ARRAY& primitive_boxes, int startIndex, int endIndex)
{
	int curIndex = m_num_nodes;
	m_num_nodes++;

	btAssert((endIndex - startIndex) > 0);

	if ((endIndex - startIndex) == 1)
	{
		//We have a leaf node
		setNodeBound(curIndex, primitive_boxes[startIndex].m_bound);
		m_node_array[curIndex].setDataIndex(primitive_boxes[startIndex].m_data);

		return;
	}
	//calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

	//split axis
	int splitIndex = _calc_splitting_axis(primitive_boxes, startIndex, endIndex);

	splitIndex = _sort_and_calc_splitting_index(
		primitive_boxes, startIndex, endIndex,
		splitIndex  //split axis
	);

	//calc this node bounding box

	btAABB node_bound;
	node_bound.invalidate();

	for (int i = startIndex; i < endIndex; i++)
	{
		node_bound.merge(primitive_boxes[i].m_bound);
	}

	setNodeBound(curIndex, node_bound);

	//build left branch
	_build_sub_tree(primitive_boxes, startIndex, splitIndex);

	//build right branch
	_build_sub_tree(primitive_boxes, splitIndex, endIndex);

	m_node_array[curIndex].setEscapeIndex(m_num_nodes - curIndex);
}

//! stackless build tree
void btQuantizedBvhTree::build_tree(
	GIM_BVH_DATA_ARRAY& primitive_boxes)
{
	calc_quantization(primitive_boxes);
	// initialize node count to 0
	m_num_nodes = 0;
	// allocate nodes
	m_node_array.resize(primitive_boxes.size() * 2);

	_build_sub_tree(primitive_boxes, 0, primitive_boxes.size());
}

////////////////////////////////////class btGImpactQuantizedBvh

void btGImpactQuantizedBvh::refit()
{
	int nodecount = getNodeCount();
	while (nodecount--)
	{
		if (isLeafNode(nodecount))
		{
			btAABB leafbox;
			m_primitive_manager->get_primitive_box(getNodeData(nodecount), leafbox);
			setNodeBound(nodecount, leafbox);
		}
		else
		{
			//const GIM_BVH_TREE_NODE * nodepointer = get_node_pointer(nodecount);
			//get left bound
			btAABB bound;
			bound.invalidate();

			btAABB temp_box;

			int child_node = getLeftNode(nodecount);
			if (child_node)
			{
				getNodeBound(child_node, temp_box);
				bound.merge(temp_box);
			}

			child_node = getRightNode(nodecount);
			if (child_node)
			{
				getNodeBound(child_node, temp_box);
				bound.merge(temp_box);
			}

			setNodeBound(nodecount, bound);
		}
	}
}

//! this rebuild the entire set
void btGImpactQuantizedBvh::buildSet()
{
	//obtain primitive boxes
	GIM_BVH_DATA_ARRAY primitive_boxes;
	primitive_boxes.resize(m_primitive_manager->get_primitive_count());

	for (int i = 0; i < primitive_boxes.size(); i++)
	{
		m_primitive_manager->get_primitive_box(i, primitive_boxes[i].m_bound);
		primitive_boxes[i].m_data = i;
	}

	m_box_tree.build_tree(primitive_boxes);
}

//! returns the indices of the primitives in the m_primitive_manager
bool btGImpactQuantizedBvh::boxQuery(const btAABB& box, btAlignedObjectArray<int>& collided_results) const
{
	int curIndex = 0;
	int numNodes = getNodeCount();

	//quantize box

	unsigned short quantizedMin[3];
	unsigned short quantizedMax[3];

	m_box_tree.quantizePoint(quantizedMin, box.m_min);
	m_box_tree.quantizePoint(quantizedMax, box.m_max);

	while (curIndex < numNodes)
	{
		//catch bugs in tree data

		bool aabbOverlap = m_box_tree.testQuantizedBoxOverlapp(curIndex, quantizedMin, quantizedMax);
		bool isleafnode = isLeafNode(curIndex);

		if (isleafnode && aabbOverlap)
		{
			collided_results.push_back(getNodeData(curIndex));
		}

		if (aabbOverlap || isleafnode)
		{
			//next subnode
			curIndex++;
		}
		else
		{
			//skip node
			curIndex += getEscapeNodeIndex(curIndex);
		}
	}
	if (collided_results.size() > 0) return true;
	return false;
}

//! returns the indices of the primitives in the m_primitive_manager
bool btGImpactQuantizedBvh::rayQuery(
	const btVector3& ray_dir, const btVector3& ray_origin,
	btAlignedObjectArray<int>& collided_results) const
{
	int curIndex = 0;
	int numNodes = getNodeCount();

	while (curIndex < numNodes)
	{
		btAABB bound;
		getNodeBound(curIndex, bound);

		//catch bugs in tree data

		bool aabbOverlap = bound.collide_ray(ray_origin, ray_dir);
		bool isleafnode = isLeafNode(curIndex);

		if (isleafnode && aabbOverlap)
		{
			collided_results.push_back(getNodeData(curIndex));
		}

		if (aabbOverlap || isleafnode)
		{
			//next subnode
			curIndex++;
		}
		else
		{
			//skip node
			curIndex += getEscapeNodeIndex(curIndex);
		}
	}
	if (collided_results.size() > 0) return true;
	return false;
}

SIMD_FORCE_INLINE bool _quantized_node_collision(
	const btGImpactQuantizedBvh* boxset0, const btGImpactQuantizedBvh* boxset1,
	const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0,
	int node0, int node1, bool complete_primitive_tests)
{
	btAABB box0;
	boxset0->getNodeBound(node0, box0);
	btAABB box1;
	boxset1->getNodeBound(node1, box1);

	return box0.overlapping_trans_cache(box1, trans_cache_1to0, complete_primitive_tests);
		//box1.appy_transform_trans_cache(trans_cache_1to0);
		//return box0.has_collision(box1);
}

// recursive collision routine
static void _find_quantized_collision_pairs_recursive_ser(
	const btGImpactQuantizedBvh* boxset0, const btGImpactQuantizedBvh* boxset1, btPairSet* collision_pairs,
	const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0,
	int node0, int node1, bool complete_primitive_tests, bool findOnlyFirstPair)
{
	if (findOnlyFirstPair)
	{
		if (collision_pairs->size() > 0)
		{
			return;
		}
	}
	else if (!findOnlyFirstPair && boxset0->isLeafNode(node0) && boxset1->isLeafNode(node1))
	{
		// Leaf vs leaf test is not done now (except for the findOnlyFirstPair mode), but deferred to be done in the parallelized for loop in collide_sat_triangles.
		// The assumption is that the tri vs tri test is comparable in complexity to the aabb vs obb test. So we should not loose much and gain significantly from the parallelization.
		collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
		return;
	}
	if (_quantized_node_collision(
			boxset0, boxset1, trans_cache_1to0,
			node0, node1, complete_primitive_tests) == false)
	{
		return;  //avoid colliding internal nodes
	}

	if (boxset0->isLeafNode(node0))
	{
		if (boxset1->isLeafNode(node1))
		{
			// collision result
			if (findOnlyFirstPair)
			{
				collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
			}
			return;
		}
		else
		{
			//collide left recursive
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, node0, boxset1->getLeftNode(node1), false, findOnlyFirstPair);
			//collide right recursive
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, node0, boxset1->getRightNode(node1), false, findOnlyFirstPair);
		}
	}
	else
	{
		if (boxset1->isLeafNode(node1))
		{
			//collide left recursive
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getLeftNode(node0), node1, false, findOnlyFirstPair);
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getRightNode(node0), node1, false, findOnlyFirstPair);
		}
		else
		{
			//collide left0 left1
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getLeftNode(node0), boxset1->getLeftNode(node1), false, findOnlyFirstPair);

			//collide left0 right1
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getLeftNode(node0), boxset1->getRightNode(node1), false, findOnlyFirstPair);

			//collide right0 left1
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getRightNode(node0), boxset1->getLeftNode(node1), false, findOnlyFirstPair);

			//collide right0 right1
			_find_quantized_collision_pairs_recursive_ser(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getRightNode(node0), boxset1->getRightNode(node1), false, findOnlyFirstPair);
		}  // else if node1 is not a leaf
	}      // else if node0 is not a leaf
}

static void _find_quantized_collision_pairs_stack_ser(
	const btGImpactQuantizedBvh* boxset0, const btGImpactQuantizedBvh* boxset1,
	btPairSet* collision_pairs,
	const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0,
	int node0, int node1, bool complete_primitive_tests, bool findOnlyFirstPair)
{
	std::stack<std::tuple<int, int, bool>> pairStack;
	pairStack.push({node0, node1, complete_primitive_tests});
	while (!pairStack.empty())
	{
		auto pair = pairStack.top();
		pairStack.pop();
		int node0 = std::get<0>(pair), node1 = std::get<1>(pair);
		bool complete_primitive_tests_local = std::get<2>(pair);
		
		if (findOnlyFirstPair)
		{
			if (collision_pairs->size() > 0)
			{
				continue;
			}
		}
		else if (!findOnlyFirstPair && boxset0->isLeafNode(node0) && boxset1->isLeafNode(node1))
		{
			// Leaf vs leaf test is not done now (except for the findOnlyFirstPair mode), but deferred to be done in the parallelized for loop in collide_sat_triangles.
			// The assumption is that the tri vs tri test is comparable in complexity to the aabb vs obb test. So we should not loose much and gain significantly from the parallelization.
			// Work in this function is not embarrassingly parallel. It was challenging to feed the threads with enough work while trying to parrallelize it.
			collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
			continue;
		}

		if (_quantized_node_collision(
				boxset0, boxset1, trans_cache_1to0,
				node0, node1, complete_primitive_tests_local) == false)
		{
			continue;  //avoid colliding internal nodes
		}

		if (boxset0->isLeafNode(node0))
		{
			if (boxset1->isLeafNode(node1))
			{
				// collision result
				if (findOnlyFirstPair)
				{
					collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
				}
				continue;
			}
			else
			{
				pairStack.push({node0, boxset1->getRightNode(node1), false});
				pairStack.push({node0, boxset1->getLeftNode(node1), false});
			}
		}
		else
		{
			if (boxset1->isLeafNode(node1))
			{
				pairStack.push({boxset0->getRightNode(node0), node1, false});
				pairStack.push({boxset0->getLeftNode(node0), node1, false});
			}
			else
			{
				pairStack.push({boxset0->getRightNode(node0), boxset1->getRightNode(node1), false});
				pairStack.push({boxset0->getRightNode(node0), boxset1->getLeftNode(node1), false});
				pairStack.push({boxset0->getLeftNode(node0), boxset1->getRightNode(node1), false});
				pairStack.push({boxset0->getLeftNode(node0), boxset1->getLeftNode(node1), false});
			}
		}
	}
}

// Do not use yet. The scheduler gets swamped because there is no level cutoff for parallelization. Also result writeback
// should be handled using thread local storage. Please note that vector is used instead of a stack. tbb::parallel_for_each probably couldn't handle a stack.
// TODO use a list instead of vector. Should be faster.
static void _find_quantized_collision_pairs_stack_par(
	const btGImpactQuantizedBvh* boxset0, const btGImpactQuantizedBvh* boxset1,
	btPairSet* collision_pairs,
	const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0,
	int node0, int node1, bool complete_primitive_tests, bool findOnlyFirstPair)
{
	std::mutex collision_pairs_mutex;
	typedef std::tuple<int, int, bool> ElemType;
	std::vector<ElemType> tupleElem;
	tupleElem.push_back({node0, node1, complete_primitive_tests});

	tbb::parallel_for_each(
		tupleElem.begin(), tupleElem.end(),
		[findOnlyFirstPair, boxset0, boxset1, trans_cache_1to0, collision_pairs, &collision_pairs_mutex](ElemType& elem, tbb::feeder<ElemType>& feeder)
		{
			int node0 = std::get<0>(elem), node1 = std::get<1>(elem);
			bool complete_primitive_tests_local = std::get<2>(elem);
			if (findOnlyFirstPair)
			{
				std::lock_guard<std::mutex> guard(collision_pairs_mutex);
				if (collision_pairs->size() > 0)
				{
					return;
				}
			}
			else if (!findOnlyFirstPair && boxset0->isLeafNode(node0) && boxset1->isLeafNode(node1))
			{
				// Leaf vs leaf test is not done now (except for the findOnlyFirstPair mode), but deferred to be done in the parallelized for loop in collide_sat_triangles.
				// The assumption is that the tri vs tri test is comparable in complexity to the aabb vs obb test. So we should not loose much and gain significantly from the parallelization.
				// Work in this function is not embarrassingly parallel. It was challenging to feed the threads with enough work while trying to parrallelize it.
				std::lock_guard<std::mutex> guard(collision_pairs_mutex);
				collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
				return;
			}

			if (_quantized_node_collision(
					boxset0, boxset1, trans_cache_1to0,
					node0, node1, complete_primitive_tests_local) == false)
			{
				return;  //avoid colliding internal nodes
			}

			if (boxset0->isLeafNode(node0))
			{
				if (boxset1->isLeafNode(node1))
				{
					// collision result
					if (findOnlyFirstPair)
					{
						std::lock_guard<std::mutex> guard(collision_pairs_mutex);
						collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
					}
					return;
				}
				else
				{
					feeder.add({node0, boxset1->getLeftNode(node1), false});
					feeder.add({node0, boxset1->getRightNode(node1), false});
				}
			}
			else
			{
				if (boxset1->isLeafNode(node1))
				{
					feeder.add({boxset0->getLeftNode(node0), node1, false});
					feeder.add({boxset0->getRightNode(node0), node1, false});
				}
				else
				{
					feeder.add({boxset0->getLeftNode(node0), boxset1->getLeftNode(node1), false});
					feeder.add({boxset0->getLeftNode(node0), boxset1->getRightNode(node1), false});
					feeder.add({boxset0->getRightNode(node0), boxset1->getLeftNode(node1), false});
					feeder.add({boxset0->getRightNode(node0), boxset1->getRightNode(node1), false});
				}
			}
		});
}

struct GroupedParams
{
	const btGImpactQuantizedBvh* boxset0;
	const btGImpactQuantizedBvh* boxset1;
	ThreadLocalGImpactResult& perThreadIntermediateResults;
	const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0;
	bool findOnlyFirstPair;
	std::atomic<bool>& firstPairFound;
	int threadLaunchStopLevel;
	btGimpactVsGimpactGroupedParams grpParams;
	GroupedParams(const btGImpactQuantizedBvh* boxset0,
				  const btGImpactQuantizedBvh* boxset1,
				  ThreadLocalGImpactResult& perThreadIntermediateResults,
				  const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0,
				  bool findOnlyFirstPair,
				  std::atomic<bool>& firstPairFound,
				  int threadLaunchStopLevel,
				  const btGimpactVsGimpactGroupedParams& grpParams) : boxset0(boxset0), boxset1(boxset1), perThreadIntermediateResults(perThreadIntermediateResults), trans_cache_1to0(trans_cache_1to0), findOnlyFirstPair(findOnlyFirstPair), firstPairFound(firstPairFound), threadLaunchStopLevel(threadLaunchStopLevel), grpParams(grpParams)
	{
	}
};

// This is the most promising candidate so far
static void _find_quantized_collision_pairs_recursive_par(GroupedParams& groupedParams, int node0, int node1, int level, bool complete_primitive_tests)
{
	if (groupedParams.findOnlyFirstPair)
	{
		if (groupedParams.firstPairFound)
		{
			return;
		}
	}
	else if (!groupedParams.findOnlyFirstPair && groupedParams.boxset0->isLeafNode(node0) && groupedParams.boxset1->isLeafNode(node1))
	{
		// Leaf vs leaf test is not done now (except for the findOnlyFirstPair mode), but deferred to be done in the parallelized for loop in collide_sat_triangles.
		// The assumption is that the tri vs tri test is comparable in complexity to the aabb vs obb test. So we should not loose much and gain significantly from the parallelization.
		btGImpactPairEval::EvalPair({groupedParams.boxset0->getNodeData(node0), groupedParams.boxset1->getNodeData(node1)}, groupedParams.grpParams, &groupedParams.perThreadIntermediateResults, nullptr);
		return;
	}
	if (_quantized_node_collision(
			groupedParams.boxset0, groupedParams.boxset1, groupedParams.trans_cache_1to0,
			node0, node1, complete_primitive_tests) == false)
	{
		return;  //avoid colliding internal nodes
	}

	if (groupedParams.boxset0->isLeafNode(node0))
	{
		if (groupedParams.boxset1->isLeafNode(node1))
		{
			// collision result
			if (groupedParams.findOnlyFirstPair)
			{
				if (btGImpactPairEval::EvalPair({groupedParams.boxset0->getNodeData(node0), groupedParams.boxset1->getNodeData(node1)}, groupedParams.grpParams, &groupedParams.perThreadIntermediateResults, nullptr))
					groupedParams.firstPairFound = true;
			}
			return;
		}
		else
		{
			if (level < groupedParams.threadLaunchStopLevel)
			{
				tbb::parallel_invoke(
					[&groupedParams, node0, node1, level]
					{
						//collide left recursive
						_find_quantized_collision_pairs_recursive_par(groupedParams, node0, groupedParams.boxset1->getLeftNode(node1), level + 1, false);
					},
					[&groupedParams, node0, node1, level]
					{
						//collide right recursive
						_find_quantized_collision_pairs_recursive_par(groupedParams, node0, groupedParams.boxset1->getRightNode(node1), level + 1, false);
					});
			}
			else
			{
				//collide left recursive
				_find_quantized_collision_pairs_recursive_par(groupedParams, node0, groupedParams.boxset1->getLeftNode(node1), level + 1, false);
				//collide right recursive
				_find_quantized_collision_pairs_recursive_par(groupedParams, node0, groupedParams.boxset1->getRightNode(node1), level + 1, false);
			}
		}
	}
	else
	{
		if (groupedParams.boxset1->isLeafNode(node1))
		{
			//collide left recursive
			if (level < groupedParams.threadLaunchStopLevel)
			{
				tbb::parallel_invoke(
					[&groupedParams, node0, node1, level]
					{
						_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getLeftNode(node0), node1, level + 1, false);
					},
					[&groupedParams, node0, node1, level]
					{
						_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getRightNode(node0), node1, level + 1, false);
					});
			}
			else
			{
				_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getLeftNode(node0), node1, level + 1, false);
				_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getRightNode(node0), node1, level + 1, false);
			}
		}
		else
		{
			if (level < groupedParams.threadLaunchStopLevel)
			{
				tbb::parallel_invoke(
					[&groupedParams, node0, node1, level]
					{
						//collide left0 left1
						_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getLeftNode(node0), groupedParams.boxset1->getLeftNode(node1), level + 1, false);
					},
					[&groupedParams, node0, node1, level]
					{
						//collide left0 right1
						_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getLeftNode(node0), groupedParams.boxset1->getRightNode(node1), level + 1, false);
					},
					[&groupedParams, node0, node1, level]
					{
						//collide right0 left1
						_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getRightNode(node0), groupedParams.boxset1->getLeftNode(node1), level + 1, false);
					},
					[&groupedParams, node0, node1, level]
					{
						//collide right0 right1
						_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getRightNode(node0), groupedParams.boxset1->getRightNode(node1), level + 1, false);
					});
			}
			else
			{
				//collide left0 left1
				_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getLeftNode(node0), groupedParams.boxset1->getLeftNode(node1), level + 1, false);
				//collide left0 right1
				_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getLeftNode(node0), groupedParams.boxset1->getRightNode(node1), level + 1, false);
				//collide right0 left1
				_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getRightNode(node0), groupedParams.boxset1->getLeftNode(node1), level + 1, false);
				//collide right0 right1
				_find_quantized_collision_pairs_recursive_par(groupedParams, groupedParams.boxset0->getRightNode(node0), groupedParams.boxset1->getRightNode(node1), level + 1, false);
			}
		}  // else if node1 is not a leaf
	}      // else if node0 is not a leaf
}

void btGImpactQuantizedBvh::find_collision(const btGImpactQuantizedBvh* boxset0, const btTransform& trans0,
										   const btGImpactQuantizedBvh* boxset1, const btTransform& trans1,
										   ThreadLocalGImpactResult& perThreadIntermediateResults, btPairSet& auxPairSet,
										   bool findOnlyFirstPair,
										   const btGimpactVsGimpactGroupedParams& grpParams)
{
	if (boxset0->getNodeCount() == 0 || boxset1->getNodeCount() == 0) return;

	BT_BOX_BOX_TRANSFORM_CACHE trans_cache_1to0;

	trans_cache_1to0.calc_from_homogenic(trans0, trans1);

#ifdef TRI_COLLISION_PROFILING
	bt_begin_gim02_q_tree_time();
#endif  //TRI_COLLISION_PROFILING

	//diagnostic::marker_series series0("col pairs serial");
	//diagnostic::span span(series0, diagnostic::high_importance, 10, "serial topmost %d %d", boxset0->getNodeCount(), boxset1->getNodeCount());
	
	//series0.write_message(diagnostic::normal_importance, 0, "start ser");

	//auto start = std::chrono::steady_clock::now();
	std::atomic<bool> firstPairFound = false;
	auto boxset0Depth = std::log2(boxset0->getNodeCount() + 1);
	auto boxset1Depth = std::log2(boxset1->getNodeCount() + 1);
	// It has been empirically observed that the best performance is obtained when the stop level is three quarters of total tree depth.
	// It needs to be verified yet if this holds also for different CPU core count. This was tested only on a cpu with 32 logical cores.
	auto threadLaunchStopLevel = static_cast<int>(max(boxset0Depth, boxset1Depth) / 1.5);
	// The tbb parallel calls increase the _find_quantized_collision_pairs_recursive_par duration from about 1us to anything between 20-80us,
	// so the tbb parallel calls pay off only when there are at least some triangle leaves visited in the BVHs
	if (grpParams.previouslyFoundPairCount <= 0 || findOnlyFirstPair)
		threadLaunchStopLevel = 0;
	GroupedParams groupedParams(boxset0, boxset1, perThreadIntermediateResults, trans_cache_1to0, findOnlyFirstPair, firstPairFound, threadLaunchStopLevel, grpParams);
	_find_quantized_collision_pairs_recursive_par(groupedParams, 0, 0, 0, true);
	//auto end = std::chrono::steady_clock::now();

	
	//auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	//size_t size = 0;
	//for (const auto& pairs : perThreadIntermediateResults)
	//	size += pairs.size();
	//if (size > 0)
	//{
	//	printf("boxset0Depth %f boxset1Depth %f threadLaunchStopLevel %d grpParams.previouslyFoundPairCount %d\n", boxset0Depth, boxset1Depth, threadLaunchStopLevel, grpParams.previouslyFoundPairCount);
	//	printf("_find_quantized_collision_pairs_recursive_par took %lld us\n", duration.count());
	//	printf("size %lld\n", size);
	//}

	//series0.write_message(diagnostic::normal_importance, 0, "end ser %d us", duration.count());
#ifdef TRI_COLLISION_PROFILING
	bt_end_gim02_q_tree_time();
#endif  //TRI_COLLISION_PROFILING
}
