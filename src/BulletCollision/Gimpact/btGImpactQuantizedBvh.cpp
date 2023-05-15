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

#include <tbb/tbb.h>
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

//stackless recursive collision routine
static void _find_quantized_collision_pairs_recursive(
	const btGImpactQuantizedBvh* boxset0, const btGImpactQuantizedBvh* boxset1, btPairSet* collision_pairs,
	const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0,
	int node0, int node1, bool complete_primitive_tests, bool findOnlyFirstPair)
{
	//printf("n0 %d n1 %d\n", node0, node1);
	if (findOnlyFirstPair)
	{
		if (collision_pairs->size() > 0)
		{
			//printf("b1\n");
			return;
		}
	}
	else if (!findOnlyFirstPair && boxset0->isLeafNode(node0) && boxset1->isLeafNode(node1))
	{
		// Leaf vs leaf test is not done now (except for the findOnlyFirstPair mode), but deferred to be done in the parallelized for loop in collide_sat_triangles.
		// The assumption is that the tri vs tri test is comparable in complexity to the aabb vs obb test. So we should not loose much and gain significantly from the parallelization.
		collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
		//printf("b2\n");
		return;
	}
	if (_quantized_node_collision(
			boxset0, boxset1, trans_cache_1to0,
			node0, node1, complete_primitive_tests) == false)
	{
		//printf("b3\n");
		return;  //avoid colliding internal nodes
	}

	if (boxset0->isLeafNode(node0))
	{
		if (boxset1->isLeafNode(node1))
		{
			// collision result
			if (findOnlyFirstPair)
			{
				//printf("b4\n");
				collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
			}
			return;
		}
		else
		{
			//printf("b5\n");
			//collide left recursive
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, node0, boxset1->getLeftNode(node1), false, findOnlyFirstPair);
			//collide right recursive
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, node0, boxset1->getRightNode(node1), false, findOnlyFirstPair);
		}
	}
	else
	{
		if (boxset1->isLeafNode(node1))
		{
			//collide left recursive
			//printf("b6\n");
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getLeftNode(node0), node1, false, findOnlyFirstPair);
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getRightNode(node0), node1, false, findOnlyFirstPair);
		}
		else
		{
			//printf("b7\n");
			//collide left0 left1
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getLeftNode(node0), boxset1->getLeftNode(node1), false, findOnlyFirstPair);

			//collide left0 right1
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getLeftNode(node0), boxset1->getRightNode(node1), false, findOnlyFirstPair);

			//collide right0 left1
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getRightNode(node0), boxset1->getLeftNode(node1), false, findOnlyFirstPair);

			//collide right0 right1
			_find_quantized_collision_pairs_recursive(boxset0, boxset1, collision_pairs, trans_cache_1to0, boxset0->getRightNode(node0), boxset1->getRightNode(node1), false, findOnlyFirstPair);
		}  // else if node1 is not a leaf
	}      // else if node0 is not a leaf
}

static void _find_quantized_collision_pairs(
	const btGImpactQuantizedBvh* boxset0, const btGImpactQuantizedBvh* boxset1,
	btPairSet* collision_pairs,
	const BT_BOX_BOX_TRANSFORM_CACHE& trans_cache_1to0,
	int node0, int node1, bool complete_primitive_tests, bool findOnlyFirstPair)
{
	if (findOnlyFirstPair)
	{
		printf("FINDONLYFIRSTPAIR\n");
		return;
	}
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
			//printf("n0 %d n1 %d\n", node0, node1);
			if (findOnlyFirstPair)
			{
				//std::lock_guard<std::mutex> guard(collision_pairs_mutex);
				if (collision_pairs->size() > 0)
				{
					//printf("b1\n");
					return;
				}
			}
			else if (!findOnlyFirstPair && boxset0->isLeafNode(node0) && boxset1->isLeafNode(node1))
			{
				// Leaf vs leaf test is not done now (except for the findOnlyFirstPair mode), but deferred to be done in the parallelized for loop in collide_sat_triangles.
				// The assumption is that the tri vs tri test is comparable in complexity to the aabb vs obb test. So we should not loose much and gain significantly from the parallelization.
				// Work in this function is not embarrassingly parallel. It was challenging to feed the threads with enough work while trying to parrallelize it.
				//std::lock_guard<std::mutex> guard(collision_pairs_mutex);
				//collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
				//printf("b2\n");
				return;
			}

			if (_quantized_node_collision(
					boxset0, boxset1, trans_cache_1to0,
					node0, node1, complete_primitive_tests_local) == false)
			{
				//printf("b3\n");
				return;  //avoid colliding internal nodes
			}

			if (boxset0->isLeafNode(node0))
			{
				if (boxset1->isLeafNode(node1))
				{
					// collision result
					if (findOnlyFirstPair)
					{
						//printf("b4\n");
						//std::lock_guard<std::mutex> guard(collision_pairs_mutex);
						//collision_pairs->push_back({boxset0->getNodeData(node0), boxset1->getNodeData(node1)});
					}
					return;
				}
				else
				{
					//printf("b5\n");
					feeder.add({node0, boxset1->getLeftNode(node1), false});
					feeder.add({node0, boxset1->getRightNode(node1), false});
				}
			}
			else
			{
				if (boxset1->isLeafNode(node1))
				{
					//printf("b6\n");
					feeder.add({boxset0->getLeftNode(node0), node1, false});
					feeder.add({boxset0->getRightNode(node0), node1, false});
				}
				else
				{
					//printf("b7\n");
					feeder.add({boxset0->getLeftNode(node0), boxset1->getLeftNode(node1), false});
					feeder.add({boxset0->getLeftNode(node0), boxset1->getRightNode(node1), false});
					feeder.add({boxset0->getRightNode(node0), boxset1->getLeftNode(node1), false});
					feeder.add({boxset0->getRightNode(node0), boxset1->getRightNode(node1), false});
				}
			}
		});
}

void btGImpactQuantizedBvh::find_collision(const btGImpactQuantizedBvh* boxset0, const btTransform& trans0,
										   const btGImpactQuantizedBvh* boxset1, const btTransform& trans1,
										   btPairSet& collision_pairs, btPairSet& collision_pairs2, bool findOnlyFirstPair)
{
	if (boxset0->getNodeCount() == 0 || boxset1->getNodeCount() == 0) return;

	BT_BOX_BOX_TRANSFORM_CACHE trans_cache_1to0;

	trans_cache_1to0.calc_from_homogenic(trans0, trans1);

#ifdef TRI_COLLISION_PROFILING
	bt_begin_gim02_q_tree_time();
#endif  //TRI_COLLISION_PROFILING

	//diagnostic::marker_series series0("col pairs serial");
	//diagnostic::span span(series0, diagnostic::high_importance, 10, "serial topmost %d %d", boxset0->getNodeCount(), boxset1->getNodeCount());
	auto start = std::chrono::steady_clock::now();
	//series0.write_message(diagnostic::normal_importance, 0, "start ser");

	printf("_find_quantized_collision_pairs_recursive start\n");
	_find_quantized_collision_pairs_recursive(boxset0, boxset1, &collision_pairs, trans_cache_1to0, 0, 0, true, findOnlyFirstPair);
	printf("_find_quantized_collision_pairs_recursive end\n");
	//_find_quantized_collision_pairs(boxset0, boxset1, &collision_pairs, trans_cache_1to0, 0, 0, true, findOnlyFirstPair);

	auto end = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	printf("_find_quantized_collision_pairs_recursive took %lld us\n", duration.count());
	printf("size %lld\n", collision_pairs.size());

	start = std::chrono::steady_clock::now();

	printf("_find_quantized_collision_pairs start\n");
	_find_quantized_collision_pairs(boxset0, boxset1, &collision_pairs2, trans_cache_1to0, 0, 0, true, findOnlyFirstPair);
	printf("_find_quantized_collision_pairs end\n");

	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	printf("_find_quantized_collision_pairs took %lld us\n", duration.count());
	printf("size %lld\n", collision_pairs2.size());

	//series0.write_message(diagnostic::normal_importance, 0, "end ser %d us", duration.count());
#ifdef TRI_COLLISION_PROFILING
	bt_end_gim02_q_tree_time();
#endif  //TRI_COLLISION_PROFILING
}
