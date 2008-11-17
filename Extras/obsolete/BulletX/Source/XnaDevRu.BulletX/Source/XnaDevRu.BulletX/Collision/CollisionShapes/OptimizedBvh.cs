/*
  Bullet for XNA Copyright (c) 2003-2007 Vsevolod Klementjev http://www.codeplex.com/xnadevru
  Bullet original C++ version Copyright (c) 2003-2007 Erwin Coumans http://bulletphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;

namespace XnaDevRu.BulletX
{
	/// <summary>
	/// OptimizedBvh store an AABB tree that can be quickly traversed on CPU (and SPU,GPU in future)
	/// </summary>
	public class OptimizedBvh
	{
		private static int _maxIterations = 0;
		private OptimizedBvhNode _rootNode;

		private OptimizedBvhNode[] _contiguousNodes;
		private int _curNodeIndex;

		private List<OptimizedBvhNode> _leafNodes = new List<OptimizedBvhNode>();

        public OptimizedBvh() { }

		public void Build(StridingMeshInterface triangles)
		{
			NodeTriangleCallback callback = new NodeTriangleCallback(_leafNodes);

			Vector3 aabbMin = new Vector3(-1e30f, -1e30f, -1e30f);
			Vector3 aabbMax = new Vector3(1e30f, 1e30f, 1e30f);

			triangles.InternalProcessAllTriangles(callback, aabbMin, aabbMax);

			//now we have an array of leafnodes in m_leafNodes
			_contiguousNodes = new OptimizedBvhNode[2 * _leafNodes.Count];
            for (int i = 0; i < _contiguousNodes.Length; i++)
                _contiguousNodes[i] = new OptimizedBvhNode();
			_curNodeIndex = 0;

			_rootNode = BuildTree(_leafNodes, 0, _leafNodes.Count);
		}

		public OptimizedBvhNode BuildTree(List<OptimizedBvhNode> leafNodes, int startIndex, int endIndex)
		{
			OptimizedBvhNode internalNode;

			int splitAxis, splitIndex, i;
			int numIndices = endIndex - startIndex;
			int curIndex = _curNodeIndex;

			if (numIndices <= 0)
				throw new BulletException();

			if (numIndices == 1)
			{
				_contiguousNodes[_curNodeIndex++] = leafNodes[startIndex];
				//return new (&m_contiguousNodes[m_curNodeIndex++]) btOptimizedBvhNode(leafNodes[startIndex]);
				return leafNodes[startIndex];
			}

			//calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.
			splitAxis = CalculateSplittingAxis(leafNodes, startIndex, endIndex);

			splitIndex = SortAndCalculateSplittingIndex(leafNodes, startIndex, endIndex, splitAxis);

			internalNode = _contiguousNodes[_curNodeIndex++];

			internalNode.AabbMax = new Vector3(-1e30f, -1e30f, -1e30f);
			internalNode.AabbMin = new Vector3(1e30f, 1e30f, 1e30f);

			for (i = startIndex; i < endIndex; i++)
			{
				internalNode.AabbMax = MathHelper.SetMax(internalNode.AabbMax, leafNodes[i].AabbMax);
				internalNode.AabbMin = MathHelper.SetMin(internalNode.AabbMin, leafNodes[i].AabbMin);
			}

			//internalNode->m_escapeIndex;
			internalNode.LeftChild = BuildTree(leafNodes, startIndex, splitIndex);
			internalNode.RightChild = BuildTree(leafNodes, splitIndex, endIndex);

			internalNode.EscapeIndex = _curNodeIndex - curIndex;
			return internalNode;
		}

		public int CalculateSplittingAxis(List<OptimizedBvhNode> leafNodes, int startIndex, int endIndex)
		{
			Vector3 means = new Vector3();
			Vector3 variance = new Vector3();
			int numIndices = endIndex - startIndex;

			for (int i = startIndex; i < endIndex; i++)
			{
				Vector3 center = 0.5f * (leafNodes[i].AabbMax + leafNodes[i].AabbMin);
				means += center;
			}
			means *= (1f / (float)numIndices);

			for (int i = startIndex; i < endIndex; i++)
			{
				Vector3 center = 0.5f * (leafNodes[i].AabbMax + leafNodes[i].AabbMin);
				Vector3 diff2 = center - means;
				diff2 = diff2 * diff2;
				variance += diff2;
			}
			variance *= (1f / ((float)numIndices - 1));

			return MathHelper.MaxAxis(variance);
		}

		public int SortAndCalculateSplittingIndex(List<OptimizedBvhNode> leafNodes, int startIndex, int endIndex, int splitAxis)
		{
			int splitIndex = startIndex;
			int numIndices = endIndex - startIndex;
			float splitValue;

			Vector3 means = new Vector3();
			for (int i = startIndex; i < endIndex; i++)
			{
				Vector3 center = 0.5f * (leafNodes[i].AabbMax + leafNodes[i].AabbMin);
				means += center;
			}
			means *= (1f / (float)numIndices);

			if (splitAxis == 0)
				splitValue = means.X;
			else if (splitAxis == 1)
				splitValue = means.Y;
			else if (splitAxis == 2)
				splitValue = means.Z;
			else
				throw new ArgumentException();

			//sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
			for (int i = startIndex; i < endIndex; i++)
			{
				Vector3 center = 0.5f * (leafNodes[i].AabbMax + leafNodes[i].AabbMin);
				float centerSplit;

				if (splitAxis == 0)
					centerSplit = means.X;
				else if (splitAxis == 1)
					centerSplit = means.Y;
				else if (splitAxis == 2)
					centerSplit = means.Z;
				else
					throw new ArgumentException();

				if (centerSplit > splitValue)
				{
					//swap
					OptimizedBvhNode tmp = leafNodes[i];
					leafNodes[i] = leafNodes[splitIndex];
					leafNodes[splitIndex] = tmp;
					splitIndex++;
				}
			}
			if ((splitIndex == startIndex) || (splitIndex == (endIndex - 1)))
			{
				splitIndex = startIndex + (numIndices >> 1);
			}
			return splitIndex;
		}

		public void WalkTree(OptimizedBvhNode rootNode, INodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax)
		{
			bool isLeafNode, aabbOverlap = MathHelper.TestAabbAgainstAabb2(aabbMin, aabbMax, rootNode.AabbMin, rootNode.AabbMax);
			if (aabbOverlap)
			{
				isLeafNode = (rootNode.LeftChild == null && rootNode.RightChild == null);
				if (isLeafNode)
				{
					nodeCallback.ProcessNode(rootNode);
				}
				else
				{
					WalkTree(rootNode.LeftChild, nodeCallback, aabbMin, aabbMax);
					WalkTree(rootNode.RightChild, nodeCallback, aabbMin, aabbMax);
				}
			}
		}

		public void WalkStacklessTree(OptimizedBvhNode[] rootNodeArray, INodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax)
		{
			int escapeIndex, curIndex = 0;
			int walkIterations = 0;
			bool aabbOverlap, isLeafNode;
            int rootNodeIndex = 0;
            OptimizedBvhNode rootNode = rootNodeArray[rootNodeIndex];

			while (curIndex < _curNodeIndex)
			{
				//catch bugs in tree data
				if (walkIterations >= _curNodeIndex)
					throw new BulletException();

				walkIterations++;
				aabbOverlap = MathHelper.TestAabbAgainstAabb2(aabbMin, aabbMax, rootNode.AabbMin, rootNode.AabbMax);
				isLeafNode = (rootNode.LeftChild == null && rootNode.RightChild == null);

				if (isLeafNode && aabbOverlap)
				{
					nodeCallback.ProcessNode(rootNode);
				}

				if (aabbOverlap || isLeafNode)
				{
					rootNodeIndex++; // this
					curIndex++;
                    if (rootNodeIndex < rootNodeArray.Length)
                        rootNode = rootNodeArray[rootNodeIndex];
                }
				else
				{
					escapeIndex = rootNode.EscapeIndex;
					rootNodeIndex += escapeIndex; // and this
					curIndex += escapeIndex;
                    if (rootNodeIndex < rootNodeArray.Length)
                        rootNode = rootNodeArray[rootNodeIndex];
				}

			}

			if (_maxIterations < walkIterations)
				_maxIterations = walkIterations;
		}

		public void ReportAabbOverlappingNodex(INodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax)
		{
			//either choose recursive traversal (walkTree) or stackless (walkStacklessTree)
			//walkTree(m_rootNode1,nodeCallback,aabbMin,aabbMax);
			//WalkStacklessTree(_rootNode, nodeCallback, aabbMin, aabbMax);
            WalkStacklessTree(_contiguousNodes, nodeCallback, aabbMin, aabbMax);
		}

		public void ReportSphereOverlappingNodex(INodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax) { }
	}

	public class NodeTriangleCallback : ITriangleIndexCallback
	{
		private List<OptimizedBvhNode> _triangleNodes;

		public NodeTriangleCallback(List<OptimizedBvhNode> triangleNodes)
		{
			_triangleNodes = triangleNodes;
		}

		public List<OptimizedBvhNode> TriangleNodes { get { return _triangleNodes; } set { _triangleNodes = value; } }

		public void ProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
		{

			OptimizedBvhNode node = new OptimizedBvhNode();
			node.AabbMin = new Vector3(1e30f, 1e30f, 1e30f);
			node.AabbMax = new Vector3(-1e30f, -1e30f, -1e30f);

			node.AabbMin = MathHelper.SetMin(node.AabbMin, triangle[0]);
			node.AabbMax = MathHelper.SetMax(node.AabbMax, triangle[0]);
			node.AabbMin = MathHelper.SetMin(node.AabbMin, triangle[1]);
			node.AabbMax = MathHelper.SetMax(node.AabbMax, triangle[1]);
			node.AabbMin = MathHelper.SetMin(node.AabbMin, triangle[2]);
			node.AabbMax = MathHelper.SetMax(node.AabbMax, triangle[2]);

			node.EscapeIndex = -1;
			node.LeftChild = null;
			node.RightChild = null;

			//for child nodes
			node.SubPart = partId;
			node.TriangleIndex = triangleIndex;

			_triangleNodes.Add(node);
		}
	}
}
