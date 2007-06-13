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
	/// OptimizedBvhNode contains both internal and leaf node information.
	/// It hasn't been optimized yet for storage. Some obvious optimizations are:
	/// Removal of the pointers (can already be done, they are not used for traversal)
	/// and storing aabbmin/max as quantized integers.
	/// 'subpart' doesn't need an integer either. It allows to re-use graphics triangle
	/// meshes stored in a non-uniform way (like batches/subparts of triangle-fans
	/// </summary>
	public class OptimizedBvhNode
	{
		private Vector3 _aabbMin;
		private Vector3 _aabbMax;

		//these 2 pointers are obsolete, the stackless traversal just uses the escape index
		private OptimizedBvhNode _leftChild;
		private OptimizedBvhNode _rightChild;

		private int _escapeIndex;

		//for child nodes
		private int _subPart;
		private int _triangleIndex;

		public Vector3 AabbMin { get { return _aabbMin; } set { _aabbMin = value; } }
		public Vector3 AabbMax { get { return _aabbMax; } set { _aabbMax = value; } }

		public OptimizedBvhNode LeftChild { get { return _leftChild; } set { _leftChild = value; } }
		public OptimizedBvhNode RightChild { get { return _rightChild; } set { _rightChild = value; } }

		public int EscapeIndex { get { return _escapeIndex; } set { _escapeIndex = value; } }

		public int SubPart { get { return _subPart; } set { _subPart = value; } }
		public int TriangleIndex { get { return _triangleIndex; } set { _triangleIndex = value; } }
	}
}
