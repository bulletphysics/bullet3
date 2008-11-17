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
	public class Triangle
	{
		private Vector3 _vertexA;
		private Vector3 _vertexB;
		private Vector3 _vertexC;
		private int _partId;
		private int _triangleIndex;

		public Vector3 VertexA { get { return _vertexA; } set { _vertexA = value; } }
		public Vector3 VertexB { get { return _vertexB; } set { _vertexB = value; } }
		public Vector3 VertexC { get { return _vertexC; } set { _vertexC = value; } }
		public int PartId { get { return _partId; } set { _partId = value; } }
		public int TriangleIndex { get { return _triangleIndex; } set { _triangleIndex = value; } }
	}

	/// <summary>
	/// example usage of this class:
	///			TriangleBuffer triBuf;
	///			concaveShape.processAllTriangles(triBuf, out aabbMin, out aabbMax);
	///			for (int i = 0; i < triBuf.getNumTriangles(); i++)
	///			{
	///				Triangle tri = triBuf.getTriangle(i);
	///				//do something useful here with the triangle
	///			}
	/// </summary>
	public class TriangleBuffer : ITriangleCallback
	{
		private List<Triangle> _triangleBuffer = new List<Triangle>();

		public int TriangleCount { get { return _triangleBuffer.Count; } }
		public Triangle this[int index] { get { return _triangleBuffer[index]; } }

		public void ClearBuffer()
		{
			_triangleBuffer.Clear();
		}

		#region ITriangleCallback Members
		public void ProcessTriangle(Vector3[] triangle, int partID, int triangleIndex)
		{
			Triangle tri = new Triangle();
			tri.VertexA = triangle[0];
			tri.VertexB = triangle[1];
			tri.VertexC = triangle[2];
			tri.PartId = partID;
			tri.TriangleIndex = triangleIndex;

			_triangleBuffer.Add(tri);
		}
		#endregion
	}
}
