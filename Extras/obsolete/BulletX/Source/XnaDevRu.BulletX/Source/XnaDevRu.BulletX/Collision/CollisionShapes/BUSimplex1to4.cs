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
	/// BUSimplex1to4 implements feature based and implicit simplex of up to 4 vertices (tetrahedron, triangle, line, vertex).
	/// </summary>
	public class BUSimplex1to4 : PolyhedralConvexShape
	{
		private int _numVertices = 0;
		private Vector3[] _vertices = new Vector3[4];

		public BUSimplex1to4() { }

		public BUSimplex1to4(Vector3 pointA)
		{
			AddVertex(pointA);
		}

		public BUSimplex1to4(Vector3 pointA, Vector3 pointB)
		{
			AddVertex(pointA);
			AddVertex(pointB);
		}

		public BUSimplex1to4(Vector3 pointA, Vector3 pointB, Vector3 pointC)
		{
			AddVertex(pointA);
			AddVertex(pointB);
			AddVertex(pointC);
		}

		public BUSimplex1to4(Vector3 pointA, Vector3 pointB, Vector3 pointC, Vector3 pointD)
		{
			AddVertex(pointA);
			AddVertex(pointB);
			AddVertex(pointC);
			AddVertex(pointD);
		}

		protected Vector3[] Vertices { get { return _vertices; } set { _vertices = value; } }

		public override int VertexCount
		{
			get
			{
				return _numVertices;
			}
		}

		public override int EdgeCount
		{
			get
			{
				//euler formula, F-E+V = 2, so E = F+V-2
				switch (_numVertices)
				{
					case 0: return 0;
					case 1: return 0;
					case 2: return 1;
					case 3: return 3;
					case 4: return 6;
				}
				return 0;
			}
		}

		public override int PlaneCount
		{
			get
			{
				switch (_numVertices)
				{
					case 0:
						return 0;
					case 1:
						return 0;
					case 2:
						return 0;
					case 3:
						return 2;
					case 4:
						return 4;
				}
				return 0;
			}
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.Tetrahedral;
			}
		}

		public override string Name
		{
			get
			{
				return "BUSimplex1to4";
			}
		}

		public void AddVertex(Vector3 v)
		{
			_vertices[_numVertices++] = v;
		}

		public void Reset()
		{
			_numVertices = 0;
		}

		public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
		{
			switch (_numVertices)
			{
				case 2:
					pa = _vertices[0];
					pb = _vertices[1];
					return;
				case 3:
					switch (i)
					{
						case 0:
							pa = _vertices[0];
							pb = _vertices[1];
							return;
						case 1:
							pa = _vertices[1];
							pb = _vertices[2];
							return;
						case 2:
							pa = _vertices[2];
							pb = _vertices[0];
							return;
					}
					break;
				case 4:
					switch (i)
					{
						case 0:
							pa = _vertices[0];
							pb = _vertices[1];
							return;
						case 1:
							pa = _vertices[1];
							pb = _vertices[2];
							return;
						case 2:
							pa = _vertices[2];
							pb = _vertices[0];
							return;
						case 3:
							pa = _vertices[0];
							pb = _vertices[3];
							return;
						case 4:
							pa = _vertices[1];
							pb = _vertices[3];
							return;
						case 5:
							pa = _vertices[2];
							pb = _vertices[3];
							return;
					}
					break;
			}

			pa = new Vector3();
			pb = new Vector3();
		}

		public override void GetVertex(int i, out Vector3 vtx)
		{
			vtx = _vertices[i];
		}

		public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
		{
			planeNormal = new Vector3();
			planeSupport = new Vector3();
		}

		public override bool IsInside(Vector3 pt, float tolerance)
		{
			return false;
		}
	}
}
