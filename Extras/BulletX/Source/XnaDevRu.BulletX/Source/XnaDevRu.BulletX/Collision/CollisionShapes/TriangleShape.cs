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
	public class TriangleShape : PolyhedralConvexShape
	{
		private Vector3[] _vertices = new Vector3[3];

		public TriangleShape(Vector3 pointA, Vector3 pointB, Vector3 pointC)
		{
			_vertices[0] = pointA;
			_vertices[1] = pointB;
			_vertices[2] = pointC;
		}

		public override int PreferredPenetrationDirectionsCount
		{
			get
			{
				return 2;
			}
		}

		public Vector3[] Vertices
		{
			get
			{
				return _vertices;
			}
		}

		public override int VertexCount
		{
			get
			{
				return 3;
			}
		}

		public override int EdgeCount
		{
			get
			{
				return 3;
			}
		}

		public override int PlaneCount
		{
			get
			{
				return 1;
			}
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.Triangle;
			}
		}

		public override string Name
		{
			get
			{
				return "Triangle";
			}
		}

		public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
		{
			CalculateNormal(out penetrationVector);
			if (index != 0)
				penetrationVector *= -1f;
		}

		public virtual void GetPlaneEquation(int i, out Vector3 planeNormal, out Vector3 planeSupport)
		{
			CalculateNormal(out planeNormal);
			planeSupport = _vertices[0];
		}

		public void CalculateNormal(out Vector3 normal)
		{
			normal = Vector3.Normalize(Vector3.Cross(_vertices[1] - _vertices[0], _vertices[2] - _vertices[0]));
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
		{
			Vector3 dots = new Vector3(Vector3.Dot(vec, _vertices[0]), Vector3.Dot(vec, _vertices[1]), Vector3.Dot(vec, _vertices[2]));
			return _vertices[MathHelper.MaxAxis(dots)];
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			for (int i = 0; i < vectors.Length; i++)
			{
				Vector3 dir = vectors[i];
				Vector3 dots = new Vector3(Vector3.Dot(dir, _vertices[0]), Vector3.Dot(dir, _vertices[1]), Vector3.Dot(dir, _vertices[2]));
				supportVerticesOut[i] = _vertices[MathHelper.MaxAxis(dots)];
			}
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			inertia = new Vector3();
			BulletDebug.Assert(false);
		}

		public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
		{
			GetVertex(i, out pa);
			GetVertex((i + 1) % 3, out pb);
		}

		public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			GetAabbSlow(t, out aabbMin, out aabbMax);
		}

		public override void GetVertex(int i, out Vector3 vtx)
		{
			vtx = _vertices[i];
		}

		public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
		{
			GetPlaneEquation(i, out planeNormal, out planeSupport);
		}

		public override bool IsInside(Vector3 pt, float tolerance)
		{
			Vector3 normal;
			CalculateNormal(out normal);
			//distance to plane
			float dist = Vector3.Dot(pt, normal);
			float planeconst = Vector3.Dot(_vertices[0], normal);
			dist -= planeconst;
			if (dist >= -tolerance && dist <= tolerance)
			{
				//inside check on edge-planes
				int i;
				for (i = 0; i < 3; i++)
				{
					Vector3 pa, pb;
					GetEdge(i, out pa, out pb);
					Vector3 edge = pb - pa;
					Vector3 edgeNormal = Vector3.Cross(edge, normal);
					edgeNormal = Vector3.Normalize(edgeNormal);
					float distance = Vector3.Dot(pt, edgeNormal);
					float edgeConst = Vector3.Dot(pa, edgeNormal);
					distance -= edgeConst;
					if (distance < -tolerance)
						return false;
				}
				return true;
			}
			return false;
		}
	}
}
