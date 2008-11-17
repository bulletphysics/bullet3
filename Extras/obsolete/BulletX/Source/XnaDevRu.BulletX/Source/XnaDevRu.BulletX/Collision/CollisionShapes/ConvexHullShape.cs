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
	/// ConvexHullShape implements an implicit (getSupportingVertex) Convex Hull of a Point Cloud (vertices)
	/// No connectivity is needed. localGetSupportingVertex iterates linearly though all vertices.
	/// on modern hardware, due to cache coherency this isn't that bad. Complex algorithms tend to trash the cash.
	/// (memory is much slower then the cpu)
	/// </summary>
	public class ConvexHullShape : PolyhedralConvexShape
	{
		private List<Vector3> _points = new List<Vector3>();

		public ConvexHullShape() { }

		public override int VertexCount
		{
			get
			{
				return _points.Count;
			}
		}

		public override int EdgeCount
		{
			get
			{
				return _points.Count;
			}
		}

		public override int PlaneCount
		{
			get
			{
				return 0;
			}
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.ConvexHull;
			}
		}

		public override string Name
		{
			get
			{
				return "Convex";
			}
		}

		public override Vector3 LocalGetSupportingVertex(Vector3 vec)
		{
			Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(vec);

			if (Margin != 0)
			{
				Vector3 vecnorm = vec;
				if (vecnorm.LengthSquared() < (MathHelper.Epsilon * MathHelper.Epsilon))
				{
					vecnorm=new Vector3(-1, -1, -1);
				}
				vecnorm = Vector3.Normalize(vecnorm);
				supVertex += Margin * vecnorm;
			}
			return supVertex;
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec0)
		{
			Vector3 supVec = new Vector3();
			float newDot, maxDot = -1e30f;

			Vector3 vec = vec0;
			float lenSqr = vec.LengthSquared();
			if (lenSqr < 0.0001f)
			{
				vec = new Vector3(1, 0, 0);
			}
			else
			{
				float rlen = 1f / (float)Math.Sqrt(lenSqr);
				vec *= rlen;
			}

			for (int i = 0; i < _points.Count; i++)
			{
				Vector3 vtx = _points[i] * LocalScaling;

				newDot = Vector3.Dot(vec, vtx);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					supVec = vtx;
				}
			}
			return supVec;
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			float newDot;
			//use 'w' component of supportVerticesOut?
			/*{
				for (int i = 0; i < numVectors; i++)
				{
					supportVerticesOut[i][3] = -1e30f;
				}
			}*/
			#warning Warning!
			for (int i = 0; i < _points.Count; i++)
			{
				Vector3 vtx = _points[i] * LocalScaling;

				for (int j = 0; j < vectors.Length; j++)
				{
					newDot = Vector3.Dot(vectors[j], vtx);
					if (newDot > -1e30f)
					{
						//WARNING: don't swap next lines, the w component would get overwritten!
						supportVerticesOut[j] = vtx;
						//supportVerticesOut[j][3] = newDot;
						#warning Warning!
					}
				}
			}
		}

		public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
		{
			int index0 = i % _points.Count;
			int index1 = (i + 1) % _points.Count;
			pa = _points[index0] * LocalScaling;
			pb = _points[index1] * LocalScaling;
		}

		public override void GetVertex(int i, out Vector3 vtx)
		{
			vtx = _points[i] * LocalScaling;
		}

		public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
		{
			planeNormal = new Vector3();
			planeSupport = new Vector3();
			BulletDebug.Assert(false);
		}

		public override bool IsInside(Vector3 pt, float tolerance)
		{
			BulletDebug.Assert(false);
			return false;
		}
	}
}
