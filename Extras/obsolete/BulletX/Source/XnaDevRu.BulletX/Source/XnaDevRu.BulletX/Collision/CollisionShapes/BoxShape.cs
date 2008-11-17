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
	public class BoxShape : PolyhedralConvexShape
	{
		public BoxShape(Vector3 boxHalfExtents)
		{
			ImplicitShapeDimensions = boxHalfExtents;
		}

		public override int VertexCount
		{
			get
			{
				return 8;
			}
		}

		public override int EdgeCount
		{
			get
			{
				return 12;
			}
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.Box;
			}
		}

		public override string Name
		{
			get
			{
				return "Box";
			}
		}

		public override int PreferredPenetrationDirectionsCount
		{
			get
			{
				return 6;
			}
		}

		public override int PlaneCount
		{
			get
			{
				return 6;
			}
		}

		public Vector3 HalfExtents { get { return ImplicitShapeDimensions * LocalScaling; } }

		public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
		{
			int edgeVert0 = 0;
			int edgeVert1 = 0;

			switch (i)
			{
				case 0:
					edgeVert0 = 0;
					edgeVert1 = 1;
					break;
				case 1:
					edgeVert0 = 0;
					edgeVert1 = 2;
					break;
				case 2:
					edgeVert0 = 1;
					edgeVert1 = 3;

					break;
				case 3:
					edgeVert0 = 2;
					edgeVert1 = 3;
					break;
				case 4:
					edgeVert0 = 0;
					edgeVert1 = 4;
					break;
				case 5:
					edgeVert0 = 1;
					edgeVert1 = 5;

					break;
				case 6:
					edgeVert0 = 2;
					edgeVert1 = 6;
					break;
				case 7:
					edgeVert0 = 3;
					edgeVert1 = 7;
					break;
				case 8:
					edgeVert0 = 4;
					edgeVert1 = 5;
					break;
				case 9:
					edgeVert0 = 4;
					edgeVert1 = 6;
					break;
				case 10:
					edgeVert0 = 5;
					edgeVert1 = 7;
					break;
				case 11:
					edgeVert0 = 6;
					edgeVert1 = 7;
					break;
				default:
					throw new BulletException();

			}

			GetVertex(edgeVert0, out pa);
			GetVertex(edgeVert1, out pb);
		}

		public override void GetVertex(int i, out Vector3 vtx)
		{
			Vector3 halfExtents = HalfExtents;

			vtx = new Vector3(
					halfExtents.X * (1 - (i & 1)) - halfExtents.X * (i & 1),
					halfExtents.Y * (1 - ((i & 2) >> 1)) - halfExtents.Y * ((i & 2) >> 1),
					halfExtents.Z * (1 - ((i & 4) >> 2)) - halfExtents.Z * ((i & 4) >> 2));
		}

		public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
		{
			//this plane might not be aligned...
			Vector4 plane;
			GetPlaneEquation(out plane, i);
			planeNormal = new Vector3(plane.X, plane.Y, plane.Z);
			planeSupport = LocalGetSupportingVertex(-planeNormal);
		}

		public override bool IsInside(Vector3 pt, float tolerance)
		{
			Vector3 halfExtents = HalfExtents;

			//btScalar minDist = 2*tolerance;

			bool result =	(pt.X <= ( halfExtents.X + tolerance)) &&
							(pt.X >= (-halfExtents.X - tolerance)) &&
							(pt.Y <= ( halfExtents.Y + tolerance)) &&
							(pt.Y >= (-halfExtents.Y - tolerance)) &&
							(pt.Z <= ( halfExtents.Z + tolerance)) &&
							(pt.Z >= (-halfExtents.Z - tolerance));

			return result;
		}

		public override Vector3 LocalGetSupportingVertex(Vector3 vec)
		{
			Vector3 halfExtents = HalfExtents;

			return new Vector3( vec.X < 0.0f ? -halfExtents.X : halfExtents.X,
								vec.Y < 0.0f ? -halfExtents.Y : halfExtents.Y,
								vec.Z < 0.0f ? -halfExtents.Z : halfExtents.Z);
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
		{
			Vector3 halfExtents = HalfExtents;
			Vector3 margin = new Vector3(Margin, Margin, Margin);
			halfExtents -= margin;

			return new Vector3( vec.X < 0.0f ? -halfExtents.X : halfExtents.X,
								vec.Y < 0.0f ? -halfExtents.Y : halfExtents.Y,
								vec.Z < 0.0f ? -halfExtents.Z : halfExtents.Z);
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			Vector3 halfExtents = HalfExtents;
			Vector3 margin = new Vector3(Margin, Margin, Margin);
			halfExtents -= margin;

			for (int i = 0; i < vectors.Length; i++)
			{
				Vector3 vec = vectors[i];
				supportVerticesOut[i] = new Vector3(vec.X < 0.0f ? -halfExtents.X : halfExtents.X,
													vec.Y < 0.0f ? -halfExtents.Y : halfExtents.Y,
													vec.Z < 0.0f ? -halfExtents.Z : halfExtents.Z);
			}
		}

		public virtual void GetPlaneEquation(out Vector4 plane, int i)
		{
			Vector3 halfExtents = HalfExtents;

			switch (i)
			{
				case 0:
					plane = new Vector4(1, 0, 0, 0);
					plane.W = -halfExtents.X;
					break;
				case 1:
					plane = new Vector4(-1, 0, 0, 0);
					plane.W = -halfExtents.X;
					break;
				case 2:
					plane = new Vector4(0, 1, 0, 0);
					plane.W = -halfExtents.Y;
					break;
				case 3:
					plane = new Vector4(0, -1, 0, 0);
					plane.W = -halfExtents.Y;
					break;
				case 4:
					plane = new Vector4(0, 0, 1, 0);
					plane.W = -halfExtents.Z;
					break;
				case 5:
					plane = new Vector4(0, 0, -1, 0);
					plane.W = -halfExtents.Z;
					break;
				default:
					throw new BulletException();
			}
		}

		public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
		{
			switch (index)
			{
				case 0:
					penetrationVector = new Vector3(1, 0, 0);
					break;
				case 1:
					penetrationVector = new Vector3(-1, 0, 0);
					break;
				case 2:
					penetrationVector = new Vector3(0, 1, 0);
					break;
				case 3:
					penetrationVector = new Vector3(0, -1, 0);
					break;
				case 4:
					penetrationVector = new Vector3(0, 0, 1);
					break;
				case 5:
					penetrationVector = new Vector3(0, 0, -1);
					break;
				default:
					throw new BulletException();
			}
		}

		public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			Vector3 halfExtents = HalfExtents;

			Matrix abs_b = MathHelper.Absolute(t);
			Vector3 center = t.Translation;
			Vector3 row1 = new Vector3(abs_b.M11, abs_b.M12, abs_b.M13);
			Vector3 row2 = new Vector3(abs_b.M21, abs_b.M22, abs_b.M23);
			Vector3 row3 = new Vector3(abs_b.M31, abs_b.M32, abs_b.M33);
			Vector3 extent = new Vector3(Vector3.Dot(row1, halfExtents),
										 Vector3.Dot(row2, halfExtents),
										 Vector3.Dot(row3, halfExtents));
			extent += new Vector3(Margin, Margin, Margin);

			aabbMin = center - extent;
			aabbMax = center + extent;
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			Vector3 halfExtents = HalfExtents;

			float lx = 2f * (halfExtents.X);
			float ly = 2f * (halfExtents.Y);
			float lz = 2f * (halfExtents.Z);

			inertia = new Vector3();
			inertia.X = mass / (12.0f) * (ly * ly + lz * lz);
			inertia.Y = mass / (12.0f) * (lx * lx + lz * lz);
			inertia.Z = mass / (12.0f) * (lx * lx + ly * ly);
		}
	}
}
