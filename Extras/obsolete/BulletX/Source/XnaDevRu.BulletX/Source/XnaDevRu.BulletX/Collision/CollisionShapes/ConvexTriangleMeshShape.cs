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
	/// ConvexTriangleMeshShape is a convex hull of a triangle mesh. If you just have a point cloud, you can use ConvexHullShape instead.
	/// It uses the StridingMeshInterface instead of a point cloud. This can avoid the duplication of the triangle mesh data.
	/// </summary>
	public class ConvexTriangleMeshShape : PolyhedralConvexShape
	{
		private StridingMeshInterface _stridingMesh;

		public ConvexTriangleMeshShape(StridingMeshInterface meshInterface)
		{
			_stridingMesh = meshInterface;
		}

		public StridingMeshInterface getStridingMesh()
		{
			return _stridingMesh;
		}

		public override int VertexCount
		{
			get
			{
				return 0;
			}
		}

		public override int EdgeCount
		{
			get
			{
				return 0;
			}
		}

		public override int PlaneCount
		{
			get
			{
				return 0;
			}
		}

		public override Vector3 LocalScaling
		{
			get
			{
				return base.LocalScaling;
			}
			set
			{
				_stridingMesh.Scaling = value;
			}
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.ConvexTriangleMesh;
			}
		}

		public override string Name
		{
			get
			{
				return "ConvexTrimesh";
			}
		}

		public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
		{
			pa = new Vector3();
			pb = new Vector3();
			BulletDebug.Assert(false);
		}

		public override void GetVertex(int i, out Vector3 vtx)
		{
			vtx = new Vector3();
			BulletDebug.Assert(false);
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

		public override Vector3 LocalGetSupportingVertex(Vector3 vec)
		{
			Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(vec);

			if (Margin != 0)
			{
				Vector3 vecnorm = vec;
				if (vecnorm.LengthSquared() < (MathHelper.Epsilon * MathHelper.Epsilon))
				{
					vecnorm = new Vector3(-1, -1, -1);
				}
				vecnorm = Vector3.Normalize(vecnorm);
				supVertex += Margin * vecnorm;
			}
			return supVertex;
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec0)
		{
			Vector3 supVec = new Vector3();

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

			LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback(vec);
			Vector3 aabbMax = new Vector3(1e30f, 1e30f, 1e30f);
			_stridingMesh.InternalProcessAllTriangles(supportCallback, -aabbMax, aabbMax);
			supVec = supportCallback.SupportVertexLocal;

			return supVec;
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			//use 'w' component of supportVerticesOut?
			/*{
				for (int i = 0; i < numVectors; i++)
				{
					supportVerticesOut[i][3] = -1e30f;
				}
			}*/
			for (int j = 0; j < vectors.Length; j++)
			{
				Vector3 vec = vectors[j];
				LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback(vec);
				Vector3 aabbMax = new Vector3(1e30f, 1e30f, 1e30f);
				_stridingMesh.InternalProcessAllTriangles(supportCallback, -aabbMax, aabbMax);
				supportVerticesOut[j] = supportCallback.SupportVertexLocal;
			}
		}
	}
}
