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
	/// Concave triangle mesh. Uses an interface to access the triangles to allow for sharing graphics/physics triangles.
	/// </summary>
	public class TriangleMeshShape : ConcaveShape
	{
		private StridingMeshInterface _meshInterface;
		private Vector3 _localAabbMin;
		private Vector3 _localAabbMax;

		public TriangleMeshShape(StridingMeshInterface meshInterface)
		{
			this._meshInterface = meshInterface;
			RecalcLocalAabb();
		}

		protected StridingMeshInterface MeshInterface { get { return _meshInterface; } set { _meshInterface = value; } }
		protected Vector3 LocalAabbMin { get { return _localAabbMin; } set { _localAabbMin = value; } }
		protected Vector3 LocalAabbMax { get { return _localAabbMax; } set { _localAabbMax = value; } }

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.TriangleMesh;
			}
		}

		public override Vector3 LocalScaling
		{
			get
			{
				return _meshInterface.Scaling;
			}
			set
			{
				_meshInterface.Scaling = value;
			}
		}

		public override string Name
		{
			get
			{
				return "TriangleMesh";
			}
		}

		public void RecalcLocalAabb()
		{
			{
				Vector3 vec = new Vector3();
				vec.X = 1f;
				Vector3 tmp = LocalGetSupportingVertex(vec);
				_localAabbMax.X = tmp.X + CollisionMargin;
				vec.X = -1f;
				tmp = LocalGetSupportingVertex(vec);
				_localAabbMin.X = tmp.X - CollisionMargin;
			}
			{
				Vector3 vec = new Vector3();
				vec.Y = 1f;
				Vector3 tmp = LocalGetSupportingVertex(vec);
				_localAabbMax.Y = tmp.Y + CollisionMargin;
				vec.Y = -1f;
				tmp = LocalGetSupportingVertex(vec);
				_localAabbMin.Y = tmp.Y - CollisionMargin;
			}
			{
				Vector3 vec = new Vector3();
				vec.Z = 1f;
				Vector3 tmp = LocalGetSupportingVertex(vec);
				_localAabbMax.Z = tmp.Z + CollisionMargin;
				vec.Z = -1f;
				tmp = LocalGetSupportingVertex(vec);
				_localAabbMin.Z = tmp.Z - CollisionMargin;
			}
		}

		public override void ProcessAllTriangles(ITriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
		{
            LocalProcessAllTriangles(callback, aabbMax, aabbMax);
		}

        protected void LocalProcessAllTriangles(ITriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
        {
            FilteredCallback filterCallback = new FilteredCallback(callback, aabbMin, aabbMax);
            _meshInterface.InternalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
        }

        public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			Vector3 localHalfExtents = 0.5f * (_localAabbMax - _localAabbMin);
			Vector3 localCenter = 0.5f * (_localAabbMax + _localAabbMin);

			Matrix abs_b = MathHelper.Absolute(t);

			Vector3 center = MathHelper.MatrixToVector(t, localCenter);

			Vector3 extent = new Vector3(Vector3.Dot(new Vector3(abs_b.M11, abs_b.M12, abs_b.M13), localHalfExtents),
				   Vector3.Dot(new Vector3(abs_b.M21, abs_b.M22, abs_b.M23), localHalfExtents),
				  Vector3.Dot(new Vector3(abs_b.M31, abs_b.M32, abs_b.M33), localHalfExtents));
			extent += new Vector3(Margin, Margin, Margin);

			aabbMin = center - extent;
			aabbMax = center + extent;
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			inertia = new Vector3();
			//moving concave objects not supported
			BulletDebug.Assert(false);
		}

		public virtual Vector3 LocalGetSupportingVertex(Vector3 vec)
		{
			Vector3 supportVertex;
			Matrix ident = Matrix.Identity;
			SupportVertexCallback supportCallback = new SupportVertexCallback(vec, ident);
			Vector3 aabbMax = new Vector3(1e30f, 1e30f, 1e30f);
			LocalProcessAllTriangles(supportCallback, -aabbMax, aabbMax);
			supportVertex = supportCallback.SupportVertexLocal;
			return supportVertex;
		}

		public virtual Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
		{
			BulletDebug.Assert(false);
			return LocalGetSupportingVertex(vec);
		}
	}
}
