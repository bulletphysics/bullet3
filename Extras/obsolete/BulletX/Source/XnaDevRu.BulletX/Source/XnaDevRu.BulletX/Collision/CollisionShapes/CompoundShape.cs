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
	/// CompoundShape allows to store multiple other CollisionShapes
	/// This allows for concave collision objects. This is more general then the Static Concave TriangleMeshShape.
	/// </summary>
	public class CompoundShape : CollisionShape
	{
		private List<Matrix> _childTransforms = new List<Matrix>();
		private List<CollisionShape> _childShapes = new List<CollisionShape>();
		private Vector3 _localAabbMin;
		private Vector3 _localAabbMax;

		private OptimizedBvh _aabbTree;
		private float _collisionMargin;
		private Vector3 _localScaling;

		public CompoundShape()
		{
			_localAabbMin = new Vector3(1e30f, 1e30f, 1e30f);
			_localAabbMax = new Vector3(-1e30f, -1e30f, -1e30f);
			_aabbTree = null;
			_collisionMargin = 0f;
			_localScaling = new Vector3(1f, 1f, 1f);
		}

		public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			Vector3 localHalfExtents = 0.5f * (_localAabbMax - _localAabbMin);
			Vector3 localCenter = 0.5f * (_localAabbMax + _localAabbMin);

			Matrix abs_b = MathHelper.Absolute(t);

			Vector3 row1 = new Vector3(abs_b.M11, abs_b.M12, abs_b.M13);
			Vector3 row2 = new Vector3(abs_b.M21, abs_b.M22, abs_b.M23);
			Vector3 row3 = new Vector3(abs_b.M31, abs_b.M32, abs_b.M33);

			Vector3 center = new Vector3(Vector3.Dot(row1, localCenter) + t.Translation.X,
										 Vector3.Dot(row2, localCenter) + t.Translation.Y,
										 Vector3.Dot(row3, localCenter) + t.Translation.Z);

			Vector3 extent = new Vector3(Vector3.Dot(row1, localHalfExtents),
										 Vector3.Dot(row2, localHalfExtents),
										 Vector3.Dot(row3, localHalfExtents));

			aabbMin = center - extent;
			aabbMax = center + extent;
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.Compound;
			}
		}

		public override Vector3 LocalScaling
		{
			get
			{
				return _localScaling;
			}
			set
			{
				_localScaling = value;
			}
		}

		public override string Name
		{
			get
			{
				return "Compound";
			}
		}

		public override float Margin
		{
			get
			{
				return _collisionMargin;
			}
			set
			{
				_collisionMargin = value;
			}
		}

		public int ChildShapeCount { get { return _childShapes.Count; } }
		//this is optional, but should make collision queries faster, by culling non-overlapping nodes
		public OptimizedBvh AabbTree { get { return _aabbTree; } }

		public CollisionShape GetChildShape(int index)
		{
			return _childShapes[index];
		}

		public Matrix GetChildTransform(int index)
		{
			return _childTransforms[index];
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			//approximation: take the inertia from the aabb for now
			Matrix ident = Matrix.Identity;
			Vector3 aabbMin, aabbMax;
			GetAabb(ident, out aabbMin, out aabbMax);

			Vector3 halfExtents = (aabbMax - aabbMin) * 0.5f;

			float lx = 2f * (halfExtents.X);
			float ly = 2f * (halfExtents.Y);
			float lz = 2f * (halfExtents.Z);

			inertia = new Vector3();
			inertia.X = mass / (12.0f) * (ly * ly + lz * lz);
			inertia.Y = mass / (12.0f) * (lx * lx + lz * lz);
			inertia.Z = mass / (12.0f) * (lx * lx + ly * ly);
		}

		public void AddChildShape(Matrix localTransform, CollisionShape shape)
		{
			_childTransforms.Add(localTransform);
			_childShapes.Add(shape);

			//extend the local aabbMin/aabbMax
			Vector3 localAabbMin, localAabbMax;
			shape.GetAabb(localTransform, out localAabbMin, out localAabbMax);
			if (_localAabbMin.X > localAabbMin.X)
			{
				_localAabbMin.X = localAabbMin.X;
			}
			if (_localAabbMax.X < localAabbMax.X)
			{
				_localAabbMax.X = localAabbMax.X;
			}
			if (_localAabbMin.Y > localAabbMin.Y)
			{
				_localAabbMin.Y = localAabbMin.Y;
			}
			if (_localAabbMax.Y < localAabbMax.Y)
			{
				_localAabbMax.Y = localAabbMax.Y;
			}
			if (_localAabbMin.Z > localAabbMin.Z)
			{
				_localAabbMin.Z = localAabbMin.Z;
			}
			if (_localAabbMax.Z < localAabbMax.Z)
			{
				_localAabbMax.Z = localAabbMax.Z;
			}
		}
	}
}
