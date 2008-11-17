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
	public class StaticPlaneShape : ConcaveShape
	{
		private Vector3 _localAabbMin;
		private Vector3 _localAabbMax;

		private Vector3 _planeNormal;
		private float _planeConstant;
		private Vector3 _localScaling;

		public StaticPlaneShape(Vector3 planeNormal, float planeConstant)
		{
			_planeNormal = planeNormal;
			_planeConstant = planeConstant;
			_localScaling = new Vector3();
		}

		protected Vector3 LocalAabbMin { get { return _localAabbMin; } set { _localAabbMin = value; } }
		protected Vector3 LocalAabbMax { get { return _localAabbMax; } set { _localAabbMax = value; } }

		protected Vector3 PlaneNormal { get { return _planeNormal; } set { _planeNormal = value; } }
		protected float PlaneConstant { get { return _planeConstant; } set { _planeConstant = value; } }

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.StaticPlane;
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
				return "StaticPlane";
			}
		}

		public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			Vector3 infvec = new Vector3(1e30f, 1e30f, 1e30f);

			Vector3 center = _planeNormal * _planeConstant;
			aabbMin = center + infvec * _planeNormal;
			aabbMax = aabbMin;
			MathHelper.SetMin(ref aabbMin, center - infvec * _planeNormal);
			MathHelper.SetMax(ref aabbMax, center - infvec * _planeNormal);

			aabbMin = new Vector3(-1e30f, -1e30f, -1e30f);
			aabbMax = new Vector3(1e30f, 1e30f, 1e30f);
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			//moving concave objects not supported
			inertia = new Vector3();
		}

        public override void ProcessAllTriangles(ITriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax) {
            Vector3 halfExtents = (aabbMax - aabbMin) * 0.5f;
            float radius = halfExtents.Length();
            Vector3 center = (aabbMax + aabbMin) * 0.5f;

            //this is where the triangles are generated, given AABB and plane equation (normal/constant)
            Vector3 tangentDir0 = new Vector3(), tangentDir1 = new Vector3();

            //tangentDir0/tangentDir1 can be precalculated
            MathHelper.PlaneSpace1(_planeNormal, ref tangentDir0, ref tangentDir1);

            Vector3 projectedCenter = center - (Vector3.Dot(_planeNormal, center) - _planeConstant) * _planeNormal;

            Vector3[] triangle = new Vector3[3];
            triangle[0] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;
            triangle[1] = projectedCenter + tangentDir0 * radius - tangentDir1 * radius;
            triangle[2] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;
            callback.ProcessTriangle(triangle, 0, 0);

            triangle[0] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;
            triangle[1] = projectedCenter - tangentDir0 * radius + tangentDir1 * radius;
            triangle[2] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;
            callback.ProcessTriangle(triangle, 0, 1);
        }
    }
}
