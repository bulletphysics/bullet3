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
	/// CollisionShape provides generic interface for collidable objects
	/// </summary>
	public abstract class CollisionShape
	{
		//debugging support
		private string _tempDebug;

		public abstract string Name { get; }
		public string ExtraDebugInfo { get { return _tempDebug; } set { _tempDebug = value; } }

		public bool IsPolyhedral
		{
			get
			{
				return BroadphaseProxy.IsPolyhedral(ShapeType);
			}
		}

		public bool IsConvex
		{
			get
			{
				return BroadphaseProxy.IsConvex(ShapeType);
			}
		}
		public bool IsConcave
		{
			get
			{
				return BroadphaseProxy.IsConcave(ShapeType);
			}
		}
		public bool IsCompound
		{
			get
			{
				return BroadphaseProxy.IsCompound(ShapeType);
			}
		}

		//isInfinite is used to catch simulation error (aabb check)
		public bool IsInfinite
		{
			get
			{
				return BroadphaseProxy.IsInfinite(ShapeType);
			}
		}

		public abstract float Margin { get; set; }
		public abstract Vector3 LocalScaling { get; set; }
		public abstract BroadphaseNativeTypes ShapeType { get; }
		

		public virtual void GetBoundingSphere(out Vector3 center, out float radius)
		{
			Matrix tr = Matrix.Identity;
			Vector3 aabbMin, aabbMax;

			GetAabb(tr, out aabbMin, out aabbMax);

			radius = (aabbMax - aabbMin).Length() * 0.5f;
			center = (aabbMin + aabbMax) * 0.5f;
		}

		public virtual float GetAngularMotionDisc()
		{
			Vector3 center;
			float disc;
			GetBoundingSphere(out center, out disc);
			disc += center.Length();
			return disc;
		}

		//calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
		//result is conservative
		public void CalculateTemporalAabb(Matrix currentTransform, Vector3 linearVelocity, Vector3 angularVelocity, float timeStep, out Vector3 temporalAabbMin, out Vector3 temporalAabbMax)
		{
			//start with static aabb
			GetAabb(currentTransform, out temporalAabbMin, out temporalAabbMax);

			float temporalAabbMaxx = temporalAabbMax.X;
			float temporalAabbMaxy = temporalAabbMax.Y;
			float temporalAabbMaxz = temporalAabbMax.Z;
			float temporalAabbMinx = temporalAabbMin.X;
			float temporalAabbMiny = temporalAabbMin.Y;
			float temporalAabbMinz = temporalAabbMin.Z;

			// add linear motion
			Vector3 linMotion = linearVelocity * timeStep;
			//todo: simd would have a vector max/min operation, instead of per-element access
			if (linMotion.X > 0)
				temporalAabbMaxx += linMotion.X;
			else
				temporalAabbMinx += linMotion.X;
			if (linMotion.Y > 0)
				temporalAabbMaxy += linMotion.Y;
			else
				temporalAabbMiny += linMotion.Y;
			if (linMotion.Z > 0)
				temporalAabbMaxz += linMotion.Z;
			else
				temporalAabbMinz += linMotion.Z;

			//add conservative angular motion
			float angularMotion = angularVelocity.Length() * GetAngularMotionDisc() * timeStep;
			Vector3 angularMotion3d = new Vector3(angularMotion, angularMotion, angularMotion);
			temporalAabbMin = new Vector3(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
			temporalAabbMax = new Vector3(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);

			temporalAabbMin -= angularMotion3d;
			temporalAabbMax += angularMotion3d;
		}

		public abstract void GetAabb(Matrix transform, out Vector3 aabbMin, out Vector3 aabbMax);

		public abstract void CalculateLocalInertia(float mass, out Vector3 inertia);
	}
}
