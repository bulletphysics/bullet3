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

namespace XnaDevRu.BulletX.Dynamics
{
	public class RaycastVehicle : TypedConstraint
	{
		public override void BuildJacobian()
		{
			throw new Exception("The method or operation is not implemented.");
		}

		public override void SolveConstraint(float timeStep)
		{
			throw new Exception("The method or operation is not implemented.");
		}

		public int getNumWheels()
		{
			throw new Exception("The method or operation is not implemented.");
		}

		public WheelInfo getWheelInfo(int v)
		{
			throw new Exception("The method or operation is not implemented.");
		}

		public void updateWheelTransform(int v, bool p)
		{
			throw new Exception("The method or operation is not implemented.");
		}

		public int getRightAxis()
		{
			throw new Exception("The method or operation is not implemented.");
		}

		public void updateVehicle(float timeStep)
		{
			throw new Exception("The method or operation is not implemented.");
		}
	}

	public class DefaultVehicleRaycaster : IVehicleRaycaster
	{
		DynamicsWorld _dynamicsWorld;

		public DefaultVehicleRaycaster(DynamicsWorld world)
		{
			_dynamicsWorld = world;
		}

		public object CastRay(Vector3 from, Vector3 to, out VehicleRaycasterResult result)
		{
			CollisionWorld.ClosestRayResultCallback rayCallback = new CollisionWorld.ClosestRayResultCallback(from, to);
			_dynamicsWorld.RayTest(from, to, rayCallback);

			result = new VehicleRaycasterResult();

			if (!rayCallback.HasHit) return 0;
			RigidBody body = RigidBody.Upcast(rayCallback.CollisionObject);
			if (body == null) return 0;

			result.HitPointInWorld = rayCallback.HitPointWorld;
			result.HitNormalInWorld = rayCallback.HitNormalWorld;
			result.HitNormalInWorld.Normalize();
			result.DistFraction = rayCallback.ClosestHitFraction;
			return body;
		}
	}
}