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
	public class SphereSphereCollisionAlgorithm : CollisionAlgorithm
	{
		private bool _ownManifold;
		private PersistentManifold _manifold;

		public SphereSphereCollisionAlgorithm(PersistentManifold manifold, CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			: base(collisionAlgorithmConstructionInfo)
		{
			_ownManifold = false;
			_manifold = manifold;

			if (_manifold == null)
			{
				_manifold = Dispatcher.GetNewManifold(bodyA, bodyB);
				_ownManifold = true;
			}
		}

		public SphereSphereCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo)
			: base(collisionAlgorithmConstructionInfo) { }

		~SphereSphereCollisionAlgorithm()
		{
			if (_ownManifold)
			{
				if (_manifold != null)
					Dispatcher.ReleaseManifold(_manifold);
			}
		}

		public override void ProcessCollision(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			if (_manifold == null)
				return;

			SphereShape sphereA = bodyA.CollisionShape as SphereShape;
			SphereShape sphereB = bodyB.CollisionShape as SphereShape;

			Vector3 diff = bodyA.WorldTransform.Translation - bodyB.WorldTransform.Translation;
			float len = diff.Length();
			float radiusA = sphereA.Radius;
			float radiusB = sphereB.Radius;

			//if distance positive, don't generate a new contact
			if (len > (radiusA + radiusB))
				return;

			//distance (negative means penetration)
			float dist = len - (radiusA + radiusB);

			Vector3 normalOnSurfaceB = diff / len;
			//point on A (worldspace)
			Vector3 posA = bodyA.WorldTransform.Translation - radiusA * normalOnSurfaceB;
			//point on B (worldspace)
			Vector3 posB = bodyB.WorldTransform.Translation + radiusB * normalOnSurfaceB;

			// report a contact. internally this will be kept persistent, and contact reduction is done
			resultOut.SetPersistentManifold(_manifold);
			resultOut.AddContactPoint(normalOnSurfaceB, posB, dist);
		}

		public override float CalculateTimeOfImpact(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			//not yet
			return 1f;
		}

		public class CreateFunc : CollisionAlgorithmCreateFunction
		{
			public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			{
				return new SphereSphereCollisionAlgorithm(null, collisionAlgorithmConstructionInfo, bodyA, bodyB);
			}
		}
	}
}
