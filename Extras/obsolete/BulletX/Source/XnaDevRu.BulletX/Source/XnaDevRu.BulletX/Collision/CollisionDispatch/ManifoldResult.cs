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
	public delegate bool ContactAddedCallback(ManifoldPoint contactPoint, CollisionObject collisionObjectA, int partIdA, int indexA, CollisionObject collisionObjectB, int partIdB, int indexB);

	public class ManifoldResult : DiscreteCollisionDetectorInterface.Result
	{
		private PersistentManifold _manifold;
		private static ContactAddedCallback _contactAddedCallback = null;

		//we need this for compounds
		private Matrix _rootTransA;
		private Matrix _rootTransB;

		private CollisionObject _bodyA;
		private CollisionObject _bodyB;
		private int _partIdA;
		private int _partIdB;
		private int _indexA;
		private int _indexB;

		public ManifoldResult()
		{
		}

		public ManifoldResult(CollisionObject bodyA, CollisionObject bodyB)
		{
			_bodyA = bodyA;
			_bodyB = bodyB;
			_rootTransA = bodyA.WorldTransform;
			_rootTransB = bodyB.WorldTransform;
		}

		public static ContactAddedCallback ContactAddedCallback { get { return _contactAddedCallback; } set { _contactAddedCallback = value; } }

		public void SetPersistentManifold(PersistentManifold manifold)
		{
			_manifold = manifold;
		}

		public override void SetShapeIdentifiers(int partIdA, int indexA, int partIdB, int indexB)
		{
			_partIdA = partIdA;
			_partIdB = partIdB;
			_indexA = indexA;
			_indexB = indexB;
		}

        public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth)
		{
			if (_manifold == null)
				throw new BulletException("Manifold Pointer is null.");

			//order in manifold needs to match

			if (depth > PersistentManifold.ContactBreakingThreshold)
				return;

			bool isSwapped = _manifold.BodyA != _bodyA;

			Vector3 pointA = pointInWorld + normalOnBInWorld * depth;
			Vector3 localA;
			Vector3 localB;

			if (isSwapped)
			{
				localA = MathHelper.InvXForm(_rootTransB, pointA);
				localB = MathHelper.InvXForm(_rootTransA, pointInWorld);
			}
			else
			{
				localA = MathHelper.InvXForm(_rootTransA, pointA);
				localB = MathHelper.InvXForm(_rootTransB, pointInWorld);
			}

			ManifoldPoint newPt = new ManifoldPoint(localA, localB, normalOnBInWorld, depth);

			int insertIndex = _manifold.GetCacheEntry(newPt);

			newPt.CombinedFriction = CalculateCombinedFriction(_bodyA, _bodyB);
			newPt.CombinedRestitution = CalculateCombinedRestitution(_bodyA, _bodyB);

			//User can override friction and/or restitution
			if (_contactAddedCallback != null &&
				//and if either of the two bodies requires custom material
				 ((_bodyA.CollisionFlags & CollisionOptions.CustomMaterialCallback) != 0 ||
				   (_bodyB.CollisionFlags & CollisionOptions.CustomMaterialCallback) != 0))
			{
				//experimental feature info, for per-triangle material etc.
				CollisionObject obj0 = isSwapped ? _bodyB : _bodyA;
				CollisionObject obj1 = isSwapped ? _bodyA : _bodyB;
				_contactAddedCallback(newPt, obj0, _partIdA, _indexA, obj1, _partIdB, _indexB);
			}

			if (insertIndex >= 0)
			{
				_manifold.ReplaceContactPoint(newPt, insertIndex);
			}
			else
			{
				_manifold.AddManifoldPoint(newPt);
			}
		}

		private float CalculateCombinedFriction(CollisionObject bodyA, CollisionObject bodyB)
		{
			float friction = bodyA.Friction * bodyB.Friction;

			float MaxFriction = 10;
			if (friction < -MaxFriction)
				friction = -MaxFriction;
			if (friction > MaxFriction)
				friction = MaxFriction;
			return friction;
		}

		private float CalculateCombinedRestitution(CollisionObject bodyA, CollisionObject bodyB)
		{
			return bodyA.Restitution * bodyB.Restitution;
		}
	}
}
