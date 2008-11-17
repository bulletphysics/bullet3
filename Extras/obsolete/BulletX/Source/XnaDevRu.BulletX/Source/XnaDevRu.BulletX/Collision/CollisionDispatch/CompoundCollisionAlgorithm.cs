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
using System.Diagnostics;
using Microsoft.Xna.Framework;

namespace XnaDevRu.BulletX
{
	public class CompoundCollisionAlgorithm : CollisionAlgorithm
	{
		private List<CollisionAlgorithm> _childCollisionAlgorithms;
		private bool _isSwapped;

		public CompoundCollisionAlgorithm(
			CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo,
			CollisionObject bodyA,
			CollisionObject bodyB, bool isSwapped)
			: base(collisionAlgorithmConstructionInfo)
		{
			//Begin
			_isSwapped = isSwapped;

			CollisionObject collisionObject = isSwapped ? bodyB : bodyA;
			CollisionObject otherObject = isSwapped ? bodyA : bodyB;

			BulletDebug.Assert(collisionObject.CollisionShape.IsCompound);

			CompoundShape compoundShape = collisionObject.CollisionShape as CompoundShape;
			int childrenNumber = compoundShape.ChildShapeCount;
			int index = 0;

			_childCollisionAlgorithms = new List<CollisionAlgorithm>(childrenNumber);

			for (index = 0; index < childrenNumber; index++)
			{
				CollisionShape childShape = compoundShape.GetChildShape(index);
				CollisionShape orgShape = collisionObject.CollisionShape;

				collisionObject.CollisionShape = childShape;
				_childCollisionAlgorithms[index] = collisionAlgorithmConstructionInfo.Dispatcher.FindAlgorithm(collisionObject, otherObject);
				collisionObject.CollisionShape = orgShape;
			}
		}

		public override void ProcessCollision(
			CollisionObject bodyA,
			CollisionObject bodyB,
			DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			//Begin

			CollisionObject collisionObject = _isSwapped ? bodyB : bodyB;
			CollisionObject otherObject = _isSwapped ? bodyA : bodyB;

			//Debug.Assert(collisionObject.getCollisionShape().isCompound());
			BulletDebug.Assert(collisionObject.CollisionShape.IsCompound);

			CompoundShape compoundShape = (CompoundShape)collisionObject.CollisionShape;

			int childrenNumber = _childCollisionAlgorithms.Count;

			for (int i = 0; i < childrenNumber; i++)
			{
				CompoundShape childShape = compoundShape.GetChildShape(i) as CompoundShape;

				Matrix orgTransform = collisionObject.WorldTransform;
				CollisionShape orgShape = collisionObject.CollisionShape;

				Matrix childTransform = compoundShape.GetChildTransform(i);
				Matrix newChildWorld = orgTransform * childTransform;

				collisionObject.WorldTransform = newChildWorld;
				collisionObject.CollisionShape = childShape;
				_childCollisionAlgorithms[i].ProcessCollision(collisionObject, otherObject, dispatchInfo, resultOut);

				collisionObject.CollisionShape = orgShape;
				collisionObject.WorldTransform = orgTransform;
			}
		}

		public override float CalculateTimeOfImpact(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			CollisionObject collisionObject = _isSwapped ? bodyB : bodyA;
			CollisionObject otherObject = _isSwapped ? bodyA : bodyB;

			BulletDebug.Assert(collisionObject.CollisionShape.IsCompound);

			CompoundShape compoundShape = (CompoundShape)collisionObject.CollisionShape;

			float hitFraction = 1.0f;

			for (int i = 0; i < _childCollisionAlgorithms.Count; i++)
			{
				CollisionShape childShape = compoundShape.GetChildShape(i);

				Matrix orgTransform = collisionObject.WorldTransform;
				CollisionShape orgShape = collisionObject.CollisionShape;

				Matrix childTransform = compoundShape.GetChildTransform(i);
				Matrix newChildWorld = orgTransform * childTransform;
				collisionObject.WorldTransform = newChildWorld;

				collisionObject.CollisionShape = childShape;
				float frac = _childCollisionAlgorithms[i].CalculateTimeOfImpact(
					collisionObject, otherObject, dispatchInfo, resultOut
				);

				if (frac < hitFraction)
				{
					hitFraction = frac;
				}

				collisionObject.CollisionShape = orgShape;
				collisionObject.WorldTransform = orgTransform;
			}

			return hitFraction;
		}

		public class CreateFunc : CollisionAlgorithmCreateFunction
		{
			public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			{
				return new CompoundCollisionAlgorithm(collisionAlgorithmConstructionInfo, bodyA, bodyB, false);
			}
		};

		public class SwappedCreateFunc : CollisionAlgorithmCreateFunction
		{
			public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			{
				return new CompoundCollisionAlgorithm(collisionAlgorithmConstructionInfo, bodyA, bodyB, true);
			}
		};
	}
}
