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
	public abstract class DiscreteCollisionDetectorInterface
	{
		public abstract class Result
		{
			public abstract void SetShapeIdentifiers(int partIdA, int indexA, int partIdB, int indexB);
			public abstract void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth);
		}

		public class ClosestPointInput
		{
			private float _maximumDistanceSquared;
			private Matrix _transformA, _transformB;

			#region Properties
			public Matrix TransformB
			{
				get { return _transformB; }
				set { _transformB = value; }
			}

			public Matrix TransformA
			{
				get { return _transformA; }
				set { _transformA = value; }
			}

			public float MaximumDistanceSquared
			{
				get { return _maximumDistanceSquared; }
				set { _maximumDistanceSquared = value; }
			}
			#endregion

			public ClosestPointInput()
			{
				_maximumDistanceSquared = 1e30f;
			}
		}

		public abstract void GetClosestPoints(ClosestPointInput input, Result output, IDebugDraw debugDraw);
	}

	public class StorageResult : DiscreteCollisionDetectorInterface.Result
	{
		private Vector3 _closestPointInB;
		private Vector3 _normalOnSurfaceB;
		private float _distance; //negative means penetration !

		#region Properties

		public float Distance
		{
			get { return _distance; }
			set { _distance = value; }
		}
		public Vector3 NormalOnSurfaceB
		{
			get { return _normalOnSurfaceB; }
			set { _normalOnSurfaceB = value; }
		}
		public Vector3 ClosestPointInB
		{
			get { return _closestPointInB; }
			set { _closestPointInB = value; }
		}

		#endregion

		public StorageResult()
		{
			_distance = 1e30f;
		}

		public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth)
		{
			if (depth < _distance)
			{
				_normalOnSurfaceB = normalOnBInWorld;
				_closestPointInB = pointInWorld;
				_distance = depth;
			}
		}

		public override void SetShapeIdentifiers(int partId0, int index0, int partId1, int index1)
		{
			throw new Exception("The method or operation is not implemented.");
		}
	}
}
