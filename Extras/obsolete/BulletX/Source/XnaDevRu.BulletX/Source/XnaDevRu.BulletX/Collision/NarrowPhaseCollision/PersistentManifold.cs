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
	public delegate bool ContactDestroyedCallback(object userPersistentData);

	public class PersistentManifold
	{
		private static ContactDestroyedCallback _contactDestroyedCallback = null;
		private static float _contactBreakingThreshold = 0.02f;

		private ManifoldPoint[] _pointCache = new ManifoldPoint[4];

		// this two body pointers can point to the physics rigidbody class.
		// object will allow any rigidbody class
		private object _bodyA;
		private object _bodyB;
		private int _cachedPoints;

		public PersistentManifold(object bodyA, object bodyB)
		{
			_bodyA = bodyA;
			_bodyB = bodyB;
			_cachedPoints = 0;
		}

		public object BodyA { get { return _bodyA; } }
		public object BodyB { get { return _bodyB; } }

		public int ContactsCount { get { return _cachedPoints; } }

		public static ContactDestroyedCallback ContactDestroyedCallback { get { return _contactDestroyedCallback; } set { _contactDestroyedCallback = value; } }
		public static float ContactBreakingThreshold { get { return _contactBreakingThreshold; } }

		public void SetBodies(object bodyA, object bodyB)
		{
			_bodyA = bodyA;
			_bodyB = bodyB;
		}

		public ManifoldPoint GetContactPoint(int index)
		{
			if (index >= _cachedPoints)
				throw new ArgumentOutOfRangeException("index", "index must be smaller than cachedPoints");

			return _pointCache[index];
		}

		public int GetCacheEntry(ManifoldPoint newPoint)
		{
			float shortestDist = ContactBreakingThreshold * ContactBreakingThreshold;
			int size = ContactsCount;
			int nearestPoint = -1;
			for (int i = 0; i < size; i++)
			{
				ManifoldPoint mp = _pointCache[i];

				Vector3 diffA = mp.LocalPointA - newPoint.LocalPointA;
				float distToManiPoint = Vector3.Dot(diffA, diffA);
				if (distToManiPoint < shortestDist)
				{
					shortestDist = distToManiPoint;
					nearestPoint = i;
				}
			}
			return nearestPoint;
		}

		public void AddManifoldPoint(ManifoldPoint newPoint)
		{
			if (!ValidContactDistance(newPoint))
				throw new BulletException();

			int insertIndex = ContactsCount;
			if (insertIndex == 4)
			{
				//sort cache so best points come first, based on area
				insertIndex = SortCachedPoints(newPoint);
			}
			else
			{
				_cachedPoints++;
			}
			ReplaceContactPoint(newPoint, insertIndex);
		}

		public void RemoveContactPoint(int index)
		{
			ClearUserCache(_pointCache[index]);

			int lastUsedIndex = ContactsCount - 1;
			_pointCache[index] = _pointCache[lastUsedIndex];
			//get rid of duplicated userPersistentData pointer
			_pointCache[lastUsedIndex].UserPersistentData = null;
			_cachedPoints--;
		}

		public void ReplaceContactPoint(ManifoldPoint newPoint, int insertIndex)
		{
			BulletDebug.Assert(ValidContactDistance(newPoint));

			if (_pointCache[insertIndex] != null)
			{
				int lifeTime = _pointCache[insertIndex].LifeTime;
				BulletDebug.Assert(lifeTime >= 0);
				object cache = _pointCache[insertIndex].UserPersistentData;

				_pointCache[insertIndex] = newPoint;

				_pointCache[insertIndex].UserPersistentData = cache;
				_pointCache[insertIndex].LifeTime = lifeTime;
			}
			else
			{
				_pointCache[insertIndex] = newPoint;
			}

			//ClearUserCache(_pointCache[insertIndex]);
			//_pointCache[insertIndex] = newPoint;
		}

		public bool ValidContactDistance(ManifoldPoint pt)
		{
			return pt.Distance <= ContactBreakingThreshold;
		}

		// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
		public void RefreshContactPoints(Matrix trA, Matrix trB)
		{
			// first refresh worldspace positions and distance
			for (int i = ContactsCount - 1; i >= 0; i--)
			{
				ManifoldPoint manifoldPoint = _pointCache[i];
				manifoldPoint.PositionWorldOnA = MathHelper.MatrixToVector(trA,manifoldPoint.LocalPointA);
				manifoldPoint.PositionWorldOnB = MathHelper.MatrixToVector(trB, manifoldPoint.LocalPointB);
				manifoldPoint.Distance = Vector3.Dot(manifoldPoint.PositionWorldOnA - manifoldPoint.PositionWorldOnB, manifoldPoint.NormalWorldOnB);
				manifoldPoint.LifeTime++;
			}

			// then 
			float distance2d;
			Vector3 projectedDifference, projectedPoint;
			for (int i = ContactsCount - 1; i >= 0; i--)
			{

				ManifoldPoint manifoldPoint = _pointCache[i];
				//contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
				if (!ValidContactDistance(manifoldPoint))
				{
					RemoveContactPoint(i);
				}
				else
				{
					//contact also becomes invalid when relative movement orthogonal to normal exceeds margin
					projectedPoint = manifoldPoint.PositionWorldOnA - manifoldPoint.NormalWorldOnB * manifoldPoint.Distance;
					projectedDifference = manifoldPoint.PositionWorldOnB - projectedPoint;
					distance2d = Vector3.Dot(projectedDifference, projectedDifference);
					if (distance2d > ContactBreakingThreshold * ContactBreakingThreshold)
					{
						RemoveContactPoint(i);
					}
				}
			}
		}

		public void ClearManifold()
		{
			for (int i = 0; i < _cachedPoints; i++)
			{
				ClearUserCache(_pointCache[i]);
			}
			_cachedPoints = 0;
		}

		private void ClearUserCache(ManifoldPoint pt)
		{
			if (pt != null)
			{
				object oldPtr = pt.UserPersistentData;

				if (oldPtr != null)
				{
					if (pt.UserPersistentData != null && _contactDestroyedCallback != null)
					{
						_contactDestroyedCallback(pt.UserPersistentData);
						pt.UserPersistentData = null;
					}
				}
			}
		}

		// sort cached points so most isolated points come first
		private int SortCachedPoints(ManifoldPoint pt)
		{
			//calculate 4 possible cases areas, and take biggest area
			//also need to keep 'deepest'

			int maxPenetrationIndex = -1;
			float maxPenetration = pt.Distance;
			for (int i = 0; i < 4; i++)
			{
				if (_pointCache[i].Distance < maxPenetration)
				{
					maxPenetrationIndex = i;
					maxPenetration = _pointCache[i].Distance;
				}
			}

			float res0 = 0, res1 = 0, res2 = 0, res3 = 0;
			if (maxPenetrationIndex != 0)
			{
				Vector3 a0 = pt.LocalPointA - _pointCache[1].LocalPointA;
				Vector3 b0 = _pointCache[3].LocalPointA - _pointCache[2].LocalPointA;
				Vector3 cross = Vector3.Cross(a0, b0);
				res0 = cross.LengthSquared();
			}
			if (maxPenetrationIndex != 1)
			{
				Vector3 a1 = pt.LocalPointA - _pointCache[0].LocalPointA;
				Vector3 b1 = _pointCache[3].LocalPointA - _pointCache[2].LocalPointA;
				Vector3 cross = Vector3.Cross(a1, b1);
				res1 = cross.LengthSquared();
			}

			if (maxPenetrationIndex != 2)
			{
				Vector3 a2 = pt.LocalPointA - _pointCache[0].LocalPointA;
				Vector3 b2 = _pointCache[3].LocalPointA - _pointCache[1].LocalPointA;
				Vector3 cross = Vector3.Cross(a2, b2);
				res2 = cross.LengthSquared();
			}

			if (maxPenetrationIndex != 3)
			{
				Vector3 a3 = pt.LocalPointA - _pointCache[0].LocalPointA;
				Vector3 b3 = _pointCache[2].LocalPointA - _pointCache[1].LocalPointA;
				Vector3 cross = Vector3.Cross(a3, b3);
				res3 = cross.LengthSquared();
			}

			Vector4 maxvec = new Vector4(res0, res1, res2, res3);
			int biggestarea = MathHelper.ClosestAxis(maxvec);
			return biggestarea;
		}

		private int FindContactPoint(ManifoldPoint unUsed, int numUnused, ManifoldPoint pt) { return 0; }
	}
}
