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

namespace XnaDevRu.BulletX
{
	public abstract class OverlappingPairCache : IBroadphase
	{
		private static int _overlappingPairCount = 0;
		private List<BroadphasePair> _overlappingPairs = new List<BroadphasePair>();
		//during the dispatch, check that user doesn't destroy/create proxy
		private bool _blockedForChanges;

		public List<BroadphasePair> OverlappingPairs { get { return _overlappingPairs; } set { _overlappingPairs = value; } }
		public bool BlockedForChanges { get { return _blockedForChanges; } set { _blockedForChanges = value; } }

		public static int OverlappingPairCount { get { return _overlappingPairCount; } set { _overlappingPairCount = value; } }

		public void RemoveOverlappingPair(BroadphasePair pair)
		{
			if (!_overlappingPairs.Contains(pair))
				return;

			CleanOverlappingPair(ref pair);
			_overlappingPairs.Remove(pair);
		}

		public void AddOverlappingPair(BroadphaseProxy proxyA, BroadphaseProxy proxyB)
		{
			//don't add overlap with own
			bool test = proxyA != proxyB;
            BulletDebug.Assert(proxyA != proxyB);

			if (!NeedsBroadphaseCollision(proxyA, proxyB))
				return;

			BroadphasePair pair = new BroadphasePair(proxyA, proxyB);
			_overlappingPairs.Add(pair);
			_overlappingPairCount++;
		}

		//this FindPair becomes really slow. Either sort the list to speedup the query, or
		//use a different solution. It is mainly used for Removing overlapping pairs. Removal could be delayed.
		//we could keep a linked list in each proxy, and store pair in one of the proxies (with lowest memory address)
		//Also we can use a 2D bitmap, which can be useful for a future GPU implementation
		public BroadphasePair FindPair(BroadphaseProxy proxyA, BroadphaseProxy proxyB)
		{
			if (!NeedsBroadphaseCollision(proxyA, proxyB))
				return null;

			BroadphasePair pair = new BroadphasePair(proxyA, proxyB);
			for (int i = 0; i < _overlappingPairs.Count; i++)
			{
				if (_overlappingPairs[i] == pair)
				{
					return _overlappingPairs[i];
				}
			}

			return null;
		}

		public void CleanProxyFromPairs(BroadphaseProxy proxy)
		{
			for (int i = 0; i < _overlappingPairs.Count; i++)
			{
				BroadphasePair pair = _overlappingPairs[i];
				if (pair.ProxyA == proxy ||
					pair.ProxyB == proxy)
				{
                    CleanOverlappingPair(ref pair);
                    _overlappingPairs[i] = pair;
				}
			}
		}

		public void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy)
		{
			for (int i = _overlappingPairs.Count - 1; i >= 0; i--)
			{
				BroadphasePair pair = _overlappingPairs[i];
				if (pair.ProxyA == proxy ||
					pair.ProxyB == proxy)
				{
					RemoveOverlappingPair(pair);
					i++;
				}
			}
		}

		public bool NeedsBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
		{
			bool collides = (proxy0.CollisionFilterGroup & proxy1.CollisionFilterMask) != 0;
			collides = collides && ((proxy1.CollisionFilterGroup & proxy0.CollisionFilterMask) != 0);

			return collides;
		}

		public virtual void ProcessAllOverlappingPairs(IOverlapCallback callback)
		{
			List<BroadphasePair> deleting = new List<BroadphasePair>();
            for (int i = 0; i < _overlappingPairs.Count; i++)
			{
                BroadphasePair p = _overlappingPairs[i];
				if (callback.ProcessOverlap(ref p))
				{
                    CleanOverlappingPair(ref p);
					deleting.Add(p);
					_overlappingPairCount--;
				}
			}

			for (int i = 0; i < deleting.Count; i++)
				_overlappingPairs.Remove(deleting[i]);
		}

		public void CleanOverlappingPair(ref BroadphasePair pair)
		{
            if (pair.CollisionAlgorithm != null)
            {
				if (pair.CollisionAlgorithm is IDisposable)
					(pair.CollisionAlgorithm as IDisposable).Dispose();
                pair.CollisionAlgorithm = null;
            }
		}

		public abstract void RefreshOverlappingPairs();

		#region IBroadphase Members
		public abstract BroadphaseProxy CreateProxy(Microsoft.Xna.Framework.Vector3 min, Microsoft.Xna.Framework.Vector3 max, BroadphaseNativeTypes shapeType, object userData, BroadphaseProxy.CollisionFilterGroups collisionFilterGroup, BroadphaseProxy.CollisionFilterGroups collisionFilterMask);

		public abstract void DestroyProxy(BroadphaseProxy proxy);

		public abstract void SetAabb(BroadphaseProxy proxy, Microsoft.Xna.Framework.Vector3 aabbMin, Microsoft.Xna.Framework.Vector3 aabbMax);

		#endregion
	}
}
