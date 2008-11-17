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
	public class SimpleBroadphase : OverlappingPairCache
	{
		private int _maxProxies;
		private List<SimpleBroadphaseProxy> _proxies = new List<SimpleBroadphaseProxy>();

		public SimpleBroadphase()
			: this(16384) { }

		public SimpleBroadphase(int maxProxies)
			: base()
		{
			_maxProxies = maxProxies;
		}

		public override BroadphaseProxy CreateProxy(Vector3 min, Vector3 max, BroadphaseNativeTypes shapeType, object userData, BroadphaseProxy.CollisionFilterGroups collisionFilterGroup, BroadphaseProxy.CollisionFilterGroups collisionFilterMask)
		{
			if (_proxies.Count >= _maxProxies)
			{
                BulletDebug.Assert(false);
				return null; //should never happen, but don't let the game crash ;-)
			}
            BulletDebug.Assert(min.X <= max.X && min.Y <= max.Y && min.Z <= max.Z);

			SimpleBroadphaseProxy proxy = new SimpleBroadphaseProxy(min, max, shapeType, userData, collisionFilterGroup, collisionFilterMask);
			_proxies.Add(proxy);

			return proxy;
		}


		public override void DestroyProxy(BroadphaseProxy proxy)
		{
			RemoveOverlappingPairsContainingProxy(proxy);
			_proxies.Remove(proxy as SimpleBroadphaseProxy);
		}

		public override void SetAabb(BroadphaseProxy proxy, Vector3 aabbMin, Vector3 aabbMax)
		{
			SimpleBroadphaseProxy simpleProxy = GetSimpleProxyFromProxy(proxy);
			simpleProxy.Minimum = aabbMin;
			simpleProxy.Maximum = aabbMax;
		}

		private SimpleBroadphaseProxy GetSimpleProxyFromProxy(BroadphaseProxy proxy)
		{
			return proxy as SimpleBroadphaseProxy;
		}

		public override void RefreshOverlappingPairs()
		{
			for (int i = 0; i < _proxies.Count; i++)
			{
				SimpleBroadphaseProxy proxyA = _proxies[i];

				for (int j = i + 1; j < _proxies.Count; j++)
				{
					SimpleBroadphaseProxy proxyB = _proxies[j];

					if (AabbOverlap(proxyA, proxyB))
					{
						if (FindPair(proxyA, proxyB) == null)
						{
							AddOverlappingPair(proxyA, proxyB);
						}
					}
				}
			}

			CheckOverlapCallback check = new CheckOverlapCallback();
			ProcessAllOverlappingPairs(check);
		}

		public static bool AabbOverlap(SimpleBroadphaseProxy proxyA, SimpleBroadphaseProxy proxyB)
		{
			return proxyA.Minimum.X <= proxyB.Maximum.X && proxyB.Minimum.X <= proxyA.Maximum.X &&
				   proxyA.Minimum.Y <= proxyB.Maximum.Y && proxyB.Minimum.Y <= proxyA.Maximum.Y &&
				   proxyA.Minimum.Z <= proxyB.Maximum.Z && proxyB.Minimum.Z <= proxyA.Maximum.Z;
		}

		private void Validate()
		{
			for (int i = 0; i < _proxies.Count; i++)
			{
				for (int j = i + 1; j < _proxies.Count; j++)
				{
					if (_proxies[i] == _proxies[j])
						throw new BulletException();
				}
			}
		}
	}

	public class CheckOverlapCallback : IOverlapCallback
	{
		public bool ProcessOverlap(ref BroadphasePair pair)
		{
			return (!SimpleBroadphase.AabbOverlap(pair.ProxyA as SimpleBroadphaseProxy, pair.ProxyB as SimpleBroadphaseProxy));
		}
	}
}
