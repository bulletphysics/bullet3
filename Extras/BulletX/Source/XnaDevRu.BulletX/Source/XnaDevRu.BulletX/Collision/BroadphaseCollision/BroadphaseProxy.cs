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
	public class BroadphaseProxy
	{
		//Usually the client CollisionObject or Rigidbody class
		private object _clientObject;
		private CollisionFilterGroups _collisionFilterGroup;
		private CollisionFilterGroups _collisionFilterMask;
		private readonly int _comparisonID;

		private static int _globalCount = 0;

		public BroadphaseProxy()
		{
			_comparisonID = _globalCount++;
		}

		public BroadphaseProxy(object userData, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask)
			: this()
		{
			_clientObject = userData;
			_collisionFilterGroup = collisionFilterGroup;
			_collisionFilterMask = collisionFilterMask;
		}

		public object ClientData { get { return _clientObject; } set { _clientObject = value; } }
		public CollisionFilterGroups CollisionFilterGroup { get { return _collisionFilterGroup; } set { _collisionFilterGroup = value; } }
		public CollisionFilterGroups CollisionFilterMask { get { return _collisionFilterMask; } set { _collisionFilterMask = value; } }
		internal int ComparisonID { get { return _comparisonID; } }

		public static bool IsPolyhedral(BroadphaseNativeTypes proxyType)
		{
			return (proxyType < BroadphaseNativeTypes.ImplicitConvexShapes);
		}

		public static bool IsConvex(BroadphaseNativeTypes proxyType)
		{
			return (proxyType < BroadphaseNativeTypes.ConcaveShapesStart);
		}

		public static bool IsConcave(BroadphaseNativeTypes proxyType)
		{
			return ((proxyType > BroadphaseNativeTypes.ConcaveShapesStart) &&
				(proxyType < BroadphaseNativeTypes.ConcaveShapesEnd));
		}
		public static bool IsCompound(BroadphaseNativeTypes proxyType)
		{
			return (proxyType == BroadphaseNativeTypes.Compound);
		}
		public static bool IsInfinite(BroadphaseNativeTypes proxyType)
		{
			return (proxyType == BroadphaseNativeTypes.StaticPlane);
		}

		//optional filtering to cull potential collisions
		public enum CollisionFilterGroups
		{
			Default = 1,
			Static = 2,
			Kinematic = 4,
			Debris = 8,
			Sensor = 16,
			All = Default | Static | Kinematic | Debris | Sensor,
		}
	}
}
