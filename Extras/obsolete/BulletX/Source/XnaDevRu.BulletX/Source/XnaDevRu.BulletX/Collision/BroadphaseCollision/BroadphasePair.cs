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
	public class BroadphasePair
	{
		private BroadphaseProxy _proxyA;
		private BroadphaseProxy _proxyB;

		private CollisionAlgorithm _algorithm;
		private object _userInfo;

		public BroadphasePair()
		{
		}

		public BroadphasePair(BroadphasePair other)
		{
			_proxyA = other._proxyA;
			_proxyB = other._proxyB;

			_algorithm = other._algorithm;
			_userInfo = null;
		}

		public BroadphasePair(BroadphaseProxy proxyA, BroadphaseProxy proxyB)
		{
			_proxyA = proxyA;
			_proxyB = proxyB;

			_algorithm = null;
			_userInfo = null;
		}

		public BroadphaseProxy ProxyA { get { return _proxyA; } set { _proxyA = value; } }
		public BroadphaseProxy ProxyB { get { return _proxyB; } set { _proxyB = value; } }

		public CollisionAlgorithm CollisionAlgorithm { get { return _algorithm; } set { _algorithm = value; } }
		public object UserInfo { get { return _userInfo; } set { _userInfo = value; } }

		public override int GetHashCode()
		{
			return _proxyA.GetHashCode() ^ _proxyB.GetHashCode();
		}

		public override bool Equals(object obj)
		{
			if (obj is BroadphasePair)
				return this == (BroadphasePair)obj;
			return false;
		}

		public static int ComparisonSort(BroadphasePair a, BroadphasePair b)
		{
			int aAId = a.ProxyA != null ? a.ProxyA.ComparisonID : -1;
			int aBId = a.ProxyB != null ? a.ProxyB.ComparisonID : -1;
			int aCId = a.CollisionAlgorithm != null ? a.CollisionAlgorithm.ComparisonID : -1;
			int bAId = b.ProxyA != null ? b.ProxyA.ComparisonID : -1;
			int bBId = b.ProxyB != null ? b.ProxyB.ComparisonID : -1;
			int bCId = b.CollisionAlgorithm != null ? b.CollisionAlgorithm.ComparisonID : -1;
			
			if	(aAId > bAId ||
				(a.ProxyA == b.ProxyA && aBId > bBId) ||
				(a.ProxyA == b.ProxyA && a.ProxyB == b.ProxyB && aCId > bCId))
				return -1;
			else
				return 1;
		}

		public static bool operator ==(BroadphasePair a, BroadphasePair b)
		{
			if (object.Equals(a, null) && object.Equals(b, null))
				return true;
			if (object.Equals(a, null) || object.Equals(b, null))
				return false;

			return (a.ProxyA == b.ProxyA) && (a.ProxyB == b.ProxyB);
		}

		public static bool operator !=(BroadphasePair a, BroadphasePair b)
		{
			if (object.Equals(a, null) && object.Equals(b, null))
				return true;
			if (object.Equals(a, null) || object.Equals(b, null))
				return false;

			return (a.ProxyA != b.ProxyA) || (a.ProxyB != b.ProxyB);
		}
	}
}
