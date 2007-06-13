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
	public class SimpleBroadphaseProxy : BroadphaseProxy
	{
		private Vector3 _min;
		private Vector3 _max;

		public SimpleBroadphaseProxy() { }

		public SimpleBroadphaseProxy(Vector3 minPoint, Vector3 maxPoint, BroadphaseNativeTypes shapeType, object userData, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask)
			: base(userData, collisionFilterGroup, collisionFilterMask)
		{
			_min = minPoint;
			_max = maxPoint;
		}

		public Vector3 Minimum { get { return _min; } set { _min = value; } }
		public Vector3 Maximum { get { return _max; } set { _max = value; } }
	}
}
