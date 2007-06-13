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
	public class PointCollector : DiscreteCollisionDetectorInterface.Result
	{
		private Vector3 _normalOnBInWorld;
		private Vector3 _pointInWorld;
		private float _distance; //negative means penetration
		private bool _hasResult;

		public PointCollector()
		{
			_distance = 1e30f;
			_hasResult = false;
		}

		public Vector3 NormalOnBInWorld { get { return _normalOnBInWorld; } }
		public Vector3 PointInWorld { get { return _pointInWorld; } }
		public float Distance { get { return _distance; } }
		public bool HasResult { get { return _hasResult; } }

		public override void SetShapeIdentifiers(int partIdA, int indexA, int partIdB, int indexB)
		{
			//??
		}

		public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth)
		{
			if (depth < _distance)
			{
				_hasResult = true;
				_normalOnBInWorld = normalOnBInWorld;
				_pointInWorld = pointInWorld;
				//negative means penetration
				_distance = depth;
			}
		}
	}
}
