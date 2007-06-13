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
	public class ManifoldPoint
	{
		private Vector3 _localPointA;
		private Vector3 _localPointB;
		private Vector3 _positionWorldOnB;
		private Vector3 _positionWorldOnA;
		private Vector3 _normalWorldOnB;

		private float _distance;
		private float _combinedFriction;
		private float _combinedRestitution;

		private object _userPersistentData;

		private int _lifeTime;//lifetime of the contactpoint in frames

		public ManifoldPoint()
			: this(new Vector3(), new Vector3(), new Vector3(), 0f)
		{
		}

		public ManifoldPoint(Vector3 pointA, Vector3 pointB,
				Vector3 normal,
				float distance)
		{
			_localPointA = pointA;
			_localPointB = pointB;
			_normalWorldOnB = normal;
			_distance = distance;
			_positionWorldOnA = new Vector3();
			_positionWorldOnB = new Vector3();
		}

		public float Distance { get { return _distance; } set { _distance = value; } }
		public int LifeTime { get { return _lifeTime; } set { _lifeTime = value; } }

		public Vector3 PositionWorldOnA { get { return _positionWorldOnA; } set { _positionWorldOnA = value; } }
		public Vector3 PositionWorldOnB { get { return _positionWorldOnB; } set { _positionWorldOnB = value; } }

		public Vector3 LocalPointA { get { return _localPointA; } set { _localPointA = value; } }
		public Vector3 LocalPointB { get { return _localPointB; } set { _localPointB = value; } }

		public Vector3 NormalWorldOnB { get { return _normalWorldOnB; } }

		public float CombinedFriction { get { return _combinedFriction; } set { _combinedFriction = value; } }
		public float CombinedRestitution { get { return _combinedRestitution; } set { _combinedRestitution = value; } }

		public object UserPersistentData { get { return _userPersistentData; } set { _userPersistentData = value; } }
	}
}
