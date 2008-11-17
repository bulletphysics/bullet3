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
	/// <summary>
	/// CollisionAlgorithm is an collision interface that is compatible with the Broadphase and Dispatcher.
	/// It is persistent over frames
	/// </summary>
	public abstract class CollisionAlgorithm
	{
		private IDispatcher _dispatcher;
		private readonly int _comparisonID = 0;

		private static int _globalCount = 0;

		public CollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo)
		{
			_comparisonID = _globalCount++;
			_dispatcher = collisionAlgorithmConstructionInfo.Dispatcher;
		}

		protected IDispatcher Dispatcher { get { return _dispatcher; } set { _dispatcher = value; } }
		internal int ComparisonID { get { return _comparisonID; } }

		public abstract void ProcessCollision(CollisionObject colA, CollisionObject colB, DispatcherInfo dispatchInfo, ManifoldResult resultOut);
		public abstract float CalculateTimeOfImpact(CollisionObject colA, CollisionObject colB, DispatcherInfo dispatchInfo, ManifoldResult resultOut);
	}
}
