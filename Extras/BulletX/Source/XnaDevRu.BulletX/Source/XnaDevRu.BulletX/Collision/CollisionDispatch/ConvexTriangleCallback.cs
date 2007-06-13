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
	public class ConvexTriangleCallback : ITriangleCallback, IDisposable
	{
		private CollisionObject _convexBody;
		private CollisionObject _triBody;

		private Vector3 _aabbMin;
		private Vector3 _aabbMax;

		private ManifoldResult _resultOut;

		private IDispatcher _dispatcher;
		private DispatcherInfo _dispatchInfo;
		private float _collisionMarginTriangle;

		private int _triangleCount;

		private PersistentManifold _manifold;

		public ConvexTriangleCallback(IDispatcher dispatcher, CollisionObject bodyA, CollisionObject bodyB, bool isSwapped)
		{
			_dispatcher = dispatcher;
			_dispatchInfo = null;
			_convexBody = isSwapped ? bodyB : bodyA;
			_triBody = isSwapped ? bodyA : bodyB;

			// create the manifold from the dispatcher 'manifold pool'
			_manifold = _dispatcher.GetNewManifold(_convexBody, _triBody);
			ClearCache();
		}

		public Vector3 AabbMin { get { return _aabbMin; } }
		public Vector3 AabbMax { get { return _aabbMax; } }
		public int TriangleCount { get { return _triangleCount; } set { _triangleCount = value; } }
		public PersistentManifold Manifold { get { return _manifold; } set { _manifold = value; } }

		public void SetTimeStepAndCounters(float collisionMarginTriangle, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			_dispatchInfo = dispatchInfo;
			_collisionMarginTriangle = collisionMarginTriangle;
			_resultOut = resultOut;

			//recalc aabbs
            Matrix convexInTriangleSpace = MathHelper.InvertMatrix(_triBody.WorldTransform) * _convexBody.WorldTransform;
			CollisionShape convexShape = _convexBody.CollisionShape;
			//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
			convexShape.GetAabb(convexInTriangleSpace, out _aabbMin, out _aabbMax);
			float extraMargin = collisionMarginTriangle;
			Vector3 extra = new Vector3(extraMargin, extraMargin, extraMargin);

			_aabbMax += extra;
			_aabbMin -= extra;
		}

		public void ClearCache()
		{
			_dispatcher.ClearManifold(_manifold);
		}

		#region ITriangleCallback Members
		public void ProcessTriangle(Vector3[] triangle, int partID, int triangleIndex)
		{
			//aabb filter is already applied!	
			CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo = new CollisionAlgorithmConstructionInfo();
			collisionAlgorithmConstructionInfo.Dispatcher = _dispatcher;

			CollisionObject collisionObject = _triBody;

			//debug drawing of the overlapping triangles
			/*if (m_dispatchInfoPtr && m_dispatchInfoPtr.m_debugDraw && m_dispatchInfoPtr->m_debugDraw->getDebugMode() > 0)
			{
				Vector3 color = new Vector3(255, 255, 0);
				btTransform & tr = ob->WorldTransform;
				m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]), tr(triangle[1]), color);
				m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]), tr(triangle[2]), color);
				m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]), tr(triangle[0]), color);
			}*/

			if (_convexBody.CollisionShape.IsConvex)
			{
				TriangleShape triangleShape = new TriangleShape(triangle[0], triangle[1], triangle[2]);
				triangleShape.Margin=_collisionMarginTriangle;

				CollisionShape tempShape = collisionObject.CollisionShape;
				collisionObject.CollisionShape = triangleShape;

				CollisionAlgorithm collisionAlgorithm = collisionAlgorithmConstructionInfo.Dispatcher.FindAlgorithm(_convexBody, _triBody, _manifold);

				_resultOut.SetShapeIdentifiers(-1, -1, partID, triangleIndex);
				collisionAlgorithm.ProcessCollision(_convexBody, _triBody, _dispatchInfo, _resultOut);
				collisionObject.CollisionShape = tempShape;
			}
		}
		#endregion
		#region IDisposable Members
		public void Dispose()
		{
			ClearCache();
			_dispatcher.ReleaseManifold(_manifold);
		}
		#endregion
	}
}
