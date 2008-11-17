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
	public class ConvexConcaveCollisionAlgorithm : CollisionAlgorithm
	{
		private bool _isSwapped;
		private ConvexTriangleCallback _convexTriangleCallback;

		public ConvexConcaveCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB, bool isSwapped)
			: base(collisionAlgorithmConstructionInfo)
		{
			_isSwapped = isSwapped;
			_convexTriangleCallback = new ConvexTriangleCallback(collisionAlgorithmConstructionInfo.Dispatcher, bodyA, bodyB, isSwapped);
		}

		public void ClearCache()
		{
			_convexTriangleCallback.ClearCache();
		}

		public override void ProcessCollision(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			CollisionObject convexBody = _isSwapped ? bodyB : bodyA;
			CollisionObject triBody = _isSwapped ? bodyA : bodyB;

			if (triBody.CollisionShape.IsConcave)
			{
				CollisionObject triOb = triBody;
				ConcaveShape concaveShape = triOb.CollisionShape as ConcaveShape;

				if (convexBody.CollisionShape.IsConvex)
				{
					float collisionMarginTriangle = concaveShape.Margin;

					resultOut.SetPersistentManifold(_convexTriangleCallback.Manifold);
					_convexTriangleCallback.SetTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, resultOut);

					//Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
					//m_dispatcher->clearManifold(m_btConvexTriangleCallback.m_manifoldPtr);

					_convexTriangleCallback.Manifold.SetBodies(convexBody, triBody);
					concaveShape.ProcessAllTriangles(_convexTriangleCallback, _convexTriangleCallback.AabbMin, _convexTriangleCallback.AabbMax);
				}
			}
		}

		public override float CalculateTimeOfImpact(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			CollisionObject convexbody = _isSwapped ? bodyB : bodyA;
			CollisionObject triBody = _isSwapped ? bodyA : bodyB;


			//quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

			//only perform CCD above a certain threshold, this prevents blocking on the long run
			//because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
			float squareMot0 = (convexbody.InterpolationWorldTransform.Translation - convexbody.WorldTransform.Translation).LengthSquared();
			if (squareMot0 < convexbody.CcdSquareMotionThreshold)
			{
				return 1;
			}

            Matrix triInv = MathHelper.InvertMatrix(triBody.WorldTransform);
			Matrix convexFromLocal = triInv * convexbody.WorldTransform;
			Matrix convexToLocal = triInv * convexbody.InterpolationWorldTransform;

			if (triBody.CollisionShape.IsConcave)
			{
				Vector3 rayAabbMin = convexFromLocal.Translation;
				MathHelper.SetMin(ref rayAabbMin, convexToLocal.Translation);
				Vector3 rayAabbMax = convexFromLocal.Translation;
				MathHelper.SetMax(ref rayAabbMax, convexToLocal.Translation);
				float ccdRadius0 = convexbody.CcdSweptSphereRadius;
				rayAabbMin -= new Vector3(ccdRadius0, ccdRadius0, ccdRadius0);
				rayAabbMax += new Vector3(ccdRadius0, ccdRadius0, ccdRadius0);

				float curHitFraction = 1f; //is this available?
				LocalTriangleSphereCastCallback raycastCallback = new LocalTriangleSphereCastCallback(convexFromLocal, convexToLocal,
					convexbody.CcdSweptSphereRadius, curHitFraction);

				raycastCallback.HitFraction = convexbody.HitFraction;

				CollisionObject concavebody = triBody;

				ConcaveShape triangleMesh = concavebody.CollisionShape as ConcaveShape;

				if (triangleMesh != null)
				{
					triangleMesh.ProcessAllTriangles(raycastCallback, rayAabbMin, rayAabbMax);
				}

				if (raycastCallback.HitFraction < convexbody.HitFraction)
				{
					convexbody.HitFraction = raycastCallback.HitFraction;
					return raycastCallback.HitFraction;
				}
			}

			return 1;
		}

		public class CreateFunc : CollisionAlgorithmCreateFunction
		{
			public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			{
				return new ConvexConcaveCollisionAlgorithm(collisionAlgorithmConstructionInfo, bodyA, bodyB, false);
			}
		}

		public class SwappedCreateFunc : CollisionAlgorithmCreateFunction
		{
			public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			{
				return new ConvexConcaveCollisionAlgorithm(collisionAlgorithmConstructionInfo, bodyA, bodyB, true);
			}
		}

		private class LocalTriangleSphereCastCallback : ITriangleCallback
		{
			private Matrix _ccdSphereFromTrans;
			private Matrix _ccdSphereToTrans;
			private Matrix _meshTransform;

			private float _ccdSphereRadius;
			private float _hitFraction;

			public LocalTriangleSphereCastCallback(Matrix from, Matrix to, float ccdSphereRadius, float hitFraction)
			{
				_ccdSphereFromTrans = from;
				_ccdSphereToTrans = to;
				_ccdSphereRadius = ccdSphereRadius;
				_hitFraction = hitFraction;
			}

			public Matrix CcdSphereFromTrans { get { return _ccdSphereFromTrans; } set { _ccdSphereFromTrans = value; } }
			public Matrix CcdSphereToTrans { get { return _ccdSphereToTrans; } set { _ccdSphereToTrans = value; } }
			public Matrix MeshTransform { get { return _meshTransform; } set { _meshTransform = value; } }
			public float CcdSphereRadius { get { return _ccdSphereRadius; } set { _ccdSphereRadius = value; } }
			public float HitFraction { get { return _hitFraction; } set { _hitFraction = value; } }

			public void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
			{
				//do a swept sphere for now
				Matrix ident = Matrix.Identity;
				CastResult castResult = new CastResult();
				castResult.Fraction = _hitFraction;
				SphereShape pointShape = new SphereShape(_ccdSphereRadius);
				TriangleShape triShape = new TriangleShape(triangle[0], triangle[1], triangle[2]);
				VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
				SubsimplexConvexCast convexCaster = new SubsimplexConvexCast(pointShape, triShape, simplexSolver);
				//GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
				//ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
				//local space?

				if (convexCaster.CalcTimeOfImpact(_ccdSphereFromTrans, _ccdSphereToTrans,
					ident, ident, castResult))
				{
					if (_hitFraction > castResult.Fraction)
						_hitFraction = castResult.Fraction;
				}
			}
		}
	}
}
