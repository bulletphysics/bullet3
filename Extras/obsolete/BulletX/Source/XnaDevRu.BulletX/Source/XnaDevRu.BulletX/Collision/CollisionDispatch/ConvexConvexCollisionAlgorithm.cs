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
	public class ConvexConvexAlgorithm : CollisionAlgorithm, IDisposable
	{
		private const bool DisableCcd = false;
		private GjkPairDetector _gjkPairDetector;
		private bool _ownManifold;
		private PersistentManifold _manifold;
		private bool _lowLevelOfDetail;

		public ConvexConvexAlgorithm(PersistentManifold manifold, CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB, ISimplexSolver simplexSolver, IConvexPenetrationDepthSolver penetrationDepthSolver)
			: base(collisionAlgorithmConstructionInfo)
		{
			_gjkPairDetector = new GjkPairDetector(null, null, simplexSolver, penetrationDepthSolver);
			_ownManifold = false;
			_manifold = manifold;
			_lowLevelOfDetail = false;
		}

		public bool LowLevelOfDetail { get { return _lowLevelOfDetail; } set { _lowLevelOfDetail = value; } }
		public bool OwnManifold { get { return _ownManifold; } set { _ownManifold = value; } }
		public PersistentManifold Manifold { get { return _manifold; } set { _manifold = value; } }

		public override void ProcessCollision(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			if (_manifold == null)
			{
				//swapped?
				_manifold = Dispatcher.GetNewManifold(bodyA, bodyB);
				_ownManifold = true;
			}
			resultOut.SetPersistentManifold(_manifold);

			ConvexShape min0 = bodyA.CollisionShape as ConvexShape;
			ConvexShape min1 = bodyB.CollisionShape as ConvexShape;

			GjkPairDetector.ClosestPointInput input = new DiscreteCollisionDetectorInterface.ClosestPointInput();

			//TODO: if (dispatchInfo.m_useContinuous)
			_gjkPairDetector.setMinkowskiA(min0);
			_gjkPairDetector.setMinkowskiB(min1);
			input.MaximumDistanceSquared = min0.Margin + min1.Margin + PersistentManifold.ContactBreakingThreshold;
			input.MaximumDistanceSquared *= input.MaximumDistanceSquared;

			//	input.m_maximumDistanceSquared = 1e30f;

			input.TransformA = bodyA.WorldTransform;
			input.TransformB = bodyB.WorldTransform;

			_gjkPairDetector.GetClosestPoints(input, resultOut, dispatchInfo.DebugDraw);
		}

		public override float CalculateTimeOfImpact(CollisionObject colA, CollisionObject colB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			//Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

			//Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
			//col0->m_worldTransform,
			float resultFraction = 1f;

			float squareMotA = (colA.InterpolationWorldTransform.Translation - colA.WorldTransform.Translation).LengthSquared();
			float squareMotB = (colB.InterpolationWorldTransform.Translation - colB.WorldTransform.Translation).LengthSquared();

			if (squareMotA < colA.CcdSquareMotionThreshold &&
				squareMotB < colB.CcdSquareMotionThreshold)
				return resultFraction;

			if (DisableCcd)
				return 1f;

			//An adhoc way of testing the Continuous Collision Detection algorithms
			//One object is approximated as a sphere, to simplify things
			//Starting in penetration should report no time of impact
			//For proper CCD, better accuracy and handling of 'allowed' penetration should be added
			//also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)

			// Convex0 against sphere for Convex1
			{
				ConvexShape convexA = colA.CollisionShape as ConvexShape;

				SphereShape sphereB = new SphereShape(colB.CcdSweptSphereRadius); //todo: allow non-zero sphere sizes, for better approximation
				CastResult result = new CastResult();
				VoronoiSimplexSolver voronoiSimplex = new VoronoiSimplexSolver();
				//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
				//Simplification, one object is simplified as a sphere
				GjkConvexCast ccdB = new GjkConvexCast(convexA, sphereB, voronoiSimplex);
				//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
				if (ccdB.CalcTimeOfImpact(colA.WorldTransform, colA.InterpolationWorldTransform,
					colB.WorldTransform, colB.InterpolationWorldTransform, result))
				{
					//store result.m_fraction in both bodies
					if (colA.HitFraction > result.Fraction)
						colA.HitFraction = result.Fraction;

					if (colB.HitFraction > result.Fraction)
						colB.HitFraction = result.Fraction;

					if (resultFraction > result.Fraction)
						resultFraction = result.Fraction;
				}
			}

			// Sphere (for convex0) against Convex1
			{
				ConvexShape convexB = colB.CollisionShape as ConvexShape;

				SphereShape sphereA = new SphereShape(colA.CcdSweptSphereRadius); //todo: allow non-zero sphere sizes, for better approximation
				CastResult result = new CastResult();
				VoronoiSimplexSolver voronoiSimplex = new VoronoiSimplexSolver();
				//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
				///Simplification, one object is simplified as a sphere
				GjkConvexCast ccdB = new GjkConvexCast(sphereA, convexB, voronoiSimplex);
				//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
				if (ccdB.CalcTimeOfImpact(colA.WorldTransform, colA.InterpolationWorldTransform,
					colB.WorldTransform, colB.InterpolationWorldTransform, result))
				{
					//store result.m_fraction in both bodies
					if (colA.HitFraction > result.Fraction)
						colA.HitFraction = result.Fraction;

					if (colB.HitFraction > result.Fraction)
						colB.HitFraction = result.Fraction;

					if (resultFraction > result.Fraction)
						resultFraction = result.Fraction;
				}
			}
			return resultFraction;
		}

		public class CreateFunc : CollisionAlgorithmCreateFunction
		{
			private IConvexPenetrationDepthSolver _penetrationDepthSolver;
			private ISimplexSolver _simplexSolver;
			//private bool _ownsSolvers;

			public CreateFunc()
			{
				//_ownsSolvers = true;
				_simplexSolver = new VoronoiSimplexSolver();
				_penetrationDepthSolver = new GjkEpaPenetrationDepthSolver();
			}

			public CreateFunc(ISimplexSolver simplexSolver, IConvexPenetrationDepthSolver penetrationDepthSolver)
			{
				//_ownsSolvers = false;
				_simplexSolver = simplexSolver;
				_penetrationDepthSolver = penetrationDepthSolver;
			}

			public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			{
				return new ConvexConvexAlgorithm(collisionAlgorithmConstructionInfo.Manifold, collisionAlgorithmConstructionInfo, bodyA, bodyB, _simplexSolver, _penetrationDepthSolver);
			}
		}

		#region IDisposable Members
		public void Dispose()
		{
			if (_ownManifold)
			{
				if (_manifold != null)
					Dispatcher.ReleaseManifold(_manifold);
			}
		}
		#endregion
	}
}
