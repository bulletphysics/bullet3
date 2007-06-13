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
	/// <summary>
	/// ContinuousConvexCollision implements angular and linear time of impact for convex objects.
	/// Based on Brian Mirtich's Conservative Advancement idea (PhD thesis).
	/// Algorithm operates in worldspace, in order to keep inbetween motion globally consistent.
	/// It uses GJK at the moment. Future improvement would use minkowski sum / supporting vertex, merging innerloops
	/// </summary>
	public class ContinuousConvexCollision : IConvexCast
	{
		/// <summary>
		/// This maximum should not be necessary. It allows for untested/degenerate cases in production code.
		/// You don't want your game ever to lock-up.
		/// </summary>
		private const int MaxIterations = 1000;

		private ISimplexSolver _simplexSolver;
		private IConvexPenetrationDepthSolver _penetrationDepthSolver;
		private ConvexShape _convexA;
		private ConvexShape _convexB;

		public ContinuousConvexCollision(ConvexShape convexA, ConvexShape convexB,
			ISimplexSolver simplexSolver, IConvexPenetrationDepthSolver penetrationDepthSolver)
		{
			_simplexSolver = simplexSolver;
			_penetrationDepthSolver = penetrationDepthSolver;
			_convexA = convexA;
			_convexB = convexB;
		}

		public bool CalcTimeOfImpact(Matrix fromA, Matrix toA, Matrix fromB, Matrix toB, CastResult result)
		{
			_simplexSolver.Reset();

			// compute linear and angular velocity for this interval, to interpolate
			Vector3 linVelA = new Vector3(), angVelA = new Vector3(), linVelB = new Vector3(), angVelB = new Vector3();
			TransformUtil.CalculateVelocity(fromA, toA, 1f, ref linVelA, ref angVelA);
			TransformUtil.CalculateVelocity(fromB, toB, 1f, ref linVelB, ref angVelB);

			float boundingRadiusA = _convexA.GetAngularMotionDisc();
			float boundingRadiusB = _convexB.GetAngularMotionDisc();

			float maxAngularProjectedVelocity = angVelA.Length() * boundingRadiusA +
				angVelB.Length() * boundingRadiusB;

			float radius = 0.001f;

			float lambda = 0f;
			Vector3 v = new Vector3(1f, 0f, 0f);

			int maxIter = MaxIterations;

			Vector3 n = new Vector3();
			bool hasResult = false;
			Vector3 c;

			float lastLambda = lambda;
			//float epsilon = 0.001f;

			int numIter = 0;
			//first solution, using GJK


			Matrix identityTrans = Matrix.Identity;

			SphereShape raySphere = new SphereShape(0f);
			raySphere.Margin=0f;


			//result.drawCoordSystem(sphereTr);

			PointCollector pointCollector1 = new PointCollector();

			GjkPairDetector gjk = new GjkPairDetector(_convexA, _convexB, (VoronoiSimplexSolver)_simplexSolver, _penetrationDepthSolver);
			GjkPairDetector.ClosestPointInput input = new DiscreteCollisionDetectorInterface.ClosestPointInput();

			//we don't use margins during CCD
			gjk.setIgnoreMargin(true);

			input.TransformA = fromA;
			input.TransformB = fromB;

			DiscreteCollisionDetectorInterface.Result r = (DiscreteCollisionDetectorInterface.Result)pointCollector1;
			gjk.GetClosestPoints(input, r, null);

			hasResult = pointCollector1.HasResult;
			c = pointCollector1.PointInWorld;

			if (hasResult)
			{
				float dist;
				dist = pointCollector1.Distance;
				n = pointCollector1.NormalOnBInWorld;

				//not close enough
				while (dist > radius)
				{
					numIter++;
					if (numIter > maxIter)
						return false; //todo: report a failure

					float dLambda = 0f;

					//calculate safe moving fraction from distance / (linear+rotational velocity)

					//float clippedDist  = GEN_min(angularConservativeRadius,dist);
					//float clippedDist  = dist;

					float projectedLinearVelocity = Vector3.Dot(linVelB - linVelA, n);

					dLambda = dist / (projectedLinearVelocity + maxAngularProjectedVelocity);

					lambda = lambda + dLambda;

					if (lambda > 1f) return false;
					if (lambda < 0f) return false;

					//todo: next check with relative epsilon
					if (lambda <= lastLambda)
						break;
					lastLambda = lambda;


					//interpolate to next lambda
					Matrix interpolatedTransA = new Matrix(), interpolatedTransB = new Matrix(), relativeTrans;

					TransformUtil.IntegrateTransform(fromA, linVelA, angVelA, lambda, ref interpolatedTransA);
					TransformUtil.IntegrateTransform(fromB, linVelB, angVelB, lambda, ref interpolatedTransB);

					relativeTrans = MathHelper.InverseTimes(interpolatedTransB, interpolatedTransA);

					result.DebugDraw(lambda);

					PointCollector pointCollector = new PointCollector();
					gjk = new GjkPairDetector(_convexA, _convexB, (VoronoiSimplexSolver)_simplexSolver, _penetrationDepthSolver);
					input = new DiscreteCollisionDetectorInterface.ClosestPointInput();
					input.TransformA = interpolatedTransA;
					input.TransformB = interpolatedTransB;

					// !!!!!!!!!!
					r = (DiscreteCollisionDetectorInterface.Result)pointCollector1;
					gjk.GetClosestPoints(input, r, null);

					if (pointCollector.HasResult)
					{
						if (pointCollector.Distance < 0f)
						{
							//degenerate ?!
							result.Fraction = lastLambda;
							result.Normal = n;
							return true;
						}
						c = pointCollector.PointInWorld;

						dist = pointCollector.Distance;
					}
					else
					{
						//??
						return false;
					}

				}

				result.Fraction = lambda;
				result.Normal = n;
				return true;
			}

			return false;
		}
	}
}
