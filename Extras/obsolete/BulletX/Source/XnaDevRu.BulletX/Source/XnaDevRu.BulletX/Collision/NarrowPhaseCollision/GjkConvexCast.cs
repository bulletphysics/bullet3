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
    /// GjkConvexCast performs a raycast on a convex object using support mapping.
    /// </summary>
	public class GjkConvexCast : IConvexCast
	{
		private VoronoiSimplexSolver _simplexSolver;
		private ConvexShape _convexA, _convexB;

		public GjkConvexCast(ConvexShape convexShapeA, ConvexShape convexShapeB, VoronoiSimplexSolver solver)
		{
			_simplexSolver = solver;

			_convexA = convexShapeA;
			_convexB = convexShapeB;
		}

		#region IConvexCast Members

        /// <summary>
        /// cast a convex against another convex object
        /// </summary>
        /// <param name="fromA"></param>
        /// <param name="toA"></param>
        /// <param name="fromB"></param>
        /// <param name="toB"></param>
        /// <param name="result"></param>
        /// <returns></returns>
		public bool CalcTimeOfImpact(Matrix fromA, Matrix toA, Matrix fromB, Matrix toB, CastResult result)
		{
			MinkowskiSumShape combined = new MinkowskiSumShape(_convexA, _convexB);

            Matrix rayFromLocalA = MathHelper.InvertMatrix(fromA) * fromB;
            Matrix rayToLocalA = MathHelper.InvertMatrix(toA) * toB;

			Matrix transformA = fromA;
			Matrix transformB = fromB;

			transformA.Translation = new Vector3(0, 0, 0);
			transformB.Translation = new Vector3(0, 0, 0);

			combined.TransformA = transformA;
			combined.TransformB = transformB;

			float radius = 0.01f;
			float lambda = 0;

			Vector3 s = rayFromLocalA.Translation;
			Vector3 r = rayToLocalA.Translation - rayFromLocalA.Translation;
			Vector3 x = s;
			Vector3 n = new Vector3();
			Vector3 c = new Vector3();

			bool hasResult = false;
			float lastLambda = lambda;

			IConvexPenetrationDepthSolver penSolver = null;
			Matrix identityTransform = Matrix.Identity;

			SphereShape raySphere = new SphereShape(0.0f);
			raySphere.Margin=0.0f;

			Matrix sphereTransform = Matrix.Identity;
			sphereTransform.Translation = rayFromLocalA.Translation;

			result.DrawCoordSystem(sphereTransform);

			{
				PointCollector pointCollector = new PointCollector();
				GjkPairDetector gjk = new GjkPairDetector(raySphere, combined, _simplexSolver, penSolver);

				GjkPairDetector.ClosestPointInput input = new DiscreteCollisionDetectorInterface.ClosestPointInput();
				input.TransformA = sphereTransform;
				input.TransformB = identityTransform;

				gjk.GetClosestPoints(input, pointCollector, null);

				hasResult = pointCollector.HasResult;

				c = pointCollector.PointInWorld;
				n = pointCollector.NormalOnBInWorld;
			}

			if (hasResult)
			{
				float dist = (c - x).Length();

				if (dist < radius)
				{
					lastLambda = 1.0f;
				}

				while (dist > radius)
				{
					n = x - c;
					float dot = Vector3.Dot(n, r);

					if (dot >= -(MathHelper.Epsilon * MathHelper.Epsilon)) return false;

					lambda = lambda - Vector3.Distance(n, n) / dot;
					if (lambda <= lastLambda) break;

					lastLambda = lambda;

					x = s + lambda * r;

					sphereTransform.Translation = x;
					result.DrawCoordSystem(sphereTransform);
					PointCollector pointCollector = new PointCollector();

					GjkPairDetector gjk = new GjkPairDetector(raySphere, combined, _simplexSolver, penSolver);
					GjkPairDetector.ClosestPointInput input = new DiscreteCollisionDetectorInterface.ClosestPointInput();
					input.TransformA = sphereTransform;
					input.TransformB = identityTransform;

					gjk.GetClosestPoints(input, pointCollector, null);

					if (pointCollector.HasResult)
					{
						if (pointCollector.Distance < 0.0f)
						{
							result.Fraction = lastLambda;
							result.Normal = n;
							return true;
						}

						c = pointCollector.PointInWorld;
						dist = (c - x).Length();
					}
					else
					{
						return false;
					}
				}

				if (lastLambda < 1.0f)
				{
					result.Fraction = lastLambda;
					result.Normal = n;
					return true;
				}
			}

			return false;
		}

		#endregion
	}
}
