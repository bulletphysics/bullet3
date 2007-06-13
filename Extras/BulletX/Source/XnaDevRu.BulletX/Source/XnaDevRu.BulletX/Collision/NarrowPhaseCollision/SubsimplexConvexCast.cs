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
	/// SubsimplexConvexCast implements Gino van den Bergens' paper
	/// "Ray Casting against bteral Convex Objects with Application to Continuous Collision Detection"
	/// GJK based Ray Cast, optimized version
	/// Objects should not start in overlap, otherwise results are not defined.
	/// </summary>
	public class SubsimplexConvexCast : IConvexCast
	{
		private ISimplexSolver _simplexSolver;
		private ConvexShape _convexA;
		private ConvexShape _convexB;

		/// <summary>
		/// Typically the conservative advancement reaches solution in a few iterations, clip it to 32 for degenerate cases.
		/// See discussion about this here http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=565
		/// </summary>
		private const int MaxIterations = 32;

		public SubsimplexConvexCast(ConvexShape shapeA, ConvexShape shapeB, ISimplexSolver simplexSolver)
		{
			_simplexSolver = simplexSolver;
			_convexA = shapeA;
			_convexB = shapeB;
		}

		#region IConvexCast Members
		/// <summary>
		/// SimsimplexConvexCast calculateTimeOfImpact calculates the time of impact+normal for the linear cast (sweep) between two moving objects.
		/// Precondition is that objects should not penetration/overlap at the start from the interval. Overlap can be tested using GjkPairDetector.
		/// </summary>
		/// <param name="fromA"></param>
		/// <param name="toA"></param>
		/// <param name="fromB"></param>
		/// <param name="toB"></param>
		/// <param name="result"></param>
		/// <returns></returns>
		public bool CalcTimeOfImpact(Matrix fromA, Matrix toA, Matrix fromB, Matrix toB, CastResult result)
		{
			MinkowskiSumShape convex = new MinkowskiSumShape(_convexA, _convexB);

			Matrix rayFromLocalA;
			Matrix rayToLocalA;

            rayFromLocalA = MathHelper.InvertMatrix(fromA) * fromB;
            rayToLocalA = MathHelper.InvertMatrix(toA) * toB;

			_simplexSolver.Reset();

			convex.TransformB = rayFromLocalA;

			float lambda = 0;
			//todo: need to verify this:
			//because of minkowski difference, we need the inverse direction

			Vector3 s = -rayFromLocalA.Translation;
			Vector3 r = -(rayToLocalA.Translation - rayFromLocalA.Translation);
			Vector3 x = s;
			Vector3 v;
			Vector3 arbitraryPoint = convex.LocalGetSupportingVertex(r);

			v = x - arbitraryPoint;

			int maxIter = MaxIterations;

			Vector3 n = new Vector3();
			float lastLambda = lambda;

			float dist2 = v.LengthSquared();
			float epsilon = 0.0001f;

			Vector3 w, p;
			float VdotR;

			while ((dist2 > epsilon) && (maxIter-- != 0))
			{
				p = convex.LocalGetSupportingVertex(v);
				w = x - p;

				float VdotW = Vector3.Dot(v, w);

				if (VdotW > 0)
				{
					VdotR = Vector3.Dot(v, r);

					if (VdotR >= -(MathHelper.Epsilon * MathHelper.Epsilon))
						return false;
					else
					{
						lambda = lambda - VdotW / VdotR;
						x = s + lambda * r;
						_simplexSolver.Reset();
						//check next line
						w = x - p;
						lastLambda = lambda;
						n = v;
					}
				}
				_simplexSolver.AddVertex(w, x, p);
				if (_simplexSolver.Closest(out v))
				{
					dist2 = v.LengthSquared();
				}
				else
				{
					dist2 = 0f;
				}
			}
			result.Fraction = lambda;
			result.Normal = n;
			return true;
		}
		#endregion
	}
}
