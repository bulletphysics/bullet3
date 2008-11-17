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
	public class GjkPairDetector : DiscreteCollisionDetectorInterface
	{
		private Vector3 _cachedSeparatingAxis;
		private IConvexPenetrationDepthSolver _penetrationDepthSolver;
		private ISimplexSolver _simplexSolver;
		private ConvexShape _minkowskiA, _minkowskiB;
		private bool _ignoreMargin;

		private int _lastUsedMethod;
		private int _currentIteration;
		private int _degenerateSimplex;
		private int _catchDegeneracies;

		private static int _numDeepPenetrationChecks = 0;
		private static int _numGjkChecks = 0;

		private const float RelativeError2 = 1.0e-6f;

		#region Properties
		public int LastUsedMethod
		{
			get { return _lastUsedMethod; }
			set { _lastUsedMethod = value; }
		}

		public int CurrentIteration
		{
			get { return _currentIteration; }
			set { _currentIteration = value; }
		}

		public int DegenerateSimplex
		{
			get { return _degenerateSimplex; }
			set { _degenerateSimplex = value; }
		}

		public int CatchDegeneracies
		{
			get { return _catchDegeneracies; }
			set { _catchDegeneracies = value; }
		}

		public static int DeepPenetrationChecksCount { get { return _numDeepPenetrationChecks; } }
		public static int GjkChecksCount { get { return _numGjkChecks; } }
		#endregion

		public GjkPairDetector(ConvexShape objectA, ConvexShape objectB,
							   ISimplexSolver simplexSolver,
							   IConvexPenetrationDepthSolver penetrationDepthSolver)
		{
			_cachedSeparatingAxis = new Vector3(0, 0, 1);

			_penetrationDepthSolver = penetrationDepthSolver;
			_simplexSolver = simplexSolver;
			_minkowskiA = objectA;
			_minkowskiB = objectB;
			_ignoreMargin = false;
			_lastUsedMethod = -1;
			_catchDegeneracies = 1;
		}

		public void setMinkowskiA(ConvexShape minkA)
		{
			_minkowskiA = minkA;
		}

		public void setMinkowskiB(ConvexShape minkB)
		{
			_minkowskiB = minkB;
		}
		public void setCachedSeperatingAxis(Vector3 seperatingAxis)
		{
			_cachedSeparatingAxis = seperatingAxis;
		}

		public void setPenetrationDepthSolver(IConvexPenetrationDepthSolver penetrationDepthSolver)
		{
			this._penetrationDepthSolver = penetrationDepthSolver;
		}

		public void setIgnoreMargin(bool ignoreMargin)
		{
			this._ignoreMargin = ignoreMargin;
		}

		public override void GetClosestPoints(DiscreteCollisionDetectorInterface.ClosestPointInput input, DiscreteCollisionDetectorInterface.Result output, IDebugDraw debugDraw)
		{
			float distance = 0;

			Vector3 normalInB = new Vector3();
			Vector3 pointOnA = new Vector3(), pointOnB = new Vector3();

			Matrix localTransA = input.TransformA;
			Matrix localTransB = input.TransformB;

			Vector3 positionOffset = (localTransA.Translation + localTransB.Translation) * 0.5f;
			localTransA.Translation -= positionOffset;
			localTransB.Translation -= positionOffset;

			float marginA = _minkowskiA.Margin;
			float marginB = _minkowskiB.Margin;

			_numGjkChecks++;

			if (_ignoreMargin)
			{
				marginA = 0;
				marginB = 0;
			}

			_currentIteration = 0;

			int gjkMaxIter = 1000;
			_cachedSeparatingAxis = new Vector3(0, 1, 0);

			bool isValid = false;
			bool checkSimplex = false;
			bool checkPenetration = true;
			_degenerateSimplex = 0;

			_lastUsedMethod = -1;

			{
				float squaredDistance = MathHelper.Infinity;
				float delta = 0;

				float margin = marginA + marginB;

				_simplexSolver.Reset();

				while (true)
				{
					Matrix transABasis = input.TransformA;
					transABasis.Translation = Vector3.Zero;

					Matrix transBBasis = input.TransformB;
					transBBasis.Translation = Vector3.Zero;

					Vector3 seperatingAxisInA = Vector3.TransformNormal(-_cachedSeparatingAxis, transABasis);
					Vector3 seperatingAxisInB = Vector3.TransformNormal(_cachedSeparatingAxis, transBBasis);

					Vector3 pInA = _minkowskiA.LocalGetSupportingVertexWithoutMargin(seperatingAxisInA);
					Vector3 qInB = _minkowskiB.LocalGetSupportingVertexWithoutMargin(seperatingAxisInB);
					Vector3 pWorld = MathHelper.MatrixToVector(localTransA, pInA);
					Vector3 qWorld = MathHelper.MatrixToVector(localTransB, qInB);

					Vector3 w = pWorld - qWorld;
					delta = Vector3.Dot(_cachedSeparatingAxis, w);

					if ((delta > 0.0) && (delta * delta > squaredDistance * input.MaximumDistanceSquared))
					{
						checkPenetration = false;
						break;
					}

					if (_simplexSolver.InSimplex(w))
					{
						_degenerateSimplex = 1;
						checkSimplex = true;
						break;
					}

					float f0 = squaredDistance - delta;
					float f1 = squaredDistance * RelativeError2;

					if (f0 <= f1)
					{
						if (f0 <= 0.0f)
						{
							_degenerateSimplex = 2;
						}

						checkSimplex = true;
						break;
					}

					_simplexSolver.AddVertex(w, pWorld, qWorld);

					if (!_simplexSolver.Closest(out _cachedSeparatingAxis))
					{
						_degenerateSimplex = 3;
						checkSimplex = true;
						break;
					}

					float previouseSquaredDistance = squaredDistance;
					squaredDistance = _cachedSeparatingAxis.LengthSquared();

					if (previouseSquaredDistance - squaredDistance <= MathHelper.Epsilon * previouseSquaredDistance)
					{
						_simplexSolver.BackupClosest(out _cachedSeparatingAxis);
						checkSimplex = true;
						break;
					}

					if (_currentIteration++ > gjkMaxIter)
					{
#if DEBUG
						Console.WriteLine("GjkPairDetector maxIter exceeded: {0}", _currentIteration);
						Console.WriteLine("sepAxis=({0},{1},{2}), squaredDistance = {3}, shapeTypeA={4}, shapeTypeB={5}",
							_cachedSeparatingAxis.X,
							_cachedSeparatingAxis.Y,
							_cachedSeparatingAxis.Z,
							squaredDistance,
							_minkowskiA.ShapeType,
							_minkowskiB.ShapeType
						);
#endif
						break;
					}

					bool check = (!_simplexSolver.FullSimplex);

					if (!check)
					{
						_simplexSolver.BackupClosest(out _cachedSeparatingAxis);
						break;
					}
				}

				if (checkSimplex)
				{
					_simplexSolver.ComputePoints(out pointOnA, out pointOnB);
					normalInB = pointOnA - pointOnB;
					float lenSqr = _cachedSeparatingAxis.LengthSquared();

					if (lenSqr < 0.0001f)
					{
						_degenerateSimplex = 5;
					}

					if (lenSqr > MathHelper.Epsilon * MathHelper.Epsilon)
					{
						float rlen = 1.0f / (float)Math.Sqrt((float)lenSqr);
						normalInB *= rlen;
						float s = (float)Math.Sqrt((float)squaredDistance);

						BulletDebug.Assert(s > 0);
						pointOnA -= _cachedSeparatingAxis * (marginA / s);
						pointOnB += _cachedSeparatingAxis * (marginB / s);
						distance = ((1 / rlen) - margin);

						isValid = true;

						_lastUsedMethod = 1;
					}
					else
					{
						_lastUsedMethod = 2;
					}
				}

				bool catchDegeneratePenetrationCase =
					(_catchDegeneracies != 0 && _penetrationDepthSolver != null && _degenerateSimplex != 0 && ((distance + margin) < 0.01f));

				if (checkPenetration && (!isValid || catchDegeneratePenetrationCase))
				{
#warning Check this
					if (_penetrationDepthSolver != null)
					{
						Vector3 tmpPointOnA, tmpPointOnB;

						_numDeepPenetrationChecks++;

						bool isValid2 = _penetrationDepthSolver.CalculatePenetrationDepth(
							_simplexSolver, _minkowskiA, _minkowskiB, localTransA, localTransB,
							_cachedSeparatingAxis, out tmpPointOnA, out tmpPointOnB,
							debugDraw
						);

						if (isValid2)
						{
							Vector3 tmpNormalInB = tmpPointOnB - tmpPointOnA;
							float lengSqr = tmpNormalInB.LengthSquared();

							if (lengSqr > (MathHelper.Epsilon * MathHelper.Epsilon))
							{
								tmpNormalInB /= (float)Math.Sqrt((float)lengSqr);
								float distance2 = -(tmpPointOnA - tmpPointOnB).Length();

								if (!isValid || (distance2 < distance))
								{
									distance = distance2;
									pointOnA = tmpPointOnA;
									pointOnB = tmpPointOnB;
									normalInB = tmpNormalInB;
									isValid = true;
									_lastUsedMethod = 3;
								}
								else
								{

								}
							}
							else
							{
								_lastUsedMethod = 4;
							}
						}
						else
						{
							_lastUsedMethod = 5;
						}
					}
				}

				if (isValid)
				{
					output.AddContactPoint(normalInB, pointOnB + positionOffset, distance);
				}
			}
		}
	}
}