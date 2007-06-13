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
	/// MinkowskiPenetrationDepthSolver implements bruteforce penetration depth estimation.
	/// Implementation is based on sampling the depth using support mapping, and using GJK step to get the witness points.
	/// </summary>
	public class MinkowskiPenetrationDepthSolver : IConvexPenetrationDepthSolver
	{
		private const int UnitSpherePointsCount = 42;

		private static Vector3[] penetrationDirections = 
														{
														new Vector3(0.000000f , -0.000000f,-1.000000f),
														new Vector3(0.723608f , -0.525725f,-0.447219f),
														new Vector3(-0.276388f , -0.850649f,-0.447219f),
														new Vector3(-0.894426f , -0.000000f,-0.447216f),
														new Vector3(-0.276388f , 0.850649f,-0.447220f),
														new Vector3(0.723608f , 0.525725f,-0.447219f),
														new Vector3(0.276388f , -0.850649f,0.447220f),
														new Vector3(-0.723608f , -0.525725f,0.447219f),
														new Vector3(-0.723608f , 0.525725f,0.447219f),
														new Vector3(0.276388f , 0.850649f,0.447219f),
														new Vector3(0.894426f , 0.000000f,0.447216f),
														new Vector3(-0.000000f , 0.000000f,1.000000f),
														new Vector3(0.425323f , -0.309011f,-0.850654f),
														new Vector3(-0.162456f , -0.499995f,-0.850654f),
														new Vector3(0.262869f , -0.809012f,-0.525738f),
														new Vector3(0.425323f , 0.309011f,-0.850654f),
														new Vector3(0.850648f , -0.000000f,-0.525736f),
														new Vector3(-0.525730f , -0.000000f,-0.850652f),
														new Vector3(-0.688190f , -0.499997f,-0.525736f),
														new Vector3(-0.162456f , 0.499995f,-0.850654f),
														new Vector3(-0.688190f , 0.499997f,-0.525736f),
														new Vector3(0.262869f , 0.809012f,-0.525738f),
														new Vector3(0.951058f , 0.309013f,0.000000f),
														new Vector3(0.951058f , -0.309013f,0.000000f),
														new Vector3(0.587786f , -0.809017f,0.000000f),
														new Vector3(0.000000f , -1.000000f,0.000000f),
														new Vector3(-0.587786f , -0.809017f,0.000000f),
														new Vector3(-0.951058f , -0.309013f,-0.000000f),
														new Vector3(-0.951058f , 0.309013f,-0.000000f),
														new Vector3(-0.587786f , 0.809017f,-0.000000f),
														new Vector3(-0.000000f , 1.000000f,-0.000000f),
														new Vector3(0.587786f , 0.809017f,-0.000000f),
														new Vector3(0.688190f , -0.499997f,0.525736f),
														new Vector3(-0.262869f , -0.809012f,0.525738f),
														new Vector3(-0.850648f , 0.000000f,0.525736f),
														new Vector3(-0.262869f , 0.809012f,0.525738f),
														new Vector3(0.688190f , 0.499997f,0.525736f),
														new Vector3(0.525730f , 0.000000f,0.850652f),
														new Vector3(0.162456f , -0.499995f,0.850654f),
														new Vector3(-0.425323f , -0.309011f,0.850654f),
														new Vector3(-0.425323f , 0.309011f,0.850654f),
														new Vector3(0.162456f , 0.499995f,0.850654f)
														};

		private class IntermediateResult : DiscreteCollisionDetectorInterface.Result
		{
			private Vector3 _normalOnBInWorld;
			private Vector3 _pointInWorld;
			private float _depth;
			private bool _hasResult;

			public IntermediateResult()
			{
				_hasResult = false;
			}

			public bool HasResult { get { return _hasResult; } }
			public float Depth { get { return _depth; } }
			public Vector3 PointInWorld { get { return _pointInWorld; } }

			public override void SetShapeIdentifiers(int partId0, int index0, int partId1, int index1)
			{
			}

			public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth)
			{
				_normalOnBInWorld = normalOnBInWorld;
				_pointInWorld = pointInWorld;
				_depth = depth;
				_hasResult = true;
			}
		}

		#region IConvexPenetrationDepthSolver Members
		public bool CalculatePenetrationDepth(ISimplexSolver simplexSolver,
			ConvexShape convexA, ConvexShape convexB,
			Matrix transformA, Matrix transformB,
			Vector3 v, out Vector3 pa, out Vector3 pb, IDebugDraw debugDraw)
		{
			pa = new Vector3();
			pb = new Vector3();
			//just take fixed number of orientation, and sample the penetration depth in that direction
			float minProj = 1e30f;
			Vector3 minNorm = new Vector3();
			Vector3 minA = new Vector3(), minB = new Vector3();
			Vector3 seperatingAxisInA, seperatingAxisInB;
			Vector3 pInA, qInB, pWorld, qWorld, w;

			Vector3[] supportVerticesABatch = new Vector3[UnitSpherePointsCount + ConvexShape.MaxPreferredPenetrationDirections * 2];
			Vector3[] supportVerticesBBatch = new Vector3[UnitSpherePointsCount + ConvexShape.MaxPreferredPenetrationDirections * 2];
			Vector3[] seperatingAxisInABatch = new Vector3[UnitSpherePointsCount + ConvexShape.MaxPreferredPenetrationDirections * 2];
			Vector3[] seperatingAxisInBBatch = new Vector3[UnitSpherePointsCount + ConvexShape.MaxPreferredPenetrationDirections * 2];

			int numSampleDirections = UnitSpherePointsCount;

			for (int i = 0; i < numSampleDirections; i++)
			{
				Vector3 norm = penetrationDirections[i];
				seperatingAxisInABatch[i] = Vector3.TransformNormal((-norm), transformA);
				seperatingAxisInBBatch[i] = Vector3.TransformNormal(norm, transformB);
			}

			{
				int numPDA = convexA.PreferredPenetrationDirectionsCount;
				if (numPDA != 0)
				{
					for (int i = 0; i < numPDA; i++)
					{
						Vector3 norm;
						convexA.GetPreferredPenetrationDirection(i, out norm);
						norm = Vector3.TransformNormal(norm, transformA);
						penetrationDirections[numSampleDirections] = norm;
						seperatingAxisInABatch[numSampleDirections] = Vector3.TransformNormal((-norm), transformA);
						seperatingAxisInBBatch[numSampleDirections] = Vector3.TransformNormal(norm, transformB);
						numSampleDirections++;
					}
				}
			}

			{
				int numPDB = convexB.PreferredPenetrationDirectionsCount;
				if (numPDB != 0)
				{
					for (int i = 0; i < numPDB; i++)
					{
						Vector3 norm;
						convexB.GetPreferredPenetrationDirection(i, out norm);
						norm = Vector3.TransformNormal(norm, transformB);
						penetrationDirections[numSampleDirections] = norm;
						seperatingAxisInABatch[numSampleDirections] = Vector3.TransformNormal((-norm), transformA);
						seperatingAxisInBBatch[numSampleDirections] = Vector3.TransformNormal(norm, transformB);
						numSampleDirections++;
					}
				}
			}

			convexA.BatchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch, supportVerticesABatch); //, numSampleDirections);
			convexB.BatchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch, supportVerticesBBatch); //, numSampleDirections);

			for (int i = 0; i < numSampleDirections; i++)
			{
				Vector3 norm = penetrationDirections[i];
				seperatingAxisInA = seperatingAxisInABatch[i];
				seperatingAxisInB = seperatingAxisInBBatch[i];

				pInA = supportVerticesABatch[i];
				qInB = supportVerticesBBatch[i];

				pWorld = MathHelper.MatrixToVector(transformA, pInA);
				qWorld = MathHelper.MatrixToVector(transformB, qInB);
				w = qWorld - pWorld;
				float delta = Vector3.Dot(norm, w);
				//find smallest delta
				if (delta < minProj)
				{
					minProj = delta;
					minNorm = norm;
					minA = pWorld;
					minB = qWorld;
				}
			}

			//add the margins
			minA += minNorm * convexA.Margin;
			minB -= minNorm * convexB.Margin;
			//no penetration
			if (minProj < 0)
				return false;

			minProj += (convexA.Margin + convexB.Margin);

			GjkPairDetector gjkdet = new GjkPairDetector(convexA, convexB, simplexSolver, null);

			float offsetDist = minProj;
			Vector3 offset = minNorm * offsetDist;

			GjkPairDetector.ClosestPointInput input = new DiscreteCollisionDetectorInterface.ClosestPointInput();

			Vector3 newOrg = transformA.Translation + offset;

			Matrix displacedTrans = transformA;
			displacedTrans.Translation = newOrg;

			input.TransformA = displacedTrans;
			input.TransformB = transformB;
			input.MaximumDistanceSquared = 1e30f;//minProj;

			IntermediateResult res = new IntermediateResult();
			gjkdet.GetClosestPoints(input, res, debugDraw);

			float correctedMinNorm = minProj - res.Depth;

			//the penetration depth is over-estimated, relax it
			float penetration_relaxation = 1;
			minNorm *= penetration_relaxation;

			if (res.HasResult)
			{

				pa = res.PointInWorld - minNorm * correctedMinNorm;
				pb = res.PointInWorld;
			}

			return res.HasResult;
		}
		#endregion
	}
}
