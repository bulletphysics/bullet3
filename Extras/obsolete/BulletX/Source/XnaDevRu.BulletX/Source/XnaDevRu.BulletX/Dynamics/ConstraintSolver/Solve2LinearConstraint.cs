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

namespace XnaDevRu.BulletX.Dynamics
{
	/// <summary>
	/// constraint class used for lateral tyre friction
	/// </summary>
	public class Solve2LinearConstraint
	{
		private float _tau;
		private float _damping;

		public Solve2LinearConstraint(float tau, float damping)
		{
			_tau = tau;
			_damping = damping;
		}

		// solve unilateral constraint (equality, direct method)
		public void ResolveUnilateralPairConstraint(
							RigidBody body1, RigidBody body2,
							Matrix world2A,
							Matrix world2B,
							Vector3 invInertiaADiag,
							float invMassA,
							Vector3 linvelA, Vector3 angvelA,
							Vector3 rel_posA1,
							Vector3 invInertiaBDiag,
							float invMassB,
							Vector3 linvelB, Vector3 angvelB,
							Vector3 rel_posA2,
							float depthA, Vector3 normalA,
							Vector3 rel_posB1, Vector3 rel_posB2,
							float depthB, Vector3 normalB,
							out float imp0, out float imp1)
		{
			imp0 = 0;
			imp1 = 0;

			float len = Math.Abs(normalA.Length()) - 1f;
			if (Math.Abs(len) >= float.Epsilon)
				return;

			BulletDebug.Assert(len < float.Epsilon);

			//this jacobian entry could be re-used for all iterations
			JacobianEntry jacA = new JacobianEntry(world2A, world2B, rel_posA1, rel_posA2, normalA, invInertiaADiag, invMassA,
				invInertiaBDiag, invMassB);
			JacobianEntry jacB = new JacobianEntry(world2A, world2B, rel_posB1, rel_posB2, normalB, invInertiaADiag, invMassA,
				invInertiaBDiag, invMassB);

			float vel0 = Vector3.Dot(normalA, body1.GetVelocityInLocalPoint(rel_posA1) - body2.GetVelocityInLocalPoint(rel_posA1));
			float vel1 = Vector3.Dot(normalB, body1.GetVelocityInLocalPoint(rel_posB1) - body2.GetVelocityInLocalPoint(rel_posB1));

			//	btScalar penetrationImpulse = (depth*contactTau*timeCorrection)  * massTerm;//jacDiagABInv
			float massTerm = 1f / (invMassA + invMassB);

			// calculate rhs (or error) terms
			float dv0 = depthA * _tau * massTerm - vel0 * _damping;
			float dv1 = depthB * _tau * massTerm - vel1 * _damping;

			float nonDiag = jacA.GetNonDiagonal(jacB, invMassA, invMassB);
			float invDet = 1.0f / (jacA.Diagonal * jacB.Diagonal - nonDiag * nonDiag);

			imp0 = dv0 * jacA.Diagonal * invDet + dv1 * -nonDiag * invDet;
			imp1 = dv1 * jacB.Diagonal * invDet + dv0 * -nonDiag * invDet;
		}

		// solving 2x2 lcp problem (inequality, direct solution )
		public void ResolveBilateralPairConstraint(
							RigidBody body1, RigidBody body2,
							Matrix world2A, Matrix world2B,
							Vector3 invInertiaADiag,
							float invMassA,
							Vector3 linvelA, Vector3 angvelA,
							Vector3 rel_posA1,
							Vector3 invInertiaBDiag,
							float invMassB,
							Vector3 linvelB, Vector3 angvelB,
							Vector3 rel_posA2,
							float depthA, Vector3 normalA,
							Vector3 rel_posB1, Vector3 rel_posB2,
							float depthB, Vector3 normalB,
							out float imp0, out float imp1)
		{
			imp0 = 0f;
			imp1 = 0f;

			float len = Math.Abs(normalA.Length()) - 1f;
			if (Math.Abs(len) >= float.Epsilon)
				return;

			BulletDebug.Assert(len < float.Epsilon);

			JacobianEntry jacA = new JacobianEntry(world2A, world2B, rel_posA1, rel_posA2, normalA, invInertiaADiag, invMassA,
				invInertiaBDiag, invMassB);
			JacobianEntry jacB = new JacobianEntry(world2A, world2B, rel_posB1, rel_posB2, normalB, invInertiaADiag, invMassA,
				invInertiaBDiag, invMassB);

			float vel0 = Vector3.Dot(normalA, body1.GetVelocityInLocalPoint(rel_posA1) - body2.GetVelocityInLocalPoint(rel_posA1));
			float vel1 = Vector3.Dot(normalB, body1.GetVelocityInLocalPoint(rel_posB1) - body2.GetVelocityInLocalPoint(rel_posB1));

			// calculate rhs (or error) terms
			float dv0 = depthA * _tau - vel0 * _damping;
			float dv1 = depthB * _tau - vel1 * _damping;

			float nonDiag = jacA.GetNonDiagonal(jacB, invMassA, invMassB);
			float invDet = 1.0f / (jacA.Diagonal * jacB.Diagonal - nonDiag * nonDiag);

			imp0 = dv0 * jacA.Diagonal * invDet + dv1 * -nonDiag * invDet;
			imp1 = dv1 * jacB.Diagonal * invDet + dv0 * -nonDiag * invDet;

			if (imp0 > 0.0f)
			{
				if (imp1 <= 0.0f)
				{
					imp1 = 0f;

					// now imp0>0 imp1<0
					imp0 = dv0 / jacA.Diagonal;
					if (imp0 < 0.0f)
						imp0 = 0f;
				}
			}
			else
			{
				imp0 = 0f;

				imp1 = dv1 / jacB.Diagonal;
				if (imp1 <= 0.0f)
				{
					imp1 = 0f;
					// now imp0>0 imp1<0
					imp0 = dv0 / jacA.Diagonal;
					if (imp0 > 0.0f)
					{
					}
					else
					{
						imp0 = 0f;
					}
				}
			}
		}

		//public void ResolveAngularConstraint(
		//                Matrix invInertiaAWS,
		//                float invMassA,
		//                Vector3 linvelA, Vector3 angvelA,
		//                Vector3 rel_posA1,
		//                Matrix invInertiaBWS,
		//                float invMassB,
		//                Vector3 linvelB, Vector3 angvelB,
		//                Vector3 rel_posA2,
		//                float depthA, Vector3 normalA,
		//                Vector3 rel_posB1, Vector3 rel_posB2,
		//                float depthB, Vector3 normalB,
		//                out float imp0, out float imp1)
		//{
		//    imp0 = 0;
		//    imp1 = 0;
		//}
	}
}
