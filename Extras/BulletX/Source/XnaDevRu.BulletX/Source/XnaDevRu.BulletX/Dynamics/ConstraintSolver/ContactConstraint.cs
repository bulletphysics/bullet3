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
	public delegate float ContactSolverFunc (RigidBody bodyA, RigidBody bodyB, ManifoldPoint contactPoint, ContactSolverInfo info);

	public enum ContactSolverType
	{
		Default = 0,
		TypeA,
		TypeB,
		User,
		MaxContactSolverType,
	}

	public class ConstraintPersistentData
	{
		// total applied impulse during most recent frame
		private float _appliedImpulse;
		private float _previousAppliedImpulse;
		private float _accumulatedTangentImpulse0;
		private float _accumulatedTangentImpulse1;

		private float _jacDiagABInv;
		private float _jacDiagABInvTangentA;
		private float _jacDiagABInvTangentB;
		private int _persistentLifeTime;
		private float _restitution;
		private float _friction;
		private float _penetration;
		private Vector3 _frictionWorldTangentialA;
		private Vector3 _frictionWorldTangentialB;

		private Vector3 _frictionAngularComponent0A;
		private Vector3 _frictionAngularComponent0B;
		private Vector3 _frictionAngularComponent1A;
		private Vector3 _frictionAngularComponent1B;

		//some data doesn't need to be persistent over frames: todo: clean/reuse this
		private Vector3 _angularComponentA;
		private Vector3 _angularComponentB;

		private ContactSolverFunc _contactSolverFunc;
		private ContactSolverFunc _frictionSolverFunc;

		public float AppliedImpulse { get { return _appliedImpulse; } set { _appliedImpulse = value; } }
		public float PreviousAppliedImpulse { get { return _previousAppliedImpulse; } set { _previousAppliedImpulse = value; } }
		public float AccumulatedTangentImpulseA { get { return _accumulatedTangentImpulse0; } set { _accumulatedTangentImpulse0 = value; } }
		public float AccumulatedTangentImpulseB { get { return _accumulatedTangentImpulse1; } set { _accumulatedTangentImpulse1 = value; } }
		
		public float JacDiagABInv { get { return _jacDiagABInv; } set { _jacDiagABInv = value; } }
		public float JacDiagABInvTangentA { get { return _jacDiagABInvTangentA; } set { _jacDiagABInvTangentA = value; } }
		public float JacDiagABInvTangentB { get { return _jacDiagABInvTangentB; } set { _jacDiagABInvTangentB = value; } }
		public int PersistentLifeTime { get { return _persistentLifeTime; } set { _persistentLifeTime = value; } }
		public float Restitution { get { return _restitution; } set { _restitution = value; } }
		public float Friction { get { return _friction; } set { _friction = value; } }
		public float Penetration { get { return _penetration; } set { _penetration = value; } }
		public Vector3 FrictionWorldTangentialA { get { return _frictionWorldTangentialA; } set { _frictionWorldTangentialA = value; } }
		public Vector3 FrictionWorldTangentialB { get { return _frictionWorldTangentialB; } set { _frictionWorldTangentialB = value; } }

		public Vector3 FrictionAngularComponent0A { get { return _frictionAngularComponent0A; } set { _frictionAngularComponent0A = value; } }
		public Vector3 FrictionAngularComponent0B { get { return _frictionAngularComponent0B; } set { _frictionAngularComponent0B = value; } }
		public Vector3 FrictionAngularComponent1A { get { return _frictionAngularComponent1A; } set { _frictionAngularComponent1A = value; } }
		public Vector3 FrictionAngularComponent1B { get { return _frictionAngularComponent1B; } set { _frictionAngularComponent1B = value; } }

		public Vector3 AngularComponentA { get { return _angularComponentA; } set { _angularComponentA = value; } }
		public Vector3 AngularComponentB { get { return _angularComponentB; } set { _angularComponentB = value; } }

		public ContactSolverFunc ContactSolverFunc { get { return _contactSolverFunc; } set { _contactSolverFunc = value; } }
		public ContactSolverFunc FrictionSolverFunc { get { return _frictionSolverFunc; } set { _frictionSolverFunc = value; } }
	}

	public static class ContactConstraint
	{
		private const int UseInternalApplyImpulse = 1;

		/// <summary>
		/// bilateral constraint between two dynamic objects
		/// positive distance = separation, negative distance = penetration
		/// </summary>
		/// <param name="body1"></param>
		/// <param name="pos1"></param>
		/// <param name="body2"></param>
		/// <param name="pos2"></param>
		/// <param name="distance"></param>
		/// <param name="normal"></param>
		/// <param name="impulse"></param>
		/// <param name="timeStep"></param>
		public static void ResolveSingleBilateral(RigidBody bodyA, Vector3 posA,
							  RigidBody bodyB, Vector3 posB,
							  float distance, Vector3 normal, out float impulse, float timeStep)
		{
			float normalLenSqr = normal.LengthSquared();

			if (Math.Abs(normalLenSqr) >= 1.1f)
				throw new BulletException();

			/*if (normalLenSqr > 1.1f)
			{
				impulse = 0f;
				return;
			}*/
			Vector3 rel_pos1 = posA - bodyA.CenterOfMassPosition;
			Vector3 rel_pos2 = posB - bodyB.CenterOfMassPosition;
			//this jacobian entry could be re-used for all iterations

			Vector3 vel1 = bodyA.GetVelocityInLocalPoint(rel_pos1);
			Vector3 vel2 = bodyB.GetVelocityInLocalPoint(rel_pos2);
			Vector3 vel = vel1 - vel2;


			JacobianEntry jac = new JacobianEntry(Matrix.Transpose(bodyA.CenterOfMassTransform),
				Matrix.Transpose(bodyB.CenterOfMassTransform),
				rel_pos1, rel_pos2, normal, bodyA.InvInertiaDiagLocal, bodyA.InverseMass,
				bodyB.InvInertiaDiagLocal, bodyB.InverseMass);

			float jacDiagAB = jac.Diagonal;
			float jacDiagABInv = 1f / jacDiagAB;

			float rel_vel = jac.GetRelativeVelocity(
			  bodyA.LinearVelocity,
			  Vector3.TransformNormal(bodyA.AngularVelocity, Matrix.Transpose(bodyA.CenterOfMassTransform)),
			  bodyB.LinearVelocity,
			  Vector3.TransformNormal(bodyB.AngularVelocity, Matrix.Transpose(bodyB.CenterOfMassTransform)));
			float a;
			a = jacDiagABInv;


			rel_vel = Vector3.Dot(normal, vel);

			float contactDamping = 0.2f;

			float velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
			impulse = velocityImpulse;
		}


		/// <summary>
		/// contact constraint resolution:
		/// calculate and apply impulse to satisfy non-penetration and non-negative relative velocity constraint
		/// positive distance = separation, negative distance = penetration
		/// </summary>
		/// <param name="body1"></param>
		/// <param name="body2"></param>
		/// <param name="contactPoint"></param>
		/// <param name="info"></param>
		/// <returns></returns>
		public static float ResolveSingleCollision(RigidBody bodyA, RigidBody bodyB,
				ManifoldPoint contactPoint, ContactSolverInfo solverInfo)
		{
			Vector3 pos1 = contactPoint.PositionWorldOnA;
			Vector3 pos2 = contactPoint.PositionWorldOnB;


			//	printf("distance=%f\n",distance);

			Vector3 normal = contactPoint.NormalWorldOnB;

			Vector3 rel_pos1 = pos1 - bodyA.CenterOfMassPosition;
			Vector3 rel_pos2 = pos2 - bodyB.CenterOfMassPosition;

			Vector3 vel1 = bodyA.GetVelocityInLocalPoint(rel_pos1);
			Vector3 vel2 = bodyB.GetVelocityInLocalPoint(rel_pos2);
			Vector3 vel = vel1 - vel2;
			float rel_vel;
			rel_vel = Vector3.Dot(normal, vel);


			float Kfps = 1f / solverInfo.TimeStep;

			//float damping = solverInfo.m_damping;
			float Kerp = solverInfo.Erp;

			float Kcor = Kerp * Kfps;

			//printf("dist=%f\n",distance);

			ConstraintPersistentData cpd = contactPoint.UserPersistentData as ConstraintPersistentData;
			if (cpd == null)
				throw new BulletException();

			float distance = cpd.Penetration;//contactPoint.getDistance();


			//distance = 0.f;
			float positionalError = Kcor * -distance;
			//jacDiagABInv;
			float velocityError = cpd.Restitution - rel_vel;// * damping;


			float penetrationImpulse = positionalError * cpd.JacDiagABInv;
			float velocityImpulse = velocityError * cpd.JacDiagABInv;
			float normalImpulse = penetrationImpulse + velocityImpulse;

			// See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
			float oldNormalImpulse = cpd.AppliedImpulse;
			float sum = oldNormalImpulse + normalImpulse;
			cpd.AppliedImpulse = 0f > sum ? 0f : sum;

			normalImpulse = cpd.AppliedImpulse - oldNormalImpulse;

			if (bodyA.InverseMass != 0)
			{
				bodyA.InternalApplyImpulse(contactPoint.NormalWorldOnB * bodyA.InverseMass, cpd.AngularComponentA, normalImpulse);
			}
			if (bodyB.InverseMass != 0)
			{
				bodyB.InternalApplyImpulse(contactPoint.NormalWorldOnB * bodyB.InverseMass, cpd.AngularComponentB, -normalImpulse);
			}

			/*body1.applyImpulse(normal * (normalImpulse), rel_pos1);
			body2.applyImpulse(-normal * (normalImpulse), rel_pos2);*/

			return normalImpulse;
		}

		public static float ResolveSingleFriction(RigidBody bodyA, RigidBody bodyB,
			ManifoldPoint contactPoint, ContactSolverInfo solverInfo)
		{

			Vector3 pos1 = contactPoint.PositionWorldOnA;
			Vector3 pos2 = contactPoint.PositionWorldOnB;

			Vector3 rel_pos1 = pos1 - bodyA.CenterOfMassPosition;
			Vector3 rel_pos2 = pos2 - bodyB.CenterOfMassPosition;

			ConstraintPersistentData cpd = contactPoint.UserPersistentData as ConstraintPersistentData;
			if (cpd == null)
				throw new BulletException();

			float combinedFriction = cpd.Friction;

			float limit = cpd.AppliedImpulse * combinedFriction;

			//friction
			if (cpd.AppliedImpulse > 0)
			{
				//apply friction in the 2 tangential directions

				// 1st tangent
				Vector3 vel1 = bodyA.GetVelocityInLocalPoint(rel_pos1);
				Vector3 vel2 = bodyB.GetVelocityInLocalPoint(rel_pos2);
				Vector3 vel = vel1 - vel2;

				float j1, j2;

				{

					float vrel = Vector3.Dot(cpd.FrictionWorldTangentialA, vel);

					// calculate j that moves us to zero relative velocity
					j1 = -vrel * cpd.JacDiagABInvTangentA;
					float oldTangentImpulse = cpd.AccumulatedTangentImpulseA;
					cpd.AccumulatedTangentImpulseA = oldTangentImpulse + j1;
					float atia = cpd.AccumulatedTangentImpulseA;
					MathHelper.SetMin(ref atia, limit);
					MathHelper.SetMax(ref atia, -limit);
					cpd.AccumulatedTangentImpulseA = atia;
					j1 = cpd.AccumulatedTangentImpulseA - oldTangentImpulse;

				}
				{
					// 2nd tangent

					float vrel = Vector3.Dot(cpd.FrictionWorldTangentialB, vel);

					// calculate j that moves us to zero relative velocity
					j2 = -vrel * cpd.JacDiagABInvTangentB;
					float oldTangentImpulse = cpd.AccumulatedTangentImpulseB;
					cpd.AccumulatedTangentImpulseB = oldTangentImpulse + j2;
					float atib = cpd.AccumulatedTangentImpulseB;
					MathHelper.SetMin(ref atib, limit);
					MathHelper.SetMax(ref atib, -limit);
					cpd.AccumulatedTangentImpulseB = atib;
					j2 = cpd.AccumulatedTangentImpulseB - oldTangentImpulse;
				}

				if (bodyA.InverseMass != 0)
				{
					bodyA.InternalApplyImpulse(cpd.FrictionWorldTangentialA * bodyA.InverseMass, cpd.FrictionAngularComponent0A, j1);
					bodyA.InternalApplyImpulse(cpd.FrictionWorldTangentialB * bodyA.InverseMass, cpd.FrictionAngularComponent1A, j2);
				}
				if (bodyB.InverseMass != 0)
				{
					bodyB.InternalApplyImpulse(cpd.FrictionWorldTangentialA * bodyB.InverseMass, cpd.FrictionAngularComponent0B, -j1);
					bodyB.InternalApplyImpulse(cpd.FrictionWorldTangentialB * bodyB.InverseMass, cpd.FrictionAngularComponent1B, -j2);
				}

			}
			return cpd.AppliedImpulse;
		}

		public static float ResolveSingleFrictionOriginal(
			RigidBody bodyA,
			RigidBody bodyB,
			ManifoldPoint contactPoint,
			ContactSolverInfo solverInfo)
		{
			Vector3 posA = contactPoint.PositionWorldOnA;
			Vector3 posB = contactPoint.PositionWorldOnB;

			Vector3 relPosA = posA - bodyA.CenterOfMassPosition;
			Vector3 relPosB = posB - bodyB.CenterOfMassPosition;

			ConstraintPersistentData cpd = contactPoint.UserPersistentData as ConstraintPersistentData;
			if (cpd == null)
				throw new BulletException();

			float combinedFriction = cpd.Friction;

			float limit = cpd.AppliedImpulse * combinedFriction;
			//if (contactPoint.m_appliedImpulse>0.f)
			//friction
			{
				//apply friction in the 2 tangential directions

				{
					// 1st tangent
					Vector3 velA = bodyA.GetVelocityInLocalPoint(relPosA);
					Vector3 velB = bodyB.GetVelocityInLocalPoint(relPosB);
					Vector3 vel = velA - velB;

					float vrel = Vector3.Dot(cpd.FrictionWorldTangentialA, vel);

					// calculate j that moves us to zero relative velocity
					float j = -vrel * cpd.JacDiagABInvTangentA;
					float total = cpd.AccumulatedTangentImpulseA + j;
					if (limit < total)
						total = limit;
					if (total < -limit)
						total = -limit;
					j = total - cpd.AccumulatedTangentImpulseA;
					cpd.AccumulatedTangentImpulseA = total;
					bodyA.ApplyImpulse(j * cpd.FrictionWorldTangentialA, relPosA);
					bodyB.ApplyImpulse(j * -cpd.FrictionWorldTangentialA, relPosB);
				}


				{
					// 2nd tangent
					Vector3 velA = bodyA.GetVelocityInLocalPoint(relPosA);
					Vector3 velB = bodyB.GetVelocityInLocalPoint(relPosB);
					Vector3 vel = velA - velB;

					float vrel = Vector3.Dot(cpd.FrictionWorldTangentialB, vel);

					// calculate j that moves us to zero relative velocity
					float j = -vrel * cpd.JacDiagABInvTangentB;
					float total = cpd.AccumulatedTangentImpulseB + j;
					if (limit < total)
						total = limit;
					if (total < -limit)
						total = -limit;
					j = total - cpd.AccumulatedTangentImpulseB;
					cpd.AccumulatedTangentImpulseB = total;
					bodyA.ApplyImpulse(j * cpd.FrictionWorldTangentialB, relPosA);
					bodyB.ApplyImpulse(j * -cpd.FrictionWorldTangentialB, relPosB);
				}
			}

			return cpd.AppliedImpulse;
		}

		//velocity + friction
		//response  between two dynamic objects with friction
		public static float ResolveSingleCollisionCombined(
			RigidBody bodyA,
			RigidBody bodyB,
			ManifoldPoint contactPoint,
			ContactSolverInfo solverInfo)
		{

			Vector3 posA = contactPoint.PositionWorldOnA;
			Vector3 posB = contactPoint.PositionWorldOnB;
			Vector3 normal = contactPoint.NormalWorldOnB;

			Vector3 relPosA = posA - bodyA.CenterOfMassPosition;
			Vector3 relPosB = posB - bodyB.CenterOfMassPosition;

			Vector3 velA = bodyA.GetVelocityInLocalPoint(relPosA);
			Vector3 velB = bodyB.GetVelocityInLocalPoint(relPosB);
			Vector3 vel = velA - velB;
			float relVel;
			relVel = Vector3.Dot(normal, vel);

			float Kfps = 1f / solverInfo.TimeStep;

			//float damping = solverInfo.m_damping;
			float Kerp = solverInfo.Erp;
			float Kcor = Kerp * Kfps;

			ConstraintPersistentData cpd = contactPoint.UserPersistentData as ConstraintPersistentData;
			if (cpd == null)
				throw new BulletException();

			float distance = cpd.Penetration;
			float positionalError = Kcor * -distance;
			float velocityError = cpd.Restitution - relVel;// * damping;

			float penetrationImpulse = positionalError * cpd.JacDiagABInv;

			float velocityImpulse = velocityError * cpd.JacDiagABInv;

			float normalImpulse = penetrationImpulse + velocityImpulse;

			// See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
			float oldNormalImpulse = cpd.AppliedImpulse;
			float sum = oldNormalImpulse + normalImpulse;
			cpd.AppliedImpulse = 0 > sum ? 0 : sum;

			normalImpulse = cpd.AppliedImpulse - oldNormalImpulse;

			if (bodyA.InverseMass != 0)
			{
				bodyA.InternalApplyImpulse(contactPoint.NormalWorldOnB * bodyA.InverseMass, cpd.AngularComponentA, normalImpulse);
			}
			if (bodyB.InverseMass != 0)
			{
				bodyB.InternalApplyImpulse(contactPoint.NormalWorldOnB * bodyB.InverseMass, cpd.AngularComponentB, -normalImpulse);
			}

			{
				//friction
				Vector3 vel12 = bodyA.GetVelocityInLocalPoint(relPosA);
				Vector3 vel22 = bodyB.GetVelocityInLocalPoint(relPosB);
				Vector3 vel3 = vel12 - vel22;

				relVel = Vector3.Dot(normal, vel3);


				Vector3 latVel = vel3 - normal * relVel;
				float lat_rel_vel = latVel.Length();

				float combinedFriction = cpd.Friction;

				if (cpd.AppliedImpulse > 0)
					if (lat_rel_vel > float.Epsilon)
					{
						latVel /= lat_rel_vel;
						Vector3 temp1 = Vector3.TransformNormal(Vector3.Cross(relPosA, latVel), bodyA.InvInertiaTensorWorld);
						Vector3 temp2 = Vector3.TransformNormal(Vector3.Cross(relPosB, latVel), bodyB.InvInertiaTensorWorld);
						float friction_impulse = lat_rel_vel /
							(bodyA.InverseMass + bodyB.InverseMass + Vector3.Dot(latVel, Vector3.Cross(temp1, relPosA) + Vector3.Cross(temp2, relPosB)));
						float normal_impulse = cpd.AppliedImpulse * combinedFriction;

						MathHelper.SetMin(ref friction_impulse, normal_impulse);
						MathHelper.SetMin(ref friction_impulse, -normal_impulse);
						bodyA.ApplyImpulse(latVel * -friction_impulse, relPosA);
						bodyB.ApplyImpulse(latVel * friction_impulse, relPosB);
					}
			}
			return normalImpulse;
		}

		public static float ResolveSingleFrictionEmpty(
			RigidBody bodyA,
			RigidBody bodyB,
			ManifoldPoint contactPoint,
			ContactSolverInfo solverInfo)
		{
			return 0;
		}
	}
}
