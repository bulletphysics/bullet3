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
	[Flags]
	public enum SolverMode
	{
		None = 0,
		RandomizeOrder = 1,
		FrictionSeperate = 2,
		UseWarmstarting = 4,
		CacheFriendly = 8,
	}

	public class SequentialImpulseConstraintSolver : IConstraintSolver
	{
		private static int _totalContactPoints = 0;

		private SolverMode _solverMode;
		private int _totalCpd = 0;
		private ContactSolverFunc[,] _contactDispatch = new ContactSolverFunc[(int)ContactSolverType.MaxContactSolverType, (int)ContactSolverType.MaxContactSolverType];
		private ContactSolverFunc[,] _frictionDispatch = new ContactSolverFunc[(int)ContactSolverType.MaxContactSolverType, (int)ContactSolverType.MaxContactSolverType];

		private float _penetrationResolveFactor = 0.9f;
		private List<SolverBody> _tmpSolverBodyPool = new List<SolverBody>();
		private List<SolverConstraint> _tmpSolverConstraintPool = new List<SolverConstraint>();
		private List<SolverConstraint> _tmpSolverFrictionConstraintPool = new List<SolverConstraint>();

		private const int _sequentialImpulseMaxSolverPoints = 16384;
		private static OrderIndex[] _order = new OrderIndex[SequentialImpulseMaxSolverPoints];
		private static long _seed2 = 0;

		public SequentialImpulseConstraintSolver()
		{
			_solverMode = SolverMode.RandomizeOrder | SolverMode.CacheFriendly;
			PersistentManifold.ContactDestroyedCallback = MyContactDestroyedCallback;

			//initialize default friction/contact funcs
			int i, j;
			for (i = 0; i < (int)ContactSolverType.MaxContactSolverType; i++)
				for (j = 0; j < (int)ContactSolverType.MaxContactSolverType; j++)
				{

					_contactDispatch[i, j] = ContactConstraint.ResolveSingleCollision;
					_frictionDispatch[i, j] = ContactConstraint.ResolveSingleFriction;
				}
		}

		public SolverMode SolverMode { get { return _solverMode; } set { _solverMode = value; } }
		public static int SequentialImpulseMaxSolverPoints { get { return _sequentialImpulseMaxSolverPoints; } }
		protected static OrderIndex[] Order { get { return _order; } set { _order = value; } }
		public static long RandSeed { get { return _seed2; } set { _seed2 = value; } }

		///<summary>
		/// Advanced: Override the default contact solving function for contacts, for certain types of rigidbody
		/// See btRigidBody::m_contactSolverType and btRigidBody::m_frictionSolverType
		///</summary>
		public void SetContactSolverFunc(ContactSolverFunc func, int typeA, int typeB)
		{
			_contactDispatch[typeA, typeB] = func;
		}

		/// <summary>
		/// Advanced: Override the default friction solving function for contacts, for certain types of rigidbody
		/// See btRigidBody::m_contactSolverType and btRigidBody::m_frictionSolverType
		///</summary>
		public void SetFrictionSolverFunc(ContactSolverFunc func, int typeA, int typeB)
		{
			_frictionDispatch[typeA, typeB] = func;
		}

		protected float Solve(RigidBody bodyA, RigidBody bodyB, ManifoldPoint cp, ContactSolverInfo info, int iter, IDebugDraw debugDraw)
		{
			float maxImpulse = 0;

			Vector3 color = new Vector3(0, 1, 0);
			if (cp.Distance <= 0)
			{
				if (iter == 0)
					if(debugDraw != null)
						debugDraw.DrawContactPoint(cp.PositionWorldOnB, cp.NormalWorldOnB, cp.Distance, cp.LifeTime, color);

				ConstraintPersistentData cpd = cp.UserPersistentData as ConstraintPersistentData;
				float impulse = cpd.ContactSolverFunc(
					bodyA, bodyB,
					cp,
					info);

				if (maxImpulse < impulse)
					maxImpulse = impulse;
			}
			return maxImpulse;
		}

		protected float Solve(RigidBody bodyA, RigidBody bodyB, ManifoldPoint cp, ContactSolverInfo info, int iter)
		{
			return Solve(bodyA, bodyB, cp, info, iter, null);
		}

		protected float SolveCombinedContactFriction(RigidBody bodyA, RigidBody bodyB, ManifoldPoint cp, ContactSolverInfo info, int iter, IDebugDraw debugDraw)
		{
			float maxImpulse = 0;

			Vector3 color = new Vector3(0, 1, 0);
			if (cp.Distance <= 0)
			{
				if (iter == 0)
					if (debugDraw != null)
						debugDraw.DrawContactPoint(cp.PositionWorldOnB, cp.NormalWorldOnB, cp.Distance, cp.LifeTime, color);

				float impulse = ContactConstraint.ResolveSingleCollisionCombined(
					bodyA, bodyB,
					cp,
					info);

				if (maxImpulse < impulse)
					maxImpulse = impulse;
			}
			return maxImpulse;
		}

		protected float SolveFriction(RigidBody bodyA, RigidBody bodyB, ManifoldPoint cp, ContactSolverInfo info, int iter, IDebugDraw debugDraw)
		{
			Vector3 color = new Vector3(0, 1, 0);

			if (cp.Distance <= 0)
			{

				ConstraintPersistentData cpd = cp.UserPersistentData as ConstraintPersistentData;
				cpd.FrictionSolverFunc(
					bodyA, bodyB,
					cp,
					info);
			}
			return 0;
		}

		protected void PrepareConstraints(PersistentManifold manifold, ContactSolverInfo info)
		{
			RigidBody body0 = manifold.BodyA as RigidBody;
			RigidBody body1 = manifold.BodyB as RigidBody;


			//only necessary to refresh the manifold once (first iteration). The integration is done outside the loop
			{
				manifold.RefreshContactPoints(body0.CenterOfMassTransform, body1.CenterOfMassTransform);

				int numpoints = manifold.ContactsCount;

				_totalContactPoints += numpoints;

				Vector3 color = new Vector3(0, 1, 0);
				for (int i = 0; i < numpoints; i++)
				{
					ManifoldPoint cp = manifold.GetContactPoint(i);
					if (cp.Distance <= 0)
					{
						Vector3 pos1 = cp.PositionWorldOnA;
						Vector3 pos2 = cp.PositionWorldOnB;

						Vector3 rel_pos1 = pos1 - body0.CenterOfMassPosition;
						Vector3 rel_pos2 = pos2 - body1.CenterOfMassPosition;


						//this jacobian entry is re-used for all iterations
						JacobianEntry jac = new JacobianEntry(MatrixOperations.Transpose(body0.CenterOfMassTransform),
							MatrixOperations.Transpose(body1.CenterOfMassTransform),
							rel_pos1, rel_pos2, cp.NormalWorldOnB, body0.InvInertiaDiagLocal, body0.InverseMass,
							body1.InvInertiaDiagLocal, body1.InverseMass);

						float jacDiagAB = jac.Diagonal;

						ConstraintPersistentData cpd = cp.UserPersistentData as ConstraintPersistentData;
						if (cpd != null)
						{
							//might be invalid
							cpd.PersistentLifeTime++;
							if (cpd.PersistentLifeTime != cp.LifeTime)
							{
								//printf("Invalid: cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd->m_persistentLifeTime,cp.getLifeTime());
								cpd = new ConstraintPersistentData();
								cpd.PersistentLifeTime = cp.LifeTime;

							}
						}
						else
						{

							cpd = new ConstraintPersistentData();
							_totalCpd++;
							//printf("totalCpd = %i Created Ptr %x\n",totalCpd,cpd);
							cp.UserPersistentData = cpd;
							cpd.PersistentLifeTime = cp.LifeTime;
							//printf("CREATED: %x . cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd,cpd->m_persistentLifeTime,cp.getLifeTime());

						}
						if (cpd == null)
							throw new BulletException();

						cpd.JacDiagABInv = 1f / jacDiagAB;

						//Dependent on Rigidbody A and B types, fetch the contact/friction response func
						//perhaps do a similar thing for friction/restutution combiner funcs...

						cpd.FrictionSolverFunc = _frictionDispatch[(int)body0.FrictionSolverType, (int)body1.FrictionSolverType];
						cpd.ContactSolverFunc = _contactDispatch[(int)body0.ContactSolverType, (int)body1.ContactSolverType];

						Vector3 vel1 = body0.GetVelocityInLocalPoint(rel_pos1);
						Vector3 vel2 = body1.GetVelocityInLocalPoint(rel_pos2);
						Vector3 vel = vel1 - vel2;
						float rel_vel;
						rel_vel = Vector3.Dot(cp.NormalWorldOnB, vel);

						float combinedRestitution = cp.CombinedRestitution;

						cpd.Penetration = cp.Distance;
						cpd.Friction = cp.CombinedFriction;
						cpd.Restitution = RestitutionCurve(rel_vel, combinedRestitution);
						if (cpd.Restitution < 0f)
						{
							cpd.Restitution = 0.0f;

						};

						//restitution and penetration work in same direction so
						//rel_vel 

						float penVel = -cpd.Penetration / info.TimeStep;

						if (cpd.Restitution > penVel)
						{
							cpd.Penetration = 0;
						}


						float relaxation = info.Damping;
						if ((_solverMode & SolverMode.UseWarmstarting) != 0)
						{
							cpd.AppliedImpulse *= relaxation;
						}
						else
						{
							cpd.AppliedImpulse = 0f;
						}

						//for friction
						cpd.PreviousAppliedImpulse = cpd.AppliedImpulse;

						//re-calculate friction direction every frame, todo: check if this is really needed
						Vector3 fwta = cpd.FrictionWorldTangentialA;
						Vector3 fwtb = cpd.FrictionWorldTangentialB;
						MathHelper.PlaneSpace1(cp.NormalWorldOnB, ref fwta, ref fwtb);
						cpd.FrictionWorldTangentialA = fwta;
						cpd.FrictionWorldTangentialB = fwtb;

						cpd.AccumulatedTangentImpulseA = 0;
						cpd.AccumulatedTangentImpulseB = 0;
						float denom0 = body0.ComputeImpulseDenominator(pos1, cpd.FrictionWorldTangentialA);
						float denom1 = body1.ComputeImpulseDenominator(pos2, cpd.FrictionWorldTangentialA);
						float denom = relaxation / (denom0 + denom1);
						cpd.JacDiagABInvTangentA = denom;


						denom0 = body0.ComputeImpulseDenominator(pos1, cpd.FrictionWorldTangentialB);
						denom1 = body1.ComputeImpulseDenominator(pos2, cpd.FrictionWorldTangentialB);
						denom = relaxation / (denom0 + denom1);
						cpd.JacDiagABInvTangentB = denom;

						Vector3 totalImpulse = cp.NormalWorldOnB * cpd.AppliedImpulse;

						{
							Vector3 torqueAxis0 = Vector3.Cross(rel_pos1, cp.NormalWorldOnB);
							cpd.AngularComponentA = Vector3.TransformNormal(torqueAxis0, body0.InvInertiaTensorWorld);
							Vector3 torqueAxis1 = Vector3.Cross(rel_pos2, cp.NormalWorldOnB);
							cpd.AngularComponentB = Vector3.TransformNormal(torqueAxis1, body1.InvInertiaTensorWorld);
						}
						{
							Vector3 ftorqueAxis0 = Vector3.Cross(rel_pos1, cpd.FrictionWorldTangentialA);
							cpd.FrictionAngularComponent0A = Vector3.TransformNormal(ftorqueAxis0, body0.InvInertiaTensorWorld);
						}
						{
							Vector3 ftorqueAxis1 = Vector3.Cross(rel_pos1, cpd.FrictionWorldTangentialB);
							cpd.FrictionAngularComponent1A = Vector3.TransformNormal(ftorqueAxis1, body0.InvInertiaTensorWorld);
						}
						{
							Vector3 ftorqueAxis0 = Vector3.Cross(rel_pos2, cpd.FrictionWorldTangentialA);
							cpd.FrictionAngularComponent0B = Vector3.TransformNormal(ftorqueAxis0, body1.InvInertiaTensorWorld);
						}
						{
							Vector3 ftorqueAxis1 = Vector3.Cross(rel_pos2, cpd.FrictionWorldTangentialB);
							cpd.FrictionAngularComponent1B = Vector3.TransformNormal(ftorqueAxis1, body1.InvInertiaTensorWorld);
						}


						//apply previous frames impulse on both bodies
						body0.ApplyImpulse(totalImpulse, rel_pos1);
						body1.ApplyImpulse(-totalImpulse, rel_pos2);
					}
				}
			}
		}

		private bool MyContactDestroyedCallback(object userPersistentData)
		{
			if (userPersistentData == null)
				throw new BulletException();
			ConstraintPersistentData cpd = userPersistentData as ConstraintPersistentData;
			_totalCpd--;
			return true;
		}

		private float RestitutionCurve(float relVel, float restitution)
		{
			float rest = restitution * -relVel;
			return rest;
		}

		//velocity + friction
		//response  between two dynamic objects with friction
		public virtual float ResolveSingleCollisionCombinedCacheFriendly(
			SolverBody bodyA,
			SolverBody bodyB,
			SolverConstraint contactConstraint,
			ContactSolverInfo solverInfo)
		{
			float normalImpulse = 0;

			if (contactConstraint.Penetration < 0)
				return 0;

			float relVel;
			float velADotn = Vector3.Dot(contactConstraint.ContactNormal,bodyA.LinearVelocity)
						+ Vector3.Dot(contactConstraint.RelPosACrossNormal,bodyA.AngularVelocity);
			float velBDotn = Vector3.Dot(contactConstraint.ContactNormal,bodyB.LinearVelocity)
						+ Vector3.Dot(contactConstraint.RelPosBCrossNormal,bodyB.AngularVelocity);

			relVel = velADotn - velBDotn;

			float positionalError = contactConstraint.Penetration;
			float velocityError = contactConstraint.Restitution - relVel;// * damping;

			float penetrationImpulse = positionalError * contactConstraint.JacDiagABInv;
			float velocityImpulse = velocityError * contactConstraint.JacDiagABInv;
			normalImpulse = penetrationImpulse + velocityImpulse;

			// See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
			float oldNormalImpulse = contactConstraint.AppliedImpulse;
			float sum = oldNormalImpulse + normalImpulse;
			contactConstraint.AppliedImpulse = 0 > sum ? 0 : sum;

			float oldVelocityImpulse = contactConstraint.AppliedVelocityImpulse;
			float velocitySum = oldVelocityImpulse + velocityImpulse;
			contactConstraint.AppliedVelocityImpulse = 0 > velocitySum ? 0 : velocitySum;

			normalImpulse = contactConstraint.AppliedImpulse - oldNormalImpulse;

			if (bodyA.InvMass != 0)
			{
				bodyA.ApplyImpulse(contactConstraint.ContactNormal * bodyA.InvMass,
					contactConstraint.AngularComponentA, normalImpulse);
			}
			if (bodyB.InvMass != 0)
			{
				bodyB.ApplyImpulse(contactConstraint.ContactNormal * bodyB.InvMass,
					contactConstraint.AngularComponentB, -normalImpulse);
			}

			return normalImpulse;
		}

		public virtual float ResolveSingleFrictionCacheFriendly(
			SolverBody bodyA,
			SolverBody bodyB,
			SolverConstraint contactConstraint,
			ContactSolverInfo solverInfo,
			float appliedNormalImpulse)
		{
			float combinedFriction = contactConstraint.Friction;
			float limit = appliedNormalImpulse * combinedFriction;

			if (appliedNormalImpulse > 0)
			//friction
			{
				float j1;
				{
					float relVel;
					float velADotn = Vector3.Dot(contactConstraint.ContactNormal, bodyA.LinearVelocity)
								+ Vector3.Dot(contactConstraint.RelPosACrossNormal, bodyA.AngularVelocity);
					float velBDotn = Vector3.Dot(contactConstraint.ContactNormal, bodyB.LinearVelocity)
						+ Vector3.Dot(contactConstraint.RelPosBCrossNormal, bodyB.AngularVelocity);
					relVel = velADotn - velBDotn;

					// calculate j that moves us to zero relative velocity
					j1 = -relVel * contactConstraint.JacDiagABInv;
					float oldTangentImpulse = contactConstraint.AppliedImpulse;
					contactConstraint.AppliedImpulse = oldTangentImpulse + j1;

					float test = contactConstraint.AppliedImpulse;
					MathHelper.SetMin(ref test, limit);
					MathHelper.SetMax(ref test, -limit);
					contactConstraint.AppliedImpulse = test;

					j1 = contactConstraint.AppliedImpulse - oldTangentImpulse;
				}

				if (bodyA.InvMass != 0)
				{
					bodyA.ApplyImpulse(contactConstraint.ContactNormal * bodyA.InvMass, contactConstraint.AngularComponentA, j1);
				}
				if (bodyB.InvMass != 0)
				{
					bodyB.ApplyImpulse(contactConstraint.ContactNormal * bodyB.InvMass, contactConstraint.AngularComponentB, -j1);
				}
			}
			return 0;
		}

		public virtual float SolveGroupCacheFriendly(List<CollisionObject> bodies, List<PersistentManifold> manifolds, int numManifolds, List<TypedConstraint> constraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
		{
			if (constraints.Count + numManifolds == 0)
			{
				return 0;
			}

			for (int i = 0; i < numManifolds; i++)
			{
				PersistentManifold manifold = manifolds[i];
				RigidBody rbA = (RigidBody)manifold.BodyA;
				RigidBody rbB = (RigidBody)manifold.BodyB;

				manifold.RefreshContactPoints(rbA.CenterOfMassTransform, rbB.CenterOfMassTransform);
			}

			int minReservation = manifolds.Count * 2;

			_tmpSolverBodyPool = new List<SolverBody>(minReservation);

			for (int i = 0; i < bodies.Count; i++)
			{
				RigidBody rb = RigidBody.Upcast(bodies[i]);
				if (rb != null && rb.IslandTag >= 0)
				{
					BulletDebug.Assert(rb.CompanionID < 0);
					int solverBodyId = _tmpSolverBodyPool.Count;
					SolverBody solverBody;
					InitSolverBody(out solverBody, rb);
					_tmpSolverBodyPool.Add(solverBody);
					rb.CompanionID = solverBodyId;
				}
			}

			_tmpSolverConstraintPool = new List<SolverConstraint>(minReservation);
			_tmpSolverFrictionConstraintPool = new List<SolverConstraint>(minReservation);

			for (int i = 0; i < numManifolds; i++)
			{
				PersistentManifold manifold = manifolds[i];
				RigidBody rb0 = (RigidBody)manifold.BodyA;
				RigidBody rb1 = (RigidBody)manifold.BodyB;

				int solverBodyIdA = -1;
				int solverBodyIdB = -1;

				//if (i == 89)
				//    System.Diagnostics.Debugger.Break();

				if (manifold.ContactsCount != 0)
				{
					if (rb0.IslandTag >= 0)
					{
						solverBodyIdA = rb0.CompanionID;
					}
					else
					{
						//create a static body
						solverBodyIdA = _tmpSolverBodyPool.Count;
						SolverBody solverBody;
						InitSolverBody(out solverBody, rb0);
						_tmpSolverBodyPool.Add(solverBody);
					}

					if (rb1.IslandTag >= 0)
					{
						solverBodyIdB = rb1.CompanionID;
					}
					else
					{
						//create a static body
						solverBodyIdB = _tmpSolverBodyPool.Count;
						SolverBody solverBody;
						InitSolverBody(out solverBody, rb1);
						_tmpSolverBodyPool.Add(solverBody);
					}
				}

				if (solverBodyIdB == -1 || solverBodyIdA == -1)
					System.Diagnostics.Debug.WriteLine(string.Format("We're in ass ! {0}", i));

				for (int j = 0; j < manifold.ContactsCount; j++)
				{
					ManifoldPoint cp = manifold.GetContactPoint(j);

					int frictionIndex = _tmpSolverConstraintPool.Count;

					if (cp.Distance <= 0)
					{

						Vector3 pos1 = cp.PositionWorldOnA;
						Vector3 pos2 = cp.PositionWorldOnB;

						Vector3 rel_pos1 = pos1 - rb0.CenterOfMassPosition;
						Vector3 rel_pos2 = pos2 - rb1.CenterOfMassPosition;

						float relaxation = 1;
						{
							SolverConstraint solverConstraint = new SolverConstraint();
							_tmpSolverConstraintPool.Add(solverConstraint);

							solverConstraint.SolverBodyIdA = solverBodyIdA;
							solverConstraint.SolverBodyIdB = solverBodyIdB;
							solverConstraint.ConstraintType = SolverConstraint.SolverConstraintType.Contact;

							//can be optimized, the cross products are already calculated
							float denom0 = rb0.ComputeImpulseDenominator(pos1, cp.NormalWorldOnB);
							float denom1 = rb1.ComputeImpulseDenominator(pos2, cp.NormalWorldOnB);
							float denom = relaxation / (denom0 + denom1);
							solverConstraint.JacDiagABInv = denom;

							solverConstraint.ContactNormal = cp.NormalWorldOnB;
							solverConstraint.RelPosACrossNormal = Vector3.Cross(rel_pos1, cp.NormalWorldOnB);
							solverConstraint.RelPosBCrossNormal = Vector3.Cross(rel_pos2, cp.NormalWorldOnB);

							Vector3 vel1 = rb0.GetVelocityInLocalPoint(rel_pos1);
							Vector3 vel2 = rb1.GetVelocityInLocalPoint(rel_pos2);

							Vector3 vel = vel1 - vel2;
							float rel_vel;
							rel_vel = Vector3.Dot(cp.NormalWorldOnB, vel);


							solverConstraint.Penetration = cp.Distance;//btScalar(infoGlobal.m_numIterations);
							solverConstraint.Friction = cp.CombinedFriction;
							float rest = RestitutionCurve(rel_vel, cp.CombinedRestitution);
							if (rest <= 0)
							{
								rest = 0;
							}

							float penVel = -solverConstraint.Penetration / infoGlobal.TimeStep;
							if (rest > penVel)
							{
								rest = 0;
							}
							solverConstraint.Restitution = rest;

							solverConstraint.Penetration *= -(infoGlobal.Erp / infoGlobal.TimeStep);

							solverConstraint.AppliedImpulse = 0f;
							solverConstraint.AppliedVelocityImpulse = 0f;

#warning Check to see if we need Vector3.Transform
							Vector3 torqueAxis0 = Vector3.Cross(rel_pos1, cp.NormalWorldOnB);
							solverConstraint.AngularComponentA = Vector3.TransformNormal(torqueAxis0, rb0.InvInertiaTensorWorld);
							Vector3 torqueAxis1 = Vector3.Cross(rel_pos2, cp.NormalWorldOnB);
							solverConstraint.AngularComponentB = Vector3.TransformNormal(torqueAxis1, rb1.InvInertiaTensorWorld);
						}
						//create 2 '1d axis' constraints for 2 tangential friction directions

						//re-calculate friction direction every frame, todo: check if this is really needed
						Vector3 frictionTangential0a = new Vector3(),
								frictionTangential1b = new Vector3();

						MathHelper.PlaneSpace1(cp.NormalWorldOnB, ref frictionTangential0a, ref frictionTangential1b);
						{
							SolverConstraint solverConstraint = new SolverConstraint();
							_tmpSolverFrictionConstraintPool.Add(solverConstraint);
							solverConstraint.ContactNormal = frictionTangential0a;

							solverConstraint.SolverBodyIdA = solverBodyIdA;
							solverConstraint.SolverBodyIdB = solverBodyIdB;
							solverConstraint.ConstraintType = SolverConstraint.SolverConstraintType.Friction;
							solverConstraint.FrictionIndex = frictionIndex;

							solverConstraint.Friction = cp.CombinedFriction;

							solverConstraint.AppliedImpulse = 0;
							solverConstraint.AppliedVelocityImpulse = 0;

							float denom0 = rb0.ComputeImpulseDenominator(pos1, solverConstraint.ContactNormal);
							float denom1 = rb1.ComputeImpulseDenominator(pos2, solverConstraint.ContactNormal);
							float denom = relaxation / (denom0 + denom1);
							solverConstraint.JacDiagABInv = denom;

							{
								Vector3 ftorqueAxis0 = Vector3.Cross(rel_pos1, solverConstraint.ContactNormal);
								solverConstraint.RelPosACrossNormal = ftorqueAxis0;
								solverConstraint.AngularComponentA = Vector3.TransformNormal(ftorqueAxis0, rb0.InvInertiaTensorWorld);
							}
							{
								Vector3 ftorqueAxis0 = Vector3.Cross(rel_pos2, solverConstraint.ContactNormal);
								solverConstraint.RelPosBCrossNormal = ftorqueAxis0;
								solverConstraint.AngularComponentB = Vector3.TransformNormal(ftorqueAxis0, rb1.InvInertiaTensorWorld);
							}
						}


						{

							SolverConstraint solverConstraint = new SolverConstraint();
							_tmpSolverFrictionConstraintPool.Add(solverConstraint);
							solverConstraint.ContactNormal = frictionTangential1b;

							solverConstraint.SolverBodyIdA = solverBodyIdA;
							solverConstraint.SolverBodyIdB = solverBodyIdB;
							solverConstraint.ConstraintType = SolverConstraint.SolverConstraintType.Friction;
							solverConstraint.FrictionIndex = frictionIndex;

							solverConstraint.Friction = cp.CombinedFriction;

							solverConstraint.AppliedImpulse = 0;
							solverConstraint.AppliedVelocityImpulse = 0;

							float denom0 = rb0.ComputeImpulseDenominator(pos1, solverConstraint.ContactNormal);
							float denom1 = rb1.ComputeImpulseDenominator(pos2, solverConstraint.ContactNormal);
							float denom = relaxation / (denom0 + denom1);
							solverConstraint.JacDiagABInv = denom;
							{
								Vector3 ftorqueAxis1 = Vector3.Cross(rel_pos1, solverConstraint.ContactNormal);
								solverConstraint.RelPosACrossNormal = ftorqueAxis1;
								solverConstraint.AngularComponentA = Vector3.TransformNormal(ftorqueAxis1, rb0.InvInertiaTensorWorld);
							}
							{
								Vector3 ftorqueAxis1 = Vector3.Cross(rel_pos2, solverConstraint.ContactNormal);
								solverConstraint.RelPosBCrossNormal = ftorqueAxis1;
								solverConstraint.AngularComponentB = Vector3.TransformNormal(ftorqueAxis1, rb1.InvInertiaTensorWorld);
							}
						}
					}
				}
			}

			ContactSolverInfo info = infoGlobal;
			{
				for (int j = 0; j < constraints.Count; j++)
				{
					TypedConstraint constraint = constraints[j];
					constraint.BuildJacobian();
				}
			}

			int numConstraintPool = _tmpSolverConstraintPool.Count;
			int numFrictionPool = _tmpSolverFrictionConstraintPool.Count;

			//todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
			List<int> gOrderTmpConstraintPool = new List<int>(numConstraintPool);
			List<int> gOrderFrictionConstraintPool = new List<int>(numFrictionPool);
			{
				for (int i = 0; i < numConstraintPool; i++)
				{
					gOrderTmpConstraintPool.Add(i);
				}
				for (int i = 0; i < numFrictionPool; i++)
				{
					gOrderFrictionConstraintPool.Add(i);
				}
			}

			//should traverse the contacts random order...
			int iteration;
			{
				for (iteration = 0; iteration < info.IterationsCount; iteration++)
				{

					int j;
					if ((_solverMode & SolverMode.RandomizeOrder) != SolverMode.None)
					{
						if ((iteration & 7) == 0)
						{
							for (j = 0; j < numConstraintPool; ++j)
							{
								int tmp = gOrderTmpConstraintPool[j];
								int swapi = RandInt2(j + 1);
								gOrderTmpConstraintPool[j] = gOrderTmpConstraintPool[swapi];
								gOrderTmpConstraintPool[swapi] = tmp;
							}

							for (j = 0; j < numFrictionPool; ++j)
							{
								int tmp = gOrderFrictionConstraintPool[j];
								int swapi = RandInt2(j + 1);
								gOrderFrictionConstraintPool[j] = gOrderFrictionConstraintPool[swapi];
								gOrderFrictionConstraintPool[swapi] = tmp;
							}
						}
					}

					for (j = 0; j < constraints.Count; j++)
					{
						TypedConstraint constraint = constraints[j];
						//todo: use solver bodies, so we don't need to copy from/to btRigidBody

						if ((constraint.RigidBodyA.IslandTag >= 0) && (constraint.RigidBodyA.CompanionID >= 0))
						{
							_tmpSolverBodyPool[constraint.RigidBodyA.CompanionID].WriteBackVelocity();
						}
						if ((constraint.RigidBodyB.IslandTag >= 0) && (constraint.RigidBodyB.CompanionID >= 0))
						{
							_tmpSolverBodyPool[constraint.RigidBodyB.CompanionID].WriteBackVelocity();
						}

						constraint.SolveConstraint(info.TimeStep);

						if ((constraint.RigidBodyA.IslandTag >= 0) && (constraint.RigidBodyA.CompanionID >= 0))
						{
							_tmpSolverBodyPool[constraint.RigidBodyA.CompanionID].ReadVelocity();
						}
						if ((constraint.RigidBodyB.IslandTag >= 0) && (constraint.RigidBodyB.CompanionID >= 0))
						{
							_tmpSolverBodyPool[constraint.RigidBodyB.CompanionID].ReadVelocity();
						}

					}

					{
						int numPoolConstraints = _tmpSolverConstraintPool.Count;
						for (j = 0; j < numPoolConstraints; j++)
						{
							SolverConstraint solveManifold = _tmpSolverConstraintPool[gOrderTmpConstraintPool[j]];
							ResolveSingleCollisionCombinedCacheFriendly(_tmpSolverBodyPool[solveManifold.SolverBodyIdA],
								_tmpSolverBodyPool[solveManifold.SolverBodyIdB], solveManifold, info);
						}
					}

					{
						int numFrictionPoolConstraints = _tmpSolverFrictionConstraintPool.Count;
						for (j = 0; j < numFrictionPoolConstraints; j++)
						{
							SolverConstraint solveManifold = _tmpSolverFrictionConstraintPool[gOrderFrictionConstraintPool[j]];
							float appliedNormalImpulse = _tmpSolverConstraintPool[solveManifold.FrictionIndex].AppliedImpulse;

							ResolveSingleFrictionCacheFriendly(_tmpSolverBodyPool[solveManifold.SolverBodyIdA],
								_tmpSolverBodyPool[solveManifold.SolverBodyIdB], solveManifold, info, appliedNormalImpulse);
						}
					}
				}
			}

			for (int i = 0; i < _tmpSolverBodyPool.Count; i++)
			{
				_tmpSolverBodyPool[i].WriteBackVelocity();
			}

			_tmpSolverBodyPool.Clear();
			_tmpSolverConstraintPool.Clear();
			_tmpSolverFrictionConstraintPool.Clear();

			return 0;
		}

		public virtual float SolveGroup(List<CollisionObject> bodies, List<PersistentManifold> manifolds, int numManifolds, List<TypedConstraint> constraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
		{
			if ((_solverMode & SolverMode.CacheFriendly) != SolverMode.None)
			{
				return SolveGroupCacheFriendly(bodies, manifolds, numManifolds, constraints, infoGlobal, debugDrawer);
			}

			ContactSolverInfo info = infoGlobal;
			int totalPoints = 0;

			int numiter = infoGlobal.IterationsCount;

			for (int j = 0; j < manifolds.Count; j++)
			{
				PersistentManifold manifold = manifolds[j];
				PrepareConstraints(manifold, info);

				for (int p = 0; p < manifolds[j].ContactsCount; p++)
				{
					_order[totalPoints].ManifoldIndex = j;
					_order[totalPoints].PointIndex = p;
					totalPoints++;
				}
			}

			for (int j = 0; j < constraints.Count; j++)
			{
				constraints[j].BuildJacobian();
			}

			//should traverse the contacts random order...
			int iteration;

			for (iteration = 0; iteration < numiter; iteration++)
			{
				int j;
				if ((_solverMode & SolverMode.RandomizeOrder) != SolverMode.None)
				{
					if ((iteration & 7) == 0)
					{
						for (j = 0; j < totalPoints; ++j)
						{
							OrderIndex tmp = _order[j];
							int swapi = RandInt2(j + 1);
							_order[j] = _order[swapi];
							_order[swapi] = tmp;
						}
					}
				}

				for (j = 0; j < constraints.Count; j++)
				{
					constraints[j].SolveConstraint(info.TimeStep);
				}

				for (j = 0; j < totalPoints; j++)
				{
					PersistentManifold manifold = manifolds[_order[j].ManifoldIndex];
					Solve((RigidBody)manifold.BodyA, (RigidBody)manifold.BodyB,
						manifold.GetContactPoint(_order[j].PointIndex), info, iteration, debugDrawer);
				}

				for (j = 0; j < totalPoints; j++)
				{
					PersistentManifold manifold = manifolds[_order[j].ManifoldIndex];
					SolveFriction((RigidBody)manifold.BodyA,
						(RigidBody)manifold.BodyB, manifold.GetContactPoint(_order[j].PointIndex), info, iteration, debugDrawer);
				}
			}

			return 0;
		}

		private void InitSolverBody(out SolverBody solverBody, RigidBody rigidBody)
		{
			solverBody = new SolverBody();
			solverBody.AngularVelocity = rigidBody.AngularVelocity;
			solverBody.CenterOfMassPosition = rigidBody.CenterOfMassPosition;
			solverBody.Friction = rigidBody.Friction;
			solverBody.InvMass = rigidBody.InverseMass;
			solverBody.LinearVelocity = rigidBody.LinearVelocity;
			solverBody.OriginalBody = rigidBody;
			solverBody.AngularFactor = rigidBody.AngularFactor;
		}

		private long Rand2()
		{
			_seed2 = (1664525L * _seed2 + 1013904223L) & 0xffffffff;
			return _seed2;
		}

		private int RandInt2(int n)
		{
			// seems good; xor-fold and modulus
			long un = n;
			long r = Rand2();

			// note: probably more aggressive than it needs to be -- might be
			//       able to get away without one or two of the innermost branches.
			if (un <= 0x00010000L)
			{
				r ^= (r >> 16);
				if (un <= 0x00000100L)
				{
					r ^= (r >> 8);
					if (un <= 0x00000010L)
					{
						r ^= (r >> 4);
						if (un <= 0x00000004L)
						{
							r ^= (r >> 2);
							if (un <= 0x00000002L)
							{
								r ^= (r >> 1);
							}
						}
					}
				}
			}
			return (int)(r % un);
		}

		protected struct OrderIndex
		{
			private int _manifoldIndex;
			private int _pointIndex;

			public int ManifoldIndex { get { return _manifoldIndex; } set { _manifoldIndex = value; } }
			public int PointIndex { get { return _pointIndex; } set { _pointIndex = value; } }
		}
	}
}
