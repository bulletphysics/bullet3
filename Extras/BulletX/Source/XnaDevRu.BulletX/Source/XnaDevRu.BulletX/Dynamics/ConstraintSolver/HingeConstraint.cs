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
	/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
	/// axis defines the orientation of the hinge axis
	/// </summary>
	public class HingeConstraint : TypedConstraint
	{
		private JacobianEntry[] _jac = new JacobianEntry[3]; //3 orthogonal linear constraints
		private JacobianEntry[] _jacAng = new JacobianEntry[3]; //2 orthogonal angular constraints + 1 for limit/motor

		private Vector3 _pivotInA;
		private Vector3 _pivotInB;
		private Vector3 _axisInA;
		private Vector3 _axisInB;

		private bool _angularOnly;

		private float _motorTargetVelocity;
		private float _maxMotorImpulse;
		private bool _enableAngularMotor;

		public HingeConstraint(RigidBody rbA, RigidBody rbB, Vector3 pivotInA, Vector3 pivotInB, Vector3 axisInA, Vector3 axisInB)
			: base(rbA, rbB)
		{
			_pivotInA = pivotInA;
			_pivotInB = pivotInB;
			_axisInA = axisInA;
			_axisInB = -axisInB;
			_angularOnly = false;
		}

		public HingeConstraint(RigidBody rbA, Vector3 pivotInA, Vector3 axisInA)
			: base(rbA)
		{
			_pivotInA = pivotInA;
			_pivotInB = MathHelper.MatrixToVector(rbA.CenterOfMassTransform, pivotInA);
			_axisInA = axisInA;
			//fixed axis in worldspace
			_axisInB = MathHelper.TransformNormal(-axisInA, rbA.CenterOfMassTransform);
			_angularOnly = false;
		}

		public HingeConstraint() { }

		public bool AngularOnly { set { _angularOnly = value; } }

		public override void BuildJacobian()
		{
			AppliedImpulse = 0f;

			Vector3 normal = new Vector3();

			if (!_angularOnly)
			{
				for (int i = 0; i < 3; i++)
				{
					MathHelper.SetElement(ref normal, i, 1);
					_jac[i] = new JacobianEntry(
							MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform),
							MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform),
							MathHelper.Transform(_pivotInA, RigidBodyA.CenterOfMassTransform) - RigidBodyA.CenterOfMassPosition,
							MathHelper.Transform(_pivotInB, RigidBodyB.CenterOfMassTransform) - RigidBodyB.CenterOfMassPosition,
							normal,
							RigidBodyA.InvInertiaDiagLocal,
							RigidBodyA.InverseMass,
							RigidBodyB.InvInertiaDiagLocal,
							RigidBodyB.InverseMass);
					MathHelper.SetElement(ref normal, i, 0);
				}
			}

			//calculate two perpendicular jointAxis, orthogonal to hingeAxis
			//these two jointAxis require equal angular velocities for both bodies
			//this is unused for now, it's a todo
			Vector3 jointAxisALocal = new Vector3();
			Vector3 jointAxisBLocal = new Vector3();
			MathHelper.PlaneSpace1(_axisInA, ref jointAxisALocal, ref jointAxisBLocal);

			Vector3 jointAxisA = MathHelper.TransformNormal(jointAxisALocal, RigidBodyA.CenterOfMassTransform);
			Vector3 jointAxisB = MathHelper.TransformNormal(jointAxisBLocal, RigidBodyA.CenterOfMassTransform);
			Vector3 hingeAxisWorld = MathHelper.TransformNormal(_axisInA, RigidBodyA.CenterOfMassTransform);

			_jacAng[0] = new JacobianEntry(jointAxisA,
				MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform),
				MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform),
				RigidBodyA.InvInertiaDiagLocal,
				RigidBodyB.InvInertiaDiagLocal);

			_jacAng[1] = new JacobianEntry(jointAxisB,
				MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform),
				MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform),
				RigidBodyA.InvInertiaDiagLocal,
				RigidBodyB.InvInertiaDiagLocal);

			_jacAng[2] = new JacobianEntry(hingeAxisWorld,
				MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform),
				MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform),
				RigidBodyA.InvInertiaDiagLocal,
				RigidBodyB.InvInertiaDiagLocal);
		}

		public override void SolveConstraint(float timeStep)
		{
			Vector3 pivotAInW = MathHelper.Transform(_pivotInA, RigidBodyA.CenterOfMassTransform);
			Vector3 pivotBInW = MathHelper.Transform(_pivotInB, RigidBodyB.CenterOfMassTransform);

			Vector3 normal = new Vector3(0, 0, 0);
			float tau = 0.3f;
			float damping = 1f;

			//linear part
			if (!_angularOnly)
			{
				for (int i = 0; i < 3; i++)
				{
					if (i == 0)
						normal = new Vector3(1, 0, 0);
					else if (i == 1)
						normal = new Vector3(0, 1, 0);
					else
						normal = new Vector3(0, 0, 1);

					float jacDiagABInv = 1f / _jac[i].Diagonal;

					Vector3 rel_pos1 = pivotAInW - RigidBodyA.CenterOfMassPosition;
					Vector3 rel_pos2 = pivotBInW - RigidBodyB.CenterOfMassPosition;

					Vector3 vel1 = RigidBodyA.GetVelocityInLocalPoint(rel_pos1);
					Vector3 vel2 = RigidBodyB.GetVelocityInLocalPoint(rel_pos2);
					Vector3 vel = vel1 - vel2;
					float rel_vel;
					rel_vel = Vector3.Dot(normal, vel);
					//positional error (zeroth order error)
					float depth = -Vector3.Dot(pivotAInW - pivotBInW, normal); //this is the error projected on the normal
					float impulse = depth * tau / timeStep * jacDiagABInv - damping * rel_vel * jacDiagABInv * damping;
					AppliedImpulse += impulse;
					Vector3 impulse_vector = normal * impulse;
					RigidBodyA.ApplyImpulse(impulse_vector, pivotAInW - RigidBodyA.CenterOfMassPosition);
					RigidBodyB.ApplyImpulse(-impulse_vector, pivotBInW - RigidBodyB.CenterOfMassPosition);
				}
			}
			//solve angular part
			// get axes in world space
			Vector3 axisA = MathHelper.TransformNormal(_axisInA, RigidBodyA.CenterOfMassTransform);
			Vector3 axisB = MathHelper.TransformNormal(_axisInB, RigidBodyB.CenterOfMassTransform);

			Vector3 angVelA = RigidBodyA.AngularVelocity;
			Vector3 angVelB = RigidBodyB.AngularVelocity;
			Vector3 angVelAroundHingeAxisA = axisA * Vector3.Dot(axisA, angVelA);
			Vector3 angVelAroundHingeAxisB = axisB * Vector3.Dot(axisB, angVelB);

			Vector3 angAOrthog = angVelA - angVelAroundHingeAxisA;
			Vector3 angBOrthog = angVelB - angVelAroundHingeAxisB;
			Vector3 velrelOrthog = angAOrthog - angBOrthog;

			//solve angular velocity correction
			float relaxation = 1f;
			float len = velrelOrthog.Length();
			if (len > 0.00001f)
			{
				Vector3 normal2 = Vector3.Normalize(velrelOrthog);
				float denom = RigidBodyA.ComputeAngularImpulseDenominator(normal2) +
					RigidBodyB.ComputeAngularImpulseDenominator(normal2);
				// scale for mass and relaxation
				velrelOrthog *= (1f / denom) * 0.9f;
			}

			//solve angular positional correction
			Vector3 angularError = -Vector3.Cross(axisA, axisB) * (1f / timeStep);
			float len2 = angularError.Length();
			if (len2 > 0.00001f)
			{
				Vector3 normal2 = Vector3.Normalize(angularError);
				float denom2 = RigidBodyA.ComputeAngularImpulseDenominator(normal2) +
						RigidBodyB.ComputeAngularImpulseDenominator(normal2);
				angularError *= (1f / denom2) * relaxation;
			}

			RigidBodyA.ApplyTorqueImpulse(-velrelOrthog + angularError);
			RigidBodyB.ApplyTorqueImpulse(velrelOrthog - angularError);

			//apply motor
			if (_enableAngularMotor)
			{
				//todo: add limits too
				Vector3 angularLimit = Vector3.Zero;

				Vector3 velrel = angVelAroundHingeAxisA - angVelAroundHingeAxisB;
				float projRelVel = Vector3.Dot(velrel, axisA);

				float desiredMotorVel = _motorTargetVelocity;
				float motorRelvel = desiredMotorVel - projRelVel;

				float denom3 = RigidBodyA.ComputeAngularImpulseDenominator(axisA) +
						RigidBodyB.ComputeAngularImpulseDenominator(axisA);

				float unclippedMotorImpulse = (1f / denom3) * motorRelvel;
				//todo: should clip against accumulated impulse
				float clippedMotorImpulse = unclippedMotorImpulse > _maxMotorImpulse ? _maxMotorImpulse : unclippedMotorImpulse;
				clippedMotorImpulse = clippedMotorImpulse < -_maxMotorImpulse ? -_maxMotorImpulse : clippedMotorImpulse;
				Vector3 motorImp = clippedMotorImpulse * axisA;

				RigidBodyA.ApplyTorqueImpulse(motorImp + angularLimit);
				RigidBodyB.ApplyTorqueImpulse(-motorImp - angularLimit);
			}
		}

		public void EnableAngularMotor(bool enableMotor, float targetVelocity, float maxMotorImpulse)
		{
			_enableAngularMotor = enableMotor;
			_motorTargetVelocity = targetVelocity;
			_maxMotorImpulse = maxMotorImpulse;
		}

		public void UpdateRHS(float timeStep)
		{
		}
	}
}
