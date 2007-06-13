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
	public class ConstraintSetting
	{
		private float _tau, _damping;

		public ConstraintSetting()
		{
			_tau = 0.3f;
			_damping = 1.0f;
		}

		public float Damping
		{
			get { return _damping; }
			set { _damping = value; }
		}

		public float Tau
		{
			get { return _tau; }
			set { _tau = value; }
		}
	}

	public class Point2PointConstraint : TypedConstraint
	{
		private JacobianEntry[] _jacobian;
		private Vector3 _pivotInA, _pivotInB;

		private ConstraintSetting _setting = new ConstraintSetting();

		public Point2PointConstraint()
		{
			_jacobian = new JacobianEntry[3];
		}

		public Point2PointConstraint(RigidBody rbA, RigidBody rbB, Vector3 pivotInA, Vector3 pivotInB)
			: base(rbA, rbB)
		{
			_jacobian = new JacobianEntry[3];

			_pivotInA = pivotInA;
			_pivotInB = pivotInB;
		}

		public Point2PointConstraint(RigidBody rbA, Vector3 pivotInA)
			: base(rbA)
		{
			_jacobian = new JacobianEntry[3];

			_pivotInA = pivotInA;
			_pivotInB = MathHelper.MatrixToVector(rbA.CenterOfMassTransform, _pivotInA);
		}

		public ConstraintSetting Settings { get { return _setting; } set { _setting = value; } }

		public Vector3 PivotInA
		{
			set
			{
				_pivotInA = value;
			}
		}

		public Vector3 PivotInB
		{
			set
			{
				_pivotInB = value;
			}
		}

		public override void BuildJacobian()
		{
			Vector3 normal = new Vector3();

			for (int i = 0; i < 3; i++)
			{
				MathHelper.SetElement(ref normal, i, 1);
				_jacobian[i] = new JacobianEntry(
					MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform),
					MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform),
					MathHelper.Transform(_pivotInA, RigidBodyA.CenterOfMassTransform) - RigidBodyA.CenterOfMassPosition,
					MathHelper.Transform(_pivotInB, RigidBodyB.CenterOfMassTransform) - RigidBodyB.CenterOfMassPosition,
					normal,
					RigidBodyA.InvInertiaDiagLocal,
					RigidBodyA.InverseMass,
					RigidBodyB.InvInertiaDiagLocal,
					RigidBodyB.InverseMass
				);
				MathHelper.SetElement(ref normal, i, 0);
			}
		}

		public override void SolveConstraint(float timeStep)
		{
			Vector3 pivotAInW = MathHelper.Transform(_pivotInA, RigidBodyA.CenterOfMassTransform);
			Vector3 pivotBInW = MathHelper.Transform(_pivotInB, RigidBodyB.CenterOfMassTransform);

			Vector3 normal = new Vector3();

			for (int i = 0; i < 3; i++)
			{
				MathHelper.SetElement(ref normal, i, 1);

				float jacDiagABInv = 1.0f / _jacobian[i].Diagonal;

				Vector3 rel_pos1 = pivotAInW - RigidBodyA.CenterOfMassPosition;
				Vector3 rel_pos2 = pivotBInW - RigidBodyB.CenterOfMassPosition;

				Vector3 vel1 = RigidBodyA.GetVelocityInLocalPoint(rel_pos1);
				Vector3 vel2 = RigidBodyB.GetVelocityInLocalPoint(rel_pos2);

				Vector3 vel = vel1 - vel2;

				float rel_vel = Vector3.Dot(normal, vel);
				float depth = -Vector3.Dot((pivotAInW - pivotBInW), normal);

				float impulse = depth * _setting.Tau / timeStep * jacDiagABInv - _setting.Damping * rel_vel * jacDiagABInv;
				AppliedImpulse += impulse;
				Vector3 impulseVector = normal * impulse;

				RigidBodyA.ApplyImpulse(impulseVector, pivotAInW - RigidBodyA.CenterOfMassPosition);
				RigidBodyB.ApplyImpulse(-impulseVector, pivotBInW - RigidBodyB.CenterOfMassPosition);

				MathHelper.SetElement(ref normal, i, 0);
			}
		}
	}
}