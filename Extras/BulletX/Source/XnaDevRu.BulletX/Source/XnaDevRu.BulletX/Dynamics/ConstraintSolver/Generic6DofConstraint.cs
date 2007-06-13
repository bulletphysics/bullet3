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
	/// Generic6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
	/// Generic6DofConstraint can leave any of the 6 degree of freedom 'free' or 'locked'
	/// Work in progress (is still a Hinge actually)
	/// </summary>
	public class Generic6DofConstraint : TypedConstraint
	{
		private static readonly float[] _sign = { 1.0f, -1.0f, 1.0f };
		private static readonly int[] _axisA = { 1, 0, 0 };
		private static readonly int[] _axisB = { 2, 2, 1 };

		private JacobianEntry[] _jacLinear = new JacobianEntry[3];			// 3 orthogonal linear constraints
		private JacobianEntry[] _jacAng = new JacobianEntry[3];		// 3 orthogonal angular constraints

		private Matrix _frameInA;			// the constraint space w.r.t body A
		private Matrix _frameInB;			// the constraint space w.r.t body B

		private float[] _lowerLimit = new float[6];	// the constraint lower limits
		private float[] _upperLimit = new float[6];	// the constraint upper limits

		private float[] _accumulatedImpulse = new float[6];

		public Generic6DofConstraint(RigidBody rbA, RigidBody rbB, Matrix frameInA, Matrix frameInB)
			: base(rbA, rbB)
		{
			_frameInA = frameInA;
			_frameInB = frameInB;
			//free means upper < lower, 
			//locked means upper == lower
			//limited means upper > lower
			//so start all locked
			for (int i = 0; i < 6; ++i)
			{
				_lowerLimit[i] = 0.0f;
				_upperLimit[i] = 0.0f;
				_accumulatedImpulse[i] = 0.0f;
			}
		}

		public Generic6DofConstraint() { }

		public void UpdateRHS(float timeStep) { }

		public float ComputeAngle(int axis)
		{
			float angle = 0;

			switch (axis)
			{
				case 0:
					{
						Vector3 v1 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInA, 1), RigidBodyA.CenterOfMassTransform);
						Vector3 v2 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, 1), RigidBodyB.CenterOfMassTransform);
						Vector3 w2 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, 2), RigidBodyB.CenterOfMassTransform);

						float s = Vector3.Dot(v1, w2);
						float c = Vector3.Dot(v1, v2);

						angle = (float)Math.Atan2(s, c);
						break;
					}
				case 1:
					{
						Vector3 w1 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInA, 2), RigidBodyA.CenterOfMassTransform);
						Vector3 w2 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, 2), RigidBodyB.CenterOfMassTransform);
						Vector3 u2 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, 0), RigidBodyB.CenterOfMassTransform);

						float s = Vector3.Dot(w1, u2);
						float c = Vector3.Dot(w1, w2);

						angle = (float)Math.Atan2(s, c);
						break;
					}
				case 2:
					{
						Vector3 u1 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInA, 0), RigidBodyA.CenterOfMassTransform);
						Vector3 u2 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, 0), RigidBodyB.CenterOfMassTransform);
						Vector3 v2 = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, 1), RigidBodyB.CenterOfMassTransform);

						float s = Vector3.Dot(u1, v2);
						float c = Vector3.Dot(u1, u2);

						angle = (float)Math.Atan2(s, c);
						break;
					}
				default: BulletDebug.Assert(false); break;
			}

			return angle;
		}

		public void SetLinearLowerLimit(Vector3 linearLower)
		{
			_lowerLimit[0] = linearLower.X;
			_lowerLimit[1] = linearLower.Y;
			_lowerLimit[2] = linearLower.Z;
		}

		public void SetLinearUpperLimit(Vector3 linearUpper)
		{
			_upperLimit[0] = linearUpper.X;
			_upperLimit[1] = linearUpper.Y;
			_upperLimit[2] = linearUpper.Z;
		}

		public void SetAngularLowerLimit(Vector3 angularLower)
		{
			_lowerLimit[3] = angularLower.X;
			_lowerLimit[4] = angularLower.Y;
			_lowerLimit[5] = angularLower.Z;
		}

		public void SetAngularUpperLimit(Vector3 angularUpper)
		{
			_upperLimit[3] = angularUpper.X;
			_upperLimit[4] = angularUpper.Y;
			_upperLimit[5] = angularUpper.Z;
		}

		//first 3 are linear, next 3 are angular
		public void SetLimit(int axis, float lo, float hi)
		{
			_lowerLimit[axis] = lo;
			_upperLimit[axis] = hi;
		}

		//free means upper < lower, 
		//locked means upper == lower
		//limited means upper > lower
		//limitIndex: first 3 are linear, next 3 are angular
		public bool IsLimited(int limitIndex)
		{
			return (_upperLimit[limitIndex] >= _lowerLimit[limitIndex]);
		}

		public override void BuildJacobian()
		{
			Vector3 localNormalInA = new Vector3(0, 0, 0);

			Vector3 pivotInA = _frameInA.Translation;
			Vector3 pivotInB = _frameInB.Translation;

			Vector3 pivotAInW = MathHelper.Transform(_frameInA.Translation, RigidBodyA.CenterOfMassTransform);
			Vector3 pivotBInW = MathHelper.Transform(_frameInB.Translation, RigidBodyB.CenterOfMassTransform);

			Vector3 rel_pos1 = pivotAInW - RigidBodyA.CenterOfMassPosition;
			Vector3 rel_pos2 = pivotBInW - RigidBodyB.CenterOfMassPosition;

			//linear part
			for (int i = 0; i < 3; i++)
			{
				if (IsLimited(i))
				{
					if (i == 0)
						localNormalInA = new Vector3(1, 0, 0);
					else if (i == 1)
						localNormalInA = new Vector3(0, 1, 0);
					else
						localNormalInA = new Vector3(0, 0, 1);

					Vector3 normalWorld = MathHelper.TransformNormal(localNormalInA, RigidBodyA.CenterOfMassTransform);

					// Create linear atom
					_jacLinear[i] = new JacobianEntry(
						MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform),
						MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform),
						MathHelper.Transform(pivotInA, RigidBodyA.CenterOfMassTransform) - RigidBodyA.CenterOfMassPosition,
						MathHelper.Transform(pivotInB, RigidBodyB.CenterOfMassTransform) - RigidBodyB.CenterOfMassPosition,
						normalWorld,
						RigidBodyA.InvInertiaDiagLocal,
						RigidBodyA.InverseMass,
						RigidBodyB.InvInertiaDiagLocal,
						RigidBodyB.InverseMass);

					//optionally disable warmstarting
					_accumulatedImpulse[i] = 0f;

					// Apply accumulated impulse
					Vector3 impulse_vector = _accumulatedImpulse[i] * normalWorld;

					RigidBodyA.ApplyImpulse(impulse_vector, rel_pos1);
					RigidBodyB.ApplyImpulse(-impulse_vector, rel_pos2);
				}
			}

			// angular part
			for (int i = 0; i < 3; i++)
			{
				if (IsLimited(i + 3))
				{
					Vector3 axisA = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInA, _axisA[i] + 1), RigidBodyA.CenterOfMassTransform);
					Vector3 axisB = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, _axisB[i] + 1), RigidBodyB.CenterOfMassTransform);

					Vector3 axis = _sign[i] * Vector3.Cross(axisA, axisB);

					// Create angular atom
					_jacAng[i] = new JacobianEntry(axis,
						MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform),
						MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform),
						RigidBodyA.InvInertiaDiagLocal,
						RigidBodyB.InvInertiaDiagLocal);

					_accumulatedImpulse[i + 3] = 0f;

					// Apply accumulated impulse
					Vector3 impulse_vector = _accumulatedImpulse[i + 3] * axis;

					RigidBodyA.ApplyTorqueImpulse(impulse_vector);
					RigidBodyB.ApplyTorqueImpulse(-impulse_vector);
				}
			}
		}

		public override void SolveConstraint(float timeStep)
		{
			float tau = 0.1f;
			float damping = 1.0f;

			Vector3 pivotAInW = MathHelper.Transform(_frameInA.Translation, RigidBodyA.CenterOfMassTransform);
			Vector3 pivotBInW = MathHelper.Transform(_frameInB.Translation, RigidBodyB.CenterOfMassTransform);

			Vector3 rel_pos1 = pivotAInW - RigidBodyA.CenterOfMassPosition;
			Vector3 rel_pos2 = pivotBInW - RigidBodyB.CenterOfMassPosition;

			Vector3 localNormalInA = new Vector3();

			// linear
			for (int i = 0; i < 3; i++)
			{
				if (IsLimited(i))
				{
					Vector3 angvelA = MathHelper.TransformNormal(RigidBodyA.AngularVelocity, MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform));
					Vector3 angvelB = MathHelper.TransformNormal(RigidBodyB.AngularVelocity, MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform));

					if (i == 0)
						localNormalInA = new Vector3(1, 0, 0);
					else if (i == 1)
						localNormalInA = new Vector3(0, 1, 0);
					else
						localNormalInA = new Vector3(0, 0, 1);

					Vector3 normalWorld = MathHelper.TransformNormal(localNormalInA, RigidBodyA.CenterOfMassTransform);

					float jacDiagABInv = 1f / _jacLinear[i].Diagonal;

					//velocity error (first order error)
					float rel_vel = _jacLinear[i].GetRelativeVelocity(RigidBodyA.LinearVelocity, angvelA,
																			RigidBodyB.LinearVelocity, angvelB);

					//positional error (zeroth order error)
					float depth = -Vector3.Dot(pivotAInW - pivotBInW, normalWorld);
					float lo = -1e30f;
					float hi = 1e30f;

					//handle the limits
					if (_lowerLimit[i] < _upperLimit[i])
					{
						if (depth > _upperLimit[i])
						{
							depth -= _upperLimit[i];
							lo = 0f;
						}
						else
						{
							if (depth < _lowerLimit[i])
							{
								depth -= _lowerLimit[i];
								hi = 0f;
							}
							else
							{
								continue;
							}
						}
					}

					float normalImpulse = (tau * depth / timeStep - damping * rel_vel) * jacDiagABInv;
					float oldNormalImpulse = _accumulatedImpulse[i];
					float sum = oldNormalImpulse + normalImpulse;
					_accumulatedImpulse[i] = sum > hi ? 0f : sum < lo ? 0f : sum;
					normalImpulse = _accumulatedImpulse[i] - oldNormalImpulse;

					Vector3 impulse_vector = normalWorld * normalImpulse;
					RigidBodyA.ApplyImpulse(impulse_vector, rel_pos1);
					RigidBodyB.ApplyImpulse(-impulse_vector, rel_pos2);
				}
			}

			Vector3 axis;
			float angle;
			Matrix frameAWorld = RigidBodyA.CenterOfMassTransform * _frameInA;
			Matrix frameBWorld = RigidBodyB.CenterOfMassTransform * _frameInB;

			TransformUtil.CalculateDiffAxisAngle(frameAWorld, frameBWorld, out axis, out angle);
			Quaternion diff = new Quaternion(axis, angle);
			Matrix diffMat = Matrix.CreateFromQuaternion(diff);
			Vector3 xyz;
			// this is not perfect, we can first check which axis are limited, and choose a more appropriate order
			MatrixToEulerXYZ(diffMat, out xyz);

			// angular
			for (int i = 0; i < 3; i++)
			{
				if (IsLimited(i + 3))
				{
					Vector3 angvelA = MathHelper.TransformNormal(RigidBodyA.AngularVelocity, MatrixOperations.Transpose(RigidBodyA.CenterOfMassTransform));
					Vector3 angvelB = MathHelper.TransformNormal(RigidBodyB.AngularVelocity, MatrixOperations.Transpose(RigidBodyB.CenterOfMassTransform));

					float jacDiagABInv = 1f / _jacAng[i].Diagonal;

					//velocity error (first order error)
					float rel_vel = _jacAng[i].GetRelativeVelocity(RigidBodyA.LinearVelocity, angvelA,
																					RigidBodyB.LinearVelocity, angvelB);

					//positional error (zeroth order error)
					Vector3 axisA = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInA, _axisA[i] + 1), RigidBodyA.CenterOfMassTransform);
					Vector3 axisB = MathHelper.TransformNormal(MathHelper.GetColumn(_frameInB, _axisB[i] + 1), RigidBodyB.CenterOfMassTransform);

					float rel_pos = _sign[i] * Vector3.Dot(axisA, axisB);

					float lo = -1e30f;
					float hi = 1e30f;

					//handle the twist limit
					if (_lowerLimit[i + 3] < _upperLimit[i + 3])
					{
						//clamp the values
						float loLimit = _upperLimit[i + 3] > -3.1415 ? _lowerLimit[i + 3] : -1e30f;
						float hiLimit = _upperLimit[i + 3] < 3.1415 ? _upperLimit[i + 3] : 1e30f;

						float projAngle;

						if (i == 0)
							projAngle = -2f * xyz.Z;
						else if (i == 1)
							projAngle = -2f * xyz.Y;
						else
							projAngle = -2f * xyz.Z;

						if (projAngle < loLimit)
						{
							hi = 0f;
							rel_pos = loLimit - projAngle;
						}
						else
						{
							if (projAngle > hiLimit)
							{
								lo = 0f;
								rel_pos = (hiLimit - projAngle);
							}
							else
							{
								continue;
							}
						}
					}

					//impulse

					float normalImpulse = -(tau * rel_pos / timeStep + damping * rel_vel) * jacDiagABInv;
					float oldNormalImpulse = _accumulatedImpulse[i + 3];
					float sum = oldNormalImpulse + normalImpulse;
					_accumulatedImpulse[i + 3] = sum > hi ? 0f : sum < lo ? 0f : sum;
					normalImpulse = _accumulatedImpulse[i + 3] - oldNormalImpulse;

					Vector3 axis2 = _sign[i] * Vector3.Cross(axisA, axisB);
					Vector3 impulse_vector = axis2 * normalImpulse;

					RigidBodyA.ApplyTorqueImpulse(impulse_vector);
					RigidBodyB.ApplyTorqueImpulse(-impulse_vector);
				}
			}
		}

		//MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
		private bool MatrixToEulerXYZ(Matrix mat, out Vector3 xyz)
		{
			// rot =  cy*cz          -cy*sz           sy
			//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
			//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
			xyz = new Vector3();

			if (MathHelper.GetElement(mat, 2) < 1.0f)
			{
				if (MathHelper.GetElement(mat, 2) > -1.0f)
				{
					xyz.X = (float)Math.Atan2(-MathHelper.GetElement(mat, 5), MathHelper.GetElement(mat, 8));
					xyz.Y = (float)Math.Asin(MathHelper.GetElement(mat, 2));
					xyz.Z = (float)Math.Atan2(-MathHelper.GetElement(mat, 1), MathHelper.GetElement(mat, 0));
					return true;
				}
				else
				{
					// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
					xyz.X = -(float)Math.Atan2(MathHelper.GetElement(mat, 3), MathHelper.GetElement(mat, 4));
					xyz.Y = -(float)Math.PI / 2;
					xyz.Z = 0.0f;
					return false;
				}
			}
			else
			{
				// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
				xyz.X = (float)Math.Atan2(MathHelper.GetElement(mat, 3), MathHelper.GetElement(mat, 4));
				xyz.Y = (float)Math.PI / 2;
				xyz.Z = 0.0f;
				return false;
			}
		}
	}
}
