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
	public class RigidBody : CollisionObject
	{
		private static float _linearAirDamping = 1;
		//'temporarily' global variables
		private static float _rigidBodyDeactivationTime = 2;
		private static bool _disableDeactivation = false;

		private static float _linearSleepingThreshold = 0.8f;
		private static float _angularSleepingThreshold = 1.0f;
		private static int _uniqueId = 0;

		private Matrix _invInertiaTensorWorld;
		private Vector3 _linearVelocity;
		private Vector3 _angularVelocity;
		private float _inverseMass;
		private float _angularFactor;

		private Vector3 _gravity;
		private Vector3 _invInertiaLocal;
		private Vector3 _totalForce;
		private Vector3 _totalTorque;

		private float _linearDamping;
		private float _angularDamping;

		//m_optionalMotionState allows to automatic synchronize the world transform for active objects
		private MotionState _optionalMotionState;

		//for experimental overriding of friction/contact solver func
		private ContactSolverType _contactSolverType;
		private ContactSolverType _frictionSolverType;

		private int _debugBodyId;

		//Bullet 2.20b has experimental damping code to reduce jitter just before objects fall asleep/deactivate
		//doesn't work very well yet (value 0 disabled this damping)
		//note there this influences deactivation thresholds!
		private float _clippedAngvelThresholdSqr = 0.01f;
		private float _clippedLinearThresholdSqr = 0.01f;

		private float _jitterVelocityDampingFactor = 0.7f;

		public RigidBody(float mass, MotionState motionState, CollisionShape collisionShape, Vector3 localInertia, float linearDamping, float angularDamping, float friction, float restitution)
		{
			_optionalMotionState = motionState;
			_angularFactor = 1;
			_angularDamping = 0.5f;

			if (motionState != null)
			{
				motionState.GetWorldTransform(out _worldTransform);
			}
			else
			{
				WorldTransform = Matrix.Identity;
			}

			InterpolationWorldTransform = WorldTransform;
			InterpolationLinearVelocity = new Vector3();
			InterpolationAngularVelocity = new Vector3();

			//moved to btCollisionObject
			Friction = friction;
			Restitution = restitution;

			CollisionShape = collisionShape;
			_debugBodyId = UniqueID++;

			//m_internalOwner is to allow upcasting from collision object to rigid body
			Owner = this;

			SetMassProps(mass, localInertia);
			SetDamping(linearDamping, angularDamping);
			UpdateInertiaTensor();
		}

		public int DebugBodyID { get { return _debugBodyId; } set { _debugBodyId = value; } }

		public ContactSolverType ContactSolverType { get { return _contactSolverType; } set { _contactSolverType = value; } }
		public ContactSolverType FrictionSolverType { get { return _frictionSolverType; } set { _frictionSolverType = value; } }

		public float AngularFactor { get { return _angularFactor; } set { _angularFactor = value; } }

		//is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
		public bool IsInWorld { get { return Broadphase != null; } }

		public Vector3 Gravity
		{
			get { return _gravity; }
			set
			{
				if (_inverseMass != 0.0f)
				{
					_gravity = value * (1.0f / _inverseMass);
				}
			}
		}
		public Matrix InvInertiaTensorWorld { get { return _invInertiaTensorWorld; } }
		public float InverseMass { get { return _inverseMass; } }
		public Vector3 InvInertiaDiagLocal { get { return _invInertiaLocal; } set { _invInertiaLocal = value; } }
		public Vector3 CenterOfMassPosition { get { return WorldTransform.Translation; } }
		public Quaternion Orientation { get { return Quaternion.CreateFromRotationMatrix(WorldTransform); } }
		public Matrix CenterOfMassTransform
		{
			get { return WorldTransform; }
			set
			{
				InterpolationWorldTransform = value;
				InterpolationLinearVelocity = LinearVelocity;
				InterpolationAngularVelocity = AngularVelocity;
				WorldTransform = value;
				UpdateInertiaTensor();
			}
		}

		public Vector3 LinearVelocity
		{
			get { return _linearVelocity; }
			set
			{
				if (CollisionFlags == CollisionOptions.StaticObject)
					throw new BulletException("Static objects can't have linear velocity!");
				_linearVelocity = value;
			}
		}

		public Vector3 AngularVelocity
		{
			get { return _angularVelocity; }
			set
			{
				if (CollisionFlags == CollisionOptions.StaticObject)
					throw new BulletException("Static objects can't have angular velocity!");
				_angularVelocity = value;
			}
		}

		//MotionState allows to automatic synchronize the world transform for active objects
		public MotionState MotionState
		{
			get { return _optionalMotionState; }
			set
			{
				_optionalMotionState = value;
				if (_optionalMotionState != null)
					value.GetWorldTransform(out _worldTransform);
			}
		}

		public static float LinearAirDamping { get { return _linearAirDamping; } set { _linearAirDamping = value; } }
		public static float RigidBodyDeactivationTime { get { return _rigidBodyDeactivationTime; } set { _rigidBodyDeactivationTime = value; } }
		public static bool DisableDeactivation { get { return _disableDeactivation; } set { _disableDeactivation = value; } }
		public static float LinearSleepingThreshold { get { return _linearSleepingThreshold; } set { _linearSleepingThreshold = value; } }
		public static float AngularSleepingThreshold { get { return _angularSleepingThreshold; } set { _angularSleepingThreshold = value; } }
		public static int UniqueID { get { return _uniqueId; } set { _uniqueId = value; } }

		public void ProceedToTransform(Matrix newTrans)
		{
			CenterOfMassTransform = newTrans;
		}

		//to keep collision detection and dynamics separate we don't store a rigidbody pointer
		//but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
		public static RigidBody Upcast(CollisionObject colObj)
		{
			return colObj.Owner as RigidBody;
		}

		// continuous collision detection needs prediction
		public void PredictIntegratedTransform(float step, ref Matrix predictedTransform)
		{
			if ((_angularVelocity.LengthSquared() < _clippedAngvelThresholdSqr) &&
				(_linearVelocity.LengthSquared() < _clippedLinearThresholdSqr))
			{
				_angularVelocity *= _jitterVelocityDampingFactor;
				_linearVelocity *= _jitterVelocityDampingFactor;
			}

			TransformUtil.IntegrateTransform(WorldTransform, _linearVelocity, _angularVelocity, step, ref predictedTransform);
		}

		public void SaveKinematicState(float step)
		{
			//todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
			if (step != 0)
			{
				//if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
				if (MotionState != null)
					MotionState.GetWorldTransform(out _worldTransform);

				TransformUtil.CalculateVelocity(InterpolationWorldTransform, WorldTransform, step, ref _linearVelocity, ref _angularVelocity);
				InterpolationLinearVelocity = _linearVelocity;
				InterpolationAngularVelocity = _angularVelocity;
				InterpolationWorldTransform = WorldTransform;
			}
		}

		public void ApplyForces(float step)
		{
			if (IsStaticOrKinematicObject)
				return;

			ApplyCentralForce(_gravity);

			_linearVelocity *= (1 - step * LinearAirDamping * _linearDamping) < 0.0f ? 0.0f : (1.0f < (1 - step * LinearAirDamping * _linearDamping) ? 1.0f : (1 - step * LinearAirDamping * _linearDamping)); 
			_angularVelocity *= (1 - step * _angularDamping) < 0.0f ? 0.0f : (1.0f < (1 - step * _angularDamping) ? 1.0f : (1 - step * _angularDamping)); 

			float speed = _linearVelocity.Length();
			if (speed < _linearDamping)
			{
				float dampVel = 0.005f;
				if (speed > dampVel)
				{
					Vector3 dir = _linearVelocity;
					dir.Normalize();
					_linearVelocity -= dir * dampVel;
				}
				else
				{
					_linearVelocity = new Vector3();
				}
			}

			float angSpeed = _angularVelocity.Length();
			if (angSpeed < _angularDamping)
			{
				float angDampVel = 0.005f;
				if (angSpeed > angDampVel)
				{
					Vector3 dir = _angularVelocity;
					dir.Normalize();
					_angularVelocity -= dir * angDampVel;
				}
				else
				{
					_angularVelocity = new Vector3();
				}
			}
		}

		public void SetDamping(float linDamping, float angDamping)
		{
			_linearDamping = linDamping < 0.0f ? 0.0f : (1.0f < linDamping ? 1.0f : linDamping); 
			_angularDamping = angDamping < 0.0f ? 0.0f : (1.0f < angDamping ? 1.0f : angDamping); 
		}

		public void SetMassProps(float mass, Vector3 inertia)
		{
			if (mass == 0)
			{
				CollisionFlags |= CollisionOptions.StaticObject;
				_inverseMass = 0;
			}
			else
			{
				CollisionFlags &= (~CollisionOptions.StaticObject);
				_inverseMass = 1.0f / mass;
			}

			_invInertiaLocal = new Vector3(inertia.X != 0.0f ? 1.0f / inertia.X : 0.0f,
								   inertia.Y != 0.0f ? 1.0f / inertia.Y : 0.0f,
								   inertia.Z != 0.0f ? 1.0f / inertia.Z : 0.0f);
		}

		public void IntegrateVelocities(float step)
		{
			if (IsStaticOrKinematicObject)
				return;

			_linearVelocity += _totalForce * (_inverseMass * step);
			_angularVelocity += Vector3.TransformNormal(_totalTorque, _invInertiaTensorWorld) * step;

			float MAX_ANGVEL = Microsoft.Xna.Framework.MathHelper.PiOver2;
			/// clamp angular velocity. collision calculations will fail on higher angular velocities	
			float angvel = _angularVelocity.Length();
			if (angvel * step > MAX_ANGVEL)
			{
				_angularVelocity *= (MAX_ANGVEL / step) / angvel;
			}

			ClearForces();
		}

		public void ApplyCentralForce(Vector3 force)
		{
			_totalForce += force;
		}

		public void ApplyTorque(Vector3 torque)
		{
			_totalTorque += torque;
		}

		public void ApplyForce(Vector3 force, Vector3 rel_pos)
		{
			ApplyCentralForce(force);
			ApplyTorque(Vector3.Cross(rel_pos, force));
		}

		public void ApplyCentralImpulse(Vector3 impulse)
		{
			_linearVelocity += impulse * _inverseMass;
		}

		public void ApplyTorqueImpulse(Vector3 torque)
		{
			_angularVelocity += Vector3.TransformNormal(torque, _invInertiaTensorWorld);
		}

		public void ApplyImpulse(Vector3 impulse, Vector3 rel_pos)
		{
			if (_inverseMass != 0)
			{
				ApplyCentralImpulse(impulse);
				if (_angularFactor != 0)
					ApplyTorqueImpulse(Vector3.Cross(rel_pos, impulse) * _angularFactor);
			}
		}

		public void InternalApplyImpulse(Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude)
		{
			if (_inverseMass != 0)
			{
				_linearVelocity += linearComponent * impulseMagnitude;
				if (_angularFactor != 0)
					_angularVelocity += angularComponent * impulseMagnitude * _angularFactor;
			}
		}

		public void ClearForces()
		{
			_totalForce = new Vector3();
			_totalTorque = new Vector3();
		}

		public void UpdateInertiaTensor()
		{
			Matrix temp = WorldTransform;
			temp.Translation = Vector3.Zero;
			_invInertiaTensorWorld = MatrixOperations.Multiply(MatrixOperations.Scaled(WorldTransform, _invInertiaLocal), Matrix.Transpose(temp));
		}

		public Vector3 GetVelocityInLocalPoint(Vector3 relPos)
		{
			//we also calculate lin/ang velocity for kinematic objects
			return _linearVelocity + Vector3.Cross(_angularVelocity, relPos);

			//for kinematic objects, we could also use use:
			//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
		}

		public void Translate(Vector3 v)
		{
			Matrix m = WorldTransform;
			m.Translation += v;
			WorldTransform = m;
		}

		public void GetAabb(out Vector3 aabbMin, out Vector3 aabbMax)
		{
			CollisionShape.GetAabb(WorldTransform, out aabbMin, out aabbMax);
		}

		public float ComputeImpulseDenominator(Vector3 pos, Vector3 normal)
		{
			Vector3 r0 = pos - CenterOfMassPosition;
			Vector3 c0 = Vector3.Cross(r0, normal);
			Vector3 vec = Vector3.Cross(Vector3.TransformNormal(c0, InvInertiaTensorWorld), r0);

			return _inverseMass + Vector3.Dot(normal, vec);

		}

		public float ComputeAngularImpulseDenominator(Vector3 axis)
		{
			Vector3 vec = Vector3.TransformNormal(axis, InvInertiaTensorWorld);
			return Vector3.Dot(axis, vec);
		}

		public void UpdateDeactivation(float timeStep)
		{
			if ((ActivationState == ActivationState.IslandSleeping) || (ActivationState == ActivationState.DisableDeactivation))
				return;

			if ((LinearVelocity.LengthSquared() < LinearSleepingThreshold * LinearSleepingThreshold) &&
				(AngularVelocity.LengthSquared() < AngularSleepingThreshold * AngularSleepingThreshold))
			{
				DeactivationTime += timeStep;
			}
			else
			{
				DeactivationTime = 0;
				ActivationState = ActivationState.Nothing;
			}

		}

		public bool WantsSleeping()
		{

			if (ActivationState == ActivationState.DisableDeactivation)
				return false;

			//disable deactivation
			if (DisableDeactivation || (RigidBodyDeactivationTime == 0))
				return false;

			if ((ActivationState == ActivationState.IslandSleeping) || (ActivationState == ActivationState.WantsDeactivation))
				return true;

			if (DeactivationTime > RigidBodyDeactivationTime)
			{
				return true;
			}
			return false;
		}
	}
}
