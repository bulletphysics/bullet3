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
	public class SimpleDynamicsWorld : DynamicsWorld
	{
		private IConstraintSolver _constraintSolver;
		private bool _ownsConstraintSolver;
		private Vector3 _gravity;
		private IDebugDraw _debugDrawer;

		/// <summary>
		/// this btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache and constraintSolver
		/// </summary>
		/// <param name="dispatcher"></param>
		/// <param name="pairCache"></param>
		/// <param name="constraintSolver"></param>
		public SimpleDynamicsWorld(IDispatcher dispatcher, OverlappingPairCache pairCache, IConstraintSolver constraintSolver)
			: base(dispatcher, pairCache)
		{
			_constraintSolver = constraintSolver;
			_ownsConstraintSolver = false;
			_gravity = new Vector3(0, 0, -10);
		}

		public override Vector3 Gravity
		{
			set
			{
				_gravity = value;
				for (int i = 0; i < CollisionObjects.Count; i++)
				{
					CollisionObject colObj = CollisionObjects[i];
					RigidBody body = RigidBody.Upcast(colObj);
					if (body != null)
					{
						body.Gravity = value;
					}
				}
			}
		}

		public override IConstraintSolver ConstraintSolver
		{
			set
			{
				_ownsConstraintSolver = false;
				_constraintSolver = value;
			}
		}

		public override IDebugDraw DebugDrawer
		{
			get
			{
				return _debugDrawer;
			}
			set
			{
				_debugDrawer = value;
			}
		}

		public override void StepSimulation(float timeStep, int numSubsteps, float fixedTimeStep)
		{
			//apply gravity, predict motion
			PredictUnconstraintMotion(timeStep);

			DispatcherInfo dispatchInfo = new DispatcherInfo();
			dispatchInfo.TimeStep = timeStep;
			dispatchInfo.StepCount = 0;
			dispatchInfo.DebugDraw = DebugDrawer;
			//perform collision detection
			PerformDiscreteCollisionDetection();

			//solve contact constraints
			int numManifolds = Dispatcher.ManifoldCount;
			if (numManifolds != 0)
			{

				List<PersistentManifold> manifolds = (Dispatcher as CollisionDispatcher).Manifolds;
				//int numManifolds = m_dispatcher1.GetNumManifolds();
				ContactSolverInfo infoGlobal = new ContactSolverInfo();
				infoGlobal.TimeStep = timeStep;

				_constraintSolver.SolveGroup(new List<CollisionObject>(), manifolds, manifolds.Count, new List<TypedConstraint>(), infoGlobal, _debugDrawer);
			}
			//integrate transforms
			IntegrateTransforms(timeStep);

			UpdateAabbs();

			SynchronizeMotionStates();
		}

		public override void UpdateAabbs()
		{
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (body.IsActive && (!body.IsStaticObject))
					{
						Vector3 minAabb, maxAabb;
						colObj.CollisionShape.GetAabb(colObj.WorldTransform, out minAabb, out maxAabb);
						IBroadphase bp = Broadphase;
						bp.SetAabb(body.Broadphase, minAabb, maxAabb);
					}
				}
			}
		}

		public override void AddRigidBody(RigidBody body)
		{
			body.Gravity = _gravity;

			if (body.CollisionShape != null)
			{
				AddCollisionObject(body);
			}
		}

		public override void RemoveRigidBody(RigidBody body)
		{
			RemoveCollisionObject(body);
		}

		public void SynchronizeMotionStates()
		{
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null && body.MotionState != null)
				{
					if (body.ActivationState != ActivationState.IslandSleeping)
					{
						body.MotionState.SetWorldTransform(body.WorldTransform);
					}
				}
			}
		}

		protected void PredictUnconstraintMotion(float timeStep)
		{
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (!body.IsStaticObject)
					{
						if (body.IsActive)
						{
							body.ApplyForces(timeStep);
							body.IntegrateVelocities(timeStep);
							Matrix temp = body.InterpolationWorldTransform;
							body.PredictIntegratedTransform(timeStep, ref temp);
							body.InterpolationWorldTransform = temp;
						}
					}
				}
			}
		}

		protected void IntegrateTransforms(float timeStep)
		{
			Matrix predictedTrans = Matrix.Identity;
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (body.IsActive && (!body.IsStaticObject))
					{
						body.PredictIntegratedTransform(timeStep, ref predictedTrans);
						body.ProceedToTransform(predictedTrans);
					}
				}
			}
		}
	}
}
