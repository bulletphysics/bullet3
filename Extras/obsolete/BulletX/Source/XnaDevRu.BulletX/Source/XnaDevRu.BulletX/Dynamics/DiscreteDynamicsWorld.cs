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
	/// DiscreteDynamicsWorld provides discrete rigid body simulation
	/// those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
	/// </summary>
	public class DiscreteDynamicsWorld : DynamicsWorld
	{
		private static bool _reportMe = true;

		private IConstraintSolver _constraintSolver;
		private SimulationIslandManager _islandManager;
		private List<TypedConstraint> _constraints = new List<TypedConstraint>();
		private IDebugDraw _debugDrawer;
		private ContactSolverInfo _solverInfo = new ContactSolverInfo();
		private Vector3 _gravity;
		//for variable timesteps
		private float _localTime;
		//for variable timesteps
		private bool _ownsIslandManager;
		private bool _ownsConstraintSolver;
		private List<RaycastVehicle> _vehicles = new List<RaycastVehicle>();
		private int _profileTimings;

		public DiscreteDynamicsWorld(IDispatcher dispatcher, OverlappingPairCache pairCache)
			: this(dispatcher, pairCache, null) { }

		//this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
		public DiscreteDynamicsWorld(IDispatcher dispatcher, OverlappingPairCache pairCache, IConstraintSolver constraintSolver)
			: base(dispatcher, pairCache)
		{
			_constraintSolver = constraintSolver != null ? constraintSolver : new SequentialImpulseConstraintSolver();
			_debugDrawer = null;
			_gravity = new Vector3(0, -10, 0);
			_localTime = 1f / 60f;
			_profileTimings = 0;
			_islandManager = new SimulationIslandManager();
			_ownsIslandManager = true;
			_ownsConstraintSolver = constraintSolver == null;
		}

		public ContactSolverInfo SolverInfo { get { return _solverInfo; } }
		public SimulationIslandManager SimulationIslandManager { get { return _islandManager; } }
		public CollisionWorld CollisionWorld { get { return this; } }

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

		public override int ConstraintsCount
		{
			get
			{
				return _constraints.Count;
			}
		}

		//if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
		public override void StepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
		{
			int numSimulationSubSteps = 0;

			if (maxSubSteps != 0)
			{
				//fixed timestep with interpolation
				_localTime += timeStep;
				if (_localTime >= fixedTimeStep)
				{
					numSimulationSubSteps = (int)(_localTime / fixedTimeStep);
					_localTime -= numSimulationSubSteps * fixedTimeStep;
				}
			}
			else
			{
				//variable timestep
				fixedTimeStep = timeStep;
				_localTime = timeStep;
				if (Math.Abs(timeStep) < float.Epsilon)
				{
					numSimulationSubSteps = 0;
					maxSubSteps = 0;
				}
				else
				{
					numSimulationSubSteps = 1;
					maxSubSteps = 1;
				}
			}

			//process some debugging flags
			if (DebugDrawer != null)
			{
				RigidBody.DisableDeactivation = (DebugDrawer.DebugMode & DebugDrawModes.NoDeactivation) != 0;
			}
			if (numSimulationSubSteps != 0)
			{

				SaveKinematicState(fixedTimeStep);

				//clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
				int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps) ? maxSubSteps : numSimulationSubSteps;

				for (int i = 0; i < clampedSimulationSteps; i++)
				{
					InternalSingleStepSimulation(fixedTimeStep);
					SynchronizeMotionStates();
				}

			}

			SynchronizeMotionStates();

			//return numSimulationSubSteps;
		}

		public void StepSimulation(float timeStep, int maxSubSteps)
		{
			StepSimulation(timeStep, maxSubSteps, 1f / 60f);
		}

		public override void UpdateAabbs()
		{
			Vector3 colorvec = new Vector3(1, 0, 0);
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);

				if (body != null)
				{
					//	if (body->IsActive() && (!body->IsStatic()))
					{
						Vector3 minAabb, maxAabb;
						colObj.CollisionShape.GetAabb(colObj.WorldTransform, out minAabb, out maxAabb);
						OverlappingPairCache bp = BroadphasePairCache;

						//moving objects should be moderately sized, probably something wrong if not
						if (colObj.IsStaticObject || ((maxAabb - minAabb).LengthSquared() < 1e12f))
						{
							bp.SetAabb(body.Broadphase, minAabb, maxAabb);
						}
						else
						{
							//something went wrong, investigate
							//this assert is unwanted in 3D modelers (danger of loosing work)
							BulletDebug.Assert(false);
							body.ActivationState = ActivationState.DisableSimulation;

							if (_reportMe)
							{
								_reportMe = false;
								Console.WriteLine("Overflow in AABB, object removed from simulation \n");
								Console.WriteLine("If you can reproduce this, please email bugs@continuousphysics.com\n");
								Console.WriteLine("Please include above information, your Platform, version of OS.\n");
								Console.WriteLine("Thanks.\n");
							}
						}
						if (_debugDrawer != null && (_debugDrawer.DebugMode & DebugDrawModes.DrawAabb) != 0)
							DrawAabb(_debugDrawer, minAabb, maxAabb, colorvec);
					}
				}
			}
		}

		public override void AddConstraint(TypedConstraint constraint)
		{
			_constraints.Add(constraint);
		}

		public override void RemoveConstraint(TypedConstraint constraint)
		{
			_constraints.Remove(constraint);
		}

		public void AddVehicle(RaycastVehicle vehicle)
		{
			_vehicles.Add(vehicle);
		}

		public void RemoveVehicle(RaycastVehicle vehicle)
		{
			_vehicles.Remove(vehicle);
		}

		public override void AddRigidBody(RigidBody body)
		{
			if (!body.IsStaticOrKinematicObject)
			{
				body.Gravity = _gravity;
			}

			if (body.CollisionShape != null)
			{
				bool isDynamic = !(body.IsStaticObject || body.IsKinematicObject);
				BroadphaseProxy.CollisionFilterGroups collisionFilterGroup = isDynamic ? BroadphaseProxy.CollisionFilterGroups.Default : BroadphaseProxy.CollisionFilterGroups.Static;
				BroadphaseProxy.CollisionFilterGroups collisionFilterMask = isDynamic ? BroadphaseProxy.CollisionFilterGroups.All : (BroadphaseProxy.CollisionFilterGroups.All ^ BroadphaseProxy.CollisionFilterGroups.Static);

				AddCollisionObject(body, collisionFilterGroup, collisionFilterMask);
			}
		}

		public override void RemoveRigidBody(RigidBody body)
		{
			RemoveCollisionObject(body);
		}

		public void DebugDrawObject(Matrix worldTransform, CollisionShape shape, Vector3 color)
		{
			if (shape.ShapeType == BroadphaseNativeTypes.Compound)
			{
				CompoundShape compoundShape = shape as CompoundShape;
				for (int i = compoundShape.ChildShapeCount - 1; i >= 0; i--)
				{
					Matrix childTrans = compoundShape.GetChildTransform(i);
					CollisionShape colShape = compoundShape.GetChildShape(i);
					DebugDrawObject(worldTransform * childTrans, colShape, color);
				}

			}
			else
			{
				switch (shape.ShapeType)
				{

					case BroadphaseNativeTypes.Sphere:
						{
							SphereShape sphereShape = shape as SphereShape;
							float radius = sphereShape.Margin;//radius doesn't include the margin, so draw with margin
							Vector3 start = worldTransform.Translation;
							DebugDrawer.DrawLine(start, start + Vector3.TransformNormal(new Vector3(radius, 0, 0), worldTransform), color);
							DebugDrawer.DrawLine(start, start + Vector3.TransformNormal(new Vector3(0, radius, 0), worldTransform), color);
							DebugDrawer.DrawLine(start, start + Vector3.TransformNormal(new Vector3(0, 0, radius), worldTransform), color);
							//drawSphere					
							break;
						}
					case BroadphaseNativeTypes.MultiSphere:
					case BroadphaseNativeTypes.Cone:
						{
							ConeShape coneShape = shape as ConeShape;
							float radius = coneShape.Radius;//+coneShape->getMargin();
							float height = coneShape.Height;//+coneShape->getMargin();
							Vector3 start = worldTransform.Translation;
							DebugDrawer.DrawLine(start + Vector3.TransformNormal(new Vector3(0f, 0f, 0.5f * height), worldTransform), start + Vector3.TransformNormal(new Vector3(radius, 0f, -0.5f * height), worldTransform), color);
							DebugDrawer.DrawLine(start + Vector3.TransformNormal(new Vector3(0f, 0f, 0.5f * height), worldTransform), start + Vector3.TransformNormal(new Vector3(-radius, 0f, -0.5f * height), worldTransform), color);
							DebugDrawer.DrawLine(start + Vector3.TransformNormal(new Vector3(0f, 0f, 0.5f * height), worldTransform), start + Vector3.TransformNormal(new Vector3(0f, radius, -0.5f * height), worldTransform), color);
							DebugDrawer.DrawLine(start + Vector3.TransformNormal(new Vector3(0f, 0f, 0.5f * height), worldTransform), start + Vector3.TransformNormal(new Vector3(0f, -radius, -0.5f * height), worldTransform), color);
							break;
						}
					case BroadphaseNativeTypes.Cylinder:
						{
							CylinderShape cylinder = shape as CylinderShape;
							int upAxis = cylinder.UpAxis;
							float radius = cylinder.Radius;
							float halfHeight = MathHelper.GetElement(cylinder.HalfExtents, upAxis);
							Vector3 start = worldTransform.Translation;
							Vector3 offsetHeight = new Vector3();
							MathHelper.SetElement(ref offsetHeight, upAxis, halfHeight);
							Vector3 offsetRadius = new Vector3();
							MathHelper.SetElement(ref offsetRadius, (upAxis + 1) % 3,  radius);
							DebugDrawer.DrawLine(start + Vector3.TransformNormal(offsetHeight + offsetRadius, worldTransform), start + Vector3.TransformNormal(-offsetHeight + offsetRadius, worldTransform), color);
							DebugDrawer.DrawLine(start + Vector3.TransformNormal(offsetHeight - offsetRadius, worldTransform), start + Vector3.TransformNormal(-offsetHeight - offsetRadius, worldTransform), color);
							break;
						} 
					default:
						{
							if (shape.ShapeType == BroadphaseNativeTypes.TriangleMesh)
							{
								TriangleMeshShape concaveMesh = shape as TriangleMeshShape;
								//btVector3 aabbMax(1e30f,1e30f,1e30f);
								//btVector3 aabbMax(100,100,100);//1e30f,1e30f,1e30f);

								//todo pass camera, for some culling
								Vector3 aabbMax = new Vector3(1e30f, 1e30f, 1e30f);
								Vector3 aabbMin = new Vector3(-1e30f, -1e30f, -1e30f);

								DebugDrawCallback drawCallback = new DebugDrawCallback(DebugDrawer, worldTransform, color);
								concaveMesh.ProcessAllTriangles(drawCallback, aabbMin, aabbMax);
							}

							if (shape.ShapeType == BroadphaseNativeTypes.ConvexTriangleMesh)
							{
								ConvexTriangleMeshShape convexMesh = shape as ConvexTriangleMeshShape;
								//todo: pass camera for some culling			
								Vector3 aabbMax = new Vector3(1e30f, 1e30f, 1e30f);
								Vector3 aabbMin = new Vector3(-1e30f, -1e30f, -1e30f);
								//DebugDrawcallback drawCallback;
								DebugDrawCallback drawCallback = new DebugDrawCallback(DebugDrawer, worldTransform, color);
								convexMesh.getStridingMesh().InternalProcessAllTriangles(drawCallback, aabbMin, aabbMax);
							}

							// for polyhedral shapes
							if (shape.IsPolyhedral)
							{
								PolyhedralConvexShape polyshape = shape as PolyhedralConvexShape;

								for (int i = 0; i < polyshape.EdgeCount; i++)
								{
									Vector3 a, b;
									polyshape.GetEdge(i, out a, out b);
									a = Vector3.TransformNormal(a, worldTransform);
									b = Vector3.TransformNormal(b, worldTransform);
									DebugDrawer.DrawLine(a, b, color);
								}
							}
							break;
						}
				}
			}
		}

		public override TypedConstraint GetConstraint(int index)
		{
			return _constraints[index];
		}

		public static void DrawAabb(IDebugDraw debugDrawer, Vector3 from, Vector3 to, Vector3 color)
		{
			Vector3 halfExtents = (to - from) * 0.5f;
			Vector3 center = (to + from) * 0.5f;

			Vector3 edgecoord = new Vector3(1f, 1f, 1f), pa, pb;
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					pa = new Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
						edgecoord.Z * halfExtents.Z);
					pa += center;

					int othercoord = j % 3;
					MathHelper.SetElement(ref edgecoord, othercoord, MathHelper.GetElement(edgecoord, othercoord) * -1f);
					pb = new Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
						edgecoord.Z * halfExtents.Z);
					pb += center;

					debugDrawer.DrawLine(pa, pb, color);
				}
				edgecoord = new Vector3(-1f, -1f, -1f);
				if (i < 3)
					MathHelper.SetElement(ref edgecoord, i, MathHelper.GetElement(edgecoord, i) * -1f);
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
					if (!body.IsStaticOrKinematicObject)
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
			Matrix predictedTrans = new Matrix();
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (body.IsActive && (!body.IsStaticOrKinematicObject))
					{
						body.PredictIntegratedTransform(timeStep, ref predictedTrans);
						body.ProceedToTransform(predictedTrans);
					}
				}
			}
		}

		protected void CalculateSimulationIslands()
		{
			SimulationIslandManager.UpdateActivationState(this, Dispatcher);

			for (int i = 0; i < _constraints.Count; i++)
			{
				TypedConstraint constraint = _constraints[i];

				RigidBody colObj0 = constraint.RigidBodyA;
				RigidBody colObj1 = constraint.RigidBodyB;

				if (((colObj0 != null) && (colObj0.MergesSimulationIslands)) &&
					((colObj1 != null) && (colObj1.MergesSimulationIslands)))
				{
					if (colObj0.IsActive || colObj1.IsActive)
					{

						SimulationIslandManager.UnionFind.Unite((colObj0).IslandTag,
							(colObj1).IslandTag);
					}
				}
			}

			//Store the island id in each body
			SimulationIslandManager.StoreIslandActivationState(this);
		}

		//protected void SolveNonContactConstraints(ContactSolverInfo solverInfo)
		//{
		//    //constraint preparation: building jacobians
		//    for (int i = 0; i < _constraints.Count; i++)
		//    {
		//        TypedConstraint constraint = _constraints[i];
		//        constraint.BuildJacobian();
		//    }

		//    //solve the regular non-contact constraints (point 2 point, hinge, generic d6)
		//    for (int g = 0; g < solverInfo.IterationsCount; g++)
		//    {
		//        for (int i = 0; i < _constraints.Count; i++)
		//        {
		//            TypedConstraint constraint = _constraints[i];
		//            constraint.SolveConstraint(solverInfo.TimeStep);
		//        }
		//    }
		//}

		//protected void SolveContactConstraints(ContactSolverInfo solverInfo)
		//{
		//    InplaceSolverIslandCallback solverCallback = new InplaceSolverIslandCallback(solverInfo, _constraintSolver, _debugDrawer);

		//    // solve all the contact points and contact friction
		//    _islandManager.BuildAndProcessIslands(Dispatcher, CollisionObjects, solverCallback);
		//}

		protected void SolveConstraints(ContactSolverInfo solverInfo)
		{
			//sorted version of all btTypedConstraint, based on islandId
			List<TypedConstraint> sortedConstraints = new List<TypedConstraint>(ConstraintsCount);

			for (int i = 0; i < ConstraintsCount; i++)
			{
				sortedConstraints.Add(_constraints[i]);
			}

			sortedConstraints.Sort(new Comparison<TypedConstraint>(TypedConstraint.SortConstraintOnIslandPredicate));
			List<TypedConstraint> constraintsPtr = ConstraintsCount != 0 ? sortedConstraints : new List<TypedConstraint>();

			InplaceSolverIslandCallback solverCallback = new InplaceSolverIslandCallback(solverInfo, _constraintSolver, constraintsPtr,  _debugDrawer);

			// solve all the constraints for this island
			_islandManager.BuildAndProcessIslands(CollisionWorld.Dispatcher, CollisionWorld.CollisionObjects, solverCallback);
		}

		protected void UpdateActivationState(float timeStep)
		{
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					body.UpdateDeactivation(timeStep);

					if (body.WantsSleeping())
					{
						if (body.IsStaticOrKinematicObject)
						{
							body.ActivationState = ActivationState.IslandSleeping;
						}
						else
						{
							if (body.ActivationState == ActivationState.Active)
								body.ActivationState = ActivationState.WantsDeactivation;
						}
					}
					else
					{
						if (body.ActivationState != ActivationState.DisableDeactivation)
							body.ActivationState = ActivationState.Active;
					}
				}
			}
		}

		protected void UpdateVehicles(float timeStep)
		{
			for (int i = 0; i < _vehicles.Count; i++)
			{
				RaycastVehicle vehicle = _vehicles[i];
				vehicle.updateVehicle(timeStep);
			}
		}

		protected void StartProfiling(float timeStep) { }

		protected virtual void InternalSingleStepSimulation(float timeStep)
		{
			StartProfiling(timeStep);

			//update aabbs information
			UpdateAabbs();

			//apply gravity, predict motion
			PredictUnconstraintMotion(timeStep);

			DispatcherInfo dispatchInfo = DispatchInfo;
			dispatchInfo.TimeStep = timeStep;
			dispatchInfo.StepCount = 0;
			dispatchInfo.DebugDraw = DebugDrawer;

			//perform collision detection
			PerformDiscreteCollisionDetection();

			CalculateSimulationIslands();

			SolverInfo.TimeStep = timeStep;

			//solve contact and other joint constraints
			SolveConstraints(SolverInfo);

			//CallbackTriggers();

			//integrate transforms
			IntegrateTransforms(timeStep);

			//update vehicle simulation
			UpdateVehicles(timeStep);

			UpdateActivationState(timeStep);
		}

		protected void SynchronizeMotionStates()
		{
			//debug vehicle wheels
			{
				//todo: iterate over awake simulation islands!
				for (int i = 0; i < CollisionObjects.Count; i++)
				{
					CollisionObject colObj = CollisionObjects[i];
					if (DebugDrawer != null && (DebugDrawer.DebugMode & DebugDrawModes.DrawWireframe) != 0)
					{
						Vector3 color = new Vector3(255f, 255f, 255f);
						switch (colObj.ActivationState)
						{
							case ActivationState.Active:
								color = new Vector3(255f, 255f, 255f); break;
							case ActivationState.IslandSleeping:
								color = new Vector3(0f, 255f, 0f); break;
							case ActivationState.WantsDeactivation:
								color = new Vector3(0f, 255f, 255f); break;
							case ActivationState.DisableDeactivation:
								color = new Vector3(255f, 0f, 0f); break;
							case ActivationState.DisableSimulation:
								color = new Vector3(255f, 255f, 0f); break;
							default:
								color = new Vector3(255f, 0f, 0f); break;
						}

						DebugDrawObject(colObj.WorldTransform, colObj.CollisionShape, color);
					}
					RigidBody body = RigidBody.Upcast(colObj);
					if (body != null && body.MotionState != null && !body.IsStaticOrKinematicObject)
					{
						//if (body.ActivationState != ActivationState.IslandSleeping)
						{
							Matrix interpolatedTransform = new Matrix();
							TransformUtil.IntegrateTransform(body.InterpolationWorldTransform,
								body.InterpolationLinearVelocity, body.InterpolationAngularVelocity, _localTime, ref interpolatedTransform);
							body.MotionState.SetWorldTransform(interpolatedTransform);
						}
					}
				}
			}

			if (DebugDrawer != null && (DebugDrawer.DebugMode & DebugDrawModes.DrawWireframe) != 0)
			{
				for (int i = 0; i < _vehicles.Count; i++)
				{
					for (int v = 0; v < _vehicles[i].getNumWheels(); v++)
					{
						Vector3 wheelColor = new Vector3(0, 255, 255);
						if (_vehicles[i].getWheelInfo(v).RaycastInfo.IsInContact)
						{
							wheelColor = new Vector3(0, 0, 255);
						}
						else
						{
							wheelColor = new Vector3(255, 0, 255);
						}

						//synchronize the wheels with the (interpolated) chassis worldtransform
						_vehicles[i].updateWheelTransform(v, true);

						Vector3 wheelPosWS = _vehicles[i].getWheelInfo(v).WorldTransform.Translation;

						Vector3 axle = new Vector3(
							MathHelper.GetElement(_vehicles[i].getWheelInfo(v).WorldTransform, 0, _vehicles[i].getRightAxis()),
							MathHelper.GetElement(_vehicles[i].getWheelInfo(v).WorldTransform, 1, _vehicles[i].getRightAxis()),
							MathHelper.GetElement(_vehicles[i].getWheelInfo(v).WorldTransform, 2, _vehicles[i].getRightAxis()));


						//m_vehicles[i]->getWheelInfo(v).m_raycastInfo.m_wheelAxleWS
						//debug wheels (cylinders)
						_debugDrawer.DrawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
						_debugDrawer.DrawLine(wheelPosWS, _vehicles[i].getWheelInfo(v).RaycastInfo.ContactPointWS, wheelColor);
					}
				}
			}
		}

		protected void SaveKinematicState(float timeStep)
		{
			for (int i = 0; i < CollisionObjects.Count; i++)
			{
				CollisionObject colObj = CollisionObjects[i];
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (body.ActivationState != ActivationState.IslandSleeping)
					{
						if (body.IsKinematicObject)
						{
							//to calculate velocities next frame
							body.SaveKinematicState(timeStep);
						}
					}
				}
			}
		}

		internal class InplaceSolverIslandCallback : SimulationIslandManager.IIslandCallback
		{
			private ContactSolverInfo _solverInfo;
			private IConstraintSolver _solver;
			private IDebugDraw _debugDrawer;
			private List<TypedConstraint> _sortedConstraints;

			public InplaceSolverIslandCallback(
				ContactSolverInfo solverInfo,
				IConstraintSolver solver,
				List<TypedConstraint> sortedConstraints,
				IDebugDraw debugDrawer)
			{
				_solverInfo = solverInfo;
				_solver = solver;
				_sortedConstraints = sortedConstraints;
				_debugDrawer = debugDrawer;
			}

			public ContactSolverInfo SolverInfo { get { return _solverInfo; } set { _solverInfo = value; } }
			public IConstraintSolver Solver { get { return _solver; } set { _solver = value; } }
			public List<TypedConstraint> Constraints { get { return _sortedConstraints; } set { _sortedConstraints = value; } }
			public IDebugDraw DebugDrawer { get { return _debugDrawer; } set { _debugDrawer = value; } }

			public void ProcessIsland(List<CollisionObject> bodies, List<PersistentManifold> manifolds, int numManifolds, int islandID)
			{
				//also add all non-contact constraints/joints for this island
				List<TypedConstraint> startConstraint = new List<TypedConstraint>();
				int numCurConstraints = 0;
				int startIndex = 0;
				int i;

				//find the first constraint for this island
				for (i = 0; i < _sortedConstraints.Count; i++)
				{
					if (TypedConstraint.GetConstraintIslandId(_sortedConstraints[i]) == islandID)
					{
						//startConstraint = &m_sortedConstraints[i];
						startIndex = i;
						break;
					}
				}
				//count the number of constraints in this island
				for (; i < _sortedConstraints.Count; i++)
				{
					if (TypedConstraint.GetConstraintIslandId(_sortedConstraints[i]) == islandID)
					{
						numCurConstraints++;
					}
				}

				for (i = startIndex; i < startIndex + numCurConstraints; i++)
				{
					startConstraint.Add(_sortedConstraints[i]);
				}

				_solver.SolveGroup(bodies, manifolds, numManifolds, startConstraint, _solverInfo, _debugDrawer);
			}
		}
	}

	internal class DebugDrawCallback : ITriangleIndexCallback, ITriangleCallback
	{
		private IDebugDraw _debugDrawer;
		private Vector3 _color;
		private Matrix _worldTrans;

		public DebugDrawCallback(IDebugDraw debugDrawer, Matrix worldTrans, Vector3 color)
		{
			_debugDrawer = debugDrawer;
			_worldTrans = worldTrans;
			_color = color;
		}

		public void ProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
		{
			ProcessTriangle(triangle, partId, triangleIndex);
		}

		#region ITriangleCallback Members

		public void ProcessTriangle(Vector3[] triangle, int partID, int triangleIndex)
		{
			Vector3 wv0, wv1, wv2;
			wv0 = Vector3.TransformNormal(triangle[0], _worldTrans);
			wv1 = Vector3.TransformNormal(triangle[1], _worldTrans);
			wv2 = Vector3.TransformNormal(triangle[2], _worldTrans);
			_debugDrawer.DrawLine(wv0, wv1, _color);
			_debugDrawer.DrawLine(wv1, wv2, _color);
			_debugDrawer.DrawLine(wv2, wv0, _color);
		}

		#endregion
	}
}