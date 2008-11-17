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

#region Using Statements
using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Storage;
#endregion
using XnaDevRu.Demos;
using XnaDevRu.BulletX;
using XnaDevRu.BulletX.Dynamics;

namespace XnaDevRu.BulletX.Demos
{
	/// <summary>
	/// This is the main type for your BulletX Demo.
	/// </summary>
	public class DemoGame : Microsoft.Xna.Framework.Game
	{
		// Main elements
		private GraphicsDeviceManager _graphics;
		private ContentManager _content;

		// Components
		private Camera _camera;
		private Framerate _fps;

		private MouseState _prevMouseState;

		// Picking
		private TypedConstraint _pickConstraint;
		private Vector3 _oldPickingPos;
		private float _oldPickingDist = 0f;
		private RigidBody _pickedBody = null; //for deactivation state

		//Physics
		private CollisionDispatcher _collisionDispatcher;
		private OverlappingPairCache _broadphase;
		private SequentialImpulseConstraintSolver _solver;
		private DiscreteDynamicsWorld _world;
		private CollisionShape[] _shapePtr;

		private const int _maxNumObjects = 32760;
		private int _numObjects = 120;
		private const int _cubeHalfExtent = 1;
		private const int _extraHeight = -20;
		private const float _collisionMargin = 0.05f;
		private float _shootBoxInitialSpeed = 50f;
		private int[] _shapeIndex;
		private Model _modelBox;
		private Model _modelSphere;
		private Model _modelCylinder;

		private bool _useDebugDrawer;
		private bool _useSweepAndPrune = true;

		/// <summary>
		/// Initializes new Demo testbed.
		/// </summary>
		public DemoGame()
		{
			_graphics = new GraphicsDeviceManager(this);
			_content = new ContentManager(Services);
			//graphics.SynchronizeWithVerticalRetrace = false;

			IsMouseVisible = true;
			_camera = new Camera(this);
			_fps = new Framerate(this);
			_fps.UpdateFrequency = 100;
			Components.Add(_camera);
			Components.Add(_fps);

			ResetPhysics();
		}

		/// <summary>
		/// Gets graphics device manager associated with this demo.
		/// </summary>
		public GraphicsDeviceManager Graphics { get { return _graphics; } }
		/// <summary>
		/// Gets content manager associated with this demo.
		/// </summary>
		public ContentManager Content { get { return _content; } }

		/// <summary>
		/// Gets camera component associated with this demo.
		/// </summary>
		public Camera Camera { get { return _camera; } }
		/// <summary>
		/// Gets framerate component associated with this demo.
		/// </summary>
		public Framerate Fps { get { return _fps; } }

		/// <summary>
		/// Gets or sets current collision dispatcher.
		/// </summary>
		public CollisionDispatcher CollisionDispatcher { get { return _collisionDispatcher; } set { _collisionDispatcher = value; } }
		/// <summary>
		/// Gets or sets current overlapping pair cache.
		/// </summary>
		public OverlappingPairCache Broadphase { get { return _broadphase; } set { _broadphase = value; } }
		/// <summary>
		/// Gets or sets current sequential impulse constraint solver.
		/// </summary>
		public SequentialImpulseConstraintSolver Solver { get { return _solver; } set { _solver = value; } }
		/// <summary>
		/// Gets or sets current dynamics world.
		/// </summary>
		public DiscreteDynamicsWorld PhysicsWorld { get { return _world; } set { _world = value; } }
		/// <summary>
		/// Gets or sets predefined shape array.
		/// </summary>
		public CollisionShape[] Shapes { get { return _shapePtr; } set { _shapePtr = value; } }

		/// <summary>
		/// Gets max number of objects in the simulation.
		/// </summary>
		public int MaxNumObjects { get { return _maxNumObjects; } }
		/// <summary>
		/// Gets or sets starting objects count in simulation.
		/// </summary>
		public int NumObjects { get { return _numObjects; } set { _numObjects = value; } }
		/// <summary>
		/// Gets half extents.
		/// </summary>
		public int HalfExtents { get { return _cubeHalfExtent; } }
		/// <summary>
		/// Gets extra height.
		/// </summary>
		public int ExtraHeight { get { return _extraHeight; } }
		/// <summary>
		/// Gets collision margin.
		/// </summary>
		public float CollisionMargin { get { return _collisionMargin; } }
		/// <summary>
		/// Gets or sets initial box speed when shot.
		/// </summary>
		public float ShootBoxInitialSpeed { get { return _shootBoxInitialSpeed; } set { _shootBoxInitialSpeed = value; } }
		/// <summary>
		/// Gets or sets predefined shape index reference for simulated objects.
		/// </summary>
		public int[] ShapeIndex { get { return _shapeIndex; } set { _shapeIndex = value; } }
		/// <summary>
		/// Gets or sets box model.
		/// </summary>
		public Model ModelBox { get { return _modelBox; } set { _modelBox = value; } }
		/// <summary>
		/// Gets or sets sphere model.
		/// </summary>
		public Model ModelSphere { get { return _modelSphere; } set { _modelSphere = value; } }
		/// <summary>
		/// Gets or sets cylinder model.
		/// </summary>
		public Model ModelCylinder { get { return _modelCylinder; } set { _modelCylinder = value; } }

		/// <summary>
		/// Gets or sets the ability to draw debug information.
		/// </summary>
		public bool UseDebugDrawer { get { return _useDebugDrawer; } set { _useDebugDrawer = value; } }
		/// <summary>
		/// Gets or sets the sweep and prune for broadphase.
		/// </summary>
		public bool UseSweepAndPrune { get { return _useSweepAndPrune; } set { _useSweepAndPrune = value; } }

		/// <summary>
		/// Reinitializes physics.
		/// </summary>
		public void ResetPhysics()
		{
			_collisionDispatcher = new CollisionDispatcher();

			if (_useSweepAndPrune)
			{
				Vector3 worldAabbMin = new Vector3(-10000, -10000, -10000);
				Vector3 worldAabbMax = new Vector3(10000, 10000, 10000);
				const int maxProxies = 32766;
				_broadphase = new AxisSweep3(worldAabbMin, worldAabbMax, maxProxies);
			}
			else
				_broadphase = new SimpleBroadphase();

			_solver = new SequentialImpulseConstraintSolver();
			_world = new DiscreteDynamicsWorld(_collisionDispatcher, _broadphase, _solver);

			//world.setConstraintSolver(solver);
			_world.Gravity = new Vector3(0, -10, 0);

			_shapePtr = new CollisionShape[4];
			_shapePtr[0] = new BoxShape(new Vector3(50, 10, 50));
			_shapePtr[1] = new CylinderShape(new Vector3(_cubeHalfExtent - _collisionMargin, _cubeHalfExtent - _collisionMargin, _cubeHalfExtent - _collisionMargin));
			_shapePtr[2] = new SphereShape(_cubeHalfExtent);
			_shapePtr[3] = new BoxShape(new Vector3(_cubeHalfExtent, _cubeHalfExtent, _cubeHalfExtent));

			_shapeIndex = new int[_maxNumObjects];
			Matrix tr = Matrix.Identity;

			for (int i = 0; i < _numObjects; i++)
			{
				if (i > 0)
					// set shape
					_shapeIndex[i] = 1;
				else
					_shapeIndex[i] = 0;
			}

			GC.Collect();
		}

		/// <summary>
		/// Creates new rigid body and adds it to the world.
		/// </summary>
		/// <param name="mass">Body mass, if 0 body is static.</param>
		/// <param name="startTransform">Starting body transform.</param>
		/// <param name="shape">Body shape.</param>
		/// <returns>Created rigid body.</returns>
		public RigidBody CreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
		{
			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.0f);

			Vector3 localInertia = new Vector3();
			if (isDynamic)
				shape.CalculateLocalInertia(mass, out localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

			DefaultMotionState myMotionState = new DefaultMotionState(startTransform, Matrix.Identity);
			RigidBody body = new RigidBody(mass, myMotionState, shape, localInertia, 0.0f, 0.0f, 0.5f, 0.0f);

			_world.AddRigidBody(body);

			return body;
		}

		/// <summary>
		/// Called after the Game and Graphics.GraphicsDevice are created, but before Game.LoadGraphicsContent.
		/// </summary>
		protected override void Initialize()
		{
			base.Initialize();

			_camera.Position = new Vector3(0, 3, 75);
			_camera.Target = new Vector3(0, 0, 0);

			if (_useDebugDrawer)
			{
				XnaDebugDraw dbg = new XnaDebugDraw(_graphics.GraphicsDevice);
				dbg.DebugMode = DebugDrawModes.DrawAabb | DebugDrawModes.DrawContactPoints;
				_world.DebugDrawer = dbg;
			}
		}

		/// <summary>
		/// Loads graphics content needed by this demo.
		/// </summary>
		/// <param name="loadAllContent">If need to reload all content.</param>
		protected override void LoadGraphicsContent(bool loadAllContent)
		{
			if (loadAllContent)
			{
				_modelBox = _content.Load<Model>("content\\box");
				_modelSphere = _content.Load<Model>("content\\sphere");
				_modelCylinder = _content.Load<Model>("content\\cylinder");
			}
		}

		/// <summary>
		/// Unloads graphics content needed by this demo.
		/// </summary>
		/// <param name="unloadAllContent">If need to unload all content.</param>
		protected override void UnloadGraphicsContent(bool unloadAllContent)
		{
			if (unloadAllContent == true)
			{
				_content.Unload();
			}
		}

		/// <summary>
		/// Called when the game has determined that game logic needs to be processed.
		/// </summary>
		/// <param name="gameTime">Time passed since the last call to this function.</param>
		protected override void Update(GameTime gameTime)
		{
			MouseState mouseState = Mouse.GetState();
			Vector3 rayTo = getRayTo(mouseState.X, mouseState.Y);

			if (mouseState.LeftButton == ButtonState.Pressed && _prevMouseState.LeftButton == ButtonState.Released)
			{
				shootBox(rayTo);
			}
			if (mouseState.MiddleButton == ButtonState.Pressed && _prevMouseState.MiddleButton == ButtonState.Released)
			{
				if (_world != null)
				{
					CollisionWorld.ClosestRayResultCallback rayCallback = new CollisionWorld.ClosestRayResultCallback(_camera.Position, rayTo);
					_world.RayTest(_camera.Position, rayTo, rayCallback);
					if (rayCallback.HasHit)
					{
						RigidBody body = RigidBody.Upcast(rayCallback.CollisionObject);
						if (body != null)
						{
							//other exclusions?
							if (!(body.IsStaticObject || body.IsKinematicObject))
							{
								_pickedBody = body;
								_pickedBody.ActivationState = ActivationState.DisableDeactivation;

								Vector3 pickPos = rayCallback.HitPointWorld;

								Vector3 localPivot = Vector3.Transform(pickPos, XnaDevRu.BulletX.MathHelper.InvertMatrix(body.CenterOfMassTransform));

								Point2PointConstraint p2p = new Point2PointConstraint(body, localPivot);
								_world.AddConstraint(p2p);
								_pickConstraint = p2p;

								//save mouse position for dragging
								_oldPickingPos = rayTo;

								Vector3 eyePos = new Vector3(_camera.Position.X, _camera.Position.Y, _camera.Position.Z);

								_oldPickingDist = (eyePos - pickPos).Length();

								//very weak constraint for picking
								p2p.Settings.Tau = 1.1f;
							}
						}
					}
				}
			}
			else if (mouseState.MiddleButton == ButtonState.Released && _prevMouseState.MiddleButton == ButtonState.Pressed)
			{
				if (_pickConstraint != null && _world != null)
				{
					_world.RemoveConstraint(_pickConstraint);
					_pickConstraint = null;
					_pickedBody.ForceActivationState(ActivationState.Active);
					_pickedBody.DeactivationTime = 0f;
					_pickedBody = null;
				}
			}

			if (_pickConstraint != null)
			{
				//move the constraint pivot
				Point2PointConstraint p2p = _pickConstraint as Point2PointConstraint;
				if (p2p != null)
				{
					//keep it at the same picking distance
					Vector3 dir = rayTo - _camera.Position;
					dir.Normalize();
					dir *= _oldPickingDist;

					Vector3 newPos = _camera.Position + dir;
					p2p.PivotInB = newPos;
				}
			}

			_prevMouseState = mouseState;

			if (Keyboard.GetState().IsKeyDown(Keys.Space))
			{
				//world.stepSimulation(1.0f/60.0f,0);
				int numObjects = _world.CollisionObjectsCount;

				for (int i = 0; i < numObjects; i++)
				{
					CollisionObject colObj = _world.CollisionObjects[i];
					RigidBody body = RigidBody.Upcast(colObj);
					if (body != null)
					{
						if (body.MotionState != null)
						{
							DefaultMotionState myMotionState = (DefaultMotionState)body.MotionState;
							myMotionState.GraphicsWorldTransform = myMotionState.StartWorldTransform;
							colObj.WorldTransform = myMotionState.GraphicsWorldTransform;
							colObj.InterpolationWorldTransform = myMotionState.StartWorldTransform;
							colObj.Activate();
						}

						//removed cached contact points
						_world.Broadphase.CleanProxyFromPairs(colObj.Broadphase);

						if (body != null && !body.IsStaticObject)
						{
							RigidBody.Upcast(colObj).LinearVelocity = new Vector3(0, 0, 0);
							RigidBody.Upcast(colObj).AngularVelocity = new Vector3(0, 0, 0);
						}
					}
				}
			}
			else if (Keyboard.GetState().IsKeyDown(Keys.Escape) || GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
			{
				Exit();
			}
			else
			{
				//world.stepSimulation(1.0f / 60.0f, 1);
			}

			base.Update(gameTime);
		}

		/// <summary>
		/// Called when the gamedetermines it is time to draw a frame.
		/// </summary>
		/// <param name="gameTime">Time passed since the last call to this function.</param>
		protected override void Draw(GameTime gameTime)
		{
			_graphics.GraphicsDevice.Clear(Color.CornflowerBlue);

			if (_useDebugDrawer)
				((XnaDebugDraw)_world.DebugDrawer).Update(_camera.View, _camera.Projection);

			_world.StepSimulation(1.0f / 60.0f, 1);

			Matrix objMatrix;

			if (_world != null)
			{
				int numObjects = _world.CollisionObjectsCount;
				Vector3 wireColor = Vector3.UnitX;
				for (int i = 0; i < numObjects; i++)
				{
					CollisionObject colObj = _world.CollisionObjects[i];
					RigidBody body = RigidBody.Upcast(colObj);

					if (body != null && body.MotionState != null)
					{
						DefaultMotionState myMotionState = (DefaultMotionState)body.MotionState;
						objMatrix = myMotionState.GraphicsWorldTransform;
					}
					else
					{
						objMatrix = colObj.WorldTransform;
					}

					wireColor = new Vector3(1.0f, 1.0f, 0.5f); //wants deactivation
					if ((i & 1) == 1)
					{
						wireColor = new Vector3(0.0f, 0.0f, 1.0f);
					}

					//color differently for active, sleeping, wantsdeactivation states
					if (colObj.ActivationState == ActivationState.Active)
					{
						if ((i & 1) == 1)
						{
							wireColor += new Vector3(1.0f, 0.0f, 0.0f);
						}
						else
						{
							wireColor += new Vector3(0.5f, 0.0f, 0.0f);
						}
					}

					if (colObj.ActivationState == ActivationState.IslandSleeping)
					{
						if ((i & 1) == 1)
						{
							wireColor += new Vector3(0.0f, 1.0f, 0.0f);
						}
						else
						{
							wireColor += new Vector3(0.0f, 0.5f, 0.0f);
						}
					}

					// draw box
					objMatrix = XnaDevRu.BulletX.MathHelper.GetDisplayMatrix(objMatrix);

					if (i == 0)
						wireColor = Vector3.Zero;

					DrawModel(objMatrix, wireColor, colObj);
				}
			}

			base.Draw(gameTime);
		}

		private void DrawModel(Matrix modelTransform, Vector3 color, CollisionObject obj)
		{
			Model model = _modelBox;
			Matrix scale = Matrix.Identity;

			if (obj.CollisionShape is CylinderShape)
			{
				CylinderShape cylinderShape = (CylinderShape)obj.CollisionShape;
				Vector3 halfExtent = cylinderShape.HalfExtents;
				scale = Matrix.CreateScale(halfExtent.X + _collisionMargin, halfExtent.Y + _collisionMargin, halfExtent.Z / 2 + _collisionMargin * 3.5f) * Matrix.CreateRotationX((float)Math.PI / 2);
				model = _modelCylinder;
			}
			else if (obj.CollisionShape is BoxShape)
			{
				BoxShape boxShape = (BoxShape)obj.CollisionShape;
				Vector3 halfExtent = boxShape.HalfExtents;
				scale = Matrix.CreateScale(2 * halfExtent.X, 2 * halfExtent.Y, 2 * halfExtent.Z);
				model = _modelBox;
			}
			else if (obj.CollisionShape is SphereShape)
			{
				SphereShape sphereShape = (SphereShape)obj.CollisionShape;
				float radius = sphereShape.Radius;
				scale = Matrix.CreateScale(radius, radius, radius);
				model = _modelSphere;
			}

			Matrix[] transforms = new Matrix[model.Bones.Count];
			model.CopyAbsoluteBoneTransformsTo(transforms);

			foreach (ModelMesh mesh in model.Meshes)
			{
				foreach (BasicEffect effect in mesh.Effects)
				{
					effect.EnableDefaultLighting();
					effect.PreferPerPixelLighting = true;
					effect.AmbientLightColor = color;

					effect.View = _camera.View;
					effect.Projection = _camera.Projection;
					effect.World = transforms[mesh.ParentBone.Index] * scale * modelTransform;
				}
				mesh.Draw();
			}
		}

		private void shootBox(Vector3 destination)
		{
			if (_world != null)
			{
				float mass = 1f;
				Matrix startTransform = Matrix.Identity;
				Vector3 camPos = _camera.Position;
				startTransform.Translation = camPos;
				//CollisionShape boxShape = new SphereShape(1);
				CollisionShape boxShape = new BoxShape(Vector3.One);
				RigidBody body = CreateRigidBody(mass, startTransform, boxShape);

				Vector3 linVel = new Vector3(destination.X - camPos.X, destination.Y - camPos.Y, destination.Z - camPos.Z);
				linVel.Normalize();
				linVel *= _shootBoxInitialSpeed;

				//body.getWorldTransform().setOrigin(camPos);
				//body.getWorldTransform().setRotation(btQuaternion(0, 0, 0, 1));
				body.LinearVelocity = linVel;
				body.AngularVelocity = Vector3.Zero;
			}
		}

		private Vector3 getRayTo(int x, int y)
		{
			Vector3 nearSource = new Vector3(x, y, 0);
			Vector3 farSource = new Vector3(x, y, 1);

			Matrix world = Matrix.CreateTranslation(0, 0, 0);

			Vector3 nearPoint = Graphics.GraphicsDevice.Viewport.Unproject(nearSource, Camera.Projection, Camera.View, world);
			Vector3 farPoint = Graphics.GraphicsDevice.Viewport.Unproject(farSource, Camera.Projection, Camera.View, world);

			Vector3 direction = farPoint - nearPoint;
			//direction.Normalize();
			return direction;
		}
	}
}
