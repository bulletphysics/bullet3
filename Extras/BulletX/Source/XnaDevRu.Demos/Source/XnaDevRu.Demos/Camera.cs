using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace XnaDevRu.Demos
{
	/// <summary>
	/// First person camera component for the demos, rotated by mouse.
	/// </summary>
	public class Camera : GameComponent
	{
		private Matrix _view;
		private Matrix _projection;

		private Vector3 _position = new Vector3(130, 50, 30);
		private Vector2 _angles = Vector2.Zero;

		private int _widthOver2;
		private int _heightOver2;

		private float _fieldOfView = Microsoft.Xna.Framework.MathHelper.PiOver4;
		private float _aspectRatio;
		private float _nearPlaneDistance = 0.1f;
		private float _farPlaneDistance = 1000.0f;

		private MouseState _prevMouseState = new MouseState();

		/// <summary>
		/// Initializes new camera component.
		/// </summary>
		/// <param name="game">Game to which attach this camera.</param>
		public Camera(Game game)
			: base(game)
		{
			_widthOver2 = game.Window.ClientBounds.Width / 2;
			_heightOver2 = game.Window.ClientBounds.Height / 2;
			_aspectRatio = (float)game.Window.ClientBounds.Width / (float)game.Window.ClientBounds.Height;
			UpdateProjection();
			Mouse.SetPosition(_widthOver2, _heightOver2);
		}

		/// <summary>
		/// Gets camera view matrix.
		/// </summary>
		public Matrix View { get { return _view; } }
		/// <summary>
		/// Gets or sets camera projection matrix.
		/// </summary>
		public Matrix Projection { get { return _projection; } set { _projection = value; } }
		/// <summary>
		/// Gets camera view matrix multiplied by projection matrix.
		/// </summary>
		public Matrix ViewProjection { get { return _view * _projection; } }

		/// <summary>
		/// Gets or sets camera position.
		/// </summary>
		public Vector3 Position { get { return _position; } set { _position = value; } }

		/// <summary>
		/// Gets or sets camera field of view.
		/// </summary>
		public float FieldOfView { get { return _fieldOfView; } set { _fieldOfView = value; UpdateProjection(); } }
		/// <summary>
		/// Gets or sets camera aspect ratio.
		/// </summary>
		public float AspectRatio { get { return _aspectRatio; } set { _aspectRatio = value; UpdateProjection(); } }
		/// <summary>
		/// Gets or sets camera near plane distance.
		/// </summary>
		public float NearPlaneDistance { get { return _nearPlaneDistance; } set { _nearPlaneDistance = value; UpdateProjection(); } }
		/// <summary>
		/// Gets or sets camera far plane distance.
		/// </summary>
		public float FarPlaneDistance { get { return _farPlaneDistance; } set { _farPlaneDistance = value; UpdateProjection(); } }

		/// <summary>
		/// Gets or sets camera's target.
		/// </summary>
		public Vector3 Target
		{
			get
			{
				Matrix cameraRotation = Matrix.CreateRotationX(_angles.X) * Matrix.CreateRotationY(_angles.Y);
				return _position + Vector3.Transform(Vector3.Forward, cameraRotation);
			}
			set
			{
				Vector3 forward = Vector3.Normalize(_position - value);
				Vector3 right = Vector3.Normalize(Vector3.Cross(forward, Vector3.Up));
				Vector3 up = Vector3.Normalize(Vector3.Cross(right, forward));

				Matrix test = Matrix.Identity;
				test.Forward = forward;
				test.Right = right;
				test.Up = up;
				_angles.X = -(float)Math.Asin(test.M32);
				_angles.Y = -(float)Math.Asin(test.M13);
			}
		}

		/// <summary>
		/// Updates camera with input and updates view matrix.
		/// </summary>
		/// <param name="gameTime"></param>
		public override void Update(GameTime gameTime)
		{
			if (Enabled)
			{
				ProcessInput((float)gameTime.ElapsedGameTime.Milliseconds / 30.0f);
				UpdateView();

				base.Update(gameTime);
			}
		}

		private void ProcessInput(float amountOfMovement)
		{
			Vector3 moveVector = new Vector3();

			KeyboardState keys = Keyboard.GetState();
			if (keys.IsKeyDown(Keys.D))
				moveVector.X += amountOfMovement;
			if (keys.IsKeyDown(Keys.A))
				moveVector.X -= amountOfMovement;
			if (keys.IsKeyDown(Keys.S))
				moveVector.Z += amountOfMovement;
			if (keys.IsKeyDown(Keys.W))
				moveVector.Z -= amountOfMovement;

			Matrix cameraRotation = Matrix.CreateRotationX(_angles.X) * Matrix.CreateRotationY(_angles.Y);
			_position += Vector3.Transform(moveVector, cameraRotation);

			MouseState currentMouseState = Mouse.GetState();

			if (currentMouseState.RightButton == ButtonState.Pressed && _prevMouseState.RightButton == ButtonState.Released)
			{
				Mouse.SetPosition(_widthOver2, _heightOver2);
			}
			else if (currentMouseState.RightButton == ButtonState.Pressed)
			{
				if (currentMouseState.X != _widthOver2)
					_angles.Y -= amountOfMovement / 80.0f * (currentMouseState.X - _widthOver2);
				if (currentMouseState.Y != _heightOver2)
					_angles.X -= amountOfMovement / 80.0f * (currentMouseState.Y - _heightOver2);

				if (_angles.X > 1.4) _angles.X = 1.4f;
				if (_angles.X < -1.4) _angles.X = -1.4f;
				if (_angles.Y > Math.PI) _angles.Y -= 2 * (float)Math.PI;
				if (_angles.Y < -Math.PI) _angles.Y += 2 * (float)Math.PI;

				Mouse.SetPosition(_widthOver2, _heightOver2);
			}

			_prevMouseState = currentMouseState;
		}

		private void UpdateProjection()
		{
			_projection = Matrix.CreatePerspectiveFieldOfView(_fieldOfView, _aspectRatio, _nearPlaneDistance, _farPlaneDistance);
		}

		private void UpdateView()
		{
			Matrix cameraRotation = Matrix.CreateRotationX(_angles.X) * Matrix.CreateRotationY(_angles.Y);
			Vector3 targetPos = _position + Vector3.Transform(Vector3.Forward, cameraRotation);

			Vector3 upVector = Vector3.Transform(Vector3.Up, cameraRotation);

			_view = Matrix.CreateLookAt(_position, targetPos, upVector);
		}
	}
}
