using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Components;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Storage;

namespace Physics
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    partial class Game1 : Microsoft.Xna.Framework.Game
    {
        #region Field: viewTransformation

        /// <summary>
        /// The used view transformation.
        /// </summary>
        private Matrix viewTransformation; 

        #endregion

        #region Field: projectionTransformation

        /// <summary>
        /// The used projection transformation.
        /// </summary>
        private Matrix projectionTransformation;

        #endregion

        #region Field: effect

        /// <summary>
        /// The used effect.
        /// </summary>
        private Effect effect;

        #endregion

        #region Field: transform

        /// <summary>
        /// The used transform effect parameter.
        /// </summary>
        private EffectParameter transform;

        #endregion

        #region Field: color

        /// <summary>
        /// The used color effect parameter.
        /// </summary>
        private EffectParameter color;

        #endregion

        #region Field: volumes

        /// <summary>
        /// The list of bounding volumes.
        /// </summary>
        private List<BoundingVolume> volumes;

        #endregion


        #region Constructor: Game1()

        /// <summary>
        /// Create a new game.
        /// </summary>
        public Game1()
        {
            InitializeComponent();

            float aspectRatio = 640.0f / 480.0f;
            float fov = MathHelper.PiOver4;

            // Initialize the matrices.
            this.viewTransformation = Matrix.CreateLookAt(new Vector3(0, 0, 150), new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            this.projectionTransformation = Matrix.CreatePerspectiveFieldOfView(fov, aspectRatio, 1, 600);

            // Initialize the shader.
            CompiledEffect cEffect = Effect.CompileEffectFromFile("color.fx", null, null, CompilerOptions.None, TargetPlatform.Windows);
            this.effect = new Effect(this.graphics.GraphicsDevice, cEffect.GetShaderCode(), CompilerOptions.None, null);

            // Get the parameters.
            EffectParameterCollection coll = this.effect.Parameters;
            this.transform = coll.GetParameterBySemantic("WorldViewProjection");
            this.color = coll.GetParameterBySemantic("Color");

            // Create and add volumes.
            this.volumes = new List<BoundingVolume>();
            this.AddBoundingVolumes();
        } 

        #endregion


        #region Method: AddBoundingVolumes()

        /// <summary>
        /// Add bounding volumes to the world.
        /// </summary>
        private void AddBoundingVolumes()
        {
            // ----------
            //
            // Add bounding volumes here.
            // 
            // ----------

            // Box 1
            BoundingBox box1 = new BoundingBox(5, 5, 5, Color.Green, this.graphics.GraphicsDevice);
            box1.TranslateWorld(new Vector3(10, 0, 0));
            this.volumes.Add(box1);

            // Box 2
            this.volumes.Add(new BoundingBox(10, 10, 10, Color.LightSkyBlue, this.graphics.GraphicsDevice));
        } 

        #endregion

        #region Method: Update()

        /// <summary>
        /// Update components of this physics application.
        /// </summary>
        protected override void Update()
        {
            // The time since Update was called last
            float elapsed = (float)ElapsedTime.TotalSeconds;

            this.ProcessUserInput();

            // Let the GameComponents update
            UpdateComponents();
        } 

        #endregion

        #region Method: ProcessUserInput()

        /// <summary>
        /// Process input from the user.
        /// </summary>
        private void ProcessUserInput()
        {
            KeyboardState state = Keyboard.GetState();

            // Move left and right.
            if (state.IsKeyDown(Keys.D)) this.volumes[0].TranslateWorld(new Vector3(0.1f, 0, 0));
            if (state.IsKeyDown(Keys.A)) this.volumes[0].TranslateWorld(new Vector3(-0.1f, 0, 0));

            // Check for collisions here somewhere.
        } 

        #endregion

        #region Method: Draw()

        /// <summary>
        /// Draw the content of the physics application.
        /// </summary>
        protected override void Draw()
        {
            // Make sure we have a valid device
            if (!graphics.EnsureDevice())
                return;

            graphics.GraphicsDevice.Clear(ClearOptions.DepthBuffer | ClearOptions.Target, Color.Blue, 1, 0);
            graphics.GraphicsDevice.BeginScene();


            // Render all the bounding volumes.
            foreach (BoundingVolume currentVolume in this.volumes)
            {
                // Set the color.
                this.color.SetValue((currentVolume.Color.ToVector4()));

                // Set the transformation.
                this.transform.SetValue(currentVolume.TransformationMatrix * this.viewTransformation * this.projectionTransformation);

                // Start the effect.
                this.effect.Begin(EffectStateOptions.Default);

                foreach (EffectPass pass in effect.CurrentTechnique.Passes)
                {
                    pass.Begin();

                    // Draw the volume.
                    currentVolume.Draw();

                    pass.End();
                }


                // End the effect.
                this.effect.End();
            }

            // Let the GameComponents draw
            DrawComponents();

            graphics.GraphicsDevice.EndScene();
            graphics.GraphicsDevice.Present();
        } 

        #endregion
    }
}