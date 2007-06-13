//////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2007, Eric Lebetsamer (http://tehone.com)                                  //
//                                                                                          //
// Permission is hereby granted, free of charge, to any person obtaining a copy             //
// of this software and associated documentation files (the "Software"), to deal            //
// in the Software without restriction, including without limitation the rights to          //
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of         //
// the Software, and to permit persons to whom the Software is furnished to do so,          //
// subject to the following conditions:                                                     //
//                                                                                          //
// The above copyright notice and this permission notice shall be included in all copies    //
// or substantial portions of the Software.                                                 //
//                                                                                          //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,      //
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR //
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE       //
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,      //
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE        //
// USE OR OTHER DEALINGS IN THE SOFTWARE.                                                   //
//////////////////////////////////////////////////////////////////////////////////////////////

using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;

namespace XnaDevRu.Demos
{
	/// <summary>
	/// This is a <see cref="DrawableGameComponent"/> that is used to calculate and display the current framerate (FPS, FramesPerSecond) of the game.
    /// </summary>
    public class Framerate : DrawableGameComponent
    {
        private double _fps = 0;
        private float _elapsedRealTime = 0;
        private string _gameWindowTitle;
        private ushort _updateFrequency = 1000;
        private float _updateFrequencyRatio;
  
        /// <summary>
        /// The CurrentFramerate property represents the current frames per second (FPS) for the game.
        /// </summary>
        /// <value>The CurrentFramerate property gets the <see cref="_fps"/> data member.</value>
        public double CurrentFramerate
        {
            get
            {
                return _fps;
            }
        }
  
        /// <summary>
        /// The UpdateFrequency property represents, in milliseconds, how often we are updating the <see cref="_fps"/> count.
        /// </summary>
        /// <value>The UpdateFrequency property gets/sets the <see cref="_updateFrequency"/> data member.</value>
        /// <remarks>
        /// This is used to set how often we are calculating the frames per second count.
        /// <para>The default value is <b><c>1000</c></b>.</para>
        /// </remarks>
        /// <exception cref="System.ArgumentOutOfRangeException">This exception will be thrown when the <b><c>value</c></b> is set less then <b><c>100</c></b> or when <b><c>value</c></b> is not divisible by <b><c>100</c></b>.</exception>
        public ushort UpdateFrequency
        {
            get
            {
                return _updateFrequency;
            }
            set
            {
                if(value % 100 != 0 || value < 100)
                    throw new ArgumentOutOfRangeException("UpdateFrequency", value, "The UpdateFrequency for the Framerate must is based on milliseconds and must be must be a positive number that is greater then or equal to 100, and the number must be divisable by 100.");
  
                _updateFrequency = value;
  
                // Figure out the new ratio, this way we are reporting the correct frames per second even when we are not calculating ever second.
                _updateFrequencyRatio = 1000 / (float)_updateFrequency;
            }
        }
  
        /// <summary>
        /// The main constructor for the class.
        /// </summary>
        /// <param name="game">The <see cref="Microsoft.Xna.Framework.Game" /> instance for this <see cref="DrawableGameComponent"/> to use.</param>
        /// <remarks>Sets the <see cref="_gameWindowTitle"/> data member to the value of <see cref="Microsoft.Xna.Framework.GameWindow.Title"/>.</remarks>
        public Framerate(Game game) : base(game)
        {
            // Save the original game window title
            _gameWindowTitle = game.Window.Title;
  
            // We are basing the ratio on 1 second (1000 milliseconds)
            _updateFrequencyRatio = 1000 / (float)_updateFrequency;
        }
  
        /// <summary>
        /// Allows the game component to perform any initialization it needs to before starting
        /// to run.  This is where it can query for any required services and load content.
        /// </summary>
        public override void Initialize()
        {
            base.Initialize();
        }
  
        /// <summary>
        /// Allows the game component to update itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        public override void Update(GameTime gameTime)
        {
            base.Update(gameTime);
  
            if (!this.Enabled)
                return;
  
            // The total real time in milliseconds since the last Update().
            float elapsed = (float)gameTime.ElapsedRealTime.TotalMilliseconds;
  
            // Adds the Update() elapsed real time to the cumulative elapsed real time.
            _elapsedRealTime += elapsed;
  
            // If the elapsed time is greater than our update frequency then: calculate the framerate, and reduce the elapsed real time count.
            if(_elapsedRealTime > _updateFrequency)
            {
                _fps = (_updateFrequency / elapsed) * _updateFrequencyRatio; // calculate the framerate
                _elapsedRealTime -= _updateFrequency; // adjust the elapsedRealTime
            }
        }
  
        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        public override void Draw(GameTime gameTime)
        {
            base.Draw(gameTime);
  
            if (!this.Visible || !this.Enabled)
                return;
  
            this.Game.Window.Title = _gameWindowTitle + " FPS: " + _fps.ToString("F");
        }
  
        /// <summary>
        /// This method is the <see cref="System.EventHandler"/> for the <see cref="Microsoft.Xna.Framework.GameComponent.EnabledChanged"/> event.
        /// </summary>
        /// <param name="sender">The events sender</param>
        /// <param name="args">The events arguments.</param>
        protected override void OnEnabledChanged(object sender, EventArgs args)
        {
            if(!this.Enabled)
                this.Game.Window.Title = _gameWindowTitle;
  
            base.OnEnabledChanged(sender, args);
        }
  
        /// <summary>
        /// This method is the <see cref="System.EventHandler"/> for the <see cref="Microsoft.Xna.Framework.GameComponent.EnabledChanged"/> event.
        /// </summary>
        /// <param name="sender">The events sender</param>
        /// <param name="args">The events arguments.</param>
        protected override void OnVisibleChanged(object sender, EventArgs args)
        {
            if (!this.Visible)
                this.Game.Window.Title = _gameWindowTitle;
  
            base.OnVisibleChanged(sender, args);
        }
    }
}