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

namespace XnaDevRu.BulletX
{
	// DefaultMotionState provides a common implementation to synchronize world transforms with offsets
	public class DefaultMotionState : MotionState
	{
		private Matrix _graphicsWorldTransform;
		private Matrix _centerOfMassOffset;
		private Matrix _startWorldTransform;
		private object _userData;

		public DefaultMotionState()
			: this(Matrix.Identity, Matrix.Identity) { }

		public DefaultMotionState(Matrix startTransform, Matrix centerOfMassOffset)
		{
			_graphicsWorldTransform = startTransform;
			_centerOfMassOffset = centerOfMassOffset;
			_startWorldTransform = startTransform;
		}

		public Matrix GraphicsWorldTransform { get { return _graphicsWorldTransform; } set { _graphicsWorldTransform = value; } }
		public Matrix CenterOfMassOffset { get { return _centerOfMassOffset; } set { _centerOfMassOffset = value; } }
		public Matrix StartWorldTransform { get { return _startWorldTransform; } set { _startWorldTransform = value; } }
		public object UserData { get { return _userData; } set { _userData = value; } }

		// synchronizes world transform from user to physics
		public override void GetWorldTransform(out Matrix centerOfMassWorldTrans)
		{
            centerOfMassWorldTrans = MathHelper.InvertMatrix(_centerOfMassOffset) * _graphicsWorldTransform;
		}

		// synchronizes world transform from physics to user
		// Bullet only calls the update of worldtransform for active objects
		public override void SetWorldTransform(Matrix centerOfMassWorldTrans)
		{
			_graphicsWorldTransform = MatrixOperations.Multiply(centerOfMassWorldTrans, _centerOfMassOffset);
			_graphicsWorldTransform.Translation = centerOfMassWorldTrans.Translation;
		}
	}
}
