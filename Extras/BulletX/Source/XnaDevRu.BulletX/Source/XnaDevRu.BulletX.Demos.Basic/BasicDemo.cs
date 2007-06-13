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
using XnaDevRu.BulletX;
using XnaDevRu.BulletX.Dynamics;

namespace XnaDevRu.BulletX.Demos.Basic
{
	/// <summary>
	/// This is the main type for your game
	/// </summary>
	public class BasicDemo : XnaDevRu.BulletX.Demos.DemoGame
	{
		public BasicDemo()
		{
			SphereBoxCollisionAlgorithm.CreateFunc boxAlgo = new SphereBoxCollisionAlgorithm.CreateFunc();
			boxAlgo.IsSwapped = true;
			CollisionDispatcher.RegisterCollisionCreateFunc(BroadphaseNativeTypes.Sphere, BroadphaseNativeTypes.Sphere, new SphereSphereCollisionAlgorithm.CreateFunc());
			CollisionDispatcher.RegisterCollisionCreateFunc(BroadphaseNativeTypes.Sphere, BroadphaseNativeTypes.Box, new SphereBoxCollisionAlgorithm.CreateFunc());
			CollisionDispatcher.RegisterCollisionCreateFunc(BroadphaseNativeTypes.Box, BroadphaseNativeTypes.Sphere, boxAlgo);

			Shapes[0] = new SphereShape(50);
			Shapes[2] = new SphereShape(HalfExtents - CollisionMargin);

			Matrix tr = Matrix.Identity;
			tr.Translation = new Vector3(0, -50, 0);
			CreateRigidBody(0, tr, Shapes[0]);

			for (int i = 0; i < NumObjects; i++)
			{
				Shapes[2].Margin = CollisionMargin;
				int colsize = 2;
				int row = (int)((i * HalfExtents * 2) / (colsize * 2 * HalfExtents));
				int row2 = row;
				int col = i % colsize - colsize / 2;
				tr.Translation = new Vector3(col * 2 * HalfExtents + (row2 % 2) * HalfExtents,
					row * 2 * HalfExtents + HalfExtents, 0);

				CreateRigidBody(1, tr, Shapes[2]);
			}
		}
	}
}
