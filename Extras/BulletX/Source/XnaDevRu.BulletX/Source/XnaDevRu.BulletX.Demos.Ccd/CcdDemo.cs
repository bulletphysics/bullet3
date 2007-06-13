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

namespace XnaDevRu.BulletX.Demos.Ccd
{
	/// <summary>
	/// This is the main type for your game
	/// </summary>
	public class CcdDemo : XnaDevRu.BulletX.Demos.DemoGame
	{
		public CcdDemo()
		{
			NumObjects = 120;

			Shapes[0] = new BoxShape(new Vector3(200, 10, 200));

			for (int i = 0; i < NumObjects; i++)
			{
				CollisionShape shape = Shapes[ShapeIndex[i]];
				shape.Margin = CollisionMargin;

				bool isDynamic = i > 0;

				Matrix trans = Matrix.Identity;

				if (i > 0)
				{
					//stack them
					int colsize = 10;
					int row = (i * HalfExtents * 2) / (colsize * 2 * HalfExtents);
					int row2 = row;
					int col = (i) % (colsize) - colsize / 2;

					if (col > 3)
					{
						col = 11;
						row2 |= 1;
					}

					Vector3 pos;
					if (NumObjects > 2)
					{
						pos = new Vector3(col * 2 * HalfExtents + (row2 % 2) * HalfExtents,
							row * 2 * HalfExtents + HalfExtents + ExtraHeight, 0);
					}
					else
					{
						pos = new Vector3(0, -30, 0);
					}

					if (shape is BoxShape)
					{
						trans.Right = new Vector3(0, -1, 0);
						trans.Up = new Vector3(1, 0, 0);
						trans.Backward = new Vector3(0, 0, 1);
					}

					trans *= Matrix.CreateRotationZ(Microsoft.Xna.Framework.MathHelper.PiOver2);
					trans.Translation = pos;
				}
				else
				{
					trans.Translation = new Vector3(0, -30, 0);
				}

				float mass = 1.0f;
				if (!isDynamic)
					mass = 0.0f;

				RigidBody body = CreateRigidBody(mass, trans, shape);

				body.CcdSquareMotionThreshold = HalfExtents;
				body.CcdSweptSphereRadius = 0.2f * HalfExtents;
			}
		}
	}
}