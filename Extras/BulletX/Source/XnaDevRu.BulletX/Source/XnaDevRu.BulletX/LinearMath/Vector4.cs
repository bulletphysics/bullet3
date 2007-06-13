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

namespace XnaDevRu.BulletX.LinearMath
{
	internal class Vector4 : Vector3
	{
		public Vector4() { }

		public Vector4(float x, float y, float z, float w)
			: base(x, y, z) { W = w; }

		public static Vector4 Absolute4(Vector4 a)
		{
			return new Vector4(
				Math.Abs(a.X),
				Math.Abs(a.Y),
				Math.Abs(a.Z),
				Math.Abs(a.W));
		}

		public int MaxAxis4()
		{
			int maxIndex = -1;
			float maxVal = -1e30f;
			if (X > maxVal)
			{
				maxIndex = 0;
				maxVal = X;
			}
			if (Y > maxVal)
			{
				maxIndex = 1;
				maxVal = Y;
			}
			if (Z > maxVal)
			{
				maxIndex = 2;
				maxVal = Z;
			}
			if (W > maxVal)
			{
				maxIndex = 3;
				maxVal = W;
			}

			return maxIndex;
		}

		public int MinAxis4()
		{
			int minIndex = -1;
			float minVal = 1e30f;
			if (X < minVal)
			{
				minIndex = 0;
				minVal = X;
			}
			if (Y < minVal)
			{
				minIndex = 1;
				minVal = Y;
			}
			if (Z < minVal)
			{
				minIndex = 2;
				minVal = Z;
			}
			if (W < minVal)
			{
				minIndex = 3;
				minVal = W;
			}

			return minIndex;
		}

		public int ClosestAxis4()
		{
			return Absolute4(this).MaxAxis4();
		}

		public static explicit operator Microsoft.Xna.Framework.Vector4(Vector4 a)
		{
			return new Microsoft.Xna.Framework.Vector4(a.X, a.Y, a.Z, a.W);
		}
	}
}
