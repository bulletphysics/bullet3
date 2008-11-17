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
    internal abstract class QuadWord
    {
        private float x;
        private float y;
        private float z;
        private float w;

        public float X { get { return x; } set { x = value; } }
        public float Y { get { return y; } set { y = value; } }
        public float Z { get { return z; } set { z = value; } }
        public float W { get { return w; } set { w = value; } }

        public QuadWord() { }

        public QuadWord(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public QuadWord(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
            W = 0;
        }

        public void SetMax(QuadWord other)
        {
            if (other.X > X)
                X = other.X;

            if (other.Y > Y)
                Y = other.Y;

            if (other.Z > Z)
                Z = other.Z;

            if (other.W > W)
                W = other.W;
        }

        public void SetMin(QuadWord other)
        {
            if (other.X < X)
                X = other.X;

            if (other.Y < Y)
                Y = other.Y;

            if (other.Z < Z)
                Z = other.Z;

            if (other.W < W)
                W = other.W;
        }
    }
}
