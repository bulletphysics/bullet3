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
using System.Diagnostics;

namespace XnaDevRu.BulletX
{
    public class EmptyShape : ConcaveShape
    {
        private Vector3 _localScaling;

		public override string Name
		{
			get
			{
				return "Empty";
			}
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.Empty;
			}
		}

		public override Vector3 LocalScaling
		{
			get
			{
				return _localScaling;
			}
			set
			{
				_localScaling = value;
			}
		}

        public override void ProcessAllTriangles(ITriangleCallback callback, Microsoft.Xna.Framework.Vector3 aabbMin, Microsoft.Xna.Framework.Vector3 aabbMax)
        {
            throw new Exception("The method or operation is not implemented.");
        }

        public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
        {
			Vector3 margin = new Vector3(Margin, Margin, Margin);
            aabbMin = t.Translation - margin;
            aabbMax = t.Translation + margin;
        }

        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
        {
            inertia = new Vector3();
            BulletDebug.Assert(false);
        }
    }
}
