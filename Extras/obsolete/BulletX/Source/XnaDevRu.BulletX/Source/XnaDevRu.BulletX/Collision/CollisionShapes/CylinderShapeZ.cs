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
	public class CylinderShapeZ : CylinderShape
	{
		public CylinderShapeZ(Vector3 halfExtents)
			: base(halfExtents) { }

		public override int UpAxis
		{
			get
			{
				return 2;
			}
		}

		public override float Radius
		{
			get
			{
				return HalfExtents.X;
			}
		}

		//debugging
		public override string Name
		{
			get
			{
				return "CylinderZ";
			}
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
		{
			return CylinderLocalSupportZ(HalfExtents, vec);
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			for (int i = 0; i < vectors.Length; i++)
			{
				supportVerticesOut[i] = CylinderLocalSupportZ(HalfExtents, vectors[i]);
			}
		}

		Vector3 CylinderLocalSupportZ(Vector3 halfExtents, Vector3 v)
		{
			//mapping depends on how cylinder local orientation is
			// extents of the cylinder is: X,Y is for radius, and Z for height
			float radius = halfExtents.X;
			float halfHeight = halfExtents.Z;

			Vector3 tmp = new Vector3();
			float d;

			float s = (float)Math.Sqrt(v.X * v.X + v.Y * v.Y);
			if (s != 0)
			{
				d = radius / s;
				tmp.X = v.X * d;
				tmp.Z = v.Z < 0 ? -halfHeight : halfHeight;
				tmp.Y = v.Y * d;
				return tmp;
			}
			else
			{
				tmp.X = radius;
				tmp.Z = v.Z < 0 ? -halfHeight : halfHeight;
				tmp.Y = 0;
				return tmp;
			}
		}
	}
}
