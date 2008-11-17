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
	/// <summary>
	/// btSphereShape implements an implicit (getSupportingVertex) Sphere
	/// </summary>
	public class SphereShape : ConvexShape
	{
		public SphereShape(float radius)
			: base()
		{
			Vector3 temp = ImplicitShapeDimensions;
			temp.X = radius;
			ImplicitShapeDimensions = temp;
		}

		public float Radius { get { return ImplicitShapeDimensions.X; } }

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.Sphere;
			}
		}

		public override string Name
		{
			get
			{
				return "Sphere";
			}
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
		{
			return new Vector3();
		}

		/// <summary>
		/// to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		/// this means, non-uniform scaling is not supported anymore
		/// </summary>
		public override float Margin
		{
			get
			{
				return LocalScaling.X * Radius + base.Margin;
			}
			set
			{
				base.Margin = value;
			}
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			if (supportVerticesOut != null)
				for (int i = 0; i < supportVerticesOut.Length; i++)
					supportVerticesOut[i] = new Vector3();
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			float elem = 0.4f * mass * Margin * Margin;
			inertia = new Vector3(elem, elem, elem);
		}

		public override Vector3 LocalGetSupportingVertex(Vector3 vec)
		{
			Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(vec);

			Vector3 vecnorm = vec;
			if (vecnorm.LengthSquared() < (MathHelper.Epsilon * MathHelper.Epsilon))
			{
				vecnorm = new Vector3(-1f, -1f, -1f);
			}
			vecnorm.Normalize();
			supVertex += Margin * vecnorm;
			return supVertex;
		}

		public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			Vector3 center = t.Translation;
			Vector3 extent = new Vector3(Margin, Margin, Margin);
			aabbMin = center - extent;
			aabbMax = center + extent;
		}
	}
}
