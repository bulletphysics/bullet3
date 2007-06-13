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
	/// MinkowskiSumShape represents implicit (getSupportingVertex) based minkowski sum of two convex implicit shapes.
	/// </summary>
	public class MinkowskiSumShape : ConvexShape
	{
		private Matrix _transformA;
		private Matrix _transformB;
		private ConvexShape _shapeA;
		private ConvexShape _shapeB;

		public MinkowskiSumShape(ConvexShape shapeA, ConvexShape shapeB)
		{
			_shapeA = shapeA;
			_shapeB = shapeB;
			_transformA = Matrix.Identity;
			_transformB = Matrix.Identity;
		}

		public Matrix TransformA { get { return _transformA; } set { _transformA = value; } }
		public Matrix TransformB { get { return _transformB; } set { _transformB = value; } }
		public ConvexShape ShapeA { get { return _shapeA; } }
		public ConvexShape ShapeB { get { return _shapeB; } }

		public override float Margin
		{
			get
			{
				return _shapeA.Margin + _shapeB.Margin;
			}
			set
			{
				base.Margin = value;
			}
		}

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.MinkowskiDifference;
			}
		}

		public override string Name
		{
			get
			{
				return "MinkowskiSum";
			}
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
		{
			Vector3 supVertexA = MathHelper.MatrixToVector(_transformA, _shapeA.LocalGetSupportingVertexWithoutMargin(Vector3.TransformNormal(vec, _transformA)));
			Vector3 supVertexB = MathHelper.MatrixToVector(_transformB, _shapeB.LocalGetSupportingVertexWithoutMargin(Vector3.TransformNormal(vec, _transformB)));
			return supVertexA + supVertexB;
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			for (int i = 0; i < vectors.Length; i++)
				supportVerticesOut[i] = LocalGetSupportingVertexWithoutMargin(vectors[i]);
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			inertia = new Vector3();
			BulletDebug.Assert(false);
		}
	}
}
