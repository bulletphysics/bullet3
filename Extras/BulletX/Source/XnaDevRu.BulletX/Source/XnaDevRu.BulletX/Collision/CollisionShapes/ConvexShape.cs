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
	/// ConvexShape is an abstract shape interface.
	/// The explicit part provides plane-equations, the implicit part provides GetClosestPoint interface.
	/// used in combination with GJK or btConvexCast
	/// </summary>
	public abstract class ConvexShape : CollisionShape
	{
		private const int _maxPreferredPenetrationDirections = 10;
		private const float _convexDistanceMargin = 0.04f;

		private Vector3 _localScaling;
		private Vector3 _implicitShapeDimensions;
		private float _collisionMargin;

		public ConvexShape()
			: base()
		{
			_localScaling = Vector3.One;
			_collisionMargin = ConvexDistanceMargin;
		}

		public static int MaxPreferredPenetrationDirections { get { return _maxPreferredPenetrationDirections; } }
		public static float ConvexDistanceMargin { get { return _convexDistanceMargin; } }

		public Vector3 ImplicitShapeDimensions { get { return _implicitShapeDimensions; } protected set { _implicitShapeDimensions = value; } }
		public virtual int PreferredPenetrationDirectionsCount { get { return 0; } }

		protected float CollisionMargin { get { return _collisionMargin; } set { _collisionMargin = value; } }

		public virtual void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
		{
			penetrationVector = new Vector3();
			BulletDebug.Assert(false);
		}

		public abstract Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec);
		//notice that the vectors should be unit length
		public abstract void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut);

		/// <summary>
		/// getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
		/// </summary>
		/// <param name="t"></param>
		/// <param name="aabbMin"></param>
		/// <param name="aabbMax"></param>
		public override void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			GetAabbSlow(t, out aabbMin, out aabbMax);
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

		public override float Margin
		{
			get
			{
				return _collisionMargin;
			}
			set
			{
				_collisionMargin = value;
			}
		}

		public virtual Vector3 LocalGetSupportingVertex(Vector3 vec)
		{
			Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(vec);

			if (Margin != 0f)
			{
				Vector3 vecnorm = vec;
				if (vecnorm.LengthSquared() < (MathHelper.Epsilon * MathHelper.Epsilon))
				{
					vecnorm = new Vector3(-1f, -1f, -1f);
				}
				vecnorm.Normalize();
				supVertex += Margin * vecnorm;
			}
			return supVertex;
		}

		public virtual void GetAabbSlow(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
		{
			float margin = Margin;
			aabbMax = new Vector3();
			aabbMin = new Vector3();

			for (int i = 0; i < 3; i++)
			{
				Vector3 vec = new Vector3(0f, 0f, 0f);
				MathHelper.SetElement(ref vec, i, 1);

				Vector3 sv = LocalGetSupportingVertex(Vector3.TransformNormal(vec, t));

				Vector3 tmp = MathHelper.MatrixToVector(t, sv);
				MathHelper.SetElement(ref aabbMax, i, MathHelper.GetElement(tmp, i) + margin);
				MathHelper.SetElement(ref vec, i, -1f);
				tmp = MathHelper.MatrixToVector(t, LocalGetSupportingVertex(Vector3.TransformNormal(vec, t)));
				MathHelper.SetElement(ref aabbMin, i, MathHelper.GetElement(tmp, i) - margin);
			}
		}
	}
}
