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
	/// ConeShape implements a Cone shape, around the X axis
	/// </summary>
	public class ConeShapeX : ConeShape
	{
		public ConeShapeX(float radius, float height)
			: base(radius, height)
		{
			ConeUpIndex = 0;
		}
	}

	/// <summary>
	/// ConeShape implements a Cone shape, around the Z axis
	/// </summary>
	public class ConeShapeZ : ConeShape
	{
		public ConeShapeZ(float radius, float height)
			: base(radius, height)
		{
			ConeUpIndex = 2;
		}
	}

	/// <summary>
	/// ConeShape implements a Cone shape, around the Y axis
	/// </summary>
	public class ConeShape : ConvexShape
	{
		private float _sinAngle;
		private float _radius;
		private float _height;
		private int[] _coneIndices = new int[3];

		public ConeShape(float radius, float height)
		{
			_radius = radius;
			_height = height;
			ConeUpIndex = 1;
			_sinAngle = (_radius / (float)Math.Sqrt(_radius * _radius + _height * _height));
		}

		public float Radius { get { return _radius; } }
		public float Height { get { return _height; } }

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.Cone;
			}
		}

		public override string Name
		{
			get
			{
				return "Cone";
			}
		}

		//choose upAxis index
		public int ConeUpIndex
		{
			get { return _coneIndices[1]; }
			set
			{
				switch (value)
				{
					case 0:
						_coneIndices[0] = 1;
						_coneIndices[1] = 0;
						_coneIndices[2] = 2;
						break;
					case 1:
						_coneIndices[0] = 0;
						_coneIndices[1] = 1;
						_coneIndices[2] = 2;
						break;
					case 2:
						_coneIndices[0] = 0;
						_coneIndices[1] = 2;
						_coneIndices[2] = 1;
						break;
					default:
						BulletDebug.Assert(false);
						break;
				}
			}
		}

		private Vector3 ConeLocalSupport(Vector3 v)
		{
			float halfHeight = _height * 0.5f;
			bool condition;

			if (_coneIndices[1] == 0)
				condition = v.X > v.Length() * _sinAngle;
			else if (_coneIndices[1] == 1)
				condition = v.Y > v.Length() * _sinAngle;
			else
				condition = v.Z > v.Length() * _sinAngle;

			if (condition)
			{
				Vector3 tmp = new Vector3();
				MathHelper.SetValueByIndex(ref tmp, _coneIndices[1], halfHeight);
				return tmp;
			}
			else
			{
				float s = (float)Math.Sqrt(MathHelper.GetValueByIndex(v, _coneIndices[0]) * MathHelper.GetValueByIndex(v, _coneIndices[0])
					+ MathHelper.GetValueByIndex(v, _coneIndices[2]) * MathHelper.GetValueByIndex(v, _coneIndices[2]));
				if (s > MathHelper.Epsilon)
				{
					float d = _radius / s;
					Vector3 tmp = new Vector3();
					MathHelper.SetValueByIndex(ref tmp, _coneIndices[0], MathHelper.GetValueByIndex(v, _coneIndices[0]) * d);
					MathHelper.SetValueByIndex(ref tmp, _coneIndices[1], -halfHeight);
					MathHelper.SetValueByIndex(ref tmp, _coneIndices[2], MathHelper.GetValueByIndex(v, _coneIndices[2]) * d);
					return tmp;
				}
				else
				{
					Vector3 tmp = new Vector3();
					MathHelper.SetValueByIndex(ref tmp, _coneIndices[1], -halfHeight);
					return tmp;
				}
			}
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
		{
			return ConeLocalSupport(vec);
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			for (int i = 0; i < vectors.Length; i++)
				supportVerticesOut[i] = ConeLocalSupport(vectors[i]);
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			Matrix identity = Matrix.Identity;
			Vector3 aabbMin, aabbMax;
			GetAabb(identity, out aabbMin, out aabbMax);

			Vector3 halfExtents = (aabbMax - aabbMin) * 0.5f;

			float margin = Margin;

			float lx = 2f * (halfExtents.X + margin);
			float ly = 2f * (halfExtents.Y + margin);
			float lz = 2f * (halfExtents.Z + margin);
			float x2 = lx * lx;
			float y2 = ly * ly;
			float z2 = lz * lz;
			float scaledmass = mass * 0.08333333f;

			inertia = scaledmass * (new Vector3(y2 + z2, x2 + z2, x2 + y2));
		}

		public override Vector3 LocalGetSupportingVertex(Vector3 vec)
		{
			Vector3 supVertex = ConeLocalSupport(vec);
			if (Margin != 0)
			{
				Vector3 vecnorm = vec;
				if (vecnorm.LengthSquared() < (MathHelper.Epsilon * MathHelper.Epsilon))
				{
					vecnorm = new Vector3(-1f, -1f, -1f);
				}
				vecnorm = Vector3.Normalize(vecnorm);
				supVertex += Margin * vecnorm;
			}
			return supVertex;
		}
	}
}
