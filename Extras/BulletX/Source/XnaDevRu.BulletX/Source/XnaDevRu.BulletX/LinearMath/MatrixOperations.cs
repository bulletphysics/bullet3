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
	internal static class MatrixOperations
	{
		public static void SetRotation(ref Matrix m, Quaternion q)
		{
			float d = q.LengthSquared();
			BulletDebug.Assert(d != 0);
			float s = 2f / d;
			float xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
			float wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
			float xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
			float yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
			m = new Matrix(1 - (yy + zz), xy - wz, xz + wy, 0,
							xy + wz, 1 - (xx + zz), yz - wx, 0,
							xz - wy, yz + wx, 1 - (xx + yy), 0,
							m.M41, m.M42, m.M43, 1);
		}

		public static Quaternion GetRotation(Matrix m)
		{
			Quaternion q = new Quaternion();

			float trace = m.M11 + m.M22 + m.M33;

			if (trace > 0)
			{
				float s = (float)Math.Sqrt(trace + 1);
				q.W = s * 0.5f;
				s = 0.5f / s;

				q.X = (m.M32 - m.M23) * s;
				q.Y = (m.M13 - m.M31) * s;
				q.Z = (m.M21 - m.M12) * s;
			}
			else
			{
				int i = m.M11 < m.M22 ?
					(m.M22 < m.M33 ? 2 : 1) :
					(m.M11 < m.M33 ? 2 : 0);
				int j = (i + 1) % 3;
				int k = (i + 2) % 3;

				float s = (float)Math.Sqrt(MathHelper.GetElement(m, i, i) - MathHelper.GetElement(m, j, j) - MathHelper.GetElement(m, k, k) + 1);
				MathHelper.SetElement(ref q, i, s * 0.5f);
				s = 0.5f / s;

				q.W = (MathHelper.GetElement(m, k, j) - MathHelper.GetElement(m, j, k)) * s;
				MathHelper.SetElement(ref q, j, (MathHelper.GetElement(m, j, i) + MathHelper.GetElement(m, i, j)) * s);
				MathHelper.SetElement(ref q, k, (MathHelper.GetElement(m, k, i) + MathHelper.GetElement(m, i, k)) * s);
			}

			return q;
		}

		public static Matrix Scaled(Matrix m, Vector3 v)
		{
			return new Matrix(  m.M11 * v.X, m.M12 * v.Y, m.M13 * v.Z, 0,
								m.M21 * v.X, m.M22 * v.Y, m.M23 * v.Z, 0,
								m.M31 * v.X, m.M32 * v.Y, m.M33 * v.Z, 0,
								0, 0, 0, 1);
		}
		
		public static Matrix Multiply(Matrix a, Matrix b)
		{
			/*return btMatrix3x3(
			m2.tdot(0, m1[0]), m2.tdot(1, m1[0]), m2.tdot(2, m1[0]),
			m2.tdot(0, m1[1]), m2.tdot(1, m1[1]), m2.tdot(2, m1[1]),
			m2.tdot(0, m1[2]), m2.tdot(1, m1[2]), m2.tdot(2, m1[2]));*/
			return new Matrix(
				Dot(b, 0, MathHelper.GetRow(a, 1)), Dot(b, 1, MathHelper.GetRow(a, 1)), Dot(b, 2, MathHelper.GetRow(a, 1)), 0,
				Dot(b, 0, MathHelper.GetRow(a, 2)), Dot(b, 1, MathHelper.GetRow(a, 2)), Dot(b, 2, MathHelper.GetRow(a, 2)), 0,
				Dot(b, 0, MathHelper.GetRow(a, 3)), Dot(b, 1, MathHelper.GetRow(a, 3)), Dot(b, 2, MathHelper.GetRow(a, 3)), 0,
				0, 0, 0, 1);
		}

		public static float Dot(Matrix m, int c, Vector3 v) 
		{
			return MathHelper.GetElement(m, 0, c) * v.X + MathHelper.GetElement(m, 1, c) * v.Y + MathHelper.GetElement(m, 2, c) * v.Z;
		}

		public static Matrix Transpose(Matrix m)
		{
			return new Matrix(	m.M11, m.M21, m.M31, 0,
								m.M12, m.M22, m.M32, 0,
								m.M13, m.M23, m.M33, 0,
								0, 0, 0, 1);
		}
	}
}
