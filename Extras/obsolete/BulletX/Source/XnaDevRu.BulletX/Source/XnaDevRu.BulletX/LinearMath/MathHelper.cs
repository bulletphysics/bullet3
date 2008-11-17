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
	public static class MathHelper
	{
		internal const float Sqrt12 = 0.7071067811865475244008443621048490f;
		internal const float Infinity = 3.402823466e+38f;
		internal const float Epsilon = 1.192092896e-07f;

		public static Vector3 MatrixToVector(Matrix m, Vector3 v)
		{
			return new Vector3(
				Vector3.Dot(new Vector3(m.M11, m.M12, m.M13), v) + m.Translation.X,
				Vector3.Dot(new Vector3(m.M21, m.M22, m.M23), v) + m.Translation.Y,
				Vector3.Dot(new Vector3(m.M31, m.M32, m.M33), v) + m.Translation.Z
				);
		}

		internal static int ClosestAxis(Vector4 v)
		{
			return MaxAxis(Absolute(v));
		}

		internal static Vector4 Absolute(Vector4 v)
		{
			return new Vector4(Math.Abs(v.X), Math.Abs(v.Y), Math.Abs(v.Z), Math.Abs(v.W));
		}

		internal static int MaxAxis(Vector4 v)
		{
			int maxIndex = -1;
			float maxVal = float.MinValue;
			if (v.X > maxVal)
			{
				maxIndex = 0;
				maxVal = v.X;
			}
			if (v.Y > maxVal)
			{
				maxIndex = 1;
				maxVal = v.Y;
			}
			if (v.Z > maxVal)
			{
				maxIndex = 2;
				maxVal = v.Z;
			}
			if (v.W > maxVal)
			{
				maxIndex = 3;
				maxVal = v.W;
			}

			return maxIndex;
		}

		internal static int MaxAxis(Vector3 v)
		{
			return v.X < v.Y ? (v.Y < v.Z ? 2 : 1) : (v.X < v.Z ? 2 : 0);
		}

		// conservative test for overlap between two aabbs
		internal static bool TestAabbAgainstAabb2(Vector3 aabbMinA, Vector3 aabbMaxA, Vector3 aabbMinB, Vector3 aabbMaxB)
		{
			bool overlap = true;
			overlap = (aabbMinA.X > aabbMaxB.X || aabbMaxA.X < aabbMinB.X) ? false : overlap;
			overlap = (aabbMinA.Z > aabbMaxB.Z || aabbMaxA.Z < aabbMinB.Z) ? false : overlap;
			overlap = (aabbMinA.Y > aabbMaxB.Y || aabbMaxA.Y < aabbMinB.Y) ? false : overlap;
			return overlap;
		}

		internal static bool TestTriangleAgainstAabb2(Vector3[] vertices, Vector3 aabbMin, Vector3 aabbMax)
		{
			Vector3 p1 = vertices[0];
			Vector3 p2 = vertices[1];
			Vector3 p3 = vertices[2];

			if (Math.Min(Math.Min(p1.X, p2.X), p3.X) > aabbMax.X) return false;
			if (Math.Max(Math.Max(p1.X, p2.X), p3.X) < aabbMin.X) return false;

			if (Math.Min(Math.Min(p1.Z, p2.Z), p3.Z) > aabbMax.Z) return false;
			if (Math.Max(Math.Max(p1.Z, p2.Z), p3.Z) < aabbMin.Z) return false;

			if (Math.Min(Math.Min(p1.Y, p2.Y), p3.Y) > aabbMax.Y) return false;
			if (Math.Max(Math.Max(p1.Y, p2.Y), p3.Y) < aabbMin.Y) return false;
			return true;
		}

		internal static void SetInterpolate3(Vector3 vA, Vector3 vB, float rt, ref Vector3 interpolated)
		{
			float s = 1.0f - rt;
			interpolated.X = s * vA.X + rt * vB.X;
			interpolated.Y = s * vA.Y + rt * vB.Y;
			interpolated.Z = s * vA.Z + rt * vB.Z;
		}

		internal static void PlaneSpace1(Vector3 n, ref Vector3 p, ref Vector3 q)
		{
			if (Math.Abs(n.Z) > Sqrt12)
			{
				// choose p in y-z plane
				float a = n.Y * n.Y + n.Z * n.Z;
				float k = 1f / (float)Math.Sqrt(a);
				p.X = 0;
				p.Y = -n.Z * k;
				p.Z = n.Y * k;
				// set q = n x p
				q.X = a * k;
				q.Y = -n.X * p.Z;
				q.Z = n.X * p.Y;
			}
			else
			{
				// choose p in x-y plane
				float a = n.X * n.X + n.Y * n.Y;
				float k = 1f / (float)Math.Sqrt(a);
				p.X = -n.Y * k;
				p.Y = n.X * k;
				p.Z = 0;
				// set q = n x p
				q.X = -n.Z * p.Y;
				q.Y = n.Z * p.X;
				q.Z = a * k;
			}
		}

		internal static bool RayAabb(Vector3 rayFrom,
								 Vector3 rayTo,
								 Vector3 aabbMin,
								 Vector3 aabbMax,
					  float param, Vector3 normal)
		{
			Vector3 aabbHalfExtent = (aabbMax - aabbMin) * 0.5f;
			Vector3 aabbCenter = (aabbMax + aabbMin) * 0.5f;
			Vector3 source = rayFrom - aabbCenter;
			Vector3 target = rayTo - aabbCenter;
			int sourceOutcode = Outcode(source, aabbHalfExtent);
			int targetOutcode = Outcode(target, aabbHalfExtent);
			if ((sourceOutcode & targetOutcode) == 0x0)
			{
				float lambda_enter = 0;
				float lambda_exit = param;
				Vector3 r = target - source;
				float normSign = 1;
				Vector3 hitNormal = new Vector3();
				int bit = 1;

				for (int j = 0; j < 2; j++)
				{
					{
						if ((sourceOutcode & bit) != 0)
						{
							float lambda = (-source.X - aabbHalfExtent.X * normSign) / r.X;
							if (lambda_enter <= lambda)
							{
								lambda_enter = lambda;
								hitNormal = new Vector3();
								hitNormal.X = normSign;
							}
						}
						else if ((targetOutcode & bit) != 0)
						{
							float lambda = (-source.X - aabbHalfExtent.X * normSign) / r.X;
							SetMin(ref lambda_exit, lambda);
						}
						bit <<= 1;
					}
					{
						if ((sourceOutcode & bit) != 0)
						{
							float lambda = (-source.Y - aabbHalfExtent.Y * normSign) / r.Y;
							if (lambda_enter <= lambda)
							{
								lambda_enter = lambda;
								hitNormal = new Vector3();
								hitNormal.Y = normSign;
							}
						}
						else if ((targetOutcode & bit) != 0)
						{
							float lambda = (-source.Y - aabbHalfExtent.Y * normSign) / r.Y;
							SetMin(ref lambda_exit, lambda);
						}
						bit <<= 1;
					}
					{
						if ((sourceOutcode & bit) != 0)
						{
							float lambda = (-source.Z - aabbHalfExtent.Z * normSign) / r.Z;
							if (lambda_enter <= lambda)
							{
								lambda_enter = lambda;
								hitNormal = new Vector3();
								hitNormal.Z = normSign;
							}
						}
						else if ((targetOutcode & bit) != 0)
						{
							float lambda = (-source.Z - aabbHalfExtent.Z * normSign) / r.Z;
							SetMin(ref lambda_exit, lambda);
						}
						bit <<= 1;
					}
					normSign = -1;
				}
				if (lambda_enter <= lambda_exit)
				{
					param = lambda_enter;
					normal = hitNormal;
					return true;
				}
			}
			return false;
		}

		internal static void SetMin(ref float a, float b)
		{
			if (a > b)
				a = b;
		}

		internal static void SetMax(ref float a, float b)
		{
			if (a < b)
				a = b;
		}

		internal static void SetMax(ref Vector3 self, Vector3 other)
		{
			if (other.X > self.X)
				self.X = other.X;

			if (other.Y > self.Y)
				self.Y = other.Y;

			if (other.Z > self.Z)
				self.Z = other.Z;
		}

		internal static Vector3 SetMax(Vector3 self, Vector3 other)
		{
			if (other.X > self.X)
				self.X = other.X;

			if (other.Y > self.Y)
				self.Y = other.Y;

			if (other.Z > self.Z)
				self.Z = other.Z;

			return self;
		}

		internal static void SetMin(ref Vector3 self, Vector3 other)
		{
			if (other.X < self.X)
				self.X = other.X;

			if (other.Y < self.Y)
				self.Y = other.Y;

			if (other.Z < self.Z)
				self.Z = other.Z;
		}

		internal static Vector3 SetMin(Vector3 self, Vector3 other)
		{
			if (other.X < self.X)
				self.X = other.X;

			if (other.Y < self.Y)
				self.Y = other.Y;

			if (other.Z < self.Z)
				self.Z = other.Z;

			return self;
		}

		internal static int Outcode(Vector3 p, Vector3 halfExtent)
		{
			return (p.X < -halfExtent.X ? 0x01 : 0x0) |
				   (p.X > halfExtent.X ? 0x08 : 0x0) |
				   (p.Y < -halfExtent.Y ? 0x02 : 0x0) |
				   (p.Y > halfExtent.Y ? 0x10 : 0x0) |
				   (p.Z < -halfExtent.Z ? 0x4 : 0x0) |
				   (p.Z > halfExtent.Z ? 0x20 : 0x0);
		}

		internal static Matrix Absolute(Matrix m)
		{
			return new Matrix(Math.Abs(m.M11), Math.Abs(m.M12), Math.Abs(m.M13), Math.Abs(m.M14),
							  Math.Abs(m.M21), Math.Abs(m.M22), Math.Abs(m.M23), Math.Abs(m.M24),
							  Math.Abs(m.M31), Math.Abs(m.M32), Math.Abs(m.M33), Math.Abs(m.M34),
							  Math.Abs(m.M41), Math.Abs(m.M42), Math.Abs(m.M43), Math.Abs(m.M44));
		}

		internal static void SetValueByIndex(ref Vector3 v, int i, float value)
		{
			if (i == 0)
				v.X = value;
			else if (i == 1)
				v.Y = value;
			else
				v.Z = value;
		}

		internal static float GetValueByIndex(Vector3 v, int i)
		{
			if (i == 0)
				return v.X;
			else if (i == 1)
				return v.Y;
			else
				return v.Z;
		}

		internal static Vector3 InvXForm(Matrix m, Vector3 v)
		{
			v -= m.Translation;
			m.Translation = new Vector3();
			return MathHelper.Transform(v, Matrix.Transpose(m));
		}

		internal static Matrix InverseTimes(Matrix m, Matrix t)
		{
			Vector3 v = t.Translation - m.Translation;

			Matrix mat = TransposeTimes(m, t);
			mat.Translation = Vector3.Transform(v, m);
			return mat;
		}

		internal static Matrix TransposeTimes(Matrix mA, Matrix mB)
		{
			return new Matrix(
				mA.M11 * mB.M11 + mA.M21 * mB.M21 + mA.M31 * mB.M31,
				mA.M11 * mB.M12 + mA.M21 * mB.M22 + mA.M31 * mB.M32,
				mA.M11 * mB.M13 + mA.M21 * mB.M23 + mA.M31 * mB.M33,
				0,
				mA.M12 * mB.M11 + mA.M22 * mB.M21 + mA.M32 * mB.M31,
				mA.M12 * mB.M12 + mA.M22 * mB.M22 + mA.M32 * mB.M32,
				mA.M12 * mB.M13 + mA.M22 * mB.M23 + mA.M32 * mB.M33,
				0,
				mA.M13 * mB.M11 + mA.M23 * mB.M21 + mA.M33 * mB.M31,
				mA.M13 * mB.M12 + mA.M23 * mB.M22 + mA.M33 * mB.M32,
				mA.M13 * mB.M13 + mA.M23 * mB.M23 + mA.M33 * mB.M33,
				0, 0, 0, 0, 1);
		}

		internal static Vector3 GetColumn(Matrix m, int column)
		{
			switch (column)
			{
				case 1:
					return new Vector3(m.M11, m.M21, m.M31);
				case 2:
					return new Vector3(m.M12, m.M22, m.M32);
				case 3:
					return new Vector3(m.M13, m.M23, m.M33);
				default:
					throw new ArgumentOutOfRangeException("column");
			}
		}

		internal static Vector3 GetRow(Matrix m, int row)
		{
			switch (row)
			{
				case 1:
					return new Vector3(m.M11, m.M12, m.M13);
				case 2:
					return new Vector3(m.M21, m.M22, m.M23);
				case 3:
					return new Vector3(m.M31, m.M32, m.M33);
				default:
					throw new ArgumentOutOfRangeException("row");
			}
		}

		internal static Quaternion GetRotation(Matrix m)
		{
			float trace = m.M11 + m.M22 + m.M33;
			Quaternion q = new Quaternion();

			if (trace > 0)
			{
				float s = (float)Math.Sqrt(trace + 1.0f);
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

				float s = (float)Math.Sqrt(GetElement(m, i, i) - GetElement(m, j, j) - GetElement(m, k, k) + 1.0f);
				SetElement(ref q, i, s * 0.5f);
				s = 0.5f / s;

				q.W = (GetElement(m, k, j) - GetElement(m, j, k)) * s;
				SetElement(ref q, j, (GetElement(m, j, i) + GetElement(m, i, j)) * s);
				SetElement(ref q, k, (GetElement(m, k, i) + GetElement(m, i, k)) * s);
			}
			return q;
		}

		internal static float SetElement(ref Quaternion q, int index, float value)
		{
			switch (index)
			{
				case 0:
					q.X = value; break;
				case 1:
					q.Y = value; break;
				case 2:
					q.Z = value; break;
				case 3:
					q.W = value; break;
			}

			return 0;
		}

		internal static float GetElement(Quaternion q, int index)
		{
			switch (index)
			{
				case 0:
					return q.X;
				case 1:
					return q.Y;
				case 2:
					return q.Z;
				default:
					return q.W;
			}
		}

		internal static float GetElement(Matrix mat, int index)
		{
			int row = index % 3;
			int col = index / 3;

			return GetElement(mat, row, col);
		}

		internal static float GetElement(Matrix mat, int row, int col)
		{
			switch (row)
			{
				case 0:
					switch (col)
					{
						case 0:
							return mat.M11;
						case 1:
							return mat.M12;
						case 2:
							return mat.M13;
					} break;
				case 1:
					switch (col)
					{
						case 0:
							return mat.M21;
						case 1:
							return mat.M22;
						case 2:
							return mat.M23;
					} break;
				case 2:
					switch (col)
					{
						case 0:
							return mat.M31;
						case 1:
							return mat.M32;
						case 2:
							return mat.M33;
					} break;
			}

			return 0;
		}

		internal static float GetElement(Vector3 v, int index)
		{
			if (index == 0)
				return v.X;
			if (index == 1)
				return v.Y;
			if (index == 2)
				return v.Z;

			throw new ArgumentOutOfRangeException("index");
		}

		internal static void SetElement(ref Vector3 v, int index, float value)
		{
			if (index == 0)
				v.X = value;
			else if (index == 1)
				v.Y = value;
			else if (index == 2)
				v.Z = value;
			else
				throw new ArgumentOutOfRangeException("index");
		}

		public static Matrix InvertMatrix(Matrix m)
		{
			Vector3 pos = m.Translation;
			m.Translation = Vector3.Zero;
			Matrix inv = Matrix.Transpose(m);
			pos = Vector3.Transform(-pos, m);
			inv.Translation = pos;
			return inv;
		}

		public static Matrix GetDisplayMatrix(Matrix m)
		{
			Matrix displayMatrix = m;
			displayMatrix.Translation = Vector3.Zero;
			displayMatrix = Matrix.Transpose(displayMatrix);
			displayMatrix.Translation = m.Translation;
			return displayMatrix;
		}

		internal static Vector3 Transform(Vector3 position, Matrix matrix)
		{
			Vector3 vector = new Vector3();
			vector.X = (((position.X * matrix.M11) + (position.Y * matrix.M12)) + (position.Z * matrix.M13)) + matrix.M41;
			vector.Y = (((position.X * matrix.M21) + (position.Y * matrix.M22)) + (position.Z * matrix.M23)) + matrix.M42;
			vector.Z = (((position.X * matrix.M31) + (position.Y * matrix.M32)) + (position.Z * matrix.M33)) + matrix.M43;
			return vector;
		}

		internal static Vector3 TransformNormal(Vector3 position, Matrix matrix)
		{
			Vector3 vector = new Vector3();
			vector.X = (((position.X * matrix.M11) + (position.Y * matrix.M12)) + (position.Z * matrix.M13));
			vector.Y = (((position.X * matrix.M21) + (position.Y * matrix.M22)) + (position.Z * matrix.M23));
			vector.Z = (((position.X * matrix.M31) + (position.Y * matrix.M32)) + (position.Z * matrix.M33));
			return vector;
		}
	}
}
