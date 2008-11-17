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
	internal class Vector3 : QuadWord
	{
		public Vector3() { }

		public Vector3(float x, float y, float z)
			: base(x, y, z) { }

		public void SetInterpolate3(Vector3 a, Vector3 b, float c)
		{
			float s = 1.0f - c;
			X = s * a.X + c * b.X;
			Y = s * a.Y + c * b.Y;
			Z = s * a.Z + c * b.Z;
		}

		public float LengthSquared()
		{
			return Dot(this, this);
		}

		public float Length()
		{
			return (float)Math.Sqrt(LengthSquared());
		}

		public int MinAxis()
		{
			return X < Y ? (X < Z ? 0 : 2) : (Y < Z ? 1 : 2);
		}

		public int MaxAxis()
		{
			return X < Y ? (Y < Z ? 2 : 1) : (X < Z ? 2 : 0);
		}

		public int FurthestAxis()
		{
			return Absolute(this).MinAxis();
		}

		public int ClosestAxis()
		{
			return Absolute(this).MaxAxis();
		}

		public Vector3 Rotate(Vector3 axis, float angle)
		{
			Vector3 o = axis * Dot(axis, this);
			Vector3 x = this - o;
			Vector3 y = Cross(axis, this);

			return (o + x * (float)Math.Cos(angle) + y * (float)Math.Sin(angle));
		}

		public static Vector3 Lerp(Vector3 a, Vector3 b, float c)
		{
			return new Vector3(
				a.X + (b.X - a.X) * c,
				a.Y + (b.Y - a.Y) * c,
				a.Z + (b.Z - a.Z) * c);
		}

		public static float Angle(Vector3 a, Vector3 b)
		{
			float s = (float)Math.Sqrt(a.LengthSquared() * b.LengthSquared());
			if (s == 0) throw new DivideByZeroException();
			return (float)Math.Acos(Dot(a, b) / s);
		}

		public static Vector3 Absolute(Vector3 a)
		{
			return new Vector3(
				Math.Abs(a.X),
				Math.Abs(a.Y),
				Math.Abs(a.Z));
		}

		public static Vector3 Normalize(Vector3 a)
		{
			return a / a.Length();
		}

		public static Vector3 Cross(Vector3 a, Vector3 b)
		{
			return new Vector3(
				a.Y * b.Z - a.Z * b.Y,
				a.Z * b.X - a.X * b.Z,
				a.X * b.Y - a.Y * b.X);
		}

		public static float Dot(Vector3 a, Vector3 b)
		{
			return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
		}

		public static float Triple(Vector3 a, Vector3 b, Vector3 c)
		{
			return a.X * (b.Y * c.Z - b.Z * c.Y) +
				a.Y * (b.Z * c.X - b.X * c.Z) +
				a.Z * (b.X * c.Y - b.Y * c.X);
		}

		public static float Distance(Vector3 a, Vector3 b)
		{
			return (b - a).Length();
		}

		public static float DistanceSquared(Vector3 a, Vector3 b)
		{
			return (b - a).LengthSquared();
		}

		public static Vector3 Rotate(Vector3 a, Vector3 axis, float angle)
		{
			Vector3 o = axis * Dot(axis, a);
			Vector3 x = a - o;
			Vector3 y = Cross(axis, a);

			return (o + x * (float)Math.Cos(angle) + y * (float)Math.Sin(angle));
		}

		public static Vector3 operator +(Vector3 a, Vector3 b)
		{
			return new Vector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
		}

		public static Vector3 operator -(Vector3 a, Vector3 b)
		{
			return new Vector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
		}

		public static Vector3 operator -(Vector3 a)
		{
			return new Vector3(-a.X, -a.Y, -a.Z);
		}

		public static Vector3 operator *(float b, Vector3 a)
		{
			return new Vector3(a.X * b, a.Y * b, a.Z * b);
		}

		public static Vector3 operator *(Vector3 a, float b)
		{
			return new Vector3(a.X * b, a.Y * b, a.Z * b);
		}

		public static Vector3 operator *(Vector3 a, Vector3 b)
		{
			return new Vector3(a.X * b.X, a.Y * b.Y, a.Z * b.Z);
		}

		public static Vector3 operator /(Vector3 a, float b)
		{
			if (b == 0) throw new DivideByZeroException();
			return new Vector3(a.X / b, a.Y / b, a.Z / b);
		}

		public static Vector3 operator /(Vector3 a, Vector3 b)
		{
			if (b.X == 0 || b.Y == 0 || b.Z == 0) throw new DivideByZeroException();
			return new Vector3(a.X / b.X, a.Y / b.Y, a.Z / b.Z);
		}

		public static bool operator ==(Vector3 a, Vector3 b)
		{
			return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
		}

		public static bool operator !=(Vector3 a, Vector3 b)
		{
			return a.X != b.X || a.Y != b.Y || a.Z != b.Z;
		}

		public static explicit operator Microsoft.Xna.Framework.Vector3(Vector3 a)
		{
			return new Microsoft.Xna.Framework.Vector3(a.X, a.Y, a.Z);
		}

		public override bool Equals(object obj)
		{
			return object.Equals(this, obj);
		}

		public override int GetHashCode()
		{
			return X.GetHashCode() & Y.GetHashCode() & Z.GetHashCode();
		}

		public override string ToString()
		{
			return string.Format("{0}, {1}, {2}", X, Y, Z);
		}
	}
}
