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
	internal class Quaternion : QuadWord
	{
		public Quaternion() { }

		public Quaternion(float x, float y, float z, float w)
			: base(x, y, z, w) { }

		public Quaternion(Vector3 axis, float angle)
		{
			SetRotation(axis, angle);
		}

		public Quaternion(float yaw, float pitch, float roll)
		{
			SetEuler(yaw, pitch, roll);
		}

		public void SetRotation(Vector3 axis, float angle)
		{
			float d = axis.Length();
			if (d == 0) throw new DivideByZeroException();
			float s = (float)Math.Sin(angle * 0.5f) / d;
			X = axis.X * s;
			Y = axis.Y * s;
			Z = axis.Z * s;
			W = (float)Math.Cos(angle * 0.5f);
		}

		public void SetEuler(float yaw, float pitch, float roll)
		{
			float halfYaw = yaw * 0.5f;
			float halfPitch = pitch * 0.5f;
			float halfRoll = roll * 0.5f;
			float cosYaw = (float)Math.Cos(halfYaw);
			float sinYaw = (float)Math.Sin(halfYaw);
			float cosPitch = (float)Math.Cos(halfPitch);
			float sinPitch = (float)Math.Sin(halfPitch);
			float cosRoll = (float)Math.Cos(halfRoll);
			float sinRoll = (float)Math.Sin(halfRoll);
			X = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
			Y = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
			Z = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
			W = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
		}

		public float LengthSquared()
		{
			return Dot(this, this);
		}

		public float Length()
		{
			return (float)Math.Sqrt(LengthSquared());
		}

		public float Angle()
		{
			return 2f * (float)Math.Acos(W);
		}

		public static float Angle(Quaternion a, Quaternion b)
		{
			float s = (float)Math.Sqrt(a.LengthSquared() * b.LengthSquared());
			if (s == 0) throw new DivideByZeroException();
			return (float)Math.Acos(Dot(a, b) / s);
		}

		public static Quaternion Farthest(Quaternion a, Quaternion b)
		{
			Quaternion diff, sum;
			diff = a - b;
			sum = a + b;
			if (Dot(diff, diff) > Dot(sum, sum))
				return b;
			return -b;
		}

		public static Quaternion Slerp(Quaternion a, Quaternion b, float c)
		{
			float theta = Angle(a, b);
			if (theta != 0)
			{
				float d = 1f / (float)Math.Sin(theta);
				float s0 = (float)Math.Sin((1f - c) * theta);
				float s1 = (float)Math.Sin(c * theta);
				return new Quaternion(
					(a.X * s0 + b.X * s1) * d,
					(a.Y * s0 + b.Y * s1) * d,
					(a.Z * s0 + b.Z * s1) * d,
					(a.W * s0 + b.W * s1) * d);
			}
			else
			{
				return a;
			}
		}

		public static Quaternion Inverse(Quaternion a)
		{
			return new Quaternion(a.X, a.Y, a.Z, -a.W);
		}

		public static Quaternion Normalize(Quaternion a)
		{
			return a / a.Length();
		}

		public static float Dot(Quaternion a, Quaternion b)
		{
			return a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
		}

		public static Quaternion operator +(Quaternion a, Quaternion b)
		{
			return new Quaternion(a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W);
		}

		public static Quaternion operator -(Quaternion a, Quaternion b)
		{
			return new Quaternion(a.X - b.X, a.Y - b.Y, a.Z - b.Z, a.W - b.W);
		}

		public static Quaternion operator -(Quaternion a)
		{
			return new Quaternion(-a.X, -a.Y, -a.Z, -a.W);
		}

		public static Quaternion operator *(Quaternion a, float b)
		{
			return new Quaternion(a.X * b, a.Y * b, a.Z * b, a.W * b);
		}

		public static Quaternion operator *(Quaternion a, Quaternion b)
		{
			return new Quaternion(
				a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
				a.W * b.Y + a.Y * b.W + a.Z * b.X - a.X * b.Z,
				a.W * b.Z + a.Z * b.W + a.X * b.Y - a.Y * b.X,
				a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);
		}

		public static Quaternion operator *(Quaternion a, Vector3 b)
		{
			return new Quaternion(
				a.W * b.X + a.Y * b.Z - a.Z * b.Y,
				a.W * b.Y + a.Z * b.X - a.X * b.Z,
				a.W * b.Z + a.X * b.Y - a.Y * b.X,
				-a.X * b.X - a.Y * b.Y - a.Z * b.Z);
		}

		public static Quaternion operator *(Vector3 w, Quaternion q)
		{
			return new Quaternion(
				w.X * q.W + w.Y * q.Z - w.Z * q.Y,
				w.Y * q.W + w.Z * q.X - w.X * q.Z,
				w.Z * q.W + w.X * q.Y - w.Y * q.X,
				-w.X * q.X - w.Y * q.Y - w.Z * q.Z);
		}

		public static Quaternion operator /(Quaternion a, float b)
		{
			if (b == 0) throw new DivideByZeroException();
			return new Quaternion(a.X / b, a.Y / b, a.Z / b, a.W / b);
		}

		public static explicit operator Microsoft.Xna.Framework.Quaternion(Quaternion a)
		{
			return new Microsoft.Xna.Framework.Quaternion(a.X, a.Y, a.Z, a.W);
		}
	}
}
