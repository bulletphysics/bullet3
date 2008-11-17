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
	public static class TransformUtil
	{
		const float AngularMotionTreshold = 0.5f * Microsoft.Xna.Framework.MathHelper.PiOver2;

		public static void IntegrateTransform(Matrix currentTransform, Vector3 linearVelocity, Vector3 angularVelocity, float timeStep, ref Matrix predictedTransform)
		{
			predictedTransform.Translation = currentTransform.Translation + linearVelocity * timeStep;
			//exponential map
			Vector3 axis;
			float angle = angularVelocity.Length();
			//limit the angular motion
			if (angle * timeStep > AngularMotionTreshold)
			{
				angle = AngularMotionTreshold / timeStep;
			}

			if (angle < 0.001f)
			{
				// use Taylor's expansions of sync function
				axis = angularVelocity * (0.5f * timeStep - (timeStep * timeStep * timeStep) * (0.020833333333f) * angle * angle);
			}
			else
			{
				// sync(fAngle) = sin(c*fAngle)/t
				axis = angularVelocity * ((float)Math.Sin(0.5f * angle * timeStep) / angle);
			}
			Quaternion dorn = new Quaternion(axis.X, axis.Y, axis.Z, (float)Math.Cos(angle * timeStep * 0.5f));
			Quaternion ornA = MatrixOperations.GetRotation(currentTransform);

			Quaternion predictedOrn = dorn * ornA;
			predictedOrn.Normalize();

			MatrixOperations.SetRotation(ref predictedTransform, predictedOrn);

			Matrix test = Matrix.CreateFromQuaternion(predictedOrn);
		}

		public static void CalculateVelocity(Matrix transformA, Matrix transformB, float timeStep, ref Vector3 linearVelocity, ref Vector3 angularVelocity)
		{
			linearVelocity = (transformB.Translation - transformA.Translation) / timeStep;
            Matrix dmat = transformB * MathHelper.InvertMatrix(transformA);
			Quaternion dorn = Quaternion.CreateFromRotationMatrix(dmat);

			Vector3 axis;
			float angle = 2 * (float)Math.Acos(dorn.W);
			axis = new Vector3(dorn.X, dorn.Y, dorn.Z);
			//axis[3] = 0.f;
			//check for axis length
			float len = axis.LengthSquared();
			if (len < MathHelper.Epsilon * MathHelper.Epsilon)
				axis = new Vector3(1f, 0f, 0f);
			else
				axis /= (float)Math.Sqrt(len);

			angularVelocity = axis * angle / timeStep;
		}

		public static void CalculateDiffAxisAngle(Matrix transformA, Matrix transformB, out Vector3 axis, out float angle)
		{
			Matrix dmat = transformB * MathHelper.InvertMatrix(transformA);
			Quaternion dorn = MathHelper.GetRotation(dmat);

			angle = 2f * (float)Math.Acos(dorn.W);
			axis = new Vector3(dorn.X, dorn.Y, dorn.Z);
			//check for axis length
			float len = axis.LengthSquared();
			if (len < MathHelper.Epsilon * MathHelper.Epsilon)
				axis = new Vector3(1f, 0f, 0f);
			else
				axis /= (float)Math.Sqrt(len);
		}
	}
}
