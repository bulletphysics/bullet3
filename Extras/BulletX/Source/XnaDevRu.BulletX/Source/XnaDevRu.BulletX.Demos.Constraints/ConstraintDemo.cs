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

#region Using Statements
using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Storage;
#endregion
using XnaDevRu.BulletX;
using XnaDevRu.BulletX.Dynamics;

namespace XnaDevRu.BulletX.Demos.Constraints
{
	/// <summary>
	/// This is the main type for your game
	/// </summary>
	public class ConstraintDemo : XnaDevRu.BulletX.Demos.DemoGame
	{
		Matrix _sliderTransform;
		Vector3 _lowerSliderLimit = new Vector3(-10, 0, 0);
		Vector3 _hiSliderLimit = new Vector3(10, 0, 0);

		RigidBody _d6BodyA = null;

		public ConstraintDemo()
		{
			CollisionShape shape = new BoxShape(new Vector3(HalfExtents, HalfExtents, HalfExtents));
			Matrix trans = Matrix.Identity;
			trans.Translation = new Vector3(0, 20, 0);

			float mass = 1f;
			//Point to Point constraint (ball socket)
			{
				RigidBody bodyA = CreateRigidBody(mass, trans, shape);
				trans.Translation = new Vector3(2 * HalfExtents, 20, 0);

				mass = 1f;
				RigidBody bodyB = null;
				//RigidBody bodyB = CreateRigidBody(mass, trans, shape);
				//bodyB.ActivationState = ActivationState.DisableDeactivation;
				//bodyB.SetDamping(0.3f, 0.3f);

				Vector3 pivotInA = new Vector3(HalfExtents, -HalfExtents, -HalfExtents);
				Vector3 axisInA = new Vector3(0, 0, 1);

				Vector3 pivotInB = bodyB != null ? MathHelper.MatrixToVector(MathHelper.InvertMatrix(bodyB.CenterOfMassTransform), MathHelper.MatrixToVector(bodyA.CenterOfMassTransform, pivotInA)) : pivotInA;
				Vector3 axisInB = bodyB != null ?
					Vector3.TransformNormal(Vector3.TransformNormal(axisInA, bodyB.CenterOfMassTransform), MathHelper.InvertMatrix(bodyB.CenterOfMassTransform)) :
				Vector3.TransformNormal(axisInA, bodyA.CenterOfMassTransform);

				//TypedConstraint p2p = new Point2PointConstraint(bodyA, bodyB, pivotInA, pivotInB);
				//TypedConstraint hinge = new HingeConstraint(bodyA, bodyB, pivotInA, pivotInB, axisInA, axisInB);
				HingeConstraint hinge = new HingeConstraint(bodyA, pivotInA, axisInA);

				//use zero targetVelocity and a small maxMotorImpulse to simulate joint friction
				//float	targetVelocity = 0.0f;
				//float	maxMotorImpulse = 0.01;
				float targetVelocity = 1.0f;
				float maxMotorImpulse = 1.0f;
				hinge.EnableAngularMotor(true, targetVelocity, maxMotorImpulse);

				PhysicsWorld.AddConstraint(hinge);
			}

			// create a slider, using the generic D6 constraint
			{
				mass = 1f;
				Vector3 sliderWorldPos = new Vector3(0, 10, 0);
				Vector3 sliderAxis = new Vector3(1, 0, 0);
				float angle = 0;
				Matrix sliderOrientation = Matrix.CreateFromQuaternion(new Quaternion(sliderAxis, angle));

				trans = Matrix.Identity;
				trans.Translation = sliderWorldPos;
				//trans.setBasis(sliderOrientation);
				_sliderTransform = trans;

				_d6BodyA = CreateRigidBody(mass, trans, shape);
				_d6BodyA.ActivationState = ActivationState.DisableDeactivation;
				RigidBody fixedBody1 = CreateRigidBody(0, trans, null);

				Matrix frameInA, frameInB;
				frameInA = Matrix.Identity;
				frameInB = Matrix.Identity;

				Generic6DofConstraint slider = new Generic6DofConstraint(_d6BodyA, fixedBody1, frameInA, frameInB);
				slider.SetLinearLowerLimit(_lowerSliderLimit);
				slider.SetLinearUpperLimit(_hiSliderLimit);

				//range should be small, otherwise singularities will 'explode' the constraint
				slider.SetAngularLowerLimit(new Vector3(10, 0, 0));
				slider.SetAngularUpperLimit(new Vector3(0, 0, 0));

				PhysicsWorld.AddConstraint(slider);
			}
		}
	}
}