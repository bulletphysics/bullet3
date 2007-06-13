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
using System.Runtime.InteropServices;

namespace XnaDevRu.BulletX.Dynamics
{
	//1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
	public class SolverConstraint
	{
		private Vector3 _relpos1CrossNormal = new Vector3();
		private Vector3 _relpos2CrossNormal = new Vector3();
		private Vector3 _contactNormal = new Vector3();
		private Vector3 _angularComponentA = new Vector3();
		private Vector3 _angularComponentB = new Vector3();

		private float _appliedVelocityImpulse;
		private int _solverBodyIdA;
		int _solverBodyIdB;
		private float _friction;
		private float _restitution;
		private float _jacDiagABInv;
		private float _penetration;
		private float _appliedImpulse;

		private SolverConstraintType _constraintType = SolverConstraintType.Contact;
		private int _frictionIndex;
		private int[] _unusedPadding = new int[2];

		public Vector3 RelPosACrossNormal { get { return _relpos1CrossNormal; } set { _relpos1CrossNormal = value; } }
		public Vector3 RelPosBCrossNormal { get { return _relpos2CrossNormal; } set { _relpos2CrossNormal = value; } }
		public Vector3 ContactNormal { get { return _contactNormal; } set { _contactNormal = value; } }
		public Vector3 AngularComponentA { get { return _angularComponentA; } set { _angularComponentA = value; } }
		public Vector3 AngularComponentB { get { return _angularComponentB; } set { _angularComponentB = value; } }

		public float AppliedVelocityImpulse { get { return _appliedVelocityImpulse; } set { _appliedVelocityImpulse = value; } }
		public int SolverBodyIdA { get { return _solverBodyIdA; } set { _solverBodyIdA = value; } }
		public int SolverBodyIdB { get { return _solverBodyIdB; } set { _solverBodyIdB = value; } }
		public float Friction { get { return _friction; } set { _friction = value; } }
		public float Restitution { get { return _restitution; } set { _restitution = value; } }
		public float JacDiagABInv { get { return _jacDiagABInv; } set { _jacDiagABInv = value; } }
		public float Penetration { get { return _penetration; } set { _penetration = value; } }
		public float AppliedImpulse { get { return _appliedImpulse; } set { _appliedImpulse = value; } }

		public SolverConstraintType ConstraintType { get { return _constraintType; } set { _constraintType = value; } }
		public int FrictionIndex { get { return _frictionIndex; } set { _frictionIndex = value; } }
		public int[] UnusedPadding { get { return _unusedPadding; } set { _unusedPadding = value; } }

		public enum SolverConstraintType
		{
			Contact = 0,
			Friction,
		}
	}
}
