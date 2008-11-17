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

namespace XnaDevRu.BulletX.Dynamics
{
	/// <summary>
	/// Jacobian entry is an abstraction that allows to describe constraints
	/// it can be used in combination with a constraint solver
	/// Can be used to relate the effect of an impulse to the constraint error
	/// </summary>
	public class JacobianEntry
	{
		private Vector3 _linearJointAxis;
		private Vector3 _aJ;
		private Vector3 _bJ;
		private Vector3 _0MinvJt;
		private Vector3 _1MinvJt;
		private float _adiag;

		public JacobianEntry() { }

		//constraint between two different rigidbodies
		public JacobianEntry(
			Matrix world2A,
			Matrix world2B,
			Vector3 relPosA, Vector3 relPosB,
			Vector3 jointAxis,
			Vector3 inertiaInvA,
			float massInvA,
			Vector3 inertiaInvB,
			float massInvB)
		{
			_linearJointAxis = jointAxis;
			_aJ = Vector3.TransformNormal(Vector3.Cross(relPosA, _linearJointAxis), world2A);
			_bJ = Vector3.TransformNormal(Vector3.Cross(relPosB, -_linearJointAxis), world2B);
			_0MinvJt = inertiaInvA * _aJ;
			_1MinvJt = inertiaInvB * _bJ;
			_adiag = massInvA + Vector3.Dot(_0MinvJt, _aJ) + massInvB + Vector3.Dot(_1MinvJt, _bJ);

			if (_adiag <= 0.0f)
				throw new BulletException();
		}

		//angular constraint between two different rigidbodies
		public JacobianEntry(Vector3 jointAxis,
			Matrix world2A,
			Matrix world2B,
			Vector3 inertiaInvA,
			Vector3 inertiaInvB)
		{
			_linearJointAxis = new Vector3();
			_aJ = Vector3.TransformNormal(jointAxis, world2A);
			_bJ = Vector3.TransformNormal(-jointAxis, world2B);
			_0MinvJt = inertiaInvA * _aJ;
			_1MinvJt = inertiaInvB * _bJ;
			_adiag = Vector3.Dot(_0MinvJt, _aJ) + Vector3.Dot(_1MinvJt, _bJ);

			if (_adiag <= 0.0f)
				throw new BulletException();
		}

		//angular constraint between two different rigidbodies
		public JacobianEntry(Vector3 axisInA,
			Vector3 axisInB,
			Vector3 inertiaInvA,
			Vector3 inertiaInvB)
		{
			_linearJointAxis = new Vector3();
			_aJ = axisInA;
			_bJ = -axisInB;
			_0MinvJt = inertiaInvA * _aJ;
			_1MinvJt = inertiaInvB * _bJ;
			_adiag = Vector3.Dot(_0MinvJt, _aJ) + Vector3.Dot(_1MinvJt, _bJ);

			if (_adiag <= 0.0f)
				throw new BulletException();
		}

		//constraint on one rigidbody
		public JacobianEntry(
			Matrix world2A,
			Vector3 rel_pos1, Vector3 rel_pos2,
			Vector3 jointAxis,
			Vector3 inertiaInvA,
			float massInvA)
		{
			_linearJointAxis = jointAxis;
			_aJ = Vector3.TransformNormal(Vector3.Cross(rel_pos1, jointAxis), world2A);
			_bJ = Vector3.TransformNormal(Vector3.Cross(rel_pos2, -jointAxis), world2A);
			_0MinvJt = inertiaInvA * _aJ;
			_1MinvJt = new Vector3();
			_adiag = massInvA + Vector3.Dot(_0MinvJt, _aJ);

			if (_adiag <= 0.0f)
				throw new BulletException();
		}

		public float Diagonal { get { return _adiag; } }

		// for two constraints on the same rigidbody (for example vehicle friction)
		public float GetNonDiagonal(JacobianEntry jacB, float massInvA)
		{
			float lin = massInvA * Vector3.Dot(_linearJointAxis, jacB._linearJointAxis);
			float ang = Vector3.Dot(_0MinvJt, jacB._aJ);
			return lin + ang;
		}

		// for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
		public float GetNonDiagonal(JacobianEntry jacB, float massInvA, float massInvB)
		{
			Vector3 lin = _linearJointAxis * jacB._linearJointAxis;
			Vector3 ang0 = _0MinvJt * jacB._aJ;
			Vector3 ang1 = _1MinvJt * jacB._bJ;
			Vector3 lin0 = massInvA * lin;
			Vector3 lin1 = massInvB * lin;
			Vector3 sum = ang0 + ang1 + lin0 + lin1;
			return sum.X + sum.Y + sum.Z;
		}

		public float GetRelativeVelocity(Vector3 linvelA, Vector3 angvelA, Vector3 linvelB, Vector3 angvelB)
		{
			Vector3 linrel = linvelA - linvelB;
			Vector3 angvela = angvelA * _aJ;
			Vector3 angvelb = angvelB * _bJ;
			linrel *= _linearJointAxis;
			angvela += angvelb;
			angvela += linrel;
			float rel_vel2 = angvela.X + angvela.Y + angvela.Z;
			return rel_vel2 + float.Epsilon;
		}
	}
}
