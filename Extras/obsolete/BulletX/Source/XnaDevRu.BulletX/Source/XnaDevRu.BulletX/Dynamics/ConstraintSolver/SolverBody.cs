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
	public class SolverBody
	{
		private Vector3 _centerOfMassPosition = new Vector3();
		private Vector3 _linearVelocity = new Vector3();
		private Vector3 _angularVelocity = new Vector3();
		private RigidBody _originalBody = null;
		private float _invMass;
		private float _friction;
		private float _angularFactor;

		public Vector3 CenterOfMassPosition { get { return _centerOfMassPosition; } set { _centerOfMassPosition = value; } }
		public Vector3 LinearVelocity { get { return _linearVelocity; } set { _linearVelocity = value; } }
		public Vector3 AngularVelocity { get { return _angularVelocity; } set { _angularVelocity = value; } }
		public RigidBody OriginalBody { get { return _originalBody; } set { _originalBody = value; } }
		public float InvMass { get { return _invMass; } set { _invMass = value; } }
		public float Friction { get { return _friction; } set { _friction = value; } }
		public float AngularFactor { get { return _angularFactor; } set { _angularFactor = value; } }

		public void GetVelocityInLocalPoint(Vector3 relPos, out Vector3 velocity)
		{
			velocity = _linearVelocity + Vector3.Cross(_angularVelocity, relPos);
		}

		public void WriteBackVelocity()
		{
			if (_invMass != 0)
			{
				_originalBody.LinearVelocity = _linearVelocity;
				_originalBody.AngularVelocity = _angularVelocity;
			}
		}

		public void ReadVelocity()
		{
			if (_invMass != 0)
			{
				_linearVelocity = _originalBody.LinearVelocity;
				_angularVelocity = _originalBody.AngularVelocity;
			}
		}

		//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
		internal void ApplyImpulse(Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude)
		{
			_linearVelocity += linearComponent * impulseMagnitude;
			_angularVelocity += angularComponent * impulseMagnitude * _angularFactor;
		}
	}
}
