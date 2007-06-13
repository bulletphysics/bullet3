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

namespace XnaDevRu.BulletX.Dynamics
{
	public class ContactSolverInfo
	{
		private float _tau;
		private float _damping;
		private float _friction;
		private float _timeStep;
		private float _restitution;
		private int _numIterations;
		private float _maxErrorReduction;
		private float _sor;
		private float _erp;

		public ContactSolverInfo()
		{
			_tau = 0.6f;
			_damping = 1.0f;
			_friction = 0.3f;
			_restitution = 0f;
			_maxErrorReduction = 20f;
			_numIterations = 10;
			_erp = 0.4f;
			_sor = 1.3f;
		}

		public float Tau { get { return _tau; } set { _tau = value; } }
		public float Damping { get { return _damping; } set { _damping = value; } }
		public float Friction { get { return _friction; } set { _friction = value; } }
		public float TimeStep { get { return _timeStep; } set { _timeStep = value; } }
		public float Restitution { get { return _restitution; } set { _restitution = value; } }
		public int IterationsCount { get { return _numIterations; } set { _numIterations = value; } }
		public float MaxErrorReduction { get { return _maxErrorReduction; } set { _maxErrorReduction = value; } }
		public float Sor { get { return _sor; } set { _sor = value; } }
		public float Erp { get { return _erp; } set { _erp = value; } }
	}
}
