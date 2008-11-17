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

namespace XnaDevRu.BulletX
{
	public enum DispatchFunction
	{
		Discrete = 1,
		Continuous,
	}

	public class DispatcherInfo
	{
		private float _timeStep;
		private int _stepCount;
		private DispatchFunction _dispatchFunc = DispatchFunction.Discrete;
		private float _timeOfImpact = 1;
		private bool _useContinuous;
		private bool _enableSatConvex;
		private bool _enableSpu;
		private IDebugDraw _debugDraw;

		public float TimeStep { get { return _timeStep; } set { _timeStep = value; } }
		public int StepCount { get { return _stepCount; } set { _stepCount = value; } }
		public DispatchFunction DispatchFunction { get { return _dispatchFunc; } set { _dispatchFunc = value; } }
		public float TimeOfImpact { get { return _timeOfImpact; } set { _timeOfImpact = value; } }
		public bool UseContinuous { get { return _useContinuous; } set { _useContinuous = value; } }
		public bool EnableSatConvex { get { return _enableSatConvex; } set { _enableSatConvex = value; } }
		public bool enableSpu { get { return _enableSpu; } set { _enableSpu = value; } }
		public IDebugDraw DebugDraw { get { return _debugDraw; } set { _debugDraw = value; } }
	}
}
