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
	public abstract class DynamicsWorld : CollisionWorld
	{
		public DynamicsWorld(IDispatcher dispatcher, OverlappingPairCache pairCache)
			: base(dispatcher, pairCache) { }

		//once a rigidbody is added to the dynamics world, it will get this gravity assigned
		//existing rigidbodies in the world get gravity assigned too, during this method
		public abstract Vector3 Gravity { set; }
		public abstract IConstraintSolver ConstraintSolver { set; }
		public virtual int ConstraintsCount { get { return 0; } }
		public abstract IDebugDraw DebugDrawer { get; set; }

		//stepSimulation proceeds the simulation over timeStep units
		public abstract void StepSimulation(float timeStep, int numSubsteps, float fixedTimeStep);

		public void StepSimulation(float timeStep)
		{
			StepSimulation(timeStep, 1, 1f / 60f);
		}

		public abstract void UpdateAabbs();

		public virtual void AddConstraint(TypedConstraint constraint) { }
		public virtual void RemoveConstraint(TypedConstraint constraint) { }

		public abstract void AddRigidBody(RigidBody body);
		public abstract void RemoveRigidBody(RigidBody body);

		public virtual TypedConstraint GetConstraint(int index) { return null; }
	}
}
