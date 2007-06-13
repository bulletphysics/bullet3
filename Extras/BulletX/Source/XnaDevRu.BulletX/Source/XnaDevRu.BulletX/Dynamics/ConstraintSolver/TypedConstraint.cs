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
	public abstract class TypedConstraint
	{
		private static RigidBody _fixed = new RigidBody(0, null, null, new Vector3(), 0, 0, 0.5f, 0);
		private int _userConstraintType;
		private int _userConstraintId;

		private RigidBody _rbA;
		private RigidBody _rbB;
		private float _appliedImpulse;

		public TypedConstraint()
			: this(_fixed, _fixed) { }

		public TypedConstraint(RigidBody rbA)
			: this(rbA, _fixed) { }

		public TypedConstraint(RigidBody rbA, RigidBody rbB)
		{
			_userConstraintType = -1;
			_userConstraintId = -1;
			_rbA = rbA;
			_rbB = rbB;
			_appliedImpulse = 0;

			_fixed.SetMassProps(0, new Vector3());
		}

		public virtual RigidBody RigidBodyA { get { return _rbA; } protected set { _rbA = value; } }
		public virtual RigidBody RigidBodyB { get { return _rbB; } protected set { _rbB = value; } }

		public float AppliedImpulse { get { return _appliedImpulse; } protected set { _appliedImpulse = value; } }
		public int UserConstraintId { get { return _userConstraintId; } set { _userConstraintId = value; } }
		public int UserConstraintType { get { return _userConstraintType; } set { _userConstraintType = value; } }

		public abstract void BuildJacobian();
		public abstract void SolveConstraint(float timeStep);

		public static int SortConstraintOnIslandPredicate(TypedConstraint left, TypedConstraint right)
		{
			int rightIslandID, leftIslandID;
			rightIslandID = GetConstraintIslandId(right);
			leftIslandID = GetConstraintIslandId(left);
			if (leftIslandID < rightIslandID)
				return -1;
			else
				return 1;
			return 0;
		}

		internal static int GetConstraintIslandId(TypedConstraint lhs)
		{
			int islandId;

			CollisionObject colObjA = lhs.RigidBodyA;
			CollisionObject colObjB = lhs.RigidBodyB;
			islandId = colObjA.IslandTag >= 0 ? colObjA.IslandTag : colObjB.IslandTag;
			return islandId;
		}
	}
}
