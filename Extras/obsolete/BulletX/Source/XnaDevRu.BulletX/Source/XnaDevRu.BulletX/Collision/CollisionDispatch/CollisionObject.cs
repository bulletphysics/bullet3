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
	public enum ActivationState
	{
		Nothing = 0,
		Active,
		IslandSleeping,
		WantsDeactivation,
		DisableDeactivation,
		DisableSimulation,
	}

	public enum CollisionOptions
	{
		StaticObject = 1,
		KinematicObject = 2,
		NoContactResponse = 4,
		CustomMaterialCallback = 8,//this allows per-triangle material (friction/restitution)
	}

	/// <summary>
	/// btCollisionObject can be used to manage collision detection objects. 
	/// btCollisionObject maintains all information that is needed for a collision detection: Shape, Transform and AABB proxy.
	/// They can be added to the btCollisionWorld.
	/// </summary>
	public class CollisionObject
	{
		protected Matrix _worldTransform;
		private BroadphaseProxy _broadphase;
		private CollisionShape _collisionShape;

		//m_interpolationWorldTransform is used for CCD and interpolation
		//it can be either previous or future (predicted) transform
		private Matrix _interpolationWorldTransform;

		private CollisionOptions _collisionFlags;

		private int _islandTag;
		private ActivationState _activationState;
		private float _deactivationTime;

		private float _friction;
		private float _restitution;

		//users can point to their objects, m_userPointer is not used by Bullet
		private object _userData;

		//m_internalOwner one is used by optional Bullet high level interface
		private object _internalOwner;

		//time of impact calculation
		private float _hitFraction;

		//Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
		private float _ccdSweptSphereRadius;

		// Don't do continuous collision detection if square motion (in one step) is less then m_ccdSquareMotionTreshold
		private float _ccdSquareMotionThreshold;
		//those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities) 
		//without destroying the continuous interpolated motion (which uses this interpolation velocities)
		private Vector3 _interpolationLinearVelocity;
		private Vector3 _interpolationAngularVelocity;

		private int _companionID;

		public CollisionObject()
		{
			_activationState = ActivationState.Active;
			_hitFraction = 1;
		}

		public bool IsStaticObject { get { return (_collisionFlags & CollisionOptions.StaticObject) != 0; } }
		public bool IsKinematicObject { get { return (_collisionFlags & CollisionOptions.KinematicObject) != 0; } }
		public bool IsStaticOrKinematicObject { get { return (_collisionFlags & (CollisionOptions.KinematicObject | CollisionOptions.StaticObject)) != 0; } }

		public bool HasContactResponse { get { return (_collisionFlags & CollisionOptions.NoContactResponse) == 0; } }
		public bool MergesSimulationIslands
		{
			get
			{
				//static objects, kinematic and object without contact response don't merge islands
				return (_collisionFlags & (CollisionOptions.StaticObject | CollisionOptions.KinematicObject | CollisionOptions.NoContactResponse)) == 0;
			}
		}

		public ActivationState ActivationState
		{
			get { return _activationState; }
			set
			{
				if ((_activationState != ActivationState.DisableDeactivation) && (_activationState != ActivationState.DisableSimulation))
					_activationState = value;
			}
		}

		public bool IsActive { get { return ((ActivationState != ActivationState.IslandSleeping) && (ActivationState != ActivationState.DisableSimulation)); } }
		public float Restitution { get { return _restitution; } set { _restitution = value; } }
		public float Friction { get { return _friction; } set { _friction = value; } }
		public CollisionShape CollisionShape { get { return _collisionShape; } set { _collisionShape = value; } }
		public float DeactivationTime { get { return _deactivationTime; } set { _deactivationTime = value; } }
		public object Owner { get { return _internalOwner; } protected set { _internalOwner = value; } }
		public Matrix WorldTransform { get { return _worldTransform; } set { _worldTransform = value; } }
		public BroadphaseProxy Broadphase { get { return _broadphase; } set { _broadphase = value; } }
		public Matrix InterpolationWorldTransform { get { return _interpolationWorldTransform; } set { _interpolationWorldTransform = value; } }
		public Vector3 InterpolationLinearVelocity { get { return _interpolationLinearVelocity; } protected set { _interpolationLinearVelocity = value; } }
		public Vector3 InterpolationAngularVelocity { get { return _interpolationAngularVelocity; } protected set { _interpolationAngularVelocity = value; } }
		public int IslandTag { get { return _islandTag; } set { _islandTag = value; } }
		public float HitFraction { get { return _hitFraction; } set { _hitFraction = value; } }
		public CollisionOptions CollisionFlags { get { return _collisionFlags; } set { _collisionFlags = value; } }
		//Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm
		public float CcdSweptSphereRadius { get { return _ccdSweptSphereRadius; } set { _ccdSweptSphereRadius = value; } }
		// Don't do continuous collision detection if square motion (in one step) is less then m_ccdSquareMotionThreshold
		public float CcdSquareMotionThreshold { get { return _ccdSquareMotionThreshold; } set { _ccdSquareMotionThreshold = value; } }
		//users can point to their objects, userPointer is not used by Bullet
		public object UserData { get { return _userData; } set { _userData = value; } }
		public int CompanionID { get { return _companionID; } set { _companionID = value; } }

		public void ForceActivationState(ActivationState newState)
		{
			_activationState = newState;
		}

		public void Activate()
		{
			Activate(false);
		}

		public void Activate(bool forceActivation)
		{
			if (forceActivation || (_collisionFlags & (CollisionOptions.StaticObject | CollisionOptions.KinematicObject)) == 0)
			{
				ActivationState = ActivationState.Active;
				_deactivationTime = 0;
			}
		}
	}
}
