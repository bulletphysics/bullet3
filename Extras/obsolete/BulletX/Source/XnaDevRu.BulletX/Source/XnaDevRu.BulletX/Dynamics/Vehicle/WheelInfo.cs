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
	public struct WheelInfoConstructionInfo
	{
		private Vector3 _chassicConnectionCS;
		private Vector3 _wheelDirectionCS;
		private Vector3 _wheelAxisCS;

		private Single _suspensionRestLength;
		private Single _maxSuspensionTravelCm;
		private Single _wheelRadius;
		private Single _suspensionStiffness;
		private Single _wheelsDampingCompression;
		private Single _wheelsDampingRelaxation;
		private Single _frictionSlip;

		private Boolean _isFrontWheel;

		#region Basic Properties
		public Vector3 ChassicConnectionCS
		{
			get { return _chassicConnectionCS; }
			set { _chassicConnectionCS = value; }
		}

		public Vector3 WheelDirectionCS
		{
			get { return _wheelDirectionCS; }
			set { _wheelDirectionCS = value; }
		}

		public Vector3 WheelAxleCS
		{
			get { return _wheelAxisCS; }
			set { _wheelAxisCS = value; }
		}


		public Single SuspensionRestLength
		{
			get { return _suspensionRestLength; }
			set { _suspensionRestLength = value; }
		}

		public Single MaxSuspensionTravelCm
		{
			get { return _maxSuspensionTravelCm; }
			set { _maxSuspensionTravelCm = value; }
		}

		public Single WheelRadius
		{
			get { return _wheelRadius; }
			set { _wheelRadius = value; }
		}


		public Single SuspensionStiffness
		{
			get { return _suspensionStiffness; }
			set { _suspensionStiffness = value; }
		}

		public Single WheelsDampingCompression
		{
			get { return _wheelsDampingCompression; }
			set { _wheelsDampingCompression = value; }
		}

		public Single WheelsDampingRelaxation
		{
			get { return _wheelsDampingRelaxation; }
			set { _wheelsDampingRelaxation = value; }
		}

		public Single FrictionSlip
		{
			get { return _frictionSlip; }
			set { _frictionSlip = value; }
		}


		public Boolean IsFrontWheel
		{
			get { return _isFrontWheel; }
			set { _isFrontWheel = value; }
		}
		#endregion
	}

	public struct RaycastInfo
	{
		private Vector3 _contractNormalWS;
		private Vector3 _contractPointWS;

		private Vector3 _hardPointWS;
		private Vector3 _wheelDirectionWS;
		private Vector3 _wheelAxleWS;

		private Single _suspensionLength;
		private Boolean _isInContract;

		#region Basic Properties
		public Single SuspensionLength
		{
			get { return _suspensionLength; }
			set { _suspensionLength = value; }
		}

		public Boolean IsInContact
		{
			get { return _isInContract; }
			set { _isInContract = value; }
		}

		public Vector3 ContactNormalWS
		{
			get { return _contractNormalWS; }
			set { _contractNormalWS = value; }
		}

		public Vector3 ContactPointWS
		{
			get { return _contractPointWS; }
			set { _contractPointWS = value; }
		}

		public Vector3 HardPointWS
		{
			get { return _hardPointWS; }
			set { _hardPointWS = value; }
		}

		public Vector3 WheelDirectionWS
		{
			get { return _wheelDirectionWS; }
			set { _wheelDirectionWS = value; }
		}

		public Vector3 WheelAxleWS
		{
			get { return _wheelAxleWS; }
			set { _wheelAxleWS = value; }
		}
		#endregion
	}

	public struct WheelInfo
	{
		private RaycastInfo _raycastInfo;

		private Matrix _worldTransform;

		private Vector3 _chassicConnectionPointCS;
		private Vector3 _wheelDirectionCS;
		private Vector3 _wheelAxleCS;

		private Single _suspensionRestLength;
		private Single _maxSuspensionTravelCm;

		private Single _wheelsRadius;
		private Single _rollInfluence;
		private Single _suspensionStiffness;
		private Single _wheelsDampingCompression;
		private Single _wheelsDampingRelaxation;
		private Single _frictionSlip;
		private Single _steering;
		private Single _rotation;
		private Single _deltaRotation;

		private Single _engineForce;
		private Single _brake;
		private Boolean _isFrontWheel;


		private Single _clippedInvContactDotSuspension;
		private Single _skidInfo;
		private Single _wheelsSuspensionForce;
		private Single _suspensionRelativeVelocity;
		//can be used to store pointer to sync transforms...
		private object _clientInfo;

		#region Constructor
		public WheelInfo(WheelInfoConstructionInfo constructionInfo)
		{
			_suspensionRestLength = constructionInfo.SuspensionRestLength;
			_maxSuspensionTravelCm = constructionInfo.MaxSuspensionTravelCm;

			_wheelsRadius = constructionInfo.WheelRadius;
			_wheelsDampingCompression = constructionInfo.WheelsDampingCompression;
			_wheelsDampingRelaxation = constructionInfo.WheelsDampingRelaxation;
			_wheelDirectionCS = constructionInfo.WheelDirectionCS;

			_suspensionStiffness = constructionInfo.SuspensionStiffness;
			_chassicConnectionPointCS = constructionInfo.ChassicConnectionCS;

			_wheelAxleCS = constructionInfo.WheelAxleCS;
			_frictionSlip = constructionInfo.FrictionSlip;

			_clippedInvContactDotSuspension = 0;
			_suspensionRelativeVelocity = 0;
			_wheelsSuspensionForce = 0;
			_skidInfo = 0;

			_steering = 0;
			_engineForce = 0;
			_rotation = 0;
			_rotation = 0;
			_deltaRotation = 0;
			_brake = 0;
			_rollInfluence = 0.1f;
			_brake = 0;
			_rollInfluence = 0.1f;

			_isFrontWheel = constructionInfo.IsFrontWheel;

			_raycastInfo = default(RaycastInfo);
			_worldTransform = default(Matrix);
			_clientInfo = null;
		}
		#endregion

		#region BasicProperties
		public object ClientInfo { get { return _clientInfo; } set { _clientInfo = value; } }

		public RaycastInfo RaycastInfo
		{
			get { return _raycastInfo; }
			set { _raycastInfo = value; }
		}

		public Matrix WorldTransform
		{
			get { return _worldTransform; }
			set { _worldTransform = value; }
		}

		public Vector3 ChassicConnectionPointCS
		{
			get { return _chassicConnectionPointCS; }
			set { _chassicConnectionPointCS = value; }
		}
		public Vector3 WheelDirectionCS
		{
			get { return _wheelDirectionCS; }
			set { _wheelDirectionCS = value; }
		}
		public Vector3 WheelAxleCS
		{
			get { return _wheelAxleCS; }
			set { _wheelAxleCS = value; }
		}

		public Single SuspensionRestLength
		{
			get { return _suspensionRestLength; }
			set { _suspensionRestLength = value; }
		}


		public Single MaxSuspensionTravelCm
		{
			get { return _maxSuspensionTravelCm; }
			set { _maxSuspensionTravelCm = value; }
		}

		public Single WheelsRadius
		{
			get { return _wheelsRadius; }
			set { _wheelsRadius = value; }
		}

		public Single SuspensionStiffness
		{
			get { return _suspensionStiffness; }
			set { _suspensionStiffness = value; }
		}

		public Single WheelsDampingCompression
		{
			get { return _wheelsDampingCompression; }
			set { _wheelsDampingCompression = value; }
		}

		public Single WheelsDampingRelaxation
		{
			get { return _wheelsDampingRelaxation; }
			set { _wheelsDampingRelaxation = value; }
		}

		public Single FrictionSlip
		{
			get { return _frictionSlip; }
			set { _frictionSlip = value; }
		}

		public Single Steering
		{
			get { return _steering; }
			set { _steering = value; }
		}

		public Single Rotation
		{
			get { return _rotation; }
			set { _rotation = value; }
		}

		public Single DeltaRotation
		{
			get { return _deltaRotation; }
			set { _deltaRotation = value; }
		}

		public Single RollInfluence
		{
			get { return _rollInfluence; }
			set { _rollInfluence = value; }
		}

		public Single EngineForce
		{
			get { return _engineForce; }
			set { _engineForce = value; }
		}

		public Single Brake
		{
			get { return _brake; }
			set { _brake = value; }
		}

		public Boolean IsFrontWheel
		{
			get { return _isFrontWheel; }
			set { _isFrontWheel = value; }
		}

		public Single ClippedInvContactDotSuspension
		{
			get { return _clippedInvContactDotSuspension; }
			set { _clippedInvContactDotSuspension = value; }
		}

		public Single SuspensionRelativeVelocity
		{
			get { return _suspensionRelativeVelocity; }
			set { _suspensionRelativeVelocity = value; }
		}

		public Single WheelsSuspensionForce
		{
			get { return _wheelsSuspensionForce; }
			set { _wheelsSuspensionForce = value; }
		}

		public Single SkidInfo
		{
			get { return _skidInfo; }
			set { _skidInfo = value; }
		}
		#endregion

		/// <summary>
		/// 
		/// </summary>
		/// <param name="chassis"></param>
		/// <param name="paramRaycastInfo">Not used!</param>
		public void UpdateWheel(RigidBody chassis, RaycastInfo paramRaycastInfo)
		{
			if (_raycastInfo.IsInContact)
			{
				float project = Vector3.Dot(_raycastInfo.ContactNormalWS, _raycastInfo.WheelDirectionWS);

				Vector3 chassisVelocityAtContactPoint = new Vector3();
				Vector3 relpos = _raycastInfo.ContactPointWS - chassis.CenterOfMassPosition;
				chassisVelocityAtContactPoint = chassis.GetVelocityInLocalPoint(relpos);
				float projVel = Vector3.Dot(_raycastInfo.ContactNormalWS, chassisVelocityAtContactPoint);

				if (project >= -0.1f)
				{
					_suspensionRelativeVelocity = 0;
					_clippedInvContactDotSuspension = 1.0f / 0.1f;
				}
				else
				{
					float inv = -1 / project;
					_suspensionRelativeVelocity = projVel * inv;
					_clippedInvContactDotSuspension = inv;
				}
			}
			else
			{
				_raycastInfo.SuspensionLength = _suspensionRestLength;
				_suspensionRelativeVelocity = 0.0f;
				_raycastInfo.ContactNormalWS = -_raycastInfo.WheelDirectionWS;
				_clippedInvContactDotSuspension = 1.0f;
			}
		}

		//        if (m_raycastInfo.m_isInContact)

		//{
		//    btScalar	project= m_raycastInfo.m_contactNormalWS.dot( m_raycastInfo.m_wheelDirectionWS );
		//    btVector3	 chassis_velocity_at_contactPoint;
		//    btVector3 relpos = m_raycastInfo.m_contactPointWS - chassis.getCenterOfMassPosition();
		//    chassis_velocity_at_contactPoint = chassis.getVelocityInLocalPoint( relpos );
		//    btScalar projVel = m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );
		//    if ( project >= -0.1f)
		//    {
		//        m_suspensionRelativeVelocity = 0.0f;
		//        m_clippedInvContactDotSuspension = 1.0f / 0.1f;
		//    }
		//    else
		//    {
		//        btScalar inv = -1.f / project;
		//        m_suspensionRelativeVelocity = projVel * inv;
		//        m_clippedInvContactDotSuspension = inv;
		//    }

		//}

		//else	// Not in contact : position wheel in a nice (rest length) position
		//{
		//    m_raycastInfo.m_suspensionLength = this->getSuspensionRestLength();
		//    m_suspensionRelativeVelocity = 0.0f;
		//    m_raycastInfo.m_contactNormalWS = -m_raycastInfo.m_wheelDirectionWS;
		//    m_clippedInvContactDotSuspension = 1.0f;
		//}
	};

	//btScalar	m_clippedInvContactDotSuspension;
	//btScalar	m_suspensionRelativeVelocity;
	//btScalar	m_wheelsSuspensionForce;
	//btScalar	m_skidInfo;

	//void*		m_clientInfo;//can be used to store pointer to sync transforms...

	//btWheelInfo(btWheelInfoConstructionInfo& ci)

	//{

	//    m_suspensionRestLength1 = ci.m_suspensionRestLength;
	//    m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

	//    m_wheelsRadius = ci.m_wheelRadius;
	//    m_suspensionStiffness = ci.m_suspensionStiffness;
	//    m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
	//    m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
	//    m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
	//    m_wheelDirectionCS = ci.m_wheelDirectionCS;
	//    m_wheelAxleCS = ci.m_wheelAxleCS;
	//    m_frictionSlip = ci.m_frictionSlip;
	//    m_steering = 0.f;
	//    m_engineForce = 0.f;
	//    m_rotation = 0.f;
	//    m_deltaRotation = 0.f;
	//    m_brake = 0.f;
	//    m_rollInfluence = 0.1f;
	//    m_bIsFrontWheel = ci.m_bIsFrontWheel;

	//}

	//void	updateWheel(const btRigidBody& chassis,RaycastInfo& raycastInfo);

	//btScalar	m_clippedInvContactDotSuspension;
	//btScalar	m_suspensionRelativeVelocity;
	////calculated by suspension
	//btScalar	m_wheelsSuspensionForce;
	//btScalar	m_skidInfo;

	//};  

	//struct RaycastInfo
	//{
	//    //set by raycaster
	//    btVector3 m_contactNormalWS;//contactnormal
	//    btVector3 m_contactPointWS;//raycast hitpoint
	//    btScalar m_suspensionLength;
	//    btVector3 m_hardPointWS;//raycast starting point
	//    btVector3 m_wheelDirectionWS; //direction in worldspace
	//    btVector3 m_wheelAxleWS; // axle in worldspace
	//    bool m_isInContact;
	//    void* m_groundObject; //could be general void* ptr
	//};

	//struct btWheelInfoConstructionInfo
	//{
	//    btVector3 m_chassisConnectionCS;
	//    btVector3 m_wheelDirectionCS;
	//    btVector3 m_wheelAxleCS;
	//    btScalar m_suspensionRestLength;
	//    btScalar m_maxSuspensionTravelCm;
	//    btScalar m_wheelRadius;

	//    float m_suspensionStiffness;
	//    float m_wheelsDampingCompression;
	//    float m_wheelsDampingRelaxation;
	//    float m_frictionSlip;
	//    bool m_bIsFrontWheel;

	//};
}
