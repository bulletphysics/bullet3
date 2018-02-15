/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btRigidBody.h" // for pointvelocity


btScalar btWheelInfo::getSuspensionRestLength() const
{

	return m_suspensionRestLength1;

}

void	btWheelInfo::updateWheel(const btRigidBody& chassis,RaycastInfo& raycastInfo)
{
	(void)raycastInfo;

	
	if (m_raycastInfo.m_isInContact)

	{
		btScalar	project= m_raycastInfo.m_contactNormalWS.dot( m_raycastInfo.m_wheelDirectionWS );
		btVector3	 chassis_velocity_at_contactPoint;
		btVector3 relpos = m_raycastInfo.m_contactPointWS - chassis.getCenterOfMassPosition();
		chassis_velocity_at_contactPoint = chassis.getVelocityInLocalPoint( relpos );
		btScalar projVel = m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );
		if ( project >= btScalar(-0.1))
		{
			m_suspensionRelativeVelocity = btScalar(0.0);
			m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
		}
		else
		{
			btScalar inv = btScalar(-1.) / project;
			m_suspensionRelativeVelocity = projVel * inv;
			m_clippedInvContactDotSuspension = inv;
		}
		
	}

	else	// Not in contact : position wheel in a nice (rest length) position
	{
		m_raycastInfo.m_suspensionLength = this->getSuspensionRestLength();
		m_suspensionRelativeVelocity = btScalar(0.0);
		m_raycastInfo.m_contactNormalWS = -m_raycastInfo.m_wheelDirectionWS;
		m_clippedInvContactDotSuspension = btScalar(1.0);
	}
}


const char * btWheelInfo::serialize (void * dataBuffer, btSerializer * serializer) const
{
	btWheelDoubleData * data = reinterpret_cast<btWheelDoubleData*>(dataBuffer);
	m_chassisConnectionPointCS.serializeDouble(data->m_chassisConnectionCS);
	m_wheelDirectionCS.serializeDouble(data->m_wheelDirectionCS);
	m_wheelAxleCS.serializeDouble(data->m_wheelAxleCS);
	data->m_suspensionRestLength = m_suspensionRestLength1;
	data->m_maxSuspensionTravelCm = m_maxSuspensionTravelCm;
	data->m_wheelRadius = m_wheelsRadius;
	data->m_suspensionStiffness = m_suspensionStiffness;
	data->m_wheelsDampingCompression = m_wheelsDampingCompression;
	data->m_wheelsDampingRelaxation = m_wheelsDampingRelaxation;
	data->m_frictionSlip = m_frictionSlip;
	data->m_maxSuspensionForce = m_maxSuspensionForce;
	data->m_bIsFrontWheel = m_bIsFrontWheel;

	// other parameters
	data->m_steering = m_steering;
	data->m_engineForce = m_engineForce;
	data->m_rotation = m_rotation;
	data->m_deltaRotation = m_deltaRotation;
	data->m_brake = m_brake;
	data->m_rollInfluence = m_rollInfluence;
	return "btWheelDoubleData";
}
