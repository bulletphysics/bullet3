#ifndef BT_RIGID_BODY_CL
#define BT_RIGID_BODY_CL

#include "BulletCommon/btScalar.h"
#include "BulletCommon/btMatrix3x3.h"

ATTRIBUTE_ALIGNED16(struct) btRigidBodyCL
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btVector3 		m_pos;
	btQuaternion 	m_quat;
	btVector3			m_linVel;
	btVector3			m_angVel;

	int 					m_collidableIdx;
	float 				m_invMass;
	float 				m_restituitionCoeff;
	float 				m_frictionCoeff;

	float	getInvMass() const
	{
			return m_invMass;
	}
};


struct btInertiaCL
{
	btMatrix3x3 m_invInertiaWorld;
	btMatrix3x3 m_initInvInertia;
};


#endif//BT_RIGID_BODY_CL
