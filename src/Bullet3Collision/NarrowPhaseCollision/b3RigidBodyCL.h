#ifndef B3_RIGID_BODY_CL
#define B3_RIGID_BODY_CL

#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3Matrix3x3.h"

ATTRIBUTE_ALIGNED16(struct) b3RigidBodyCL
{
	B3_DECLARE_ALIGNED_ALLOCATOR();

	b3Vector3 		m_pos;
	b3Quaternion 	m_quat;
	b3Vector3			m_linVel;
	b3Vector3			m_angVel;

	int 					m_collidableIdx;
	float 				m_invMass;
	float 				m_restituitionCoeff;
	float 				m_frictionCoeff;

	float	getInvMass() const
	{
			return m_invMass;
	}
};


struct b3InertiaCL
{
	b3Matrix3x3 m_invInertiaWorld;
	b3Matrix3x3 m_initInvInertia;
};


#endif//B3_RIGID_BODY_CL
