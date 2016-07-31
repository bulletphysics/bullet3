#ifndef URDF_JOINT_TYPES_H
#define URDF_JOINT_TYPES_H


enum UrdfJointTypes
{
		URDFRevoluteJoint=1,
		URDFPrismaticJoint,
		URDFContinuousJoint,
		URDFFloatingJoint,
		URDFPlanarJoint,
		URDFFixedJoint,
};
#include "LinearMath/btScalar.h"

enum URDF_LinkContactFlags
{
	URDF_CONTACT_HAS_LATERAL_FRICTION=1,
	URDF_CONTACT_HAS_ROLLING_FRICTION=2,
    URDF_CONTACT_HAS_INERTIA_SCALING=2,
    URDF_CONTACT_HAS_CONTACT_CFM=4,
	URDF_CONTACT_HAS_CONTACT_ERP=8
};

struct URDFLinkContactInfo
{
	btScalar m_lateralFriction;
	btScalar m_rollingFriction;
    btScalar m_inertiaScaling;
	btScalar m_contactCfm;
	btScalar m_contactErp;
	int m_flags;

	URDFLinkContactInfo()
		:m_lateralFriction(0.5),
		m_rollingFriction(0),
        m_inertiaScaling(1),
		m_contactCfm(0),
		m_contactErp(0)
	{
		m_flags = URDF_CONTACT_HAS_LATERAL_FRICTION;
	}
};


#endif //URDF_JOINT_TYPES_H
