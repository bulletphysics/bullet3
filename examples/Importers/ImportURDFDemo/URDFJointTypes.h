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
    URDF_CONTACT_HAS_INERTIA_SCALING=2,
    URDF_CONTACT_HAS_CONTACT_CFM=4,
	URDF_CONTACT_HAS_CONTACT_ERP=8,
	URDF_CONTACT_HAS_STIFFNESS_DAMPING=16,
    URDF_CONTACT_HAS_ROLLING_FRICTION=32,
    URDF_CONTACT_HAS_SPINNING_FRICTION=64,
	URDF_CONTACT_HAS_RESTITUTION=128,

};

struct URDFLinkContactInfo
{
	btScalar m_lateralFriction;
	btScalar m_rollingFriction;
    btScalar m_spinningFriction;
	btScalar m_restitution;
    btScalar m_inertiaScaling;
	btScalar m_contactCfm;
	btScalar m_contactErp;
	btScalar m_contactStiffness;
	btScalar m_contactDamping;
	
	int m_flags;

	URDFLinkContactInfo()
		:m_lateralFriction(0.5),
		m_rollingFriction(0),
        m_spinningFriction(0),
		m_restitution(0),
        m_inertiaScaling(1),
		m_contactCfm(0),
		m_contactErp(0),
        m_contactStiffness(1e4),
        m_contactDamping(1)
	{
		m_flags = URDF_CONTACT_HAS_LATERAL_FRICTION;
	}
};

enum UrdfCollisionFlags
{
	URDF_FORCE_CONCAVE_TRIMESH=1,
	URDF_HAS_COLLISION_GROUP=2,
	URDF_HAS_COLLISION_MASK=4,
};

#endif //URDF_JOINT_TYPES_H
