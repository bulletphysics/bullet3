#ifndef URDF_JOINT_TYPES_H
#define URDF_JOINT_TYPES_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

enum UrdfJointTypes
{
	URDFRevoluteJoint = 1,
	URDFPrismaticJoint,
	URDFContinuousJoint,
	URDFFloatingJoint,
	URDFPlanarJoint,
	URDFFixedJoint,
	URDFSphericalJoint,

};

enum URDF_LinkContactFlags
{
	URDF_CONTACT_HAS_LATERAL_FRICTION = 1,
	URDF_CONTACT_HAS_INERTIA_SCALING = 2,
	URDF_CONTACT_HAS_CONTACT_CFM = 4,
	URDF_CONTACT_HAS_CONTACT_ERP = 8,
	URDF_CONTACT_HAS_STIFFNESS_DAMPING = 16,
	URDF_CONTACT_HAS_ROLLING_FRICTION = 32,
	URDF_CONTACT_HAS_SPINNING_FRICTION = 64,
	URDF_CONTACT_HAS_RESTITUTION = 128,
	URDF_CONTACT_HAS_FRICTION_ANCHOR = 256,

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
		: m_lateralFriction(0.5),
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
	URDF_FORCE_CONCAVE_TRIMESH = 1,
	URDF_HAS_COLLISION_GROUP = 2,
	URDF_HAS_COLLISION_MASK = 4,
};

struct UrdfMaterialColor
{
	btVector4 m_rgbaColor;
	btVector3 m_specularColor;
	UrdfMaterialColor()
		: m_rgbaColor(0.8, 0.8, 0.8, 1),
		  m_specularColor(0.4, 0.4, 0.4)
	{
	}
};

//manually sync with eURDF_Flags in SharedMemoryPublic.h!
enum ConvertURDFFlags
{
	CUF_USE_SDF = 1,
	// Use inertia values in URDF instead of recomputing them from collision shape.
	CUF_USE_URDF_INERTIA = 2,
	CUF_USE_MJCF = 4,
	CUF_USE_SELF_COLLISION = 8,
	CUF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,
	CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,
	CUF_RESERVED = 64,
	CUF_USE_IMPLICIT_CYLINDER = 128,
	CUF_GLOBAL_VELOCITIES_MB = 256,
	CUF_MJCF_COLORS_FROM_FILE = 512,
	CUF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,
	CUF_ENABLE_SLEEPING = 2048,
	CUF_INITIALIZE_SAT_FEATURES = 4096,
	CUF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,
	CUF_PARSE_SENSORS = 16384,
	CUF_USE_MATERIAL_COLORS_FROM_MTL = 32768,
	CUF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,
	CUF_MAINTAIN_LINK_ORDER = 131072,
	CUF_ENABLE_WAKEUP = 1 << 18, 
	CUF_MERGE_FIXED_LINKS = 1 << 19,
	CUF_IGNORE_VISUAL_SHAPES = 1 << 20,
	CUF_IGNORE_COLLISION_SHAPES = 1 << 21,
	CUF_PRINT_URDF_INFO = 1 << 22,
	CUF_GOOGLEY_UNDEFINED_COLORS = 1 << 23,

};

#endif  //URDF_JOINT_TYPES_H
