#ifndef _URDF2BULLET_H
#define _URDF2BULLET_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include <string>
#include "URDFJointTypes.h"//for UrdfMaterialColor cache

class btVector3;
class btTransform;
class btMultiBodyDynamicsWorld;
class btTransform;


class URDFImporterInterface;
class MultiBodyCreationInterface;


//manually sync with eURDF_Flags in SharedMemoryPublic.h!
enum ConvertURDFFlags {
  CUF_USE_SDF = 1,
  // Use inertia values in URDF instead of recomputing them from collision shape.
  CUF_USE_URDF_INERTIA = 2,
  CUF_USE_MJCF = 4,
  CUF_USE_SELF_COLLISION=8,
  CUF_USE_SELF_COLLISION_EXCLUDE_PARENT=16,
  CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS=32,
  CUF_RESERVED=64,
  CUF_USE_IMPLICIT_CYLINDER=128,
  CUF_GLOBAL_VELOCITIES_MB=256,
  CUF_MJCF_COLORS_FROM_FILE=512,
  CUF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,
  CUF_ENABLE_SLEEPING=2048,
  CUF_INITIALIZE_SAT_FEATURES=4096,
};

struct UrdfVisualShapeCache
{
	btAlignedObjectArray<UrdfMaterialColor> m_cachedUrdfLinkColors;
	btAlignedObjectArray<int> m_cachedUrdfLinkVisualShapeIndices;
};


void ConvertURDF2Bullet(const URDFImporterInterface& u2b,
			MultiBodyCreationInterface& creationCallback,
			const btTransform& rootTransformInWorldSpace,
			btMultiBodyDynamicsWorld* world,
			bool createMultiBody,
			const char* pathPrefix,
            int flags = 0,
			UrdfVisualShapeCache* cachedLinkGraphicsShapes= 0
			);


#endif //_URDF2BULLET_H

