#ifndef URDF2PHYSX_H
#define URDF2PHYSX_H

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../../Importers/ImportURDFDemo/URDFJointTypes.h"

namespace physx
{
	class PxBase;
	class PxFoundation;
	class PxPhysics;
	class PxDefaultCpuDispatcher;
	class PxScene;
	class PxCooking;
	class PxArticulationReducedCoordinate;
};

struct UrdfVisualShapeCache2
{
	b3AlignedObjectArray<UrdfMaterialColor> m_cachedUrdfLinkColors;
	b3AlignedObjectArray<int> m_cachedUrdfLinkVisualShapeIndices;
};

physx::PxBase* URDF2PhysX(physx::PxFoundation* foundation, physx::PxPhysics* physics, physx::PxCooking* cooking, physx::PxScene* scene, class PhysXURDFImporter& u2p, int flags, const char* pathPrefix, const class btTransform& rootTransformInWorldSpace,struct CommonFileIOInterface* fileIO, bool createActiculation);

#endif //URDF2PHYSX_H