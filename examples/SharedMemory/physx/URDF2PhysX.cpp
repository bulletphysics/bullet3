
#include "URDF2PhysX.h"

#include "PhysXUrdfImporter.h"
#include "Bullet3Common/b3Logging.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationJointReducedCoordinate.h"
#include "PxRigidActorExt.h"
#include "PxArticulation.h"
#include "PxRigidBodyExt.h"
#include "PxRigidBody.h"
#include "LinearMath/btThreads.h"
#include "PxRigidActorExt.h"
#include "PxArticulationBase.h"
#include "PxArticulationLink.h"
#include "PxMaterial.h"
#include "PxCooking.h"
#include "PxScene.h"
#include "PxRigidStatic.h"
#include "PxRigidDynamic.h"
#include "PxActor.h"
#include "PxAggregate.h"
#include "Bullet3Common/b3FileUtils.h"
#include "PhysXUserData.h"
#include "../../CommonInterfaces/CommonFileIOInterface.h"
#include "../../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../../Importers/ImportSTLDemo/LoadMeshFromSTL.h"
#include "../../Importers/ImportURDFDemo/UrdfParser.h"




struct URDF2PhysXCachedData
{
	URDF2PhysXCachedData()
		: m_currentMultiBodyLinkIndex(-1),
		m_articulation(0),
		m_rigidStatic(0),
		m_rigidDynamic(0),
		m_totalNumJoints1(0)
	{
	}
	//these arrays will be initialized in the 'InitURDF2BulletCache'

	btAlignedObjectArray<int> m_urdfLinkParentIndices;
	btAlignedObjectArray<int> m_urdfLinkIndices2BulletLinkIndices;
	btAlignedObjectArray<class physx::PxArticulationLink*> m_urdfLink2physxLink;
	btAlignedObjectArray<btTransform> m_urdfLinkLocalInertialFrames;

	int m_currentMultiBodyLinkIndex;

	physx::PxArticulationReducedCoordinate* m_articulation;
	physx::PxRigidStatic* m_rigidStatic;
	physx::PxRigidDynamic* m_rigidDynamic;

	btAlignedObjectArray<physx::PxTransform> m_linkTransWorldSpace;
	btAlignedObjectArray<int> m_urdfLinkIndex;
	btAlignedObjectArray<int> m_parentUrdfLinkIndex;
	btAlignedObjectArray<physx::PxReal> m_linkMasses;
	btAlignedObjectArray<physx::PxArticulationJointType::Enum> m_jointTypes;

	btAlignedObjectArray<physx::PxTransform> m_parentLocalPoses;
	btAlignedObjectArray<physx::PxTransform> m_childLocalPoses;
	btAlignedObjectArray<UrdfGeomTypes> m_geomTypes;
	btAlignedObjectArray<physx::PxVec3> m_geomDimensions;
	btAlignedObjectArray<physx::PxVec3> m_linkMaterials;
	btAlignedObjectArray<physx::PxTransform>  m_geomLocalPoses;
	


	//this will be initialized in the constructor
	int m_totalNumJoints1;
	int getParentUrdfIndex(int linkIndex) const
	{
		return m_urdfLinkParentIndices[linkIndex];
	}
	int getMbIndexFromUrdfIndex(int urdfIndex) const
	{
		if (urdfIndex == -2)
			return -2;
		return m_urdfLinkIndices2BulletLinkIndices[urdfIndex];
	}

	void registerMultiBody(int urdfLinkIndex, class physx::PxArticulationLink* body, const btTransform& worldTransform, btScalar mass, const btVector3& localInertiaDiagonal,  const btTransform& localInertialFrame)
	{
		m_urdfLink2physxLink[urdfLinkIndex] = body;
		m_urdfLinkLocalInertialFrames[urdfLinkIndex] = localInertialFrame;
	}

	class physx::PxArticulationLink* getPhysxLinkFromLink(int urdfLinkIndex)
	{
		return m_urdfLink2physxLink[urdfLinkIndex];
	}

	void registerRigidBody(int urdfLinkIndex, class physx::PxArticulationLink* body, const btTransform& worldTransform, btScalar mass, const btVector3& localInertiaDiagonal, const class btCollisionShape* compound, const btTransform& localInertialFrame)
	{
		btAssert(m_urdfLink2physxLink[urdfLinkIndex] == 0);

		m_urdfLink2physxLink[urdfLinkIndex] = body;
		m_urdfLinkLocalInertialFrames[urdfLinkIndex] = localInertialFrame;
	}
};


void ComputeParentIndices(const URDFImporterInterface& u2b, URDF2PhysXCachedData& cache, int urdfLinkIndex, int urdfParentIndex)
{
	cache.m_urdfLinkParentIndices[urdfLinkIndex] = urdfParentIndex;
	cache.m_urdfLinkIndices2BulletLinkIndices[urdfLinkIndex] = cache.m_currentMultiBodyLinkIndex++;

	btAlignedObjectArray<int> childIndices;
	u2b.getLinkChildIndices(urdfLinkIndex, childIndices);
	for (int i = 0; i < childIndices.size(); i++)
	{
		ComputeParentIndices(u2b, cache, childIndices[i], urdfLinkIndex);
	}
}

void ComputeTotalNumberOfJoints(const URDFImporterInterface& u2b, URDF2PhysXCachedData& cache, int linkIndex)
{
	btAlignedObjectArray<int> childIndices;
	u2b.getLinkChildIndices(linkIndex, childIndices);
	cache.m_totalNumJoints1 += childIndices.size();
	for (int i = 0; i < childIndices.size(); i++)
	{
		int childIndex = childIndices[i];
		ComputeTotalNumberOfJoints(u2b, cache, childIndex);
	}
}


void InitURDF2BulletCache(const URDFImporterInterface& u2b, URDF2PhysXCachedData& cache, int flags)
{
	//compute the number of links, and compute parent indices array (and possibly other cached data?)
	cache.m_totalNumJoints1 = 0;

	int rootLinkIndex = u2b.getRootLinkIndex();
	if (rootLinkIndex >= 0)
	{
		ComputeTotalNumberOfJoints(u2b, cache, rootLinkIndex);
		int numTotalLinksIncludingBase = 1 + cache.m_totalNumJoints1;

		cache.m_urdfLinkParentIndices.resize(numTotalLinksIncludingBase);
		cache.m_urdfLinkIndices2BulletLinkIndices.resize(numTotalLinksIncludingBase);
		cache.m_urdfLink2physxLink.resize(numTotalLinksIncludingBase);
		cache.m_urdfLinkLocalInertialFrames.resize(numTotalLinksIncludingBase);

		cache.m_currentMultiBodyLinkIndex = -1;  //multi body base has 'link' index -1

		bool maintainLinkOrder = (flags & CUF_MAINTAIN_LINK_ORDER) != 0;
		if (maintainLinkOrder)
		{
			URDF2PhysXCachedData cache2 = cache;

			ComputeParentIndices(u2b, cache2, rootLinkIndex, -2);

			for (int j = 0; j<numTotalLinksIncludingBase; j++)
			{
				cache.m_urdfLinkParentIndices[j] = cache2.m_urdfLinkParentIndices[j];
				cache.m_urdfLinkIndices2BulletLinkIndices[j] = j - 1;
			}
		}
		else
		{
			ComputeParentIndices(u2b, cache, rootLinkIndex, -2);
		}

	}
}


struct ArticulationCreationInterface
{
	physx::PxFoundation* m_foundation;
	physx::PxPhysics* m_physics;
	physx::PxCooking* m_cooking;
	physx::PxScene* m_scene;
	CommonFileIOInterface* m_fileIO;

	b3AlignedObjectArray<int> m_mb2urdfLink;
	void addLinkMapping(int urdfLinkIndex, int mbLinkIndex)
	{
		if (m_mb2urdfLink.size() < (mbLinkIndex + 1))
		{
			m_mb2urdfLink.resize((mbLinkIndex + 1), -2);
		}
		//    m_urdf2mbLink[urdfLinkIndex] = mbLinkIndex;
		m_mb2urdfLink[mbLinkIndex] = urdfLinkIndex;
	}
};

static btVector4 colors4[4] =
{
	btVector4(1, 0, 0, 1),
	btVector4(0, 1, 0, 1),
	btVector4(0, 1, 1, 1),
	btVector4(1, 1, 0, 1),
};

static btVector4 selectColor4()
{
	static btSpinMutex sMutex;
	sMutex.lock();
	static int curColor = 0;
	btVector4 color = colors4[curColor];
	curColor++;
	curColor &= 3;
	sMutex.unlock();
	return color;
}




static physx::PxConvexMesh* createPhysXConvex(physx::PxU32 numVerts, const physx::PxVec3* verts, ArticulationCreationInterface& creation)
{
	physx::PxCookingParams params = creation.m_cooking->getParams();

	// Use the new (default) PxConvexMeshCookingType::eQUICKHULL
	params.convexMeshCookingType = physx::PxConvexMeshCookingType::eQUICKHULL;

	// If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
	// If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
	int gaussMapLimit = 256;

	params.gaussMapLimit = gaussMapLimit;
	creation.m_cooking->setParams(params);

	// Setup the convex mesh descriptor
	physx::PxConvexMeshDesc desc;

	// We provide points only, therefore the PxConvexFlag::eCOMPUTE_CONVEX flag must be specified
	desc.points.data = verts;
	desc.points.count = numVerts;
	desc.points.stride = sizeof(physx::PxVec3);
	desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;

	physx::PxU32 meshSize = 0;

	// Directly insert mesh into PhysX
	physx::PxConvexMesh* convex = creation.m_cooking->createConvexMesh(desc, creation.m_physics->getPhysicsInsertionCallback());
	PX_ASSERT(convex);

	return convex;
}


int convertLinkPhysXShapes(const URDFImporterInterface& u2b, URDF2PhysXCachedData& cache, ArticulationCreationInterface& creation, int urdfLinkIndex, const char* pathPrefix, const btTransform& localInertiaFrame,
	physx::PxArticulationReducedCoordinate* articulation, int mbLinkIndex, physx::PxRigidActor* linkPtr)
{
	int numShapes = 0;

	URDFLinkContactInfo contactInfo;
	u2b.getLinkContactInfo(urdfLinkIndex, contactInfo);
	
	//static friction, dynamic frictoin, restitution
	cache.m_linkMaterials.push_back(physx::PxVec3(contactInfo.m_lateralFriction, contactInfo.m_lateralFriction, contactInfo.m_restitution));
	physx::PxMaterial* material = creation.m_physics->createMaterial(contactInfo.m_lateralFriction, contactInfo.m_lateralFriction, contactInfo.m_restitution);
	
	const UrdfLink* link = u2b.getUrdfLink(urdfLinkIndex);//.convertLinkCollisionShapes m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{

		for (int v = 0; v < link->m_collisionArray.size(); v++)
		{
			const UrdfCollision& col = link->m_collisionArray[v];
			const UrdfCollision* collision = &col;
			btTransform childTrans = col.m_linkLocalFrame;
			btTransform localTrans = localInertiaFrame.inverse()*childTrans;
			physx::PxTransform tr;
			tr.p = physx::PxVec3(localTrans.getOrigin()[0], localTrans.getOrigin()[1], localTrans.getOrigin()[2]);
			tr.q = physx::PxQuat(localTrans.getRotation()[0], localTrans.getRotation()[1], localTrans.getRotation()[2], localTrans.getRotation()[3]);

			physx::PxShape* shape = 0;
			cache.m_geomTypes.push_back(col.m_geometry.m_type);
			

			switch (col.m_geometry.m_type)
			{
			case URDF_GEOM_PLANE:
			{
				btVector3 planeNormal = col.m_geometry.m_planeNormal;
				btScalar planeConstant = 0;  //not available?
				
				btVector3 planeRefAxis(1, 0, 0);
				btQuaternion diffQuat = shortestArcQuat(planeRefAxis, planeNormal);
				shape = physx::PxRigidActorExt::createExclusiveShape(*linkPtr, physx::PxPlaneGeometry(), *material);
				
				btTransform diffTr;
				diffTr.setIdentity();
				diffTr.setRotation(diffQuat);
				btTransform localTrans = localInertiaFrame.inverse()*childTrans*diffTr;
				physx::PxTransform tr;
				tr.p = physx::PxVec3(localTrans.getOrigin()[0], localTrans.getOrigin()[1], localTrans.getOrigin()[2]);
				tr.q = physx::PxQuat(localTrans.getRotation()[0], localTrans.getRotation()[1], localTrans.getRotation()[2], localTrans.getRotation()[3]);

				physx::PxTransform localPose = tr;
				shape->setLocalPose(localPose);
				numShapes++;
				break;
			}
			case URDF_GEOM_CAPSULE:
			{
				btScalar radius = collision->m_geometry.m_capsuleRadius;
				btScalar height = collision->m_geometry.m_capsuleHeight;

				//static PxShape* createExclusiveShape(PxRigidActor& actor, const PxGeometry& geometry, PxMaterial*const* materials, PxU16 materialCount,
				//	PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE)
				//{
				physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE;
				
				//shape = PxGetPhysics().createShape(physx::PxCapsuleGeometry(radius, 0.5*height), &material, 1, false, shapeFlags);

				shape = physx::PxRigidActorExt::createExclusiveShape(*linkPtr, physx::PxCapsuleGeometry(radius, 0.5*height), *material);

				btTransform childTrans = col.m_linkLocalFrame;
				btTransform x2z;
				x2z.setIdentity();
				x2z.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));
				btTransform localTrans = localInertiaFrame.inverse()*childTrans*x2z;
				physx::PxTransform tr;
				tr.p = physx::PxVec3(localTrans.getOrigin()[0], localTrans.getOrigin()[1], localTrans.getOrigin()[2]);
				tr.q = physx::PxQuat(localTrans.getRotation()[0], localTrans.getRotation()[1], localTrans.getRotation()[2], localTrans.getRotation()[3]);

				shape->setLocalPose(tr);
				cache.m_geomLocalPoses.push_back(tr);
				numShapes++;
				//btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(radius, height);
				//shape = capsuleShape;
				//shape->setMargin(gUrdfDefaultCollisionMargin);
				break;
			}

			case URDF_GEOM_CYLINDER:
			{
				btScalar cylRadius = collision->m_geometry.m_capsuleRadius;
				btScalar cylHalfLength = 0.5 * collision->m_geometry.m_capsuleHeight;
				cache.m_geomDimensions.push_back(physx::PxVec3(cylRadius, cylHalfLength,0));
				//if (m_data->m_flags & CUF_USE_IMPLICIT_CYLINDER)
				//{
				//	btVector3 halfExtents(cylRadius, cylRadius, cylHalfLength);
				//	btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
				//	shape = cylZShape;
				//}
				//else
				{
					btAlignedObjectArray<physx::PxVec3> vertices;
					
					int numSteps = 32;
					for (int i = 0; i < numSteps; i++)
					{
						physx::PxVec3 vert(cylRadius * btSin(SIMD_2_PI * (float(i) / numSteps)), cylRadius * btCos(SIMD_2_PI * (float(i) / numSteps)), cylHalfLength);
						vertices.push_back(vert);
						vert[2] = -cylHalfLength;
						vertices.push_back(vert);
					}

					physx::PxConvexMesh* convexMesh = createPhysXConvex(vertices.size(), &vertices[0], creation);

					shape  = physx::PxRigidActorExt::createExclusiveShape(*linkPtr,
						physx::PxConvexMeshGeometry(convexMesh), *material);

					shape->setLocalPose(tr);
					cache.m_geomLocalPoses.push_back(tr);
					numShapes++;
					
				}
				break;
			}
			case URDF_GEOM_BOX:
			{
				btVector3 extents = collision->m_geometry.m_boxSize;
				shape = physx::PxRigidActorExt::createExclusiveShape(*linkPtr, physx::PxBoxGeometry(extents[0] * 0.5, extents[1] * 0.5, extents[2] * 0.5), *material);
				cache.m_geomDimensions.push_back(physx::PxVec3(extents[0]*0.5, extents[1] * 0.5, extents[2] * 0.5));
				shape->setLocalPose(tr);
				cache.m_geomLocalPoses.push_back(tr);
				numShapes++;
				break;
			}
			case URDF_GEOM_SPHERE:
			{
				btScalar radius = collision->m_geometry.m_sphereRadius;
				shape = physx::PxRigidActorExt::createExclusiveShape(*linkPtr, physx::PxSphereGeometry(radius), *material);
				cache.m_geomDimensions.push_back(physx::PxVec3(radius,0,0));
				shape->setLocalPose(tr);
				cache.m_geomLocalPoses.push_back(tr);
				numShapes++;
				break;
			}
			case URDF_GEOM_MESH:
			{
				btAlignedObjectArray<physx::PxVec3> vertices;
				GLInstanceGraphicsShape* glmesh = 0;
				switch (collision->m_geometry.m_meshFileType)
				{
					case UrdfGeometry::FILE_OBJ:
					{
						char relativeFileName[1024];
						char pathPrefix[1024];
						pathPrefix[0] = 0;
						if (creation.m_fileIO->findResourcePath(collision->m_geometry.m_meshFileName.c_str(), relativeFileName, 1024))
						{
							b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
						}
						glmesh = LoadMeshFromObj(collision->m_geometry.m_meshFileName.c_str(), pathPrefix, creation.m_fileIO);
						
						break;
					}
					case UrdfGeometry::FILE_STL:
					{
						glmesh = LoadMeshFromSTL(collision->m_geometry.m_meshFileName.c_str(), creation.m_fileIO);
						break;
					}
					default:
					{

					}
				}
				if (glmesh && glmesh->m_numvertices)
				{
					for (int i = 0; i < glmesh->m_numvertices; i++)
					{
						physx::PxVec3 vert(
							glmesh->m_vertices->at(i).xyzw[0] * collision->m_geometry.m_meshScale[0],
							glmesh->m_vertices->at(i).xyzw[1] * collision->m_geometry.m_meshScale[1],
							glmesh->m_vertices->at(i).xyzw[2] * collision->m_geometry.m_meshScale[2]);
						vertices.push_back(vert);
					}

					physx::PxConvexMesh* convexMesh = createPhysXConvex(vertices.size(), &vertices[0], creation);

					shape = physx::PxRigidActorExt::createExclusiveShape(*linkPtr,
						physx::PxConvexMeshGeometry(convexMesh), *material);

					shape->setLocalPose(tr);
					cache.m_geomLocalPoses.push_back(tr);
					numShapes++;
				}
				break;

			}
			default:
			{
				printf("unknown physx shape\n");
			}
			}

			if (shape)
			{
				//see https://github.com/NVIDIAGameWorks/PhysX/issues/21
				physx::PxReal contactOffset = shape->getContactOffset();
				physx::PxReal restOffset = shape->getContactOffset();

				//shape->setContactOffset(physx::PxReal(.03));
				//shape->setRestOffset(physx::PxReal(.01)); //
			}
		}

	}
	
	
	return numShapes;
}

btTransform ConvertURDF2PhysXInternal(
	const PhysXURDFImporter& u2b,
	ArticulationCreationInterface& creation,
	URDF2PhysXCachedData& cache, int urdfLinkIndex,
	const btTransform& parentTransformInWorldSpace, 
	bool createActiculation, const char* pathPrefix,
	int flags, 
	UrdfVisualShapeCache2* cachedLinkGraphicsShapesIn, 
	UrdfVisualShapeCache2* cachedLinkGraphicsShapesOut, 
	bool recursive)
{
	B3_PROFILE("ConvertURDF2PhysXInternal");
	//b3Printf("start converting/extracting data from URDF interface\n");

	btTransform linkTransformInWorldSpace;
	linkTransformInWorldSpace.setIdentity();

	int mbLinkIndex = cache.getMbIndexFromUrdfIndex(urdfLinkIndex);

	int urdfParentIndex = cache.getParentUrdfIndex(urdfLinkIndex);
	int mbParentIndex = cache.getMbIndexFromUrdfIndex(urdfParentIndex);
	
	

	//b3Printf("mb link index = %d\n",mbLinkIndex);

	btTransform parentLocalInertialFrame;
	parentLocalInertialFrame.setIdentity();
	btScalar parentMass(1);
	btVector3 parentLocalInertiaDiagonal(1, 1, 1);

	if (urdfParentIndex == -2)
	{
		//b3Printf("root link has no parent\n");
	}
	else
	{
		//b3Printf("urdf parent index = %d\n",urdfParentIndex);
		//b3Printf("mb parent index = %d\n",mbParentIndex);
		//parentRigidBody = cache.getRigidBodyFromLink(urdfParentIndex);
		u2b.getMassAndInertia2(urdfParentIndex, parentMass, parentLocalInertiaDiagonal, parentLocalInertialFrame, flags);
	}

	btScalar mass = 0;
	btTransform localInertialFrame;
	localInertialFrame.setIdentity();
	btVector3 localInertiaDiagonal(0, 0, 0);
	u2b.getMassAndInertia2(urdfLinkIndex, mass, localInertiaDiagonal, localInertialFrame, flags);

	btTransform parent2joint;
	parent2joint.setIdentity();

	int jointType;
	btVector3 jointAxisInJointSpace;
	btScalar jointLowerLimit;
	btScalar jointUpperLimit;
	btScalar jointDamping;
	btScalar jointFriction;
	btScalar jointMaxForce;
	btScalar jointMaxVelocity;

	bool hasParentJoint = u2b.getJointInfo2(urdfLinkIndex, parent2joint, linkTransformInWorldSpace, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction, jointMaxForce, jointMaxVelocity);

	btTransform axis2Reference;
	axis2Reference.setIdentity();

	switch (jointType)
	{
	case URDFContinuousJoint:
	case URDFPrismaticJoint:
	case URDFRevoluteJoint:
	{
		//rotate from revolute 'axis' to standard X axis
		btVector3 refAxis(1, 0, 0);
		btVector3 axis = jointAxisInJointSpace;
		//btQuaternion axis2ReferenceRot(btVector3(0, 1, 0), SIMD_HALF_PI);// = shortestArcQuat(refAxis, btVector3(axis[0], axis[1], axis[2]));
		btQuaternion axis2ReferenceRot = shortestArcQuat(refAxis, btVector3(axis[0], axis[1], axis[2]));
		axis2Reference.setRotation(axis2ReferenceRot);
		break;
	}
	default:
	{
	}
	};
	
	parent2joint = parent2joint*axis2Reference;
	//localInertialFrame = axis2Reference.inverse()*localInertialFrame;

	std::string linkName = u2b.getLinkName(urdfLinkIndex);

	if (flags & CUF_USE_SDF)
	{
		parent2joint = parentTransformInWorldSpace.inverse() * linkTransformInWorldSpace;
	}
	else
	{
		if (flags & CUF_USE_MJCF)
		{
			linkTransformInWorldSpace = parentTransformInWorldSpace * linkTransformInWorldSpace;
		}
		else
		{
			linkTransformInWorldSpace = parentTransformInWorldSpace * parent2joint;
		}
	}

	
	int graphicsIndex;
	{
		B3_PROFILE("convertLinkVisualShapes");


		
		graphicsIndex = u2b.convertLinkVisualShapes3(urdfLinkIndex, pathPrefix, localInertialFrame, u2b.getUrdfLink(urdfLinkIndex), u2b.getUrdfModel(), -1, u2b.getBodyUniqueId(), creation.m_fileIO);
		

#if 0
		if (cachedLinkGraphicsShapesIn && cachedLinkGraphicsShapesIn->m_cachedUrdfLinkVisualShapeIndices.size() > (mbLinkIndex + 1))
		{
			graphicsIndex = cachedLinkGraphicsShapesIn->m_cachedUrdfLinkVisualShapeIndices[mbLinkIndex + 1];
			UrdfMaterialColor matColor = cachedLinkGraphicsShapesIn->m_cachedUrdfLinkColors[mbLinkIndex + 1];
			u2b.setLinkColor2(urdfLinkIndex, matColor);
		}
		else
		{
			graphicsIndex = 
			if (cachedLinkGraphicsShapesOut)
			{
				cachedLinkGraphicsShapesOut->m_cachedUrdfLinkVisualShapeIndices.push_back(graphicsIndex);
				UrdfMaterialColor matColor;
				u2b.getLinkColor2(urdfLinkIndex, matColor);
				cachedLinkGraphicsShapesOut->m_cachedUrdfLinkColors.push_back(matColor);
			}
		}
#endif
	}
	
	if (1)
	{
		UrdfMaterialColor matColor;
		btVector4 color2 = selectColor4();
		btVector3 specular(0.5, 0.5, 0.5);
		if (u2b.getLinkColor2(urdfLinkIndex, matColor))
		{
			color2 = matColor.m_rgbaColor;
			specular = matColor.m_specularColor;
		}

		/*
		if (visual->material.get())
		{
		color.setValue(visual->material->color.r,visual->material->color.g,visual->material->color.b);//,visual->material->color.a);
		}
		*/
		if (mass)
		{
			if (!(flags & CUF_USE_URDF_INERTIA))
			{
				//b3Assert(0);
				//compoundShape->calculateLocalInertia(mass, localInertiaDiagonal);
				btAssert(localInertiaDiagonal[0] < 1e10);
				btAssert(localInertiaDiagonal[1] < 1e10);
				btAssert(localInertiaDiagonal[2] < 1e10);
			}
			URDFLinkContactInfo contactInfo;
			u2b.getLinkContactInfo(urdfLinkIndex, contactInfo);
			//temporary inertia scaling until we load inertia from URDF
			if (contactInfo.m_flags & URDF_CONTACT_HAS_INERTIA_SCALING)
			{
				localInertiaDiagonal *= contactInfo.m_inertiaScaling;
			}
		}

		
		btTransform inertialFrameInWorldSpace = linkTransformInWorldSpace * localInertialFrame;
		bool canSleep = (flags & CUF_ENABLE_SLEEPING) != 0;

		physx::PxRigidActor* linkPtr = 0;
		
		physx::PxRigidBody* rbLinkPtr = 0;
		

		physx::PxTransform tr;
		tr.p = physx::PxVec3(linkTransformInWorldSpace.getOrigin().x(), linkTransformInWorldSpace.getOrigin().y(), linkTransformInWorldSpace.getOrigin().z());
		tr.q = physx::PxQuat(linkTransformInWorldSpace.getRotation().x(), linkTransformInWorldSpace.getRotation().y(), linkTransformInWorldSpace.getRotation().z(), linkTransformInWorldSpace.getRotation().w());
		bool isFixedBase = (mass == 0);  //todo: figure out when base is fixed
		

		if (!createActiculation)
		{

			if (isFixedBase)
			{
				physx::PxRigidStatic* s = creation.m_physics->createRigidStatic(tr);
				if ((cache.m_rigidStatic == 0) && (cache.m_rigidDynamic == 0))
				{
					cache.m_rigidStatic = s;
				}
				linkPtr = s;
			}
			else
			{
				
				physx::PxRigidDynamic* d = creation.m_physics->createRigidDynamic(tr);
				linkPtr = d;
				if ((cache.m_rigidStatic == 0) && (cache.m_rigidDynamic == 0))
				{
					cache.m_rigidDynamic = d;
				}
				rbLinkPtr = d;
								
			}
		}
		else
		{
			
			cache.m_linkTransWorldSpace.push_back(tr);
			cache.m_urdfLinkIndex.push_back(urdfLinkIndex);
			cache.m_parentUrdfLinkIndex.push_back(urdfParentIndex);
			cache.m_linkMasses.push_back(mass);

			if (cache.m_articulation == 0)
			{

				

				cache.m_articulation = creation.m_physics->createArticulationReducedCoordinate();

				if (isFixedBase)
				{
					cache.m_articulation->setArticulationFlags(physx::PxArticulationFlag::eFIX_BASE);
				}



				physx::PxArticulationLink* base = cache.m_articulation->createLink(NULL,tr);
				
				linkPtr = base;
				rbLinkPtr = base;

				
				//physx::PxRigidActorExt::createExclusiveShape(*base, PxBoxGeometry(0.5f, 0.25f, 1.5f), *gMaterial);
				physx::PxRigidBody& body = *base;
				
				//Now create the slider and fixed joints...

				//cache.m_articulation->setSolverIterationCounts(4);//todo: API?
				cache.m_articulation->setSolverIterationCounts(32);//todo: API?
				
				cache.m_jointTypes.push_back(physx::PxArticulationJointType::eUNDEFINED);
				cache.m_parentLocalPoses.push_back(physx::PxTransform());
				cache.m_childLocalPoses.push_back(physx::PxTransform());

				// Stabilization can create artefacts on jointed objects so we just disable it
				//cache.m_articulation->setStabilizationThreshold(0.0f);
				//cache.m_articulation->setMaxProjectionIterations(16);
				//cache.m_articulation->setSeparationTolerance(0.001f);

#if 0
				int totalNumJoints = cache.m_totalNumJoints1;
				cache.m_bulletMultiBody = creation.allocateMultiBody(urdfLinkIndex, totalNumJoints, mass, localInertiaDiagonal, isFixedBase, canSleep);
				if (flags & CUF_GLOBAL_VELOCITIES_MB)
				{
					cache.m_bulletMultiBody->useGlobalVelocities(true);
				}
				if (flags & CUF_USE_MJCF)
				{
					cache.m_bulletMultiBody->setBaseWorldTransform(linkTransformInWorldSpace);
				}

				
#endif

				cache.registerMultiBody(urdfLinkIndex, base, inertialFrameInWorldSpace, mass, localInertiaDiagonal, localInertialFrame);
			}
			else
			{

				physx::PxArticulationLink* parentLink = cache.getPhysxLinkFromLink(urdfParentIndex);
				
				physx::PxArticulationLink* childLink = cache.m_articulation->createLink(parentLink, tr);
				linkPtr = childLink;
				rbLinkPtr = childLink;
				
				physx::PxArticulationJointReducedCoordinate* joint = static_cast<physx::PxArticulationJointReducedCoordinate*>(childLink->getInboundJoint());
			
				switch (jointType)
				{
					case URDFFixedJoint:
					{
						joint->setJointType(physx::PxArticulationJointType::eFIX);
						break;
					}
					case URDFSphericalJoint:
					{
						joint->setJointType(physx::PxArticulationJointType::eSPHERICAL);
						joint->setMotion(physx::PxArticulationAxis::eTWIST, physx::PxArticulationMotion::eFREE);
						joint->setMotion(physx::PxArticulationAxis::eSWING2, physx::PxArticulationMotion::eFREE);
						joint->setMotion(physx::PxArticulationAxis::eSWING1, physx::PxArticulationMotion::eFREE);
						break;
					}
					case URDFContinuousJoint:
					case URDFRevoluteJoint:
					{
						joint->setJointType(physx::PxArticulationJointType::eREVOLUTE);
						joint->setMotion(physx::PxArticulationAxis::eTWIST, physx::PxArticulationMotion::eFREE);
						
						break;
					}
					case URDFPrismaticJoint:
					{
						joint->setJointType(physx::PxArticulationJointType::ePRISMATIC);
						joint->setMotion(physx::PxArticulationAxis::eX, physx::PxArticulationMotion::eFREE);
						break;
					}
					default:
					{
						joint->setJointType(physx::PxArticulationJointType::eFIX);
						btAssert(0);
					}
				};
				
				cache.m_jointTypes.push_back(joint->getJointType());
				btTransform offsetInA, offsetInB;

				offsetInA = parentLocalInertialFrame.inverse() *   parent2joint;
				offsetInB = (axis2Reference.inverse()*localInertialFrame).inverse();

				physx::PxTransform parentPose(physx::PxVec3(offsetInA.getOrigin()[0], offsetInA.getOrigin()[1], offsetInA.getOrigin()[2]),
					physx::PxQuat(offsetInA.getRotation()[0], offsetInA.getRotation()[1], offsetInA.getRotation()[2], offsetInA.getRotation()[3]));

				physx::PxTransform childPose(physx::PxVec3(offsetInB.getOrigin()[0], offsetInB.getOrigin()[1], offsetInB.getOrigin()[2]),
					physx::PxQuat(offsetInB.getRotation()[0], offsetInB.getRotation()[1], offsetInB.getRotation()[2], offsetInB.getRotation()[3]));

				cache.m_parentLocalPoses.push_back(parentPose);
				cache.m_childLocalPoses.push_back(childPose);

				joint->setParentPose(parentPose);
				joint->setChildPose(childPose);
			
				cache.registerMultiBody(urdfLinkIndex, childLink, inertialFrameInWorldSpace, mass, localInertiaDiagonal, localInertialFrame);
			}
			
			
		}


		if (linkPtr)
		{
			//todo: mem leaks
			MyPhysXUserData* userData = new MyPhysXUserData();
			userData->m_graphicsUniqueId = graphicsIndex;
			userData->m_bodyUniqueId = u2b.getBodyUniqueId();
			userData->m_linkIndex = mbLinkIndex;
			linkPtr->userData = userData;
		}

		//create collision shapes

		//physx::PxRigidActorExt::createExclusiveShape
		convertLinkPhysXShapes(u2b, cache, creation, urdfLinkIndex, pathPrefix, localInertialFrame, cache.m_articulation, mbLinkIndex, linkPtr);

		
		if (rbLinkPtr && mass)
		{
			physx::PxRigidBodyExt::updateMassAndInertia(*rbLinkPtr, mass);
		}
		
		//base->setMass(massOut);
		//base->setMassSpaceInertiaTensor(diagTensor);
		//base->setCMassLocalPose(PxTransform(com, orient));


		//create a joint if necessary
		if (hasParentJoint)
		{
			btTransform offsetInA, offsetInB;
			offsetInA = parentLocalInertialFrame.inverse() * parent2joint;
			offsetInB = localInertialFrame.inverse();
			btQuaternion parentRotToThis = offsetInB.getRotation() * offsetInA.inverse().getRotation();

			bool disableParentCollision = true;

			if (createActiculation && cache.m_articulation)
			{
#if 0
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointDamping = jointDamping;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointFriction = jointFriction;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointLowerLimit = jointLowerLimit;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointUpperLimit = jointUpperLimit;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxForce = jointMaxForce;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxVelocity = jointMaxVelocity;
#endif
			}
		}


		if (createActiculation)
		{

		}
		else
		{
			
		}
	}//was if 'shape/compountShape'

	btAlignedObjectArray<int> urdfChildIndices;
	u2b.getLinkChildIndices(urdfLinkIndex, urdfChildIndices);

	int numChildren = urdfChildIndices.size();

	if (recursive)
	{
		for (int i = 0; i < numChildren; i++)
		{
			int urdfChildLinkIndex = urdfChildIndices[i];

			ConvertURDF2PhysXInternal(u2b, creation, cache, urdfChildLinkIndex, linkTransformInWorldSpace, createActiculation, pathPrefix, flags, cachedLinkGraphicsShapesIn, cachedLinkGraphicsShapesOut, recursive);
		}
	}
	return linkTransformInWorldSpace;
}







physx::PxBase* URDF2PhysX(physx::PxFoundation* foundation, physx::PxPhysics* physics, physx::PxCooking* cooking, physx::PxScene* scene, class PhysXURDFImporter& u2p, int flags, const char* pathPrefix, const btTransform& rootTransformInWorldSpace, struct CommonFileIOInterface* fileIO, bool createActiculation)
{
	URDF2PhysXCachedData cache;
	InitURDF2BulletCache(u2p, cache, flags);
	int urdfLinkIndex = u2p.getRootLinkIndex();
	int rootIndex = u2p.getRootLinkIndex();
	B3_PROFILE("ConvertURDF2Bullet");

	UrdfVisualShapeCache2 cachedLinkGraphicsShapesOut;
	UrdfVisualShapeCache2 cachedLinkGraphicsShapes;

	ArticulationCreationInterface creation;
	creation.m_foundation = foundation;
	creation.m_physics = physics;
	creation.m_cooking = cooking;
	creation.m_scene = scene;
	creation.m_fileIO = fileIO;

	

	bool recursive = (flags & CUF_MAINTAIN_LINK_ORDER) == 0;
	
	if (recursive)
	{
		ConvertURDF2PhysXInternal(u2p, creation, cache, urdfLinkIndex, rootTransformInWorldSpace, createActiculation, pathPrefix, flags, &cachedLinkGraphicsShapes, &cachedLinkGraphicsShapesOut, recursive);

	}
	else
	{
#if 0
		btAlignedObjectArray<btTransform> parentTransforms;
		if (urdfLinkIndex >= parentTransforms.size())
		{
			parentTransforms.resize(urdfLinkIndex + 1);
		}
		parentTransforms[urdfLinkIndex] = rootTransformInWorldSpace;
		btAlignedObjectArray<childParentIndex> allIndices;

		GetAllIndices(u2b, cache, urdfLinkIndex, -1, allIndices);
		allIndices.quickSort(MyIntCompareFunc);

		for (int i = 0; i < allIndices.size(); i++)
		{
			int urdfLinkIndex = allIndices[i].m_index;
			int parentIndex = allIndices[i].m_parentIndex;
			btTransform parentTr = parentIndex >= 0 ? parentTransforms[parentIndex] : rootTransformInWorldSpace;
			btTransform tr = ConvertURDF2BulletInternal(u2b, creation, cache, urdfLinkIndex, parentTr, world1, createActiculation, pathPrefix, flags, cachedLinkGraphicsShapes, &cachedLinkGraphicsShapesOut, recursive);
			if ((urdfLinkIndex + 1) >= parentTransforms.size())
			{
				parentTransforms.resize(urdfLinkIndex + 1);
			}
			parentTransforms[urdfLinkIndex] = tr;
		}
#endif

	}
#if 0
	if (cachedLinkGraphicsShapes && cachedLinkGraphicsShapesOut.m_cachedUrdfLinkVisualShapeIndices.size() > cachedLinkGraphicsShapes->m_cachedUrdfLinkVisualShapeIndices.size())
	{
		*cachedLinkGraphicsShapes = cachedLinkGraphicsShapesOut;
	}
#endif

	
	if (cache.m_articulation)
	{
#ifdef DEBUG_ARTICULATIONS
		printf("\n-----------------\n");
		printf("m_linkTransWorldSpace\n");
		for (int i = 0; i < cache.m_linkTransWorldSpace.size(); i++)
		{
			printf("PxTransform(PxVec3(%f,%f,%f), PxQuat(%f,%f,%f,%f),\n",
				cache.m_linkTransWorldSpace[i].p.x, cache.m_linkTransWorldSpace[i].p.y, cache.m_linkTransWorldSpace[i].p.z,
				cache.m_linkTransWorldSpace[i].q.x, cache.m_linkTransWorldSpace[i].q.y, cache.m_linkTransWorldSpace[i].q.z, cache.m_linkTransWorldSpace[i].q.w);
		}
		printf("m_parentLocalPoses\n");
		for (int i = 0; i < cache.m_parentLocalPoses.size(); i++)
		{
			printf("PxTransform(PxVec3(%f,%f,%f), PxQuat(%f,%f,%f,%f),\n",
				cache.m_parentLocalPoses[i].p.x, cache.m_parentLocalPoses[i].p.y, cache.m_parentLocalPoses[i].p.z,
				cache.m_parentLocalPoses[i].q.x, cache.m_parentLocalPoses[i].q.y, cache.m_parentLocalPoses[i].q.z, cache.m_parentLocalPoses[i].q.w);
		}

		printf("m_childLocalPoses\n");
		for (int i = 0; i < cache.m_childLocalPoses.size(); i++)
		{
			printf("PxTransform(PxVec3(%f,%f,%f), PxQuat(%f,%f,%f,%f),\n",
				cache.m_childLocalPoses[i].p.x, cache.m_childLocalPoses[i].p.y, cache.m_childLocalPoses[i].p.z,
				cache.m_childLocalPoses[i].q.x, cache.m_childLocalPoses[i].q.y, cache.m_childLocalPoses[i].q.z, cache.m_childLocalPoses[i].q.w);
		}

		printf("m_geomDimensions\n");
		for (int i = 0; i < cache.m_geomDimensions.size(); i++)
		{
			printf("PxVec3(%f,%f,%f),\n",
				cache.m_geomDimensions[i].x, cache.m_geomDimensions[i].y, cache.m_geomDimensions[i].z);
		}

		printf("m_geomLocalPoses\n");
		for (int i = 0; i < cache.m_geomLocalPoses.size(); i++)
		{
			printf("PxTransform(PxVec3(%f,%f,%f), PxQuat(%f,%f,%f,%f),\n",
				cache.m_geomLocalPoses[i].p.x, cache.m_geomLocalPoses[i].p.y, cache.m_geomLocalPoses[i].p.z,
				cache.m_geomLocalPoses[i].q.x, cache.m_geomLocalPoses[i].q.y, cache.m_geomLocalPoses[i].q.z, cache.m_geomLocalPoses[i].q.w);
		}

		printf("m_linkMaterials\n");
		for (int i = 0; i < cache.m_linkMaterials.size(); i++)
		{
			printf("PxVec3(%f,%f,%f),\n",
				cache.m_linkMaterials[i].x, cache.m_linkMaterials[i].y, cache.m_linkMaterials[i].z);
		}
#endif //DEBUG_ARTICULATIONS

		//see also https://github.com/NVIDIAGameWorks/PhysX/issues/43
		if ((flags & CUF_USE_SELF_COLLISION) == 0)
		{
			physx::PxU32 nbActors = cache.m_articulation->getNbLinks();;     // Max number of actors expected in the aggregate
			bool selfCollisions = false;
			physx::PxAggregate* aggregate = physics->createAggregate(nbActors, selfCollisions);
			aggregate->addArticulation(*cache.m_articulation);
			scene->addAggregate(*aggregate);
		}
		else
		{
			scene->addArticulation(*cache.m_articulation);
		}

		return  cache.m_articulation;
	}

	if (cache.m_rigidStatic)
	{
		
		scene->addActor(*cache.m_rigidStatic);
		return cache.m_rigidStatic;
	}
	if (cache.m_rigidDynamic)
	{
		scene->addActor(*cache.m_rigidDynamic);
		return cache.m_rigidDynamic;
	}
	return NULL;

}
