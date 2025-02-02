
#include <stdio.h>
#include "LinearMath/btTransform.h"
#include "LinearMath/btThreads.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodySphericalJointLimit.h"


#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "URDF2Bullet.h"
#include "URDFImporterInterface.h"
#include "MultiBodyCreationInterface.h"
#include <string>
#include "Bullet3Common/b3Logging.h"

//static int bodyCollisionFilterGroup=btBroadphaseProxy::CharacterFilter;
//static int bodyCollisionFilterMask=btBroadphaseProxy::AllFilter&(~btBroadphaseProxy::CharacterFilter);
static bool enableConstraints = true;

static btVector4 gGoogleyColors[4] =
{
		btVector4(60. / 256., 186. / 256., 84. / 256., 1),
		btVector4(244. / 256., 194. / 256., 13. / 256., 1),
		btVector4(219. / 256., 50. / 256., 54. / 256., 1),
		btVector4(72. / 256., 133. / 256., 237. / 256., 1),
};

static btVector4 selectColor2()
{
#ifdef BT_THREADSAFE
	static btSpinMutex sMutex;
	sMutex.lock();
#endif
	static int curColor = 0;
	btVector4 color = gGoogleyColors[curColor];
	curColor++;
	curColor &= 3;
#ifdef BT_THREADSAFE
	sMutex.unlock();
#endif
	return color;
}

struct URDF2BulletCachedData
{
	URDF2BulletCachedData()
		: m_currentMultiBodyLinkIndex(-1),
		  m_bulletMultiBody(0),
		  m_totalNumJoints1(0)
	{
	}
	//these arrays will be initialized in the 'InitURDF2BulletCache'

	btAlignedObjectArray<int> m_urdfLinkParentIndices;
	btAlignedObjectArray<int> m_urdfLinkIndices2BulletLinkIndices;
	btAlignedObjectArray<class btRigidBody*> m_urdfLink2rigidBodies;
	btAlignedObjectArray<btTransform> m_urdfLinkLocalInertialFrames;

	int m_currentMultiBodyLinkIndex;

	class btMultiBody* m_bulletMultiBody;

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

	void registerMultiBody(int urdfLinkIndex, class btMultiBody* /*body*/, const btTransform& /*worldTransform*/, btScalar /*mass*/, const btVector3& /*localInertiaDiagonal*/, const class btCollisionShape* /*compound*/, const btTransform& localInertialFrame)
	{
		m_urdfLinkLocalInertialFrames[urdfLinkIndex] = localInertialFrame;
	}

	class btRigidBody* getRigidBodyFromLink(int urdfLinkIndex)
	{
		return m_urdfLink2rigidBodies[urdfLinkIndex];
	}

	void registerRigidBody(int urdfLinkIndex, class btRigidBody* body, const btTransform& /*worldTransform*/, btScalar /*mass*/, const btVector3& /*localInertiaDiagonal*/, const class btCollisionShape* /*compound*/, const btTransform& localInertialFrame)
	{
		btAssert(m_urdfLink2rigidBodies[urdfLinkIndex] == 0);

		m_urdfLink2rigidBodies[urdfLinkIndex] = body;
		m_urdfLinkLocalInertialFrames[urdfLinkIndex] = localInertialFrame;
	}
};

void ComputeTotalNumberOfJoints(const URDFImporterInterface& u2b, URDF2BulletCachedData& cache, int linkIndex)
{
	btAlignedObjectArray<int> childIndices;
	u2b.getLinkChildIndices(linkIndex, childIndices);
	//b3Printf("link %s has %d children\n", u2b.getLinkName(linkIndex).c_str(),childIndices.size());
	//for (int i=0;i<childIndices.size();i++)
	//{
	//    b3Printf("child %d has childIndex%d=%s\n",i,childIndices[i],u2b.getLinkName(childIndices[i]).c_str());
	//}
	cache.m_totalNumJoints1 += childIndices.size();
	for (int i = 0; i < childIndices.size(); i++)
	{
		int childIndex = childIndices[i];
		ComputeTotalNumberOfJoints(u2b, cache, childIndex);
	}
}


void ComputeParentIndices(const URDFImporterInterface& u2b, URDF2BulletCachedData& cache, int urdfLinkIndex, int urdfParentIndex)
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

void InitURDF2BulletCache(const URDFImporterInterface& u2b, URDF2BulletCachedData& cache, int flags)
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
		cache.m_urdfLink2rigidBodies.resize(numTotalLinksIncludingBase);
		cache.m_urdfLinkLocalInertialFrames.resize(numTotalLinksIncludingBase);

		cache.m_currentMultiBodyLinkIndex = -1;  //multi body base has 'link' index -1
		
		bool maintainLinkOrder  = (flags & CUF_MAINTAIN_LINK_ORDER)!=0;
		if (maintainLinkOrder)
		{
			URDF2BulletCachedData cache2 = cache;

			ComputeParentIndices(u2b, cache2, rootLinkIndex, -2);

			for (int j=0;j<numTotalLinksIncludingBase;j++)
			{
				cache.m_urdfLinkParentIndices[j] = cache2.m_urdfLinkParentIndices[j];
				cache.m_urdfLinkIndices2BulletLinkIndices[j] = j - 1;
			}
		}else
		{
			ComputeParentIndices(u2b, cache, rootLinkIndex, -2);
		}

	}
}

void processContactParameters(const URDFLinkContactInfo& contactInfo, btCollisionObject* col)
{
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_LATERAL_FRICTION) != 0)
	{
		col->setFriction(contactInfo.m_lateralFriction);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_RESTITUTION) != 0)
	{
		col->setRestitution(contactInfo.m_restitution);
	}

	if ((contactInfo.m_flags & URDF_CONTACT_HAS_ROLLING_FRICTION) != 0)
	{
		col->setRollingFriction(contactInfo.m_rollingFriction);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_SPINNING_FRICTION) != 0)
	{
		col->setSpinningFriction(contactInfo.m_spinningFriction);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_STIFFNESS_DAMPING) != 0)
	{
		col->setContactStiffnessAndDamping(contactInfo.m_contactStiffness, contactInfo.m_contactDamping);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_FRICTION_ANCHOR) != 0)
	{
		col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);
	}
}

btScalar tmpUrdfScaling = 2;

btTransform ConvertURDF2BulletInternal(
	const URDFImporterInterface& u2b, MultiBodyCreationInterface& creation,
	URDF2BulletCachedData& cache, int urdfLinkIndex,
	const btTransform& parentTransformInWorldSpace, 
	
#ifdef USE_DISCRETE_DYNAMICS_WORLD
	btDiscreteDynamicsWorld* world1,
#else
	btMultiBodyDynamicsWorld* world1,
#endif


	bool createMultiBody, const char* pathPrefix,
	int flags, UrdfVisualShapeCache* cachedLinkGraphicsShapesIn, UrdfVisualShapeCache* cachedLinkGraphicsShapesOut, bool recursive)
{
	B3_PROFILE("ConvertURDF2BulletInternal2");
	//b3Printf("start converting/extracting data from URDF interface\n");

	btTransform linkTransformInWorldSpace;
	linkTransformInWorldSpace.setIdentity();

	int mbLinkIndex = cache.getMbIndexFromUrdfIndex(urdfLinkIndex);

	int urdfParentIndex = cache.getParentUrdfIndex(urdfLinkIndex);
	int mbParentIndex = cache.getMbIndexFromUrdfIndex(urdfParentIndex);
	btRigidBody* parentRigidBody = 0;

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
		parentRigidBody = cache.getRigidBodyFromLink(urdfParentIndex);
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
	btScalar twistLimit;
	bool hasParentJoint = u2b.getJointInfo3(urdfLinkIndex, parent2joint, linkTransformInWorldSpace, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction, jointMaxForce, jointMaxVelocity, twistLimit);
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

	btCompoundShape* tmpShape = u2b.convertLinkCollisionShapes(urdfLinkIndex, pathPrefix, localInertialFrame);
	btCollisionShape* compoundShape = tmpShape;
	if (tmpShape->getNumChildShapes() == 1 && tmpShape->getChildTransform(0) == btTransform::getIdentity())
	{
		compoundShape = tmpShape->getChildShape(0);
	}

	int graphicsIndex;
	{
		B3_PROFILE("convertLinkVisualShapes");
		if (cachedLinkGraphicsShapesIn && cachedLinkGraphicsShapesIn->m_cachedUrdfLinkVisualShapeIndices.size() > (mbLinkIndex + 1))
		{
			graphicsIndex = cachedLinkGraphicsShapesIn->m_cachedUrdfLinkVisualShapeIndices[mbLinkIndex + 1];
			UrdfMaterialColor matColor = cachedLinkGraphicsShapesIn->m_cachedUrdfLinkColors[mbLinkIndex + 1];
			u2b.setLinkColor2(urdfLinkIndex, matColor);
		}
		else
		{
			graphicsIndex = u2b.convertLinkVisualShapes(urdfLinkIndex, pathPrefix, localInertialFrame);
			if (cachedLinkGraphicsShapesOut)
			{
				cachedLinkGraphicsShapesOut->m_cachedUrdfLinkVisualShapeIndices.push_back(graphicsIndex);
				UrdfMaterialColor matColor;
				u2b.getLinkColor2(urdfLinkIndex, matColor);
				cachedLinkGraphicsShapesOut->m_cachedUrdfLinkColors.push_back(matColor);
			}
		}
	}

	if (compoundShape)
	{
		UrdfMaterialColor matColor;
		
		btVector4 color2 = (flags & CUF_GOOGLEY_UNDEFINED_COLORS) ? selectColor2() : btVector4(1, 1, 1, 1);
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
				compoundShape->calculateLocalInertia(mass, localInertiaDiagonal);
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

		btRigidBody* linkRigidBody = 0;
		btTransform inertialFrameInWorldSpace = linkTransformInWorldSpace * localInertialFrame;
		bool canSleep = (flags & CUF_ENABLE_SLEEPING) != 0;

		if (!createMultiBody)
		{
			btRigidBody* body = creation.allocateRigidBody(urdfLinkIndex, mass, localInertiaDiagonal, inertialFrameInWorldSpace, compoundShape);

			if (!canSleep)
			{
				body->forceActivationState(DISABLE_DEACTIVATION);
			}

			linkRigidBody = body;

			world1->addRigidBody(body);

			compoundShape->setUserIndex(graphicsIndex);

			URDFLinkContactInfo contactInfo;
			u2b.getLinkContactInfo(urdfLinkIndex, contactInfo);

			processContactParameters(contactInfo, body);
			creation.createRigidBodyGraphicsInstance2(urdfLinkIndex, body, color2, specular, graphicsIndex);
			cache.registerRigidBody(urdfLinkIndex, body, inertialFrameInWorldSpace, mass, localInertiaDiagonal, compoundShape, localInertialFrame);

			//untested: u2b.convertLinkVisualShapes2(linkIndex,urdfLinkIndex,pathPrefix,localInertialFrame,body);
		}
		else
		{
			if (cache.m_bulletMultiBody == 0)
			{
				bool isFixedBase = (mass == 0);  //todo: figure out when base is fixed
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

				cache.registerMultiBody(urdfLinkIndex, cache.m_bulletMultiBody, inertialFrameInWorldSpace, mass, localInertiaDiagonal, compoundShape, localInertialFrame);
			}
		}

		//create a joint if necessary
		if (hasParentJoint)
		{
			btTransform offsetInA, offsetInB;
			offsetInA = parentLocalInertialFrame.inverse() * parent2joint;
			offsetInB = localInertialFrame.inverse();
			btQuaternion parentRotToThis = offsetInB.getRotation() * offsetInA.inverse().getRotation();

			bool disableParentCollision = true;

			if (createMultiBody && cache.m_bulletMultiBody)
			{
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointDamping = jointDamping;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointFriction = jointFriction;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointLowerLimit = jointLowerLimit;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointUpperLimit = jointUpperLimit;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxForce = jointMaxForce;
				cache.m_bulletMultiBody->getLink(mbLinkIndex).m_jointMaxVelocity = jointMaxVelocity;
			}

			switch (jointType)
			{
				case URDFSphericalJoint:
				{
					if (createMultiBody && cache.m_bulletMultiBody)
					{
						creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);
						cache.m_bulletMultiBody->setupSpherical(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
							parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin(),
							disableParentCollision);


						//create a spherical joint limit, swing_x,. swing_y and twist
						//jointLowerLimit <= jointUpperLimit)
						if (jointUpperLimit > 0 && jointLowerLimit> 0 && twistLimit > 0 && jointMaxForce>0)
						{
							btMultiBodySphericalJointLimit* con = new btMultiBodySphericalJointLimit(cache.m_bulletMultiBody, mbLinkIndex, 
								jointLowerLimit,
								jointUpperLimit,
								twistLimit,
								jointMaxForce);
							world1->addMultiBodyConstraint(con);
						}

					}
					else
					{
						btAssert(0);
					}
					break;
				}
				case URDFPlanarJoint:
				{
					
					if (createMultiBody && cache.m_bulletMultiBody)
					{
#if 0
						void setupPlanar(int i,  // 0 to num_links-1
							btScalar mass,
							const btVector3 &inertia,
							int parent,
							const btQuaternion &rotParentToThis,  // rotate points in parent frame to this frame, when q = 0
							const btVector3 &rotationAxis,
							const btVector3 &parentComToThisComOffset,  // vector from parent COM to this COM, in PARENT frame
							bool disableParentCollision = false);
#endif
						creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);
						cache.m_bulletMultiBody->setupPlanar(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
							parentRotToThis, quatRotate(offsetInB.getRotation(), jointAxisInJointSpace), offsetInA.getOrigin(),
							disableParentCollision);
					}
					else
					{
#if 0
						//b3Printf("Fixed joint\n");

						btGeneric6DofSpring2Constraint* dof6 = 0;

						//backward compatibility
						if (flags & CUF_RESERVED)
						{
							dof6 = creation.createFixedJoint(urdfLinkIndex, *parentRigidBody, *linkRigidBody, offsetInA, offsetInB);
						}
						else
						{
							dof6 = creation.createFixedJoint(urdfLinkIndex, *linkRigidBody, *parentRigidBody, offsetInB, offsetInA);
						}
						if (enableConstraints)
							world1->addConstraint(dof6, true);
#endif
					}
					break;
				}
				case URDFFloatingJoint:
				
				case URDFFixedJoint:
				{
					if ((jointType == URDFFloatingJoint) || (jointType == URDFPlanarJoint))
					{
						printf("Warning: joint unsupported, creating a fixed joint instead.");
					}
					creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

					if (createMultiBody)
					{
						//todo: adjust the center of mass transform and pivot axis properly
						cache.m_bulletMultiBody->setupFixed(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
															parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin());
					}
					else
					{
						//b3Printf("Fixed joint\n");

						btGeneric6DofSpring2Constraint* dof6 = 0;

						//backward compatibility
						if (flags & CUF_RESERVED)
						{
							dof6 = creation.createFixedJoint(urdfLinkIndex, *parentRigidBody, *linkRigidBody, offsetInA, offsetInB);
						}
						else
						{
							dof6 = creation.createFixedJoint(urdfLinkIndex, *linkRigidBody, *parentRigidBody, offsetInB, offsetInA);
						}
						if (enableConstraints)
							world1->addConstraint(dof6, true);
					}
					break;
				}
				case URDFContinuousJoint:
				case URDFRevoluteJoint:
				{
					creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);
					if (createMultiBody)
					{
#ifndef USE_DISCRETE_DYNAMICS_WORLD
						cache.m_bulletMultiBody->setupRevolute(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
															   parentRotToThis, quatRotate(offsetInB.getRotation(), 
																   jointAxisInJointSpace), 
																offsetInA.getOrigin(),
															   -offsetInB.getOrigin(),
															   disableParentCollision);

						if (jointType == URDFRevoluteJoint && jointLowerLimit <= jointUpperLimit)
						{
							//std::string name = u2b.getLinkName(urdfLinkIndex);
							//printf("create btMultiBodyJointLimitConstraint for revolute link name=%s urdf link index=%d (low=%f, up=%f)\n", name.c_str(), urdfLinkIndex, jointLowerLimit, jointUpperLimit);
							btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(cache.m_bulletMultiBody, mbLinkIndex, jointLowerLimit, jointUpperLimit);
							world1->addMultiBodyConstraint(con);
						}
#endif
					}
					else

					{
						btGeneric6DofSpring2Constraint* dof6 = 0;
						if (jointType == URDFRevoluteJoint && jointLowerLimit <= jointUpperLimit)
						{
							//backwards compatibility
							if (flags & CUF_RESERVED)
							{
								dof6 = creation.createRevoluteJoint(urdfLinkIndex, *parentRigidBody, *linkRigidBody, offsetInA, offsetInB, jointAxisInJointSpace, jointLowerLimit, jointUpperLimit);
							}
							else
							{
								dof6 = creation.createRevoluteJoint(urdfLinkIndex, *linkRigidBody, *parentRigidBody, offsetInB, offsetInA, jointAxisInJointSpace, jointLowerLimit, jointUpperLimit);
							}
						}
						else
						{
							//disable joint limits
							if (flags & CUF_RESERVED)
							{
								dof6 = creation.createRevoluteJoint(urdfLinkIndex, *parentRigidBody, *linkRigidBody, offsetInA, offsetInB, jointAxisInJointSpace, 1, -1);
							}
							else
							{
								dof6 = creation.createRevoluteJoint(urdfLinkIndex, *linkRigidBody, *parentRigidBody, offsetInB, offsetInA, jointAxisInJointSpace, 1, -1);
							}
						}

						if (enableConstraints)
							world1->addConstraint(dof6, true);
						//b3Printf("Revolute/Continuous joint\n");
					}
					break;
				}
				case URDFPrismaticJoint:
				{
					creation.addLinkMapping(urdfLinkIndex, mbLinkIndex);

					if (createMultiBody && cache.m_bulletMultiBody)
					{
#ifndef USE_DISCRETE_DYNAMICS_WORLD
						cache.m_bulletMultiBody->setupPrismatic(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
																parentRotToThis, quatRotate(offsetInB.getRotation(), jointAxisInJointSpace), offsetInA.getOrigin(),  //parent2joint.getOrigin(),
																-offsetInB.getOrigin(),
																disableParentCollision);

						if (jointLowerLimit <= jointUpperLimit)
						{
							//std::string name = u2b.getLinkName(urdfLinkIndex);
							//printf("create btMultiBodyJointLimitConstraint for prismatic link name=%s urdf link index=%d (low=%f, up=%f)\n", name.c_str(), urdfLinkIndex, jointLowerLimit,jointUpperLimit);

							btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(cache.m_bulletMultiBody, mbLinkIndex, jointLowerLimit, jointUpperLimit);
							world1->addMultiBodyConstraint(con);
						}
						//printf("joint lower limit=%d, upper limit = %f\n", jointLowerLimit, jointUpperLimit);
#endif
					}
					else
					{
						btGeneric6DofSpring2Constraint* dof6 = creation.createPrismaticJoint(urdfLinkIndex, *parentRigidBody, *linkRigidBody, offsetInA, offsetInB, jointAxisInJointSpace, jointLowerLimit, jointUpperLimit);

						if (enableConstraints)
							world1->addConstraint(dof6, true);

						//b3Printf("Prismatic\n");
					}
					break;
				}
				default:
				{
					//b3Printf("Error: unsupported joint type in URDF (%d)\n", jointType);
					btAssert(0);
				}
			}
		}

		if (createMultiBody)
		{
			//if (compoundShape->getNumChildShapes()>0)
			{
				btMultiBodyLinkCollider* col = creation.allocateMultiBodyLinkCollider(urdfLinkIndex, mbLinkIndex, cache.m_bulletMultiBody);

				compoundShape->setUserIndex(graphicsIndex);

				col->setCollisionShape(compoundShape);

				if (compoundShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btBvhTriangleMeshShape* trimeshShape = (btBvhTriangleMeshShape*)compoundShape;
					if (trimeshShape->getTriangleInfoMap())
					{
						col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					}
				}

				if (compoundShape->getShapeType() == TERRAIN_SHAPE_PROXYTYPE)
				{
					col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
				}

				btTransform tr;
				tr.setIdentity();
				tr = linkTransformInWorldSpace;
				//if we don't set the initial pose of the btCollisionObject, the simulator will do this
				//when syncing the btMultiBody link transforms to the btMultiBodyLinkCollider

				col->setWorldTransform(tr);

				//base and fixed? -> static, otherwise flag as dynamic
				bool isDynamic = (mbLinkIndex < 0 && cache.m_bulletMultiBody->hasFixedBase()) ? false : true;
				int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
				int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

				int colGroup = 0, colMask = 0;
				int collisionFlags = u2b.getCollisionGroupAndMask(urdfLinkIndex, colGroup, colMask);
				if (collisionFlags & URDF_HAS_COLLISION_GROUP)
				{
					collisionFilterGroup = colGroup;
				}
				if (collisionFlags & URDF_HAS_COLLISION_MASK)
				{
					collisionFilterMask = colMask;
				}
				world1->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);

				btVector4 color2L = (flags & CUF_GOOGLEY_UNDEFINED_COLORS) ? selectColor2() : btVector4(1, 1, 1, 1);
				btVector3 specularColor(1, 1, 1);
				UrdfMaterialColor matCol;
				if (u2b.getLinkColor2(urdfLinkIndex, matCol))
				{
					color2L = matCol.m_rgbaColor;
					specularColor = matCol.m_specularColor;
				}
				{
					B3_PROFILE("createCollisionObjectGraphicsInstance2");
					creation.createCollisionObjectGraphicsInstance2(urdfLinkIndex, col, color2L, specularColor);
				}
				{
					B3_PROFILE("convertLinkVisualShapes2");
					u2b.convertLinkVisualShapes2(mbLinkIndex, urdfLinkIndex, pathPrefix, localInertialFrame, col, u2b.getBodyUniqueId());
				}
				URDFLinkContactInfo contactInfo;
				u2b.getLinkContactInfo(urdfLinkIndex, contactInfo);

				processContactParameters(contactInfo, col);

				if (mbLinkIndex >= 0)  //???? double-check +/- 1
				{
					//if the base is static and all joints in the chain between this link and the base are fixed, 
					//then this link is static too (doesn't merge islands)
					if (cache.m_bulletMultiBody->getBaseMass() == 0)
					{
						bool allJointsFixed = true;
						int testLinkIndex = mbLinkIndex;
						do
						{
							if (cache.m_bulletMultiBody->getLink(testLinkIndex).m_jointType != btMultibodyLink::eFixed)
							{
								allJointsFixed = false;
								break;
							}
							testLinkIndex = cache.m_bulletMultiBody->getLink(testLinkIndex).m_parent;
						} while (testLinkIndex> 0);
						if (allJointsFixed)
						{
							col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
						}

					}
					cache.m_bulletMultiBody->getLink(mbLinkIndex).m_collider = col;
					if (flags & CUF_USE_SELF_COLLISION_INCLUDE_PARENT)
					{
						cache.m_bulletMultiBody->getLink(mbLinkIndex).m_flags &= ~BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;
					}
					if (flags & CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
					{
						cache.m_bulletMultiBody->getLink(mbLinkIndex).m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_ALL_PARENT_COLLISION;
					}
				}
				else
				{
					//					if (canSleep)
					{
						if (cache.m_bulletMultiBody->getBaseMass() == 0)
						//&& cache.m_bulletMultiBody->getNumDofs()==0)
						{
							//col->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
							col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
						}
					}

					cache.m_bulletMultiBody->setBaseCollider(col);
				}
			}
		}
		else
		{
			/* int mbLinkIndex =*/ cache.getMbIndexFromUrdfIndex(urdfLinkIndex);
			//u2b.convertLinkVisualShapes2(mbLinkIndex, urdfLinkIndex, pathPrefix, localInertialFrame, col, u2b.getBodyUniqueId());
			u2b.convertLinkVisualShapes2(-1, urdfLinkIndex, pathPrefix, localInertialFrame, linkRigidBody, u2b.getBodyUniqueId());
		}
	}

	btAlignedObjectArray<int> urdfChildIndices;
	u2b.getLinkChildIndices(urdfLinkIndex, urdfChildIndices);

	int numChildren = urdfChildIndices.size();

	if (recursive)
	{
		for (int i = 0; i < numChildren; i++)
		{
			int urdfChildLinkIndex = urdfChildIndices[i];

			ConvertURDF2BulletInternal(u2b, creation, cache, urdfChildLinkIndex, linkTransformInWorldSpace, world1, createMultiBody, pathPrefix, flags, cachedLinkGraphicsShapesIn, cachedLinkGraphicsShapesOut, recursive);
		}
	}
	return linkTransformInWorldSpace;
}

struct childParentIndex
{
	int m_index;
	int m_mbIndex;
	int m_parentIndex;
	int m_parentMBIndex;
	
};

void GetAllIndices(const URDFImporterInterface& u2b, URDF2BulletCachedData& cache, int urdfLinkIndex, int parentIndex, btAlignedObjectArray<childParentIndex>& allIndices)
{
	childParentIndex cp;
	cp.m_index = urdfLinkIndex;
	int mbIndex = cache.getMbIndexFromUrdfIndex(urdfLinkIndex);
	cp.m_mbIndex = mbIndex;
	cp.m_parentIndex = parentIndex;
	int parentMbIndex = parentIndex>=0? cache.getMbIndexFromUrdfIndex(parentIndex) : -1;
	cp.m_parentMBIndex = parentMbIndex;

	allIndices.push_back(cp);
	btAlignedObjectArray<int> urdfChildIndices;
	u2b.getLinkChildIndices(urdfLinkIndex, urdfChildIndices);
	int numChildren = urdfChildIndices.size();
	for (int i = 0; i < numChildren; i++)
	{
		int urdfChildLinkIndex = urdfChildIndices[i];
		GetAllIndices(u2b, cache, urdfChildLinkIndex, urdfLinkIndex, allIndices);
	}
}


bool MyIntCompareFunc(childParentIndex a, childParentIndex b)
{
	return (a.m_index < b.m_index);
}


	


void ConvertURDF2Bullet(
	const URDFImporterInterface& u2b, MultiBodyCreationInterface& creation,
	const btTransform& rootTransformInWorldSpace,

#ifdef USE_DISCRETE_DYNAMICS_WORLD
	btDiscreteDynamicsWorld* world1,
#else
	btMultiBodyDynamicsWorld* world1,
#endif
	bool createMultiBody, const char* pathPrefix, int flags, UrdfVisualShapeCache* cachedLinkGraphicsShapes)
{
	URDF2BulletCachedData cache;
	InitURDF2BulletCache(u2b, cache, flags);
	int urdfLinkIndex = u2b.getRootLinkIndex();
	// int rootIndex = u2b.getRootLinkIndex();
	B3_PROFILE("ConvertURDF2Bullet");

	UrdfVisualShapeCache cachedLinkGraphicsShapesOut;


	bool recursive = (flags & CUF_MAINTAIN_LINK_ORDER)==0;
	if (recursive)
	{
		ConvertURDF2BulletInternal(u2b, creation, cache, urdfLinkIndex, rootTransformInWorldSpace, world1, createMultiBody, pathPrefix, flags, cachedLinkGraphicsShapes, &cachedLinkGraphicsShapesOut, recursive);
	}
	else
	{
		
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
			int urdfLinkIndexL = allIndices[i].m_index;
			int parentIndex = allIndices[i].m_parentIndex;
			btTransform parentTr = parentIndex >= 0 ? parentTransforms[parentIndex] : rootTransformInWorldSpace;
			btTransform tr = ConvertURDF2BulletInternal(u2b, creation, cache, urdfLinkIndexL, parentTr , world1, createMultiBody, pathPrefix, flags, cachedLinkGraphicsShapes, &cachedLinkGraphicsShapesOut, recursive);
			if ((urdfLinkIndexL+1) >= parentTransforms.size())
			{
				parentTransforms.resize(urdfLinkIndexL + 1);
			}
			parentTransforms[urdfLinkIndexL] = tr;
		}
		


	}
	if (cachedLinkGraphicsShapes && cachedLinkGraphicsShapesOut.m_cachedUrdfLinkVisualShapeIndices.size() > cachedLinkGraphicsShapes->m_cachedUrdfLinkVisualShapeIndices.size())
	{
		*cachedLinkGraphicsShapes = cachedLinkGraphicsShapesOut;
	}
#ifndef USE_DISCRETE_DYNAMICS_WORLD
	if (world1 && cache.m_bulletMultiBody)
	{
		B3_PROFILE("Post process");
		btMultiBody* mb = cache.m_bulletMultiBody;

		mb->setHasSelfCollision((flags & CUF_USE_SELF_COLLISION) != 0);

		mb->finalizeMultiDof();

		btTransform localInertialFrameRoot = cache.m_urdfLinkLocalInertialFrames[urdfLinkIndex];

		if (flags & CUF_USE_MJCF)
		{
		}
		else
		{
			mb->setBaseWorldTransform(rootTransformInWorldSpace * localInertialFrameRoot);
		}
		btAlignedObjectArray<btQuaternion> scratch_q;
		btAlignedObjectArray<btVector3> scratch_m;
		mb->forwardKinematics(scratch_q, scratch_m);
		mb->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

		world1->addMultiBody(mb);
	}
	#endif
}
