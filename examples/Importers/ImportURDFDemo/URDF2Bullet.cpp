#include "URDFImporterInterface.h"
#include <stdio.h>
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "URDFImporterInterface.h"
#include "MultiBodyCreationInterface.h"
#include <string>

static int bodyCollisionFilterGroup=btBroadphaseProxy::CharacterFilter;
static int bodyCollisionFilterMask=btBroadphaseProxy::AllFilter&(~btBroadphaseProxy::CharacterFilter);
static bool enableConstraints = true;

static btVector4 colors[4] =
{
    btVector4(1,0,0,1),
    btVector4(0,1,0,1),
    btVector4(0,1,1,1),
    btVector4(1,1,0,1),
};


static btVector4 selectColor2()
{

    static int curColor = 0;
    btVector4 color = colors[curColor];
    curColor++;
    curColor&=3;
    return color;
}



struct URDF2BulletCachedData
{
    URDF2BulletCachedData()
    :
	m_currentMultiBodyLinkIndex(-1),
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
        if (urdfIndex==-2)
            return -2;
        return m_urdfLinkIndices2BulletLinkIndices[urdfIndex];
    }


    void registerMultiBody( int urdfLinkIndex, class btMultiBody* body, const btTransform& worldTransform, btScalar mass, const btVector3& localInertiaDiagonal, const class btCompoundShape* compound, const btTransform& localInertialFrame)
    {
        m_urdfLinkLocalInertialFrames[urdfLinkIndex] = localInertialFrame;
    }

    class btRigidBody* getRigidBodyFromLink(int urdfLinkIndex)
    {
        return m_urdfLink2rigidBodies[urdfLinkIndex];
    }

    void registerRigidBody( int urdfLinkIndex, class btRigidBody* body, const btTransform& worldTransform, btScalar mass, const btVector3& localInertiaDiagonal, const class btCompoundShape* compound, const btTransform& localInertialFrame)
    {
        btAssert(m_urdfLink2rigidBodies[urdfLinkIndex]==0);

        m_urdfLink2rigidBodies[urdfLinkIndex] = body;
        m_urdfLinkLocalInertialFrames[urdfLinkIndex] = localInertialFrame;
    }

};

void ComputeTotalNumberOfJoints(const URDFImporterInterface& u2b, URDF2BulletCachedData& cache, int linkIndex)
{
    btAlignedObjectArray<int> childIndices;
    u2b.getLinkChildIndices(linkIndex,childIndices);
    //b3Printf("link %s has %d children\n", u2b.getLinkName(linkIndex).c_str(),childIndices.size());
    //for (int i=0;i<childIndices.size();i++)
    //{
    //    b3Printf("child %d has childIndex%d=%s\n",i,childIndices[i],u2b.getLinkName(childIndices[i]).c_str());
    //}
    cache.m_totalNumJoints1 += childIndices.size();
    for (int i=0;i<childIndices.size();i++)
    {
        int childIndex =childIndices[i];
        ComputeTotalNumberOfJoints(u2b,cache,childIndex);
    }
}

void ComputeParentIndices(const URDFImporterInterface& u2b, URDF2BulletCachedData& cache, int urdfLinkIndex, int urdfParentIndex)
{
    cache.m_urdfLinkParentIndices[urdfLinkIndex]=urdfParentIndex;
    cache.m_urdfLinkIndices2BulletLinkIndices[urdfLinkIndex]=cache.m_currentMultiBodyLinkIndex++;

    btAlignedObjectArray<int> childIndices;
    u2b.getLinkChildIndices(urdfLinkIndex,childIndices);
    for (int i=0;i<childIndices.size();i++)
    {
        ComputeParentIndices(u2b,cache,childIndices[i],urdfLinkIndex);
    }
}

void InitURDF2BulletCache(const URDFImporterInterface& u2b, URDF2BulletCachedData& cache)
{
    //compute the number of links, and compute parent indices array (and possibly other cached data?)
    cache.m_totalNumJoints1 = 0;

    int rootLinkIndex = u2b.getRootLinkIndex();
    if (rootLinkIndex>=0)
    {
        ComputeTotalNumberOfJoints(u2b,cache,rootLinkIndex);
        int numTotalLinksIncludingBase = 1+cache.m_totalNumJoints1;

        cache.m_urdfLinkParentIndices.resize(numTotalLinksIncludingBase);
        cache.m_urdfLinkIndices2BulletLinkIndices.resize(numTotalLinksIncludingBase);
        cache.m_urdfLink2rigidBodies.resize(numTotalLinksIncludingBase);
		cache.m_urdfLinkLocalInertialFrames.resize(numTotalLinksIncludingBase);

        cache.m_currentMultiBodyLinkIndex = -1;//multi body base has 'link' index -1
        ComputeParentIndices(u2b,cache,rootLinkIndex,-2);
    }

}

void ConvertURDF2BulletInternal(const URDFImporterInterface& u2b, MultiBodyCreationInterface& creation, URDF2BulletCachedData& cache, int urdfLinkIndex, const btTransform& parentTransformInWorldSpace, btMultiBodyDynamicsWorld* world1,bool createMultiBody, const char* pathPrefix)
{
    //b3Printf("start converting/extracting data from URDF interface\n");

    btTransform linkTransformInWorldSpace;
    linkTransformInWorldSpace.setIdentity();


    int mbLinkIndex =cache.getMbIndexFromUrdfIndex(urdfLinkIndex);

    int urdfParentIndex =   cache.getParentUrdfIndex(urdfLinkIndex);
    int mbParentIndex = cache.getMbIndexFromUrdfIndex(urdfParentIndex);
    btRigidBody* parentRigidBody = 0;

    //std::string name = u2b.getLinkName(urdfLinkIndex);
    //b3Printf("link name=%s urdf link index=%d\n",name.c_str(),urdfLinkIndex);
    //b3Printf("mb link index = %d\n",mbLinkIndex);

	btTransform parentLocalInertialFrame;
	parentLocalInertialFrame.setIdentity();
	btScalar parentMass(1);
	btVector3 parentLocalInertiaDiagonal(1,1,1);

    if (urdfParentIndex==-2)
    {
        //b3Printf("root link has no parent\n");
    } else
    {
        //b3Printf("urdf parent index = %d\n",urdfParentIndex);
        //b3Printf("mb parent index = %d\n",mbParentIndex);
        parentRigidBody = cache.getRigidBodyFromLink(urdfParentIndex);
		u2b.getMassAndInertia(urdfParentIndex, parentMass,parentLocalInertiaDiagonal,parentLocalInertialFrame);

    }

    btScalar mass = 0;
    btTransform localInertialFrame;
    localInertialFrame.setIdentity();
    btVector3 localInertiaDiagonal(0,0,0);
    u2b.getMassAndInertia(urdfLinkIndex, mass,localInertiaDiagonal,localInertialFrame);





    btTransform parent2joint;
    parent2joint.setIdentity();

    int jointType;
    btVector3 jointAxisInJointSpace;
    btScalar jointLowerLimit;
    btScalar jointUpperLimit;


    bool hasParentJoint = u2b.getJointInfo(urdfLinkIndex, parent2joint, jointAxisInJointSpace, jointType,jointLowerLimit,jointUpperLimit);


    linkTransformInWorldSpace =parentTransformInWorldSpace*parent2joint;

    int graphicsIndex = u2b.convertLinkVisualShapes(urdfLinkIndex,pathPrefix,localInertialFrame);

    btCompoundShape* compoundShape = u2b.convertLinkCollisionShapes(urdfLinkIndex,pathPrefix,localInertialFrame);

    if (compoundShape)
    {


        btVector3 color = selectColor2();
        /*
         if (visual->material.get())
         {
            color.setValue(visual->material->color.r,visual->material->color.g,visual->material->color.b);//,visual->material->color.a);
         }
         */
        //btVector3 localInertiaDiagonal(0, 0, 0);
        //if (mass)
        //{
        //	shape->calculateLocalInertia(mass, localInertiaDiagonal);
        //}

        btRigidBody* linkRigidBody = 0;
        btTransform inertialFrameInWorldSpace = linkTransformInWorldSpace*localInertialFrame;

        if (!createMultiBody)
        {
            btRigidBody* body = creation.allocateRigidBody(urdfLinkIndex, mass, localInertiaDiagonal, inertialFrameInWorldSpace, compoundShape);
            linkRigidBody = body;

            world1->addRigidBody(body, bodyCollisionFilterGroup, bodyCollisionFilterMask);

            compoundShape->setUserIndex(graphicsIndex);

            creation.createRigidBodyGraphicsInstance(urdfLinkIndex, body, color, graphicsIndex);
            cache.registerRigidBody(urdfLinkIndex, body, inertialFrameInWorldSpace, mass, localInertiaDiagonal, compoundShape, localInertialFrame);
        } else
        {
            if (cache.m_bulletMultiBody==0)
            {
                
                bool canSleep = false;
                bool isFixedBase = (mass==0);//todo: figure out when base is fixed
                int totalNumJoints = cache.m_totalNumJoints1;
                cache.m_bulletMultiBody = creation.allocateMultiBody(urdfLinkIndex, totalNumJoints,mass, localInertiaDiagonal, isFixedBase, canSleep);

                cache.registerMultiBody(urdfLinkIndex, cache.m_bulletMultiBody, inertialFrameInWorldSpace, mass, localInertiaDiagonal, compoundShape, localInertialFrame);
            }

        }

        //create a joint if necessary
        if (hasParentJoint)        {

            btTransform offsetInA,offsetInB;
            offsetInA = parentLocalInertialFrame.inverse()*parent2joint;
            offsetInB = localInertialFrame.inverse();

            bool disableParentCollision = true;
            switch (jointType)
            {
                case URDFFixedJoint:
                {
                    if (createMultiBody)
                    {
                        //todo: adjust the center of mass transform and pivot axis properly

                        //b3Printf("Fixed joint (btMultiBody)\n");
                        btQuaternion rot = offsetInA.inverse().getRotation();//parent2joint.inverse().getRotation();
                        cache.m_bulletMultiBody->setupFixed(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                                               rot*offsetInB.getRotation(), offsetInA.getOrigin(),-offsetInB.getOrigin());
                        creation.addLinkMapping(urdfLinkIndex,mbLinkIndex);


                    } else
                    {
                        //b3Printf("Fixed joint\n");
						
						btGeneric6DofSpring2Constraint* dof6 = creation.createFixedJoint(urdfLinkIndex,*parentRigidBody, *linkRigidBody, offsetInA, offsetInB);
                       
                        if (enableConstraints)
                            world1->addConstraint(dof6,true);
                    }
                    break;
                }
                case URDFContinuousJoint:
                case URDFRevoluteJoint:
                {
                    if (createMultiBody)
                    {


                        cache.m_bulletMultiBody->setupRevolute(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                                                  offsetInA.inverse().getRotation()*offsetInB.getRotation(), quatRotate(offsetInB.inverse().getRotation(),jointAxisInJointSpace), offsetInA.getOrigin(),//parent2joint.getOrigin(),
                                                                  -offsetInB.getOrigin(),
                                                                  disableParentCollision);
                        creation.addLinkMapping(urdfLinkIndex,mbLinkIndex);

                    } else
                    {

						btGeneric6DofSpring2Constraint* dof6 = creation.createRevoluteJoint(urdfLinkIndex,*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,jointAxisInJointSpace,jointLowerLimit, jointUpperLimit);

						if (enableConstraints)
                                    world1->addConstraint(dof6,true);
                        //b3Printf("Revolute/Continuous joint\n");
                    }
                    break;
                }
                case URDFPrismaticJoint:
                {
                    if (createMultiBody)
                    {

                        cache.m_bulletMultiBody->setupPrismatic(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                                                   offsetInA.inverse().getRotation()*offsetInB.getRotation(), quatRotate(offsetInB.inverse().getRotation(),jointAxisInJointSpace), offsetInA.getOrigin(),//parent2joint.getOrigin(),
                                                                   -offsetInB.getOrigin(),
                                                                   disableParentCollision);

                        creation.addLinkMapping(urdfLinkIndex,mbLinkIndex);
                        btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(cache.m_bulletMultiBody,mbLinkIndex,jointLowerLimit, jointUpperLimit);
                        world1->addMultiBodyConstraint(con);
                        //printf("joint lower limit=%d, upper limit = %f\n", jointLowerLimit, jointUpperLimit);

                    } else
                    {
                        
						btGeneric6DofSpring2Constraint* dof6 = creation.createPrismaticJoint(urdfLinkIndex,*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,jointAxisInJointSpace,jointLowerLimit,jointUpperLimit);
						
                       
                        if (enableConstraints)
                            world1->addConstraint(dof6,true);

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
                btMultiBodyLinkCollider* col= creation.allocateMultiBodyLinkCollider(urdfLinkIndex, mbLinkIndex, cache.m_bulletMultiBody);

                compoundShape->setUserIndex(graphicsIndex);

                col->setCollisionShape(compoundShape);

                btTransform tr;
                tr.setIdentity();
                tr = linkTransformInWorldSpace;
                //if we don't set the initial pose of the btCollisionObject, the simulator will do this
                //when syncing the btMultiBody link transforms to the btMultiBodyLinkCollider

                col->setWorldTransform(tr);

                bool isDynamic = true;
                short collisionFilterGroup = isDynamic? short(btBroadphaseProxy::DefaultFilter) : short(btBroadphaseProxy::StaticFilter);
                short collisionFilterMask = isDynamic? 	short(btBroadphaseProxy::AllFilter) : 	short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

                world1->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);

                btVector4 color = selectColor2();//(0.0,0.0,0.5);
				u2b.getLinkColor(urdfLinkIndex,color);
                creation.createCollisionObjectGraphicsInstance(urdfLinkIndex,col,color);

                btScalar friction = 0.5f;

                col->setFriction(friction);

                if (mbLinkIndex>=0) //???? double-check +/- 1
                {
                    cache.m_bulletMultiBody->getLink(mbLinkIndex).m_collider=col;
                } else
                {
                    cache.m_bulletMultiBody->setBaseCollider(col);
                }
            }
        }
    }


    btAlignedObjectArray<int> urdfChildIndices;
    u2b.getLinkChildIndices(urdfLinkIndex,urdfChildIndices);

    int numChildren = urdfChildIndices.size();

    for (int i=0;i<numChildren;i++)
    {
        int urdfChildLinkIndex = urdfChildIndices[i];

        ConvertURDF2BulletInternal(u2b,creation, cache,urdfChildLinkIndex,linkTransformInWorldSpace,world1,createMultiBody,pathPrefix);
    }

}

void ConvertURDF2Bullet(const URDFImporterInterface& u2b, MultiBodyCreationInterface& creation, const btTransform& rootTransformInWorldSpace, btMultiBodyDynamicsWorld* world1,bool createMultiBody, const char* pathPrefix)
{
    URDF2BulletCachedData cache;

    InitURDF2BulletCache(u2b,cache);
    int urdfLinkIndex = u2b.getRootLinkIndex();
    ConvertURDF2BulletInternal(u2b, creation, cache, urdfLinkIndex,rootTransformInWorldSpace,world1,createMultiBody,pathPrefix);

	if (world1 && cache.m_bulletMultiBody)
	{
		btMultiBody* mb = cache.m_bulletMultiBody;
		mb->setHasSelfCollision(false);
		mb->finalizeMultiDof();

		mb->setBaseWorldTransform(rootTransformInWorldSpace);
		btAlignedObjectArray<btQuaternion> scratch_q;
		btAlignedObjectArray<btVector3> scratch_m;
		mb->forwardKinematics(scratch_q,scratch_m);
		mb->updateCollisionObjectWorldTransforms(scratch_q,scratch_m);
		
		world1->addMultiBody(mb);
	}
}


