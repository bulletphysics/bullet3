#include "URDFImporterInterface.h"
#include <stdio.h>
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
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


static btVector3 selectColor2()
{
    
    static int curColor = 0;
    btVector4 color = colors[curColor];
    curColor++;
    curColor&=3;
    return color;
}

void printTree(const URDFImporterInterface& u2b, int linkIndex, int indentationLevel)
{
    btAlignedObjectArray<int> childIndices;
    u2b.getLinkChildIndices(linkIndex,childIndices);
    
    int numChildren = childIndices.size();
    
    indentationLevel+=2;
    int count = 0;
    for (int i=0;i<numChildren;i++)
    {
        int childLinkIndex = childIndices[i];
        std::string name = u2b.getLinkName(childLinkIndex);
        for(int j=0;j<indentationLevel;j++) printf("  "); //indent
        printf("child(%d).name=%s with childIndex=%d\n",(count++)+1, name.c_str(),childLinkIndex);
        // first grandchild
        printTree(u2b,childLinkIndex,indentationLevel);
    }
}


struct URDF2BulletCachedData
{
    URDF2BulletCachedData()
    :m_totalNumJoints1(0),
    m_currentMultiBodyLinkIndex(-1),
    m_bulletMultiBody(0)
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
    printf("link %s has %d children\n", u2b.getLinkName(linkIndex).c_str(),childIndices.size());
    for (int i=0;i<childIndices.size();i++)
    {
        printf("child %d has childIndex%d=%s\n",i,childIndices[i],u2b.getLinkName(childIndices[i]).c_str());
    }
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
    printf("start converting/extracting data from URDF interface\n");
    
    btTransform linkTransformInWorldSpace;
    linkTransformInWorldSpace.setIdentity();
    

    int mbLinkIndex =cache.getMbIndexFromUrdfIndex(urdfLinkIndex);

    int urdfParentIndex =   cache.getParentUrdfIndex(urdfLinkIndex);
    int mbParentIndex = cache.getMbIndexFromUrdfIndex(urdfParentIndex);
    btRigidBody* parentRigidBody = 0;
    
    std::string name = u2b.getLinkName(urdfLinkIndex);
    printf("link name=%s urdf link index=%d\n",name.c_str(),urdfLinkIndex);
    printf("mb link index = %d\n",mbLinkIndex);

	btTransform parentLocalInertialFrame;
	parentLocalInertialFrame.setIdentity();
	btScalar parentMass(1);
	btVector3 parentLocalInertiaDiagonal(1,1,1);

    if (urdfParentIndex==-2)
    {
        printf("root link has no parent\n");
    } else
    {
        printf("urdf parent index = %d\n",urdfParentIndex);
        printf("mb parent index = %d\n",mbParentIndex);
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
                bool multiDof = true;
                bool canSleep = false;
                bool isFixedBase = (mass==0);//todo: figure out when base is fixed
                int totalNumJoints = cache.m_totalNumJoints1;
                cache.m_bulletMultiBody = creation.allocateMultiBody(urdfLinkIndex, totalNumJoints,mass, localInertiaDiagonal, isFixedBase, canSleep, multiDof);
                
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
                        
                        printf("Fixed joint (btMultiBody)\n");
                        btQuaternion rot = offsetInA.inverse().getRotation();//parent2joint.inverse().getRotation();
                        cache.m_bulletMultiBody->setupFixed(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                                               rot*offsetInB.getRotation(), offsetInA.getOrigin(),-offsetInB.getOrigin(),disableParentCollision);
                        creation.addLinkMapping(urdfLinkIndex,mbLinkIndex);
                        
                                             
                    } else
                    {
                        printf("Fixed joint\n");
                        
                        btMatrix3x3 rm(offsetInA.getBasis());
                        btScalar y,p,r;
                        rm.getEulerZYX(y,p,r);
                        printf("y=%f,p=%f,r=%f\n", y,p,r);
                        
                        //we could also use btFixedConstraint but it has some issues
                        btGeneric6DofSpring2Constraint* dof6 = creation.allocateGeneric6DofSpring2Constraint(urdfLinkIndex, *parentRigidBody, *linkRigidBody, offsetInA, offsetInB);

                        dof6->setLinearLowerLimit(btVector3(0,0,0));
                        dof6->setLinearUpperLimit(btVector3(0,0,0));
                        
                        dof6->setAngularLowerLimit(btVector3(0,0,0));
                        dof6->setAngularUpperLimit(btVector3(0,0,0));
                        
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
                        //only handle principle axis at the moment,
                        //@todo(erwincoumans) orient the constraint for non-principal axis
                        int principleAxis = jointAxisInJointSpace.closestAxis();
                        switch (principleAxis)
                        {
                            case 0:
                            {
                                btGeneric6DofSpring2Constraint* dof6 = creation.allocateGeneric6DofSpring2Constraint(urdfLinkIndex,*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,RO_ZYX);
                                dof6->setLinearLowerLimit(btVector3(0,0,0));
                                dof6->setLinearUpperLimit(btVector3(0,0,0));
                                
                                dof6->setAngularUpperLimit(btVector3(-1,0,0));
                                dof6->setAngularLowerLimit(btVector3(1,0,0));
                                
                                if (enableConstraints)
                                    world1->addConstraint(dof6,true);
                                break;
                            }
                            case 1:
                            {
                                btGeneric6DofSpring2Constraint* dof6 = creation.allocateGeneric6DofSpring2Constraint(urdfLinkIndex,*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,RO_XZY);
                                dof6->setLinearLowerLimit(btVector3(0,0,0));
                                dof6->setLinearUpperLimit(btVector3(0,0,0));
                                
                                dof6->setAngularUpperLimit(btVector3(0,-1,0));
                                dof6->setAngularLowerLimit(btVector3(0,1,0));
                                
                                if (enableConstraints)
                                    world1->addConstraint(dof6,true);
                                break;
                            }
                            case 2:
                            default:
                            {
                                btGeneric6DofSpring2Constraint* dof6 = creation.allocateGeneric6DofSpring2Constraint(urdfLinkIndex,*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,RO_XYZ);
                                dof6->setLinearLowerLimit(btVector3(0,0,0));
                                dof6->setLinearUpperLimit(btVector3(0,0,0));
                                
                                dof6->setAngularUpperLimit(btVector3(0,0,-1));
                                dof6->setAngularLowerLimit(btVector3(0,0,0));
                                
                                if (enableConstraints)
                                    world1->addConstraint(dof6,true);
                            }
                        };
                        printf("Revolute/Continuous joint\n");
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
                        
                    } else
                    {
                        btGeneric6DofSpring2Constraint* dof6 = creation.allocateGeneric6DofSpring2Constraint(urdfLinkIndex,*parentRigidBody, *linkRigidBody, offsetInA, offsetInB);
                        //todo(erwincoumans) for now, we only support principle axis along X, Y or Z
                        int principleAxis = jointAxisInJointSpace.closestAxis();
                        switch (principleAxis)
                        {
                            case 0:
                            {
                                dof6->setLinearLowerLimit(btVector3(jointLowerLimit,0,0));
                                dof6->setLinearUpperLimit(btVector3(jointUpperLimit,0,0));
                                break;
                            }
                            case 1:
                            {
                                dof6->setLinearLowerLimit(btVector3(0,jointLowerLimit,0));
                                dof6->setLinearUpperLimit(btVector3(0,jointUpperLimit,0));
                                break;
                            }
                            case 2:
                            default:
                            {
                                dof6->setLinearLowerLimit(btVector3(0,0,jointLowerLimit));
                                dof6->setLinearUpperLimit(btVector3(0,0,jointUpperLimit));
                            }
                        };
                        
                        dof6->setAngularLowerLimit(btVector3(0,0,0));
                        dof6->setAngularUpperLimit(btVector3(0,0,0));
                        if (enableConstraints)
                            world1->addConstraint(dof6,true);
                        
                        printf("Prismatic\n");
                    }
                    break;
                }
                default:
                {
                    printf("Error: unsupported joint type in URDF (%d)\n", jointType);
					btAssert(0);
                }
            }
            
        }
        
        if (createMultiBody)
        {
            if (compoundShape->getNumChildShapes()>0)
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
                
                btVector3 color = selectColor2();//(0.0,0.0,0.5);
               
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

		world1->addMultiBody(mb);
	}
}


