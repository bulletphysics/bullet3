#include "URDF2Bullet.h"
#include <stdio.h>
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

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

void URDF2Bullet::printTree(int linkIndex, int indentationLevel)
{
    btAlignedObjectArray<int> childIndices;
    getLinkChildIndices(linkIndex,childIndices);
    
    int numChildren = childIndices.size();
    
    indentationLevel+=2;
    int count = 0;
    for (int i=0;i<numChildren;i++)
    {
        int childLinkIndex = childIndices[i];
        std::string name = getLinkName(childLinkIndex);
        for(int j=0;j<indentationLevel;j++) printf("  "); //indent
        printf("child(%d).name=%s with childIndex=%d\n",(count++)+1, name.c_str(),childLinkIndex);
        // first grandchild
        printTree(childLinkIndex,indentationLevel);
    }
}

void ComputeTotalNumberOfJoints(const URDF2Bullet& u2b, URDF2BulletCachedData& cache, int linkIndex)
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

void ComputeParentIndices(const URDF2Bullet& u2b, URDF2BulletCachedData& cache, int urdfLinkIndex, int urdfParentIndex)
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

void InitURDF2BulletCache(const URDF2Bullet& u2b, URDF2BulletCachedData& cache)
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
        
        cache.m_currentMultiBodyLinkIndex = -1;//multi body base has 'link' index -1
        ComputeParentIndices(u2b,cache,rootLinkIndex,-2);
    }
    
}

void ConvertURDF2Bullet(const URDF2Bullet& u2b, URDF2BulletCachedData& cache, int urdfLinkIndex, const btTransform& parentTransformInWorldSpace, btMultiBodyDynamicsWorld* world1,const URDF2BulletConfig& mappings, const char* pathPrefix)
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

    if (urdfParentIndex==-2)
    {
        printf("root link has no parent\n");
    } else
    {
        printf("urdf parent index = %d\n",urdfParentIndex);
        printf("mb parent index = %d\n",mbParentIndex);
        parentRigidBody = cache.getRigidBodyFromLink(urdfParentIndex);
        
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
    

    bool hasParentJoint = u2b.getParent2JointInfo(urdfLinkIndex, parent2joint, jointAxisInJointSpace, jointType,jointLowerLimit,jointUpperLimit);
    
    
    linkTransformInWorldSpace =parentTransformInWorldSpace*parent2joint;

    int graphicsIndex = u2b.convertLinkVisuals(urdfLinkIndex,pathPrefix,localInertialFrame);
    
    btCompoundShape* compoundShape = u2b.convertLinkCollisions(urdfLinkIndex,pathPrefix,localInertialFrame);
    
    if (compoundShape)
    {
        
        
        btVector3 color = selectColor2();
        /*                if (visual->material.get())
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
        
        //btTransform visualFrameInWorldSpace = linkTransformInWorldSpace*visual_frame;
        btTransform inertialFrameInWorldSpace = linkTransformInWorldSpace*localInertialFrame;
       // URDF_LinkInformation* linkInfo = new URDF_LinkInformation;
        
        if (!mappings.m_createMultiBody)
        {
            btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, compoundShape, localInertiaDiagonal);
            rbci.m_startWorldTransform = inertialFrameInWorldSpace;
           // linkInfo->m_bodyWorldTransform = inertialFrameInWorldSpace;//visualFrameInWorldSpace
            //rbci.m_startWorldTransform = inertialFrameInWorldSpace;//linkCenterOfMass;
            btRigidBody* body = new btRigidBody(rbci);
            linkRigidBody = body;
            
            world1->addRigidBody(body, bodyCollisionFilterGroup, bodyCollisionFilterMask);
            
            compoundShape->setUserIndex(graphicsIndex);
            
            u2b.createRigidBodyGraphicsInstance(urdfLinkIndex, body, color, graphicsIndex);
            
//            gfxBridge.createRigidBodyGraphicsObject(body, color);
            
           // linkInfo->m_bulletRigidBody = body;
            
            cache.registerRigidBody(urdfLinkIndex, body, inertialFrameInWorldSpace, mass, localInertiaDiagonal, compoundShape, localInertialFrame);
            
        } else
        {
            if (cache.m_bulletMultiBody==0)
            {
                bool multiDof = true;
                bool canSleep = false;
                bool isFixedBase = (mass==0);//todo: figure out when base is fixed
                int totalNumJoints = cache.m_totalNumJoints1;
                cache.m_bulletMultiBody = new btMultiBody(totalNumJoints,mass, localInertiaDiagonal, isFixedBase, canSleep, multiDof);
                
                cache.registerMultiBody(urdfLinkIndex, cache.m_bulletMultiBody, inertialFrameInWorldSpace, mass, localInertiaDiagonal, compoundShape, localInertialFrame);
            }
            
        }
        
        
      //  linkInfo->m_collisionShape = compoundShape;
      //  linkInfo->m_localInertiaDiagonal = localInertiaDiagonal;
      //  linkInfo->m_mass = mass;
        //linkInfo->m_localVisualFrame =visual_frame;
      //  linkInfo->m_localInertialFrame =inertialFrame;
        
        
        
      //  linkInfo->m_thisLink = link.get();
      //  const Link* p = link.get();
     //   mappings.m_link2rigidbody.insert(p, linkInfo);
        
        
        //create a joint if necessary
        if (hasParentJoint)//(*link).parent_joint && pp)
        {
            
//            const Joint* pj = (*link).parent_joint.get();
            btTransform offsetInA,offsetInB;
            static bool once = true;
            
            offsetInA.setIdentity();
            static bool toggle=false;
            
            //offsetInA = pp->m_localVisualFrame.inverse()*parent2joint;
            offsetInA = localInertialFrame.inverse()*parent2joint;
            
            offsetInB.setIdentity();
            //offsetInB = visual_frame.inverse();
            offsetInB = localInertialFrame.inverse();
            
            
            bool disableParentCollision = true;
            switch (jointType)
            {
                case URDF2Bullet::FixedJoint:
                {
                    if (mappings.m_createMultiBody)
                    {
                        //todo: adjust the center of mass transform and pivot axis properly
                        
                        printf("Fixed joint (btMultiBody)\n");
                        //btVector3 dVec = quatRotate(parentComToThisCom.getRotation(),offsetInB.inverse().getOrigin());
                        btQuaternion rot = offsetInA.inverse().getRotation();//parent2joint.inverse().getRotation();
                        //toggle=!toggle;
                        //mappings.m_bulletMultiBody->setupFixed(linkIndex - 1, mass, localInertiaDiagonal, parentIndex - 1,
                        //	rot, parent2joint.getOrigin(), btVector3(0,0,0),disableParentCollision);
                        cache.m_bulletMultiBody->setupFixed(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                                               rot*offsetInB.getRotation(), offsetInA.getOrigin(),-offsetInB.getOrigin(),disableParentCollision);
                        
                                               
                        btMatrix3x3 rm(rot);
                        btScalar y,p,r;
                        rm.getEulerZYX(y,p,r);
                        //parent2joint.inverse().getRotation(), offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                        //linkInfo->m_localVisualFrame.setIdentity();
                        printf("y=%f,p=%f,r=%f\n", y,p,r);
                        
                        
                        
                    } else
                    {
                        printf("Fixed joint\n");
                        
                        btMatrix3x3 rm(offsetInA.getBasis());
                        btScalar y,p,r;
                        rm.getEulerZYX(y,p,r);
                        //parent2joint.inverse().getRotation(), offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                        //linkInfo->m_localVisualFrame.setIdentity();
                        printf("y=%f,p=%f,r=%f\n", y,p,r);
                        
                        btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentRigidBody, *linkRigidBody, offsetInA, offsetInB);
                        //                            btVector3 bulletAxis(pj->axis.x,pj->axis.y,pj->axis.z);
                        dof6->setLinearLowerLimit(btVector3(0,0,0));
                        dof6->setLinearUpperLimit(btVector3(0,0,0));
                        
                        dof6->setAngularLowerLimit(btVector3(0,0,0));
                        dof6->setAngularUpperLimit(btVector3(0,0,0));
                        
                        if (enableConstraints)
                            world1->addConstraint(dof6,true);
                        
                        //                            btFixedConstraint* fixed = new btFixedConstraint(*parentBody, *body,offsetInA,offsetInB);
                        //                          world->addConstraint(fixed,true);
                    }
                    break;
                }
                case URDF2Bullet::ContinuousJoint:
                case URDF2Bullet::RevoluteJoint:
                {
                    if (mappings.m_createMultiBody)
                    {
                        //todo: adjust the center of mass transform and pivot axis properly
                        /*mappings.m_bulletMultiBody->setupRevolute(
                         linkIndex - 1, mass, localInertiaDiagonal, parentIndex - 1,
                         
                         parent2joint.inverse().getRotation(), jointAxis, parent2joint.getOrigin(),
                         btVector3(0,0,0),//offsetInB.getOrigin(),
                         disableParentCollision);
                         */
                        
                        
                        cache.m_bulletMultiBody->setupRevolute(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                                                  //parent2joint.inverse().getRotation(), jointAxis, offsetInA.getOrigin(),//parent2joint.getOrigin(),
                                                                  offsetInA.inverse().getRotation()*offsetInB.getRotation(), quatRotate(offsetInB.inverse().getRotation(),jointAxisInJointSpace), offsetInA.getOrigin(),//parent2joint.getOrigin(),
                                                                  
                                                                  -offsetInB.getOrigin(),
                                                                  disableParentCollision);
                        //linkInfo->m_localVisualFrame.setIdentity();
                        
                    } else
                    {
                        //only handle principle axis at the moment,
                        //@todo(erwincoumans) orient the constraint for non-principal axis
                        int principleAxis = jointAxisInJointSpace.closestAxis();
                        switch (principleAxis)
                        {
                            case 0:
                            {
                                btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,RO_ZYX);
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
                                btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,RO_XZY);
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
                                btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentRigidBody, *linkRigidBody, offsetInA, offsetInB,RO_XYZ);
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
                case URDF2Bullet::PrismaticJoint:
                {
                    if (mappings.m_createMultiBody)
                    {
                        
                        cache.m_bulletMultiBody->setupPrismatic(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                                                   offsetInA.inverse().getRotation()*offsetInB.getRotation(), quatRotate(offsetInB.inverse().getRotation(),jointAxisInJointSpace), offsetInA.getOrigin(),//parent2joint.getOrigin(),
                                                                   -offsetInB.getOrigin(),
                                                                   disableParentCollision);
                        
                        
                        
                    } else
                    {
                        
                        btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentRigidBody, *linkRigidBody, offsetInA, offsetInB);
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
                }
            }
            
        }
        
        if (mappings.m_createMultiBody)
        {
            if (compoundShape->getNumChildShapes()>0)
            {
                btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(cache.m_bulletMultiBody, mbLinkIndex); //or mbLinkIndex-1??? double-check
                
                //btCompoundShape* comp = new btCompoundShape();
                //comp->addChildShape(linkInfo->m_localVisualFrame,shape);
                
                compoundShape->setUserIndex(graphicsIndex);
                
                col->setCollisionShape(compoundShape);
                
                btTransform tr;
                tr.setIdentity();
                tr = linkTransformInWorldSpace;
                //if we don't set the initial pose of the btCollisionObject, the simulator will do this 
                //when syncing the btMultiBody link transforms to the btMultiBodyLinkCollider
                
                //tr.setOrigin(local_origin[0]);
                //tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
                col->setWorldTransform(tr);
                
                bool isDynamic = true;
                short collisionFilterGroup = isDynamic? short(btBroadphaseProxy::DefaultFilter) : short(btBroadphaseProxy::StaticFilter);
                short collisionFilterMask = isDynamic? 	short(btBroadphaseProxy::AllFilter) : 	short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
                
                world1->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);
                
                btVector3 color = selectColor2();//(0.0,0.0,0.5);
               
                u2b.createCollisionObjectGraphicsInstance(urdfLinkIndex,col,color);
                
                btScalar friction = 0.5f;
                
                col->setFriction(friction);
                
                if (mbParentIndex>=0) //???? double-check +/- 1
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
        
        ConvertURDF2Bullet(u2b,cache,urdfChildLinkIndex,linkTransformInWorldSpace,world1,mappings,pathPrefix);
    }
    
}