#include "MyMultiBodyCreator.h"

#include "../../CommonInterfaces/CommonGUIHelperInterface.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "URDFJointTypes.h"

MyMultiBodyCreator::MyMultiBodyCreator(GUIHelperInterface* guiHelper)
	:m_bulletMultiBody(0),
	m_rigidBody(0),
m_guiHelper(guiHelper)
{
}

    
 class btMultiBody* MyMultiBodyCreator::allocateMultiBody(int /* urdfLinkIndex */, int totalNumJoints,btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep)
{
//	m_urdf2mbLink.resize(totalNumJoints+1,-2);
    m_mb2urdfLink.resize(totalNumJoints+1,-2);

    m_bulletMultiBody = new btMultiBody(totalNumJoints,mass,localInertiaDiagonal,isFixedBase,canSleep);
    return m_bulletMultiBody;
}

class btRigidBody* MyMultiBodyCreator::allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape) 
{
    btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, colShape, localInertiaDiagonal);
    rbci.m_startWorldTransform = initialWorldTrans;
    m_rigidBody = new btRigidBody(rbci);
	m_rigidBody->forceActivationState(DISABLE_DEACTIVATION);
	
    return m_rigidBody;
}
    
class btMultiBodyLinkCollider* MyMultiBodyCreator::allocateMultiBodyLinkCollider(int /*urdfLinkIndex*/, int mbLinkIndex, btMultiBody* multiBody) 
{
    btMultiBodyLinkCollider* mbCol= new btMultiBodyLinkCollider(multiBody, mbLinkIndex);
    return mbCol;
}




class btGeneric6DofSpring2Constraint* MyMultiBodyCreator::allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB, int rotateOrder) 
{
    btGeneric6DofSpring2Constraint* c = new btGeneric6DofSpring2Constraint(rbA,rbB,offsetInA, offsetInB, (RotateOrder)rotateOrder);
	
    return c;
}

class btGeneric6DofSpring2Constraint* MyMultiBodyCreator::createPrismaticJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
													const btVector3&	jointAxisInJointSpace,btScalar jointLowerLimit,btScalar jointUpperLimit)
{
	int rotateOrder=0;
	btGeneric6DofSpring2Constraint* dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA , rbB, offsetInA, offsetInB, rotateOrder);
	 //todo(erwincoumans) for now, we only support principle axis along X, Y or Z
	int principleAxis = jointAxisInJointSpace.closestAxis();

	GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
	userInfo->m_jointAxisInJointSpace = jointAxisInJointSpace;
	userInfo->m_jointAxisIndex = principleAxis;

	userInfo->m_urdfJointType = URDFPrismaticJoint;
	userInfo->m_lowerJointLimit = jointLowerLimit;
	userInfo->m_upperJointLimit = jointUpperLimit;
	userInfo->m_urdfIndex = urdfLinkIndex;
	dof6->setUserConstraintPtr(userInfo);

	
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
	m_6DofConstraints.push_back(dof6);
	return dof6;
}

class btGeneric6DofSpring2Constraint* MyMultiBodyCreator::createRevoluteJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
													const btVector3&	jointAxisInJointSpace,btScalar jointLowerLimit,btScalar jointUpperLimit)
{
	btGeneric6DofSpring2Constraint* dof6 = 0;

	 //only handle principle axis at the moment,
    //@todo(erwincoumans) orient the constraint for non-principal axis
    int principleAxis = jointAxisInJointSpace.closestAxis();
    switch (principleAxis)
    {
        case 0:
        {
            dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex,rbA, rbB, offsetInA, offsetInB,RO_ZYX);
            dof6->setLinearLowerLimit(btVector3(0,0,0));
            dof6->setLinearUpperLimit(btVector3(0,0,0));

            dof6->setAngularUpperLimit(btVector3(-1,0,0));
            dof6->setAngularLowerLimit(btVector3(1,0,0));

            break;
        }
        case 1:
        {
            dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex,rbA, rbB, offsetInA, offsetInB,RO_XZY);
            dof6->setLinearLowerLimit(btVector3(0,0,0));
            dof6->setLinearUpperLimit(btVector3(0,0,0));

            dof6->setAngularUpperLimit(btVector3(0,-1,0));
            dof6->setAngularLowerLimit(btVector3(0,1,0));
            break;
        }
        case 2:
        default:
        {
            dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex,rbA, rbB, offsetInA, offsetInB,RO_XYZ);
            dof6->setLinearLowerLimit(btVector3(0,0,0));
            dof6->setLinearUpperLimit(btVector3(0,0,0));

            dof6->setAngularUpperLimit(btVector3(0,0,-1));
            dof6->setAngularLowerLimit(btVector3(0,0,0));

        }
    };

	GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
	userInfo->m_jointAxisInJointSpace = jointAxisInJointSpace;
	userInfo->m_jointAxisIndex = 3+principleAxis;

	if (jointLowerLimit > jointUpperLimit)
	{
		userInfo->m_urdfJointType = URDFContinuousJoint;
	} else
	{
		userInfo->m_urdfJointType = URDFRevoluteJoint;
		userInfo->m_lowerJointLimit = jointLowerLimit;
		userInfo->m_upperJointLimit = jointUpperLimit;
	}
	userInfo->m_urdfIndex = urdfLinkIndex;
	dof6->setUserConstraintPtr(userInfo);
	m_6DofConstraints.push_back(dof6);
	return dof6;
}


class btGeneric6DofSpring2Constraint* MyMultiBodyCreator::createFixedJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB)
{
	btGeneric6DofSpring2Constraint* dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB);
	
	GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
	userInfo->m_urdfIndex = urdfLinkIndex;
	userInfo->m_urdfJointType = URDFFixedJoint;

	dof6->setUserConstraintPtr(userInfo);

    dof6->setLinearLowerLimit(btVector3(0,0,0));
    dof6->setLinearUpperLimit(btVector3(0,0,0));

    dof6->setAngularLowerLimit(btVector3(0,0,0));
    dof6->setAngularUpperLimit(btVector3(0,0,0));

	return dof6;
}
   


void MyMultiBodyCreator::addLinkMapping(int urdfLinkIndex, int mbLinkIndex) 
{
//    m_urdf2mbLink[urdfLinkIndex] = mbLinkIndex;
    m_mb2urdfLink[mbLinkIndex] = urdfLinkIndex;
}

void MyMultiBodyCreator::createRigidBodyGraphicsInstance(int linkIndex, btRigidBody* body, const btVector3& colorRgba, int graphicsIndex) 
{
        
    m_guiHelper->createRigidBodyGraphicsObject(body, colorRgba);
}
    
void MyMultiBodyCreator::createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* colObj, const btVector3& colorRgba) 
{
    m_guiHelper->createCollisionObjectGraphicsObject(colObj,colorRgba);
}

btMultiBody* MyMultiBodyCreator::getBulletMultiBody()
{
    return m_bulletMultiBody;
}

