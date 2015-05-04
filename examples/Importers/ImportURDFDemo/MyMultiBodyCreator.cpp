#include "MyMultiBodyCreator.h"

#include "../CommonInterfaces/CommonGUIHelperInterface.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"


MyMultiBodyCreator::MyMultiBodyCreator(GUIHelperInterface* guiHelper)
	:m_guiHelper(guiHelper),
	m_bulletMultiBody(0)
{
}

    
 class btMultiBody* MyMultiBodyCreator::allocateMultiBody(int /* urdfLinkIndex */, int totalNumJoints,btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep, bool multiDof)
{
	m_urdf2mbLink.resize(totalNumJoints+1,-2);
    m_mb2urdfLink.resize(totalNumJoints+1,-2);

    m_bulletMultiBody = new btMultiBody(totalNumJoints,mass,localInertiaDiagonal,isFixedBase,canSleep,multiDof);
    return m_bulletMultiBody;
}

class btRigidBody* MyMultiBodyCreator::allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape) 
{
    btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, colShape, localInertiaDiagonal);
    rbci.m_startWorldTransform = initialWorldTrans;
    btRigidBody* body = new btRigidBody(rbci);
    return body;
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

void MyMultiBodyCreator::addLinkMapping(int urdfLinkIndex, int mbLinkIndex) 
{
    m_urdf2mbLink[urdfLinkIndex] = mbLinkIndex;
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

