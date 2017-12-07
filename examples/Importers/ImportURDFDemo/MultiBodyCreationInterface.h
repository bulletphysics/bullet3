#ifndef MULTIBODY_CREATION_INTERFACE_H
#define MULTIBODY_CREATION_INTERFACE_H

#include "LinearMath/btTransform.h"

class MultiBodyCreationInterface
{
public:

	virtual ~MultiBodyCreationInterface() {}

    
    virtual void createRigidBodyGraphicsInstance(int linkIndex, class btRigidBody* body, const btVector3& colorRgba, int graphicsIndex) = 0;
	virtual void createRigidBodyGraphicsInstance2(int linkIndex, class btRigidBody* body, const btVector3& colorRgba, const btVector3& specularColor, int graphicsIndex)
	{
		createRigidBodyGraphicsInstance(linkIndex,body,colorRgba,graphicsIndex);
	}

    ///optionally create some graphical representation from a collision object, usually for visual debugging purposes.
    virtual void createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* col, const btVector3& colorRgba) = 0;
    virtual void createCollisionObjectGraphicsInstance2(int linkIndex, class btCollisionObject* col, const btVector4& colorRgba, const btVector3& specularColor)
	{
		createCollisionObjectGraphicsInstance(linkIndex,col,colorRgba);
	}
    
    virtual class btMultiBody* allocateMultiBody(int urdfLinkIndex, int totalNumJoints,btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep)  =0;
    
    virtual class btRigidBody* allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape)  = 0;
    
    virtual class btGeneric6DofSpring2Constraint* allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB, int rotateOrder=0) = 0;
    
	virtual class btGeneric6DofSpring2Constraint* createPrismaticJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
														const btVector3&	jointAxisInJointSpace,btScalar jointLowerLimit,btScalar jointUpperLimit) = 0;
    virtual class btGeneric6DofSpring2Constraint* createRevoluteJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
														const btVector3&	jointAxisInJointSpace,btScalar jointLowerLimit,btScalar jointUpperLimit) = 0;

    virtual class btGeneric6DofSpring2Constraint* createFixedJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB) = 0;
   
    virtual class btMultiBodyLinkCollider* allocateMultiBodyLinkCollider(int urdfLinkIndex, int mbLinkIndex, btMultiBody* body) = 0;
    
    virtual void addLinkMapping(int urdfLinkIndex, int mbLinkIndex) = 0;

};

#endif //MULTIBODY_CREATION_INTERFACE_H