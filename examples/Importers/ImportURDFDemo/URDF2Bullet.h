#ifndef _URDF2BULLET_H
#define _URDF2BULLET_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include <string>
class btVector3;
class btTransform;
class btMultiBodyDynamicsWorld;
class btTransform;

class URDF2Bullet
{
    
public:

    enum {
        RevoluteJoint=1,
        PrismaticJoint,
        ContinuousJoint,
        FloatingJoint,
        PlanarJoint,
        FixedJoint,
    };
 
    ///return >=0 for the root link index, -1 if there is no root link
    virtual int getRootLinkIndex() const = 0;
    
    ///pure virtual interfaces, precondition is a valid linkIndex (you can assert/terminate if the linkIndex is out of range)
    virtual std::string getLinkName(int linkIndex) const =0;
    
    virtual std::string getJointName(int linkIndex) const = 0;

    //fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
    virtual void  getMassAndInertia(int urdfLinkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const =0;
    
    ///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
    virtual void getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const =0;
    
    virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit) const =0;
    
    virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertialFrame) const=0;
    
    ///create Bullet collision shapes from URDF 'Collision' objects, specified in inertial frame of the link.
    virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const = 0;
    
    virtual void createRigidBodyGraphicsInstance(int linkIndex, class btRigidBody* body, const btVector3& colorRgba, int graphicsIndex) const = 0;
  
    ///optionally create some graphical representation from a collision object, usually for visual debugging purposes.
    virtual void createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* col, const btVector3& colorRgba) const = 0;
    
    virtual class btMultiBody* allocateMultiBody(int urdfLinkIndex, int totalNumJoints,btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep, bool multiDof) const =0;
    
    virtual class btRigidBody* allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape) const = 0;
    
    virtual class btGeneric6DofSpring2Constraint* allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB, int rotateOrder=0) const = 0;
    
    virtual class btMultiBodyLinkCollider* allocateMultiBodyLinkCollider(int urdfLinkIndex, int mbLinkIndex, btMultiBody* body) const = 0;
    
    virtual void addLinkMapping(int urdfLinkIndex, int mbLinkIndex) const = 0;
};


void printTree(const URDF2Bullet& u2b, int linkIndex, int identationLevel=0);



void ConvertURDF2Bullet(const URDF2Bullet& u2b, const btTransform& rootTransformInWorldSpace, btMultiBodyDynamicsWorld* world,bool createMultiBody, const char* pathPrefix);


#endif //_URDF2BULLET_H

