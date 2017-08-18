
#ifndef MY_MULTIBODY_CREATOR
#define MY_MULTIBODY_CREATOR

#include "MultiBodyCreationInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"

struct GUIHelperInterface;
class btMultiBody;

struct GenericConstraintUserInfo
{
	int	m_urdfIndex;
	int m_urdfJointType;
	btVector3 m_jointAxisInJointSpace;
	int		m_jointAxisIndex;
	btScalar m_lowerJointLimit;
	btScalar m_upperJointLimit;

};

class MyMultiBodyCreator : public MultiBodyCreationInterface
{
protected:

	btMultiBody* m_bulletMultiBody;
	btRigidBody* m_rigidBody;
    
	struct GUIHelperInterface* m_guiHelper;

	btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_6DofConstraints;

public:

    btAlignedObjectArray<int> m_mb2urdfLink;
  

	MyMultiBodyCreator(GUIHelperInterface* guiHelper);

	virtual ~MyMultiBodyCreator() {}

    virtual void createRigidBodyGraphicsInstance(int linkIndex, class btRigidBody* body, const btVector3& colorRgba, int graphicsIndex) ;
    virtual void createRigidBodyGraphicsInstance2(int linkIndex, class btRigidBody* body, const btVector3& colorRgba, const btVector3& specularColor, int graphicsIndex) ;
  
    ///optionally create some graphical representation from a collision object, usually for visual debugging purposes.
    virtual void createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* col, const btVector3& colorRgba);
    virtual void createCollisionObjectGraphicsInstance2(int linkIndex, class btCollisionObject* col, const btVector4& colorRgba, const btVector3& specularColor);

    virtual class btMultiBody* allocateMultiBody(int urdfLinkIndex, int totalNumJoints,btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep);
    
    virtual class btRigidBody* allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape);
    
    virtual class btGeneric6DofSpring2Constraint* allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB, int rotateOrder=0);
    
	virtual class btGeneric6DofSpring2Constraint* createPrismaticJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
														const btVector3&	jointAxisInJointSpace,btScalar jointLowerLimit,btScalar jointUpperLimit);
    virtual class btGeneric6DofSpring2Constraint* createRevoluteJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
														const btVector3&	jointAxisInJointSpace,btScalar jointLowerLimit,btScalar jointUpperLimit);

    virtual class btGeneric6DofSpring2Constraint* createFixedJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB);
   
    virtual class btMultiBodyLinkCollider* allocateMultiBodyLinkCollider(int urdfLinkIndex, int mbLinkIndex, btMultiBody* body);
    
    virtual void addLinkMapping(int urdfLinkIndex, int mbLinkIndex);

	btMultiBody* getBulletMultiBody();
	btRigidBody* getRigidBody()
	{
	    return m_rigidBody;
    }
	
	int	getNum6DofConstraints() const
	{
		return m_6DofConstraints.size();
	}
	
	btGeneric6DofSpring2Constraint*	get6DofConstraint(int index)
	{
		return m_6DofConstraints[index];
	}
};

#endif //MY_MULTIBODY_CREATOR
