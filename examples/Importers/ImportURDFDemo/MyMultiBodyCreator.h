
#ifndef MY_MULTIBODY_CREATOR
#define MY_MULTIBODY_CREATOR

#include "MultiBodyCreationInterface.h"
#include "LinearMath/btAlignedObjectArray.h"

struct GUIHelperInterface;
class btMultiBody;

class MyMultiBodyCreator : public MultiBodyCreationInterface
{

	btMultiBody* m_bulletMultiBody;
    
	struct GUIHelperInterface* m_guiHelper;

	

public:

	btAlignedObjectArray<int> m_urdf2mbLink;
    btAlignedObjectArray<int> m_mb2urdfLink;
  

	MyMultiBodyCreator(GUIHelperInterface* guiHelper);

	virtual ~MyMultiBodyCreator() {}

    virtual void createRigidBodyGraphicsInstance(int linkIndex, class btRigidBody* body, const btVector3& colorRgba, int graphicsIndex) ;
  
    ///optionally create some graphical representation from a collision object, usually for visual debugging purposes.
    virtual void createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* col, const btVector3& colorRgba);
    
    virtual class btMultiBody* allocateMultiBody(int urdfLinkIndex, int totalNumJoints,btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep, bool multiDof);
    
    virtual class btRigidBody* allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape);
    
    virtual class btGeneric6DofSpring2Constraint* allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB, int rotateOrder=0);
    
    virtual class btMultiBodyLinkCollider* allocateMultiBodyLinkCollider(int urdfLinkIndex, int mbLinkIndex, btMultiBody* body);
    
    virtual void addLinkMapping(int urdfLinkIndex, int mbLinkIndex);

	btMultiBody* getBulletMultiBody();

};

#endif //MY_MULTIBODY_CREATOR
