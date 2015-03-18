#ifndef _URDF2BULLET_H
#define _URDF2BULLET_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include <string>
class btVector3;
class btTransform;

struct URDF2BulletConfig
{
    URDF2BulletConfig()
    :m_createMultiBody(true)
    {
        
    }
    
    //true to create a btMultiBody, false to use btRigidBody
    bool m_createMultiBody;
};

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
        
        /*
        struct URDF_LinkInformation
        {
            const Link* m_thisLink;
            int m_linkIndex;
            //int m_parentIndex;
            
            btTransform m_localInertialFrame;
            //btTransform m_localVisualFrame;
            btTransform m_bodyWorldTransform;
            btVector3 m_localInertiaDiagonal;
            btScalar m_mass;
            
            btCollisionShape* m_collisionShape;
            btRigidBody* m_bulletRigidBody;
            
            URDF_LinkInformation()
            :m_thisLink(0),
            m_linkIndex(-2),
            //m_parentIndex(-2),
            m_collisionShape(0),
            m_bulletRigidBody(0)
            {
                
            }
            virtual ~URDF_LinkInformation()
            {
                printf("~\n");
            }
        };
         */
        
    }
    
    
};

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
    void printTree(int linkIndex, int indentationLevel=0);

    ///return >=0 for the root link index, -1 if there is no root link
    virtual int getRootLinkIndex() const = 0;
    
    ///pure virtual interfaces, precondition is a valid linkIndex (you can assert/terminate if the linkIndex is out of range)
    virtual std::string getLinkName(int linkIndex) const =0;

    //fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
    virtual void  getMassAndInertia(int urdfLinkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const =0;
    
    ///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
    virtual void getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const =0;
    
    virtual bool getParent2JointInfo(int urdfLinkIndex, btTransform& parent2joint, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit) const =0;
    
    virtual int convertLinkVisuals(int linkIndex, const char* pathPrefix, const btTransform& localInertialFrame) const=0;
    
    virtual class btCompoundShape* convertLinkCollisions(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const = 0;
    
    virtual void createRigidBodyGraphicsInstance(int linkIndex, btRigidBody* body, const btVector3& colorRgba, int graphicsIndex) const = 0;
  
    ///optionally create some graphical representation from a collision object, usually for visual debugging purposes.
    virtual void createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* col, const btVector3& colorRgba) const = 0;
    
};


void InitURDF2BulletCache(const URDF2Bullet& u2b, URDF2BulletCachedData& cache);

void ConvertURDF2Bullet(const URDF2Bullet& u2b, URDF2BulletCachedData& cache, int linkIndex, const btTransform& parentTransformInWorldSpace, btMultiBodyDynamicsWorld* world1,const URDF2BulletConfig& mappings, const char* pathPrefix);
    
#endif //_URDF2BULLET_H

