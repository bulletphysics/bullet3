#ifndef URDF_IMPORTER_INTERFACE_H
#define URDF_IMPORTER_INTERFACE_H

#include <string>
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "URDFJointTypes.h"
#include "SDFAudioTypes.h"

class URDFImporterInterface
{

public:


	virtual ~URDFImporterInterface() {}
	
    virtual bool loadURDF(const char* fileName, bool forceFixedBase = false)=0;

    virtual bool loadSDF(const char* fileName, bool forceFixedBase = false) { return false;}

    virtual const char* getPathPrefix()=0;
    
    ///return >=0 for the root link index, -1 if there is no root link
    virtual int getRootLinkIndex() const = 0;
    
    ///pure virtual interfaces, precondition is a valid linkIndex (you can assert/terminate if the linkIndex is out of range)
    virtual std::string getLinkName(int linkIndex) const =0;

	//various derived class in internal source code break with new pure virtual methods, so provide some default implementation
	virtual std::string getBodyName() const
	{
		return "";
	}
    
	/// optional method to provide the link color. return true if the color is available and copied into colorRGBA, return false otherwise
	virtual bool getLinkColor(int linkIndex, btVector4& colorRGBA) const { return false;}

	virtual bool getLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const { return false;}
	virtual void setLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const {}

	virtual int getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const { return 0;}
	///this API will likely change, don't override it!
	virtual bool getLinkContactInfo(int linkIndex, URDFLinkContactInfo& contactInfo ) const  { return false;}
    
	virtual bool getLinkAudioSource(int linkIndex, SDFAudioSource& audioSource) const { return false;}

    virtual std::string getJointName(int linkIndex) const = 0;

    //fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
    virtual void  getMassAndInertia (int urdfLinkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const =0;
	virtual void  getMassAndInertia2(int urdfLinkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame, int flags) const
	{
		getMassAndInertia(urdfLinkIndex, mass, localInertiaDiagonal, inertialFrame);
	}

    ///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
    virtual void getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const =0;
    
    virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const =0;

	virtual bool getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const 
	{
		//backwards compatibility for custom file importers
		jointMaxForce = 0;
		jointMaxVelocity = 0;
		return getJointInfo(urdfLinkIndex, parent2joint, linkTransformInWorld, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction);
	};
    
    virtual bool getRootTransformInWorld(btTransform& rootTransformInWorld) const =0;
	virtual void setRootTransformInWorld(const btTransform& rootTransformInWorld){}

	///quick hack: need to rethink the API/dependencies of this
    virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame) const { return -1;}
    
    virtual void convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& inertialFrame, class btCollisionObject* colObj, int objectIndex) const  { }
    virtual void setBodyUniqueId(int bodyId) {}
    virtual int getBodyUniqueId() const { return 0;}

   //default implementation for backward compatibility 
	virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const  = 0;
	virtual int getUrdfFromCollisionShape(const class btCollisionShape* collisionShape, struct UrdfCollision& collision) const
	{
		return 0;
	}

	virtual int getNumAllocatedCollisionShapes() const { return 0;}
    virtual class btCollisionShape* getAllocatedCollisionShape(int /*index*/ ) {return 0;}
	virtual int getNumModels() const {return 0;}
    virtual void activateModel(int /*modelIndex*/) { }
	virtual int getNumAllocatedMeshInterfaces() const { return 0;}

	virtual int getNumAllocatedTextures() const { return 0; }
	virtual int getAllocatedTexture(int index) const { return 0; }

	virtual class btStridingMeshInterface* getAllocatedMeshInterface(int index) {return 0;}

};

#endif //URDF_IMPORTER_INTERFACE_H

