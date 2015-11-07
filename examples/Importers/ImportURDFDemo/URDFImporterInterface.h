#ifndef URDF_IMPORTER_INTERFACE_H
#define URDF_IMPORTER_INTERFACE_H

#include <string>
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "URDFJointTypes.h"

class URDFImporterInterface
{

public:


	virtual ~URDFImporterInterface() {}
	
 
    virtual bool loadURDF(const char* fileName, bool forceFixedBase = false)=0;

    virtual const char* getPathPrefix()=0;
    
    ///return >=0 for the root link index, -1 if there is no root link
    virtual int getRootLinkIndex() const = 0;
    
    ///pure virtual interfaces, precondition is a valid linkIndex (you can assert/terminate if the linkIndex is out of range)
    virtual std::string getLinkName(int linkIndex) const =0;
	/// optional method to provide the link color. return true if the color is available and copied into colorRGBA, return false otherwise
	virtual bool getLinkColor(int linkIndex, btVector4& colorRGBA) const { return false;}
    
    virtual std::string getJointName(int linkIndex) const = 0;

    //fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
    virtual void  getMassAndInertia(int urdfLinkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const =0;
    
    ///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
    virtual void getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const =0;
    
    virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit) const =0;
    
	///quick hack: need to rethink the API/dependencies of this
	virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame) const = 0;

	 virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const  = 0;
};

#endif //URDF_IMPORTER_INTERFACE_H

