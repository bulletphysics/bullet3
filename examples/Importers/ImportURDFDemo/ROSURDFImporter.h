#ifndef ROS_URDF_IMPORTER_H
#define ROS_URDF_IMPORTER_H 

#include "URDFImporterInterface.h"


class ROSURDFImporter : public URDFImporterInterface
{
    
	struct ROSURDFInternalData* m_data;
    

public:

	ROSURDFImporter(struct GUIHelperInterface* guiHelper);

	virtual ~ROSURDFImporter();

	// Note that forceFixedBase is ignored in this implementation.
	virtual bool loadURDF(const char* fileName, bool forceFixedBase = false);

	virtual const char* getPathPrefix();

	void printTree(); //for debugging
	
	virtual int getRootLinkIndex() const;
    
    virtual void getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const;

    virtual std::string getLinkName(int linkIndex) const;
    
    virtual std::string getJointName(int linkIndex) const;
    
    virtual void  getMassAndInertia(int linkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const;

    virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit) const;

	virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;

	virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;

};


#endif //ROS_URDF_IMPORTER_H
