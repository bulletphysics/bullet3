#ifndef BULLET_URDF_IMPORTER_H
#define BULLET_URDF_IMPORTER_H 

#include "URDFImporterInterface.h"


class BulletURDFImporter : public URDFImporterInterface
{
    
	struct BulletURDFInternalData* m_data;
    

public:

	BulletURDFImporter(struct GUIHelperInterface* guiHelper);

	virtual ~BulletURDFImporter();

	virtual bool loadURDF(const char* fileName, bool forceFixedBase = false);

	const char* getPathPrefix();

	void printTree(); //for debugging
	
	virtual int getRootLinkIndex() const;
    
    virtual void getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const;

    virtual std::string getLinkName(int linkIndex) const;

	virtual bool getLinkColor(int linkIndex, btVector4& colorRGBA) const;
    
    virtual std::string getJointName(int linkIndex) const;
    
    virtual void  getMassAndInertia(int linkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const;

    virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit) const;

	virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;

	virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;

};


#endif //BULLET_URDF_IMPORTER_H
