#ifndef MY_URDF_IMPORTER_H
#define MY_URDF_IMPORTER_H 

#include "URDFImporterInterface.h"
#include <vector> //temp, replace by btAlignedObjectArray

#include "urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h"

class MyURDFImporter : public URDFImporterInterface
{
    
	struct MyURDFInternalData* m_data;
    

public:
    

	MyURDFImporter(my_shared_ptr<urdf::ModelInterface> robot,struct GUIHelperInterface* helper);
    
	virtual ~MyURDFImporter();
	
    virtual int getRootLinkIndex() const;
    
    virtual void getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const;

    virtual std::string getLinkName(int linkIndex) const;
    
    virtual std::string getJointName(int linkIndex) const;
    
    virtual void  getMassAndInertia(int linkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const;

    virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit) const;

	virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame) const;

	virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;

};


#endif //MY_URDF_IMPORTER_H
