#ifndef BULLET_MJCF_IMPORTER_H
#define BULLET_MJCF_IMPORTER_H

#include "../ImportURDFDemo/URDFImporterInterface.h"
#include "../ImportURDFDemo/UrdfRenderingInterface.h"

struct MJCFErrorLogger
{
	virtual ~MJCFErrorLogger() {}
	virtual void reportError(const char* error) = 0;
	virtual void reportWarning(const char* warning) = 0;
	virtual void printMessage(const char* msg) = 0;
};

struct MJCFURDFTexture
{
	int m_width;
	int m_height;
	unsigned char* textureData1;
	bool m_isCached;
};

class BulletMJCFImporter : public URDFImporterInterface
{
	struct BulletMJCFImporterInternalData* m_data;

	void convertURDFToVisualShapeInternal(const struct UrdfVisual* visual, const char* urdfPathPrefix, const btTransform& visualTransform, btAlignedObjectArray<struct GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<MJCFURDFTexture>& texturesOut) const;

public:
	BulletMJCFImporter(struct GUIHelperInterface* helper, UrdfRenderingInterface* customConverter,  struct CommonFileIOInterface* fileIO, int flags);
	virtual ~BulletMJCFImporter();

	virtual bool parseMJCFString(const char* xmlString, MJCFErrorLogger* logger);

	virtual bool loadMJCF(const char* fileName, MJCFErrorLogger* logger, bool forceFixedBase = false);

	virtual bool loadURDF(const char* fileName, bool forceFixedBase = false)
	{
		return false;
	}

	virtual bool loadSDF(const char* fileName, bool forceFixedBase = false) { return false; }

	virtual const char* getPathPrefix();

	///return >=0 for the root link index, -1 if there is no root link
	virtual int getRootLinkIndex() const;

	///pure virtual interfaces, precondition is a valid linkIndex (you can assert/terminate if the linkIndex is out of range)
	virtual std::string getLinkName(int linkIndex) const;

	virtual std::string getBodyName() const;

	/// optional method to provide the link color. return true if the color is available and copied into colorRGBA, return false otherwise
	virtual bool getLinkColor(int linkIndex, btVector4& colorRGBA) const;
	bool getLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const;

	//optional method to get collision group (type) and mask (affinity)
	virtual int getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const;

	///this API will likely change, don't override it!
	virtual bool getLinkContactInfo(int linkIndex, URDFLinkContactInfo& contactInfo) const;

	virtual std::string getJointName(int linkIndex) const;

	//fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
	virtual void getMassAndInertia(int urdfLinkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame) const;

	///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
	virtual void getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const;

	virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const;
	virtual bool getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const;

	virtual bool getRootTransformInWorld(btTransform& rootTransformInWorld) const;

	virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame) const;

	virtual void convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& inertialFrame, class btCollisionObject* colObj, int objectIndex) const;
	virtual void setBodyUniqueId(int bodyId);
	virtual int getBodyUniqueId() const;

	virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;
	virtual int getNumAllocatedCollisionShapes() const;
	virtual class btCollisionShape* getAllocatedCollisionShape(int index);
	virtual int getNumModels() const;
	virtual void activateModel(int modelIndex);

	virtual int getNumAllocatedMeshInterfaces() const;
	virtual btStridingMeshInterface* getAllocatedMeshInterface(int index);
};

#endif  //BULLET_MJCF_IMPORTER_H
