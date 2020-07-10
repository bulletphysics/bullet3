#ifndef BULLET_URDF_IMPORTER_H
#define BULLET_URDF_IMPORTER_H

#include "URDFImporterInterface.h"
#include "UrdfRenderingInterface.h"
#include "UrdfParser.h"

struct BulletURDFTexture
{
	int m_width;
	int m_height;
	unsigned char* textureData1;
	bool m_isCached;
};

///BulletURDFImporter can deal with URDF and (soon) SDF files
class BulletURDFImporter : public URDFImporterInterface
{
	struct BulletURDFInternalData* m_data;

public:
	BulletURDFImporter(struct GUIHelperInterface* helper, UrdfRenderingInterface* customConverter, struct CommonFileIOInterface* fileIO=0,double globalScaling=1, int flags=0);

	virtual ~BulletURDFImporter();

	virtual bool loadURDF(const char* fileName, bool forceFixedBase = false);

	//warning: some quick test to load SDF: we 'activate' a model, so we can re-use URDF code path
	virtual bool loadSDF(const char* fileName, bool forceFixedBase = false);
	virtual int getNumModels() const;
	virtual void activateModel(int modelIndex);
	virtual void setBodyUniqueId(int bodyId);
	virtual int getBodyUniqueId() const;
	const char* getPathPrefix();

	void printTree();  //for debugging

	virtual int getRootLinkIndex() const;

	virtual void getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const;

	virtual std::string getBodyName() const;

	virtual std::string getLinkName(int linkIndex) const;

	virtual bool getLinkColor(int linkIndex, btVector4& colorRGBA) const;

	virtual bool getLinkColor2(int linkIndex, UrdfMaterialColor& matCol) const;

	virtual void setLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const;

	virtual bool getLinkContactInfo(int urdflinkIndex, URDFLinkContactInfo& contactInfo) const;

	virtual bool getLinkAudioSource(int linkIndex, SDFAudioSource& audioSource) const;

	virtual std::string getJointName(int linkIndex) const;

	virtual void getMassAndInertia(int linkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame) const;
	virtual void getMassAndInertia2(int urdfLinkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame, int flags) const;

	virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const;
	virtual bool getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const;

	virtual bool getRootTransformInWorld(btTransform& rootTransformInWorld) const;
	virtual void setRootTransformInWorld(const btTransform& rootTransformInWorld);

	virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame) const;

	virtual void convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& inertialFrame, class btCollisionObject* colObj, int bodyUniqueId) const;

	class btCollisionShape* convertURDFToCollisionShape(const struct UrdfCollision* collision, const char* urdfPathPrefix) const;

	virtual int getUrdfFromCollisionShape(const btCollisionShape* collisionShape, UrdfCollision& collision) const;

	///todo(erwincoumans) refactor this convertLinkCollisionShapes/memory allocation

	virtual const struct UrdfModel* getUrdfModel() const;

	virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;

	virtual int getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const;

	virtual int getNumAllocatedCollisionShapes() const;
	virtual class btCollisionShape* getAllocatedCollisionShape(int index);

	virtual int getNumAllocatedMeshInterfaces() const;
	virtual class btStridingMeshInterface* getAllocatedMeshInterface(int index);

	virtual int getNumAllocatedTextures() const;
	virtual int getAllocatedTexture(int index) const;

	virtual void setEnableTinyRenderer(bool enable);
	void convertURDFToVisualShapeInternal(const struct UrdfVisual* visual, const char* urdfPathPrefix, const class btTransform& visualTransform, btAlignedObjectArray<struct GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<struct BulletURDFTexture>& texturesOut, struct b3ImportMeshData& meshData) const;
	const struct UrdfDeformable& getDeformableModel() const;
};

#endif  //BULLET_URDF_IMPORTER_H
