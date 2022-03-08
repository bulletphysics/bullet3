/* Copyright (C) 2015 Google

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BulletUrdfImporter.h"
#include "../../CommonInterfaces/CommonRenderInterface.h"
#include "../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include "URDFImporterInterface.h"
#include "btBulletCollisionCommon.h"
#include "../ImportObjDemo/LoadMeshFromObj.h"
#include "../ImportSTLDemo/LoadMeshFromSTL.h"
#include "../ImportColladaDemo/LoadMeshFromCollada.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"  //to create a tesselation of a generic btConvexShape
#include "BulletCollision/CollisionShapes/btSdfCollisionShape.h"
#include "../../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../../CommonInterfaces/CommonFileIOInterface.h"
#include "Bullet3Common/b3FileUtils.h"
#include <string>
#include "../../Utils/b3ResourcePath.h"
#include "../../Utils/b3BulletDefaultFileIO.h"

#include "../OpenGLWindow/ShapeData.h"

#include "URDF2Bullet.h"  //for flags
#include "../ImportMeshUtility/b3ImportMeshUtility.h"

static btScalar gUrdfDefaultCollisionMargin = 0.001;

#include <iostream>
#include <fstream>
#include <list>
#include "UrdfParser.h"



ATTRIBUTE_ALIGNED16(struct)
BulletURDFInternalData
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	b3BulletDefaultFileIO m_defaultFileIO;
	UrdfParser m_urdfParser;
	struct GUIHelperInterface* m_guiHelper;
	struct CommonFileIOInterface* m_fileIO;
	std::string m_sourceFile;
	char m_pathPrefix[1024];
	int m_bodyId;
	btHashMap<btHashInt, UrdfMaterialColor> m_linkColors;
	btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;
	btAlignedObjectArray<int> m_allocatedTextures;
	mutable btAlignedObjectArray<btTriangleMesh*> m_allocatedMeshInterfaces;
	btHashMap<btHashPtr, UrdfCollision> m_bulletCollisionShape2UrdfCollision;

	UrdfRenderingInterface* m_customVisualShapesConverter;
	bool m_enableTinyRenderer;
	int m_flags;

	void setSourceFile(const std::string& relativeFileName, const std::string& prefix)
	{
		m_sourceFile = relativeFileName;
		m_urdfParser.setSourceFile(relativeFileName);
		strncpy(m_pathPrefix, prefix.c_str(), sizeof(m_pathPrefix));
		m_pathPrefix[sizeof(m_pathPrefix) - 1] = 0;  // required, strncpy doesn't write zero on overflow
	}

	BulletURDFInternalData(CommonFileIOInterface* fileIO)
		:m_urdfParser(fileIO? fileIO : &m_defaultFileIO),
		m_fileIO(fileIO? fileIO : &m_defaultFileIO)
	{
		m_enableTinyRenderer = true;
		m_pathPrefix[0] = 0;
		m_flags = 0;
	}

	void setGlobalScaling(btScalar scaling)
	{
		m_urdfParser.setGlobalScaling(scaling);
	}


};

void BulletURDFImporter::printTree()
{
	//	btAssert(0);
}




BulletURDFImporter::BulletURDFImporter(struct GUIHelperInterface* helper, UrdfRenderingInterface* customConverter, struct CommonFileIOInterface* fileIO,double globalScaling, int flags)
{
	m_data = new BulletURDFInternalData(fileIO);
	m_data->setGlobalScaling(globalScaling);
	m_data->m_flags = flags;
	m_data->m_guiHelper = helper;
	m_data->m_customVisualShapesConverter = customConverter;
}

struct BulletErrorLogger : public ErrorLogger
{
	int m_numErrors;
	int m_numWarnings;

	BulletErrorLogger()
		: m_numErrors(0),
		  m_numWarnings(0)
	{
	}
	virtual void reportError(const char* error)
	{
		m_numErrors++;
		b3Error(error);
	}
	virtual void reportWarning(const char* warning)
	{
		m_numWarnings++;
		b3Warning(warning);
	}

	virtual void printMessage(const char* msg)
	{
		b3Printf(msg);
	}
};

bool BulletURDFImporter::loadURDF(const char* fileName, bool forceFixedBase)
{
	if (strlen(fileName) == 0)
		return false;

	//int argc=0;
	char relativeFileName[1024];

	b3FileUtils fu;

	//bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
	bool fileFound = m_data->m_fileIO->findResourcePath(fileName, relativeFileName, 1024);

	std::string xml_string;

	if (!fileFound)
	{
		b3Warning("URDF file '%s' not found\n", fileName);
		return false;
	}
	else
	{
		char path[1024];
		fu.extractPath(relativeFileName, path, sizeof(path));
		m_data->setSourceFile(relativeFileName, path);

		//read file
		int fileId = m_data->m_fileIO->fileOpen(relativeFileName,"r");


		char destBuffer[8192];
		char* line = 0;
		do
		{
			line = m_data->m_fileIO->readLine(fileId, destBuffer, 8192);
			if (line)
			{
				xml_string += (std::string(destBuffer) + "\n");
			}
		}
		while (line);
		m_data->m_fileIO->fileClose(fileId);
#if 0
		std::fstream xml_file(relativeFileName, std::fstream::in);
		while (xml_file.good())
		{
			std::string line;
			std::getline(xml_file, line);
			xml_string += (line + "\n");
		}
		xml_file.close();
#endif

	}

	BulletErrorLogger loggie;
	m_data->m_urdfParser.setParseSDF(false);
	bool result = false;

	if (xml_string.length())
	{
			result = m_data->m_urdfParser.loadUrdf(xml_string.c_str(), &loggie, forceFixedBase, (m_data->m_flags & CUF_PARSE_SENSORS));

			if (m_data->m_flags & CUF_IGNORE_VISUAL_SHAPES)
			{
				for (int i=0; i < m_data->m_urdfParser.getModel().m_links.size(); i++)
				{
					UrdfLink* linkPtr = *m_data->m_urdfParser.getModel().m_links.getAtIndex(i);
					linkPtr->m_visualArray.clear();
				}
			}
			if (m_data->m_flags & CUF_IGNORE_COLLISION_SHAPES)
			{
				for (int i=0; i < m_data->m_urdfParser.getModel().m_links.size(); i++)
				{
					UrdfLink* linkPtr = *m_data->m_urdfParser.getModel().m_links.getAtIndex(i);
					linkPtr->m_collisionArray.clear();
				}
			}
			if (m_data->m_urdfParser.getModel().m_rootLinks.size())
			{
				if (m_data->m_flags & CUF_MERGE_FIXED_LINKS)
				{
					m_data->m_urdfParser.mergeFixedLinks(m_data->m_urdfParser.getModel(), m_data->m_urdfParser.getModel().m_rootLinks[0], &loggie, forceFixedBase, 0);
					m_data->m_urdfParser.getModel().m_links.clear();
					m_data->m_urdfParser.getModel().m_joints.clear();
					m_data->m_urdfParser.recreateModel(m_data->m_urdfParser.getModel(), m_data->m_urdfParser.getModel().m_rootLinks[0], &loggie);
				}
				if (m_data->m_flags & CUF_PRINT_URDF_INFO)
				{
					m_data->m_urdfParser.printTree(m_data->m_urdfParser.getModel().m_rootLinks[0], &loggie, 0);
				}
			}
	}

	return result;
}

int BulletURDFImporter::getNumModels() const
{
	return m_data->m_urdfParser.getNumModels();
}

void BulletURDFImporter::activateModel(int modelIndex)
{
	m_data->m_urdfParser.activateModel(modelIndex);
}

bool BulletURDFImporter::loadSDF(const char* fileName, bool forceFixedBase)
{
	//int argc=0;
	char relativeFileName[1024];

	b3FileUtils fu;

	//bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
	bool fileFound = (m_data->m_fileIO->findResourcePath(fileName, relativeFileName, 1024));

	std::string xml_string;

	if (!fileFound)
	{
		b3Warning("SDF file '%s' not found\n", fileName);
		return false;
	}
	else
	{

		char path[1024];
		fu.extractPath(relativeFileName, path, sizeof(path));
		m_data->setSourceFile(relativeFileName, path);

		//read file
		int fileId = m_data->m_fileIO->fileOpen(relativeFileName,"r");

		char destBuffer[8192];
		char* line = 0;
		do
		{
			line = m_data->m_fileIO->readLine(fileId, destBuffer, 8192);
			if (line)
			{
				xml_string += (std::string(destBuffer) + "\n");
			}
		}
		while (line);
		m_data->m_fileIO->fileClose(fileId);
	}

	BulletErrorLogger loggie;
	//todo: quick test to see if we can re-use the URDF parser for SDF or not
	m_data->m_urdfParser.setParseSDF(true);
	bool result = false;
	if (xml_string.length())
	{
		result = m_data->m_urdfParser.loadSDF(xml_string.c_str(), &loggie);
	}

	return result;
}

const char* BulletURDFImporter::getPathPrefix()
{
	return m_data->m_pathPrefix;
}

void BulletURDFImporter::setBodyUniqueId(int bodyId)
{
	m_data->m_bodyId = bodyId;
}

int BulletURDFImporter::getBodyUniqueId() const
{
	return m_data->m_bodyId;
}

BulletURDFImporter::~BulletURDFImporter()
{
	delete m_data;
}

int BulletURDFImporter::getRootLinkIndex() const
{
	if (m_data->m_urdfParser.getModel().m_rootLinks.size() == 1)
	{
		return m_data->m_urdfParser.getModel().m_rootLinks[0]->m_linkIndex;
	}
	return -1;
};

void BulletURDFImporter::getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const
{
	childLinkIndices.resize(0);
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{
		const UrdfLink* link = *linkPtr;
		//int numChildren = m_data->m_urdfParser->getModel().m_links.getAtIndex(linkIndex)->

		for (int i = 0; i < link->m_childLinks.size(); i++)
		{
			int childIndex = link->m_childLinks[i]->m_linkIndex;
			childLinkIndices.push_back(childIndex);
		}
	}
}

std::string BulletURDFImporter::getLinkName(int linkIndex) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		return link->m_name;
	}
	return "";
}

std::string BulletURDFImporter::getBodyName() const
{
	return m_data->m_urdfParser.getModel().m_name;
}

std::string BulletURDFImporter::getJointName(int linkIndex) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		if (link->m_parentJoint)
		{
			return link->m_parentJoint->m_name;
		}
	}
	return "";
}

void BulletURDFImporter::getMassAndInertia2(int urdfLinkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame, int flags) const
{
	if (flags & CUF_USE_URDF_INERTIA)
	{
		getMassAndInertia(urdfLinkIndex, mass, localInertiaDiagonal, inertialFrame);
	}
	else
	{
		//the link->m_inertia is NOT necessarily aligned with the inertial frame
		//so an additional transform might need to be computed
		UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdfLinkIndex);

		btAssert(linkPtr);
		if (linkPtr)
		{
			UrdfLink* link = *linkPtr;
			btScalar linkMass;
			if (link->m_parentJoint == 0 && m_data->m_urdfParser.getModel().m_overrideFixedBase)
			{
				linkMass = 0.f;
			}
			else
			{
				linkMass = link->m_inertia.m_mass;
			}
			mass = linkMass;
			localInertiaDiagonal.setValue(0, 0, 0);
			inertialFrame.setOrigin(link->m_inertia.m_linkLocalFrame.getOrigin());
			inertialFrame.setBasis(link->m_inertia.m_linkLocalFrame.getBasis());
		}
		else
		{
			mass = 1.f;
			localInertiaDiagonal.setValue(1, 1, 1);
			inertialFrame.setIdentity();
		}
	}
}

void BulletURDFImporter::getMassAndInertia(int linkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame) const
{
	//the link->m_inertia is NOT necessarily aligned with the inertial frame
	//so an additional transform might need to be computed
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);

	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		btMatrix3x3 linkInertiaBasis;
		btScalar linkMass, principalInertiaX, principalInertiaY, principalInertiaZ;
		if (link->m_parentJoint == 0 && m_data->m_urdfParser.getModel().m_overrideFixedBase)
		{
			linkMass = 0.f;
			principalInertiaX = 0.f;
			principalInertiaY = 0.f;
			principalInertiaZ = 0.f;
			linkInertiaBasis.setIdentity();
		}
		else
		{
			linkMass = link->m_inertia.m_mass;
			if (link->m_inertia.m_ixy == 0.0 &&
				link->m_inertia.m_ixz == 0.0 &&
				link->m_inertia.m_iyz == 0.0)
			{
				principalInertiaX = link->m_inertia.m_ixx;
				principalInertiaY = link->m_inertia.m_iyy;
				principalInertiaZ = link->m_inertia.m_izz;
				linkInertiaBasis.setIdentity();
			}
			else
			{
				principalInertiaX = link->m_inertia.m_ixx;
				btMatrix3x3 inertiaTensor(link->m_inertia.m_ixx, link->m_inertia.m_ixy, link->m_inertia.m_ixz,
										  link->m_inertia.m_ixy, link->m_inertia.m_iyy, link->m_inertia.m_iyz,
										  link->m_inertia.m_ixz, link->m_inertia.m_iyz, link->m_inertia.m_izz);
				btScalar threshold = 1.0e-6;
				int numIterations = 30;
				inertiaTensor.diagonalize(linkInertiaBasis, threshold, numIterations);
				principalInertiaX = inertiaTensor[0][0];
				principalInertiaY = inertiaTensor[1][1];
				principalInertiaZ = inertiaTensor[2][2];
			}
		}
		mass = linkMass;
		if (principalInertiaX < 0 ||
			principalInertiaX > (principalInertiaY + principalInertiaZ) ||
			principalInertiaY < 0 ||
			principalInertiaY > (principalInertiaX + principalInertiaZ) ||
			principalInertiaZ < 0 ||
			principalInertiaZ > (principalInertiaX + principalInertiaY))
		{
			b3Warning("Bad inertia tensor properties, setting inertia to zero for link: %s\n", link->m_name.c_str());
			principalInertiaX = 0.f;
			principalInertiaY = 0.f;
			principalInertiaZ = 0.f;
			linkInertiaBasis.setIdentity();
		}
		localInertiaDiagonal.setValue(principalInertiaX, principalInertiaY, principalInertiaZ);
		inertialFrame.setOrigin(link->m_inertia.m_linkLocalFrame.getOrigin());
		inertialFrame.setBasis(link->m_inertia.m_linkLocalFrame.getBasis() * linkInertiaBasis);
	}
	else
	{
		mass = 1.f;
		localInertiaDiagonal.setValue(1, 1, 1);
		inertialFrame.setIdentity();
	}
}

bool BulletURDFImporter::getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const
{
	btScalar twistLimit;
	return getJointInfo3(urdfLinkIndex, parent2joint, linkTransformInWorld, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction, jointMaxForce, jointMaxVelocity, twistLimit);
}

bool BulletURDFImporter::getJointInfo3(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity, btScalar& twistLimit) const
{
	jointLowerLimit = 0.f;
	jointUpperLimit = 0.f;
	jointDamping = 0.f;
	jointFriction = 0.f;
	jointMaxForce = 0.f;
	jointMaxVelocity = 0.f;

	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdfLinkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		linkTransformInWorld = link->m_linkTransformInWorld;

		if (link->m_parentJoint)
		{
			UrdfJoint* pj = link->m_parentJoint;
			parent2joint = pj->m_parentLinkToJointTransform;
			jointType = pj->m_type;
			jointAxisInJointSpace = pj->m_localJointAxis;
			jointLowerLimit = pj->m_lowerLimit;
			jointUpperLimit = pj->m_upperLimit;
			jointDamping = pj->m_jointDamping;
			jointFriction = pj->m_jointFriction;
			jointMaxForce = pj->m_effortLimit;
			jointMaxVelocity = pj->m_velocityLimit;
			twistLimit = pj->m_twistLimit;
			return true;
		}
		else
		{
			parent2joint.setIdentity();
			return false;
		}
	}

	return false;
};

bool BulletURDFImporter::getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const
{
	btScalar jointMaxForce;
	btScalar jointMaxVelocity;
	return getJointInfo2(urdfLinkIndex, parent2joint, linkTransformInWorld, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction, jointMaxForce, jointMaxVelocity);
}

void BulletURDFImporter::setRootTransformInWorld(const btTransform& rootTransformInWorld)
{
	m_data->m_urdfParser.getModel().m_rootTransformInWorld = rootTransformInWorld;
}

bool BulletURDFImporter::getRootTransformInWorld(btTransform& rootTransformInWorld) const
{
	rootTransformInWorld = m_data->m_urdfParser.getModel().m_rootTransformInWorld;
	return true;
}

static btCollisionShape* createConvexHullFromShapes(const bt_tinyobj::attrib_t& attribute, std::vector<bt_tinyobj::shape_t>& shapes, const btVector3& geomScale, int flags)
{
	B3_PROFILE("createConvexHullFromShapes");
	btCompoundShape* compound = new btCompoundShape();
	compound->setMargin(gUrdfDefaultCollisionMargin);

	btTransform identity;
	identity.setIdentity();

	for (int s = 0; s < (int)shapes.size(); s++)
	{
		btConvexHullShape* convexHull = new btConvexHullShape();
		convexHull->setMargin(gUrdfDefaultCollisionMargin);
		bt_tinyobj::shape_t& shape = shapes[s];
		int faceCount = shape.mesh.indices.size();

		for (int f = 0; f < faceCount; f += 3)
		{
			btVector3 pt;
			pt.setValue(attribute.vertices[3 * shape.mesh.indices[f + 0].vertex_index + 0],
						attribute.vertices[3 * shape.mesh.indices[f + 0].vertex_index + 1],
						attribute.vertices[3 * shape.mesh.indices[f + 0].vertex_index + 2]);

			convexHull->addPoint(pt * geomScale, false);

			pt.setValue(attribute.vertices[3 * shape.mesh.indices[f + 1].vertex_index + 0],
						attribute.vertices[3 * shape.mesh.indices[f + 1].vertex_index + 1],
						attribute.vertices[3 * shape.mesh.indices[f + 1].vertex_index + 2]);
			convexHull->addPoint(pt * geomScale, false);

			pt.setValue(attribute.vertices[3 * shape.mesh.indices[f + 2].vertex_index + 0],
						attribute.vertices[3 * shape.mesh.indices[f + 2].vertex_index + 1],
						attribute.vertices[3 * shape.mesh.indices[f + 2].vertex_index + 2]);
			convexHull->addPoint(pt * geomScale, false);
		}

		convexHull->recalcLocalAabb();
		convexHull->optimizeConvexHull();
		if (flags & CUF_INITIALIZE_SAT_FEATURES)
		{
			convexHull->initializePolyhedralFeatures();
		}

		compound->addChildShape(identity, convexHull);
	}

	return compound;
}

int BulletURDFImporter::getUrdfFromCollisionShape(const btCollisionShape* collisionShape, UrdfCollision& collision) const
{
	UrdfCollision* col = m_data->m_bulletCollisionShape2UrdfCollision.find(collisionShape);
	if (col)
	{
		collision = *col;
		return 1;
	}
	return 0;
}

btCollisionShape* BulletURDFImporter::convertURDFToCollisionShape(const UrdfCollision* collision, const char* urdfPathPrefix) const
{
	BT_PROFILE("convertURDFToCollisionShape");

	btCollisionShape* shape = 0;

	switch (collision->m_geometry.m_type)
	{
		case URDF_GEOM_PLANE:
		{
			btVector3 planeNormal = collision->m_geometry.m_planeNormal;
			btScalar planeConstant = 0;  //not available?
			btStaticPlaneShape* plane = new btStaticPlaneShape(planeNormal, planeConstant);
			shape = plane;
			shape->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_CAPSULE:
		{
			btScalar radius = collision->m_geometry.m_capsuleRadius;
			btScalar height = collision->m_geometry.m_capsuleHeight;
			btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(radius, height);
			shape = capsuleShape;
			shape->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}

		case URDF_GEOM_CYLINDER:
		{
			btScalar cylRadius = collision->m_geometry.m_capsuleRadius;
			btScalar cylHalfLength = 0.5 * collision->m_geometry.m_capsuleHeight;
			if (m_data->m_flags & CUF_USE_IMPLICIT_CYLINDER)
			{
				btVector3 halfExtents(cylRadius, cylRadius, cylHalfLength);
				btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
				shape = cylZShape;
				shape->setMargin(gUrdfDefaultCollisionMargin);
			}
			else
			{
				btAlignedObjectArray<btVector3> vertices;
				//int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
				int numSteps = 32;
				for (int i = 0; i < numSteps; i++)
				{
					btVector3 vert(cylRadius * btSin(SIMD_2_PI * (float(i) / numSteps)), cylRadius * btCos(SIMD_2_PI * (float(i) / numSteps)), cylHalfLength);
					vertices.push_back(vert);
					vert[2] = -cylHalfLength;
					vertices.push_back(vert);
				}
				btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
				cylZShape->setMargin(gUrdfDefaultCollisionMargin);
				cylZShape->recalcLocalAabb();
				if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES)
				{
					cylZShape->initializePolyhedralFeatures();
				}
				cylZShape->optimizeConvexHull();
				shape = cylZShape;
			}

			break;
		}
		case URDF_GEOM_BOX:
		{
			btVector3 extents = collision->m_geometry.m_boxSize;
			btBoxShape* boxShape = new btBoxShape(extents * 0.5f);
			//btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
			if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES)
			{
				boxShape->initializePolyhedralFeatures();
			}
			shape = boxShape;
			shape->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_SPHERE:
		{
			btScalar radius = collision->m_geometry.m_sphereRadius;
			btSphereShape* sphereShape = new btSphereShape(radius);
			shape = sphereShape;
			shape->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_CDF:
		{
			char relativeFileName[1024];
			char pathPrefix[1024];
			pathPrefix[0] = 0;
			if (m_data->m_fileIO->findResourcePath(collision->m_geometry.m_meshFileName.c_str(), relativeFileName, 1024))
			{
				b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);

				btAlignedObjectArray<char> sdfData;
				{
					std::streampos fsize = 0;
					std::ifstream file(relativeFileName, std::ios::binary);
					if (file.good())
					{
						fsize = file.tellg();
						file.seekg(0, std::ios::end);
						fsize = file.tellg() - fsize;
						file.seekg(0, std::ios::beg);
						sdfData.resize(fsize);
						int bytesRead = file.rdbuf()->sgetn(&sdfData[0], fsize);
						btAssert(bytesRead == fsize);
						file.close();
					}
				}

				if (sdfData.size())
				{
					btSdfCollisionShape* sdfShape = new btSdfCollisionShape();
					bool valid = sdfShape->initializeSDF(&sdfData[0], sdfData.size());
					btAssert(valid);

					if (valid)
					{
						shape = sdfShape;
					}
					else
					{
						delete sdfShape;
					}
				}
			}
			break;
		}
		case URDF_GEOM_MESH:
		{
			GLInstanceGraphicsShape* glmesh = 0;
			switch (collision->m_geometry.m_meshFileType)
			{
				case UrdfGeometry::FILE_OBJ:
					if (collision->m_flags & URDF_FORCE_CONCAVE_TRIMESH)
					{
						char relativeFileName[1024];
						char pathPrefix[1024];
						pathPrefix[0] = 0;
						if (m_data->m_fileIO->findResourcePath(collision->m_geometry.m_meshFileName.c_str(), relativeFileName, 1024))
						{
							b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
						}
						glmesh = LoadMeshFromObj(collision->m_geometry.m_meshFileName.c_str(), pathPrefix,m_data->m_fileIO);
					}
					else
					{
						std::vector<bt_tinyobj::shape_t> shapes;
						bt_tinyobj::attrib_t attribute;
						std::string err = bt_tinyobj::LoadObj(attribute, shapes, collision->m_geometry.m_meshFileName.c_str(), "", m_data->m_fileIO);
						//create a convex hull for each shape, and store it in a btCompoundShape
						shape = createConvexHullFromShapes(attribute, shapes, collision->m_geometry.m_meshScale, m_data->m_flags);
						m_data->m_bulletCollisionShape2UrdfCollision.insert(shape, *collision);
						return shape;
					}
					break;

				case UrdfGeometry::FILE_STL:
					glmesh = LoadMeshFromSTL(collision->m_geometry.m_meshFileName.c_str(), m_data->m_fileIO);
					break;

				case UrdfGeometry::FILE_COLLADA:
				{
					btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
					btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
					btTransform upAxisTrans;
					upAxisTrans.setIdentity();
					float unitMeterScaling = 1;
					LoadMeshFromCollada(collision->m_geometry.m_meshFileName.c_str(), visualShapes, visualShapeInstances, upAxisTrans, unitMeterScaling, 2, m_data->m_fileIO);

					glmesh = new GLInstanceGraphicsShape;
					glmesh->m_indices = new b3AlignedObjectArray<int>();
					glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

					for (int i = 0; i < visualShapeInstances.size(); i++)
					{
						ColladaGraphicsInstance* instance = &visualShapeInstances[i];
						GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];

						b3AlignedObjectArray<GLInstanceVertex> verts;
						verts.resize(gfxShape->m_vertices->size());

						int baseIndex = glmesh->m_vertices->size();

						for (int i = 0; i < gfxShape->m_vertices->size(); i++)
						{
							verts[i].normal[0] = gfxShape->m_vertices->at(i).normal[0];
							verts[i].normal[1] = gfxShape->m_vertices->at(i).normal[1];
							verts[i].normal[2] = gfxShape->m_vertices->at(i).normal[2];
							verts[i].uv[0] = gfxShape->m_vertices->at(i).uv[0];
							verts[i].uv[1] = gfxShape->m_vertices->at(i).uv[1];
							verts[i].xyzw[0] = gfxShape->m_vertices->at(i).xyzw[0];
							verts[i].xyzw[1] = gfxShape->m_vertices->at(i).xyzw[1];
							verts[i].xyzw[2] = gfxShape->m_vertices->at(i).xyzw[2];
							verts[i].xyzw[3] = gfxShape->m_vertices->at(i).xyzw[3];
						}

						int curNumIndices = glmesh->m_indices->size();
						int additionalIndices = gfxShape->m_indices->size();
						glmesh->m_indices->resize(curNumIndices + additionalIndices);
						for (int k = 0; k < additionalIndices; k++)
						{
							glmesh->m_indices->at(curNumIndices + k) = gfxShape->m_indices->at(k) + baseIndex;
						}

						//compensate upAxisTrans and unitMeterScaling here
						btMatrix4x4 upAxisMat;
						upAxisMat.setIdentity();
						//upAxisMat.setPureRotation(upAxisTrans.getRotation());
						btMatrix4x4 unitMeterScalingMat;
						unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling, unitMeterScaling, unitMeterScaling));
						btMatrix4x4 worldMat = unitMeterScalingMat * instance->m_worldTransform * upAxisMat;
						//btMatrix4x4 worldMat = instance->m_worldTransform;
						int curNumVertices = glmesh->m_vertices->size();
						int additionalVertices = verts.size();
						glmesh->m_vertices->reserve(curNumVertices + additionalVertices);

						for (int v = 0; v < verts.size(); v++)
						{
							btVector3 pos(verts[v].xyzw[0], verts[v].xyzw[1], verts[v].xyzw[2]);
							pos = worldMat * pos;
							verts[v].xyzw[0] = float(pos[0]);
							verts[v].xyzw[1] = float(pos[1]);
							verts[v].xyzw[2] = float(pos[2]);
							glmesh->m_vertices->push_back(verts[v]);
						}
					}
					glmesh->m_numIndices = glmesh->m_indices->size();
					glmesh->m_numvertices = glmesh->m_vertices->size();
					//glmesh = LoadMeshFromCollada(success.c_str());
					break;
				}
			}

			if (!glmesh || glmesh->m_numvertices <= 0)
			{
				b3Warning("%s: cannot extract mesh from '%s'\n", urdfPathPrefix, collision->m_geometry.m_meshFileName.c_str());
				delete glmesh;
				break;
			}

			btAlignedObjectArray<btVector3> convertedVerts;
			convertedVerts.reserve(glmesh->m_numvertices);
			for (int i = 0; i < glmesh->m_numvertices; i++)
			{
				convertedVerts.push_back(btVector3(
					glmesh->m_vertices->at(i).xyzw[0] * collision->m_geometry.m_meshScale[0],
					glmesh->m_vertices->at(i).xyzw[1] * collision->m_geometry.m_meshScale[1],
					glmesh->m_vertices->at(i).xyzw[2] * collision->m_geometry.m_meshScale[2]));
			}

			if (collision->m_flags & URDF_FORCE_CONCAVE_TRIMESH)
			{
				BT_PROFILE("convert trimesh");
				btTriangleMesh* meshInterface = new btTriangleMesh();
				m_data->m_allocatedMeshInterfaces.push_back(meshInterface);
				{
					BT_PROFILE("convert vertices");

					for (int i = 0; i < glmesh->m_numIndices / 3; i++)
					{
						const btVector3& v0 = convertedVerts[glmesh->m_indices->at(i * 3)];
						const btVector3& v1 = convertedVerts[glmesh->m_indices->at(i * 3 + 1)];
						const btVector3& v2 = convertedVerts[glmesh->m_indices->at(i * 3 + 2)];
						meshInterface->addTriangle(v0, v1, v2);
					}
				}
				{
					BT_PROFILE("create btBvhTriangleMeshShape");
					btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface, true, true);
					//trimesh->setLocalScaling(collision->m_geometry.m_meshScale);
					shape = trimesh;
				}
			}
			else
			{
				BT_PROFILE("convert btConvexHullShape");
				btConvexHullShape* convexHull = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
				convexHull->optimizeConvexHull();
				if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES)
				{
					convexHull->initializePolyhedralFeatures();
				}
				convexHull->setMargin(gUrdfDefaultCollisionMargin);
				convexHull->recalcLocalAabb();
				//convexHull->setLocalScaling(collision->m_geometry.m_meshScale);
				shape = convexHull;
			}

			delete glmesh;
			break;
		}  // mesh case

		default:
			b3Warning("Error: unknown collision geometry type %i\n", collision->m_geometry.m_type);
	}
	if (shape && collision->m_geometry.m_type == URDF_GEOM_MESH)
	{
		m_data->m_bulletCollisionShape2UrdfCollision.insert(shape, *collision);
	}
	return shape;
}

void BulletURDFImporter::convertURDFToVisualShapeInternal(const UrdfVisual* visual, const char* urdfPathPrefix, const btTransform& visualTransform, btAlignedObjectArray<GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<BulletURDFTexture>& texturesOut, struct b3ImportMeshData& meshData) const
{
	BT_PROFILE("convertURDFToVisualShapeInternal");

	GLInstanceGraphicsShape* glmesh = 0;

	btConvexShape* convexColShape = 0;

	switch (visual->m_geometry.m_type)
	{
        case URDF_GEOM_CAPSULE:
        {
           btScalar radius = visual->m_geometry.m_capsuleRadius;
			btScalar height = visual->m_geometry.m_capsuleHeight;
			btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(radius, height);
			convexColShape = capsuleShape;
			convexColShape->setMargin(gUrdfDefaultCollisionMargin);
            break;
        }
		case URDF_GEOM_CYLINDER:
		{
			btAlignedObjectArray<btVector3> vertices;

			//int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
			int numSteps = 32;
			for (int i = 0; i < numSteps; i++)
			{
				btScalar cylRadius = visual->m_geometry.m_capsuleRadius;
				btScalar cylLength = visual->m_geometry.m_capsuleHeight;

				btVector3 vert(cylRadius * btSin(SIMD_2_PI * (float(i) / numSteps)), cylRadius * btCos(SIMD_2_PI * (float(i) / numSteps)), cylLength / 2.);
				vertices.push_back(vert);
				vert[2] = -cylLength / 2.;
				vertices.push_back(vert);
			}

			btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
			cylZShape->setMargin(gUrdfDefaultCollisionMargin);
			cylZShape->recalcLocalAabb();
			convexColShape = cylZShape;
			break;
		}

		case URDF_GEOM_BOX:
		{
			btVector3 extents = visual->m_geometry.m_boxSize;
			int strideInBytes = 9 * sizeof(float);
			int numVertices = sizeof(cube_vertices_textured) / strideInBytes;
			int numIndices = sizeof(cube_indices) / sizeof(int);
			glmesh = new GLInstanceGraphicsShape;
			glmesh->m_indices = new b3AlignedObjectArray<int>();
			glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();
			glmesh->m_indices->resize(numIndices);
			for (int k = 0; k < numIndices; k++)
			{
				glmesh->m_indices->at(k) = cube_indices[k];
			}
			glmesh->m_vertices->resize(numVertices);

			btScalar halfExtentsX = extents[0] * 0.5;
			btScalar halfExtentsY = extents[1] * 0.5;
			btScalar halfExtentsZ = extents[2] * 0.5;
			GLInstanceVertex* verts = &glmesh->m_vertices->at(0);
			btScalar textureScaling = 1;

			for (int i = 0; i < numVertices; i++)
			{
				verts[i].xyzw[0] = halfExtentsX * cube_vertices_textured[i * 9];
				verts[i].xyzw[1] = halfExtentsY * cube_vertices_textured[i * 9 + 1];
				verts[i].xyzw[2] = halfExtentsZ * cube_vertices_textured[i * 9 + 2];
				verts[i].xyzw[3] = cube_vertices_textured[i * 9 + 3];
				verts[i].normal[0] = cube_vertices_textured[i * 9 + 4];
				verts[i].normal[1] = cube_vertices_textured[i * 9 + 5];
				verts[i].normal[2] = cube_vertices_textured[i * 9 + 6];
				verts[i].uv[0] = cube_vertices_textured[i * 9 + 7] * textureScaling;
				verts[i].uv[1] = cube_vertices_textured[i * 9 + 8] * textureScaling;
			}
			
			glmesh->m_numIndices = numIndices;
			glmesh->m_numvertices = numVertices;
			break;
		}

		case URDF_GEOM_SPHERE:
		{
			btScalar radius = visual->m_geometry.m_sphereRadius;
			btSphereShape* sphereShape = new btSphereShape(radius);
			convexColShape = sphereShape;
			convexColShape->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}

		case URDF_GEOM_MESH:
		{
			switch (visual->m_geometry.m_meshFileType)
			{
				case UrdfGeometry::MEMORY_VERTICES:
				{
					glmesh = new GLInstanceGraphicsShape;
					//		int index = 0;
					glmesh->m_indices = new b3AlignedObjectArray<int>();
					glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();
					glmesh->m_vertices->resize(visual->m_geometry.m_vertices.size());
					glmesh->m_indices->resize(visual->m_geometry.m_indices.size());

					for (int i = 0; i < visual->m_geometry.m_vertices.size(); i++)
					{
						glmesh->m_vertices->at(i).xyzw[0] = visual->m_geometry.m_vertices[i].x();
						glmesh->m_vertices->at(i).xyzw[1] = visual->m_geometry.m_vertices[i].y();
						glmesh->m_vertices->at(i).xyzw[2] = visual->m_geometry.m_vertices[i].z();
						glmesh->m_vertices->at(i).xyzw[3] = 1;
						btVector3 normal(visual->m_geometry.m_vertices[i]);
						if (visual->m_geometry.m_normals.size() == visual->m_geometry.m_vertices.size())
						{
							normal = visual->m_geometry.m_normals[i];
						}
						else
						{
							normal.safeNormalize();
						}

						btVector3 uv(0.5, 0.5, 0);
						if (visual->m_geometry.m_uvs.size() == visual->m_geometry.m_vertices.size())
						{
							uv = visual->m_geometry.m_uvs[i];
						}
						glmesh->m_vertices->at(i).normal[0] = normal[0];
						glmesh->m_vertices->at(i).normal[1] = normal[1];
						glmesh->m_vertices->at(i).normal[2] = normal[2];
						glmesh->m_vertices->at(i).uv[0] = uv[0];
						glmesh->m_vertices->at(i).uv[1] = uv[1];

					}
					for (int i = 0; i < visual->m_geometry.m_indices.size(); i++)
					{
						glmesh->m_indices->at(i) = visual->m_geometry.m_indices[i];

					}
					glmesh->m_numIndices = visual->m_geometry.m_indices.size();
					glmesh->m_numvertices = visual->m_geometry.m_vertices.size();

					break;
				}
				case UrdfGeometry::FILE_OBJ:
				{

					if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(visual->m_geometry.m_meshFileName, meshData, m_data->m_fileIO))
					{
						if (meshData.m_textureImage1)
						{
							BulletURDFTexture texData;
							texData.m_width = meshData.m_textureWidth;
							texData.m_height = meshData.m_textureHeight;
							texData.textureData1 = meshData.m_textureImage1;
							texData.m_isCached = meshData.m_isCached;
							texturesOut.push_back(texData);
						}
						glmesh = meshData.m_gfxShape;
					}
					break;
				}

				case UrdfGeometry::FILE_STL:
				{
					glmesh = LoadMeshFromSTL(visual->m_geometry.m_meshFileName.c_str(),m_data->m_fileIO);
					break;
				}

				case UrdfGeometry::FILE_COLLADA:
				{
					btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
					btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
					btTransform upAxisTrans;
					upAxisTrans.setIdentity();
					float unitMeterScaling = 1;
					int upAxis = 2;

					LoadMeshFromCollada(visual->m_geometry.m_meshFileName.c_str(),
										visualShapes,
										visualShapeInstances,
										upAxisTrans,
										unitMeterScaling,
										upAxis,
										m_data->m_fileIO);

					glmesh = new GLInstanceGraphicsShape;
					//		int index = 0;
					glmesh->m_indices = new b3AlignedObjectArray<int>();
					glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

					for (int i = 0; i < visualShapeInstances.size(); i++)
					{
						ColladaGraphicsInstance* instance = &visualShapeInstances[i];
						GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];

						b3AlignedObjectArray<GLInstanceVertex> verts;
						verts.resize(gfxShape->m_vertices->size());

						int baseIndex = glmesh->m_vertices->size();

						for (int i = 0; i < gfxShape->m_vertices->size(); i++)
						{
							verts[i].normal[0] = gfxShape->m_vertices->at(i).normal[0];
							verts[i].normal[1] = gfxShape->m_vertices->at(i).normal[1];
							verts[i].normal[2] = gfxShape->m_vertices->at(i).normal[2];
							verts[i].uv[0] = gfxShape->m_vertices->at(i).uv[0];
							verts[i].uv[1] = gfxShape->m_vertices->at(i).uv[1];
							verts[i].xyzw[0] = gfxShape->m_vertices->at(i).xyzw[0];
							verts[i].xyzw[1] = gfxShape->m_vertices->at(i).xyzw[1];
							verts[i].xyzw[2] = gfxShape->m_vertices->at(i).xyzw[2];
							verts[i].xyzw[3] = gfxShape->m_vertices->at(i).xyzw[3];
						}

						int curNumIndices = glmesh->m_indices->size();
						int additionalIndices = gfxShape->m_indices->size();
						glmesh->m_indices->resize(curNumIndices + additionalIndices);
						for (int k = 0; k < additionalIndices; k++)
						{
							glmesh->m_indices->at(curNumIndices + k) = gfxShape->m_indices->at(k) + baseIndex;
						}

						//compensate upAxisTrans and unitMeterScaling here
						btMatrix4x4 upAxisMat;
						upAxisMat.setIdentity();
						//								upAxisMat.setPureRotation(upAxisTrans.getRotation());
						btMatrix4x4 unitMeterScalingMat;
						unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling, unitMeterScaling, unitMeterScaling));
						btMatrix4x4 worldMat = unitMeterScalingMat * upAxisMat * instance->m_worldTransform;
						//btMatrix4x4 worldMat = instance->m_worldTransform;
						int curNumVertices = glmesh->m_vertices->size();
						int additionalVertices = verts.size();
						glmesh->m_vertices->reserve(curNumVertices + additionalVertices);

						for (int v = 0; v < verts.size(); v++)
						{
							btVector3 pos(verts[v].xyzw[0], verts[v].xyzw[1], verts[v].xyzw[2]);
							pos = worldMat * pos;
							verts[v].xyzw[0] = float(pos[0]);
							verts[v].xyzw[1] = float(pos[1]);
							verts[v].xyzw[2] = float(pos[2]);
							glmesh->m_vertices->push_back(verts[v]);
						}
					}
					glmesh->m_numIndices = glmesh->m_indices->size();
					glmesh->m_numvertices = glmesh->m_vertices->size();
					//glmesh = LoadMeshFromCollada(visual->m_geometry.m_meshFileName);

					break;
				}
			}  // switch file type

			if (!glmesh || !glmesh->m_vertices || glmesh->m_numvertices <= 0)
			{
				b3Warning("%s: cannot extract anything useful from mesh '%s'\n", urdfPathPrefix, visual->m_geometry.m_meshFileName.c_str());
				break;
			}

			//apply the geometry scaling
			for (int i = 0; i < glmesh->m_vertices->size(); i++)
			{
				glmesh->m_vertices->at(i).xyzw[0] *= visual->m_geometry.m_meshScale[0];
				glmesh->m_vertices->at(i).xyzw[1] *= visual->m_geometry.m_meshScale[1];
				glmesh->m_vertices->at(i).xyzw[2] *= visual->m_geometry.m_meshScale[2];
			}
			break;
		}
		case URDF_GEOM_PLANE:
		{
			b3Warning("No default visual for URDF_GEOM_PLANE");
			break;
		}
		default:
		{
			b3Warning("Error: unknown visual geometry type %i\n", visual->m_geometry.m_type);
		}
	}

	//if we have a convex, tesselate into localVertices/localIndices
	if ((glmesh == 0) && convexColShape)
	{
		BT_PROFILE("convexColShape");

		btShapeHull* hull = new btShapeHull(convexColShape);
		hull->buildHull(0.0);
		{
			//	int strideInBytes = 9*sizeof(float);
			int numVertices = hull->numVertices();
			int numIndices = hull->numIndices();

			glmesh = new GLInstanceGraphicsShape;
			//	int index = 0;
			glmesh->m_indices = new b3AlignedObjectArray<int>();
			glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

			for (int i = 0; i < numVertices; i++)
			{
				GLInstanceVertex vtx;
				btVector3 pos = hull->getVertexPointer()[i];
				vtx.xyzw[0] = pos.x();
				vtx.xyzw[1] = pos.y();
				vtx.xyzw[2] = pos.z();
				vtx.xyzw[3] = 1.f;
				pos.normalize();
				vtx.normal[0] = pos.x();
				vtx.normal[1] = pos.y();
				vtx.normal[2] = pos.z();
				btScalar u = btAtan2(vtx.normal[0], vtx.normal[2]) / (2 * SIMD_PI) + 0.5;
				btScalar v = vtx.normal[1] * 0.5 + 0.5;
				vtx.uv[0] = u;
				vtx.uv[1] = v;
				glmesh->m_vertices->push_back(vtx);
			}

			btAlignedObjectArray<int> indices;
			for (int i = 0; i < numIndices; i++)
			{
				glmesh->m_indices->push_back(hull->getIndexPointer()[i]);
			}

			glmesh->m_numvertices = glmesh->m_vertices->size();
			glmesh->m_numIndices = glmesh->m_indices->size();
		}
		delete hull;
		delete convexColShape;
		convexColShape = 0;
	}

	if (glmesh && glmesh->m_numIndices > 0 && glmesh->m_numvertices > 0)
	{
		BT_PROFILE("glmesh");
		int baseIndex = verticesOut.size();

		for (int i = 0; i < glmesh->m_indices->size(); i++)
		{
			indicesOut.push_back(glmesh->m_indices->at(i) + baseIndex);
		}

		for (int i = 0; i < glmesh->m_vertices->size(); i++)
		{
			GLInstanceVertex& v = glmesh->m_vertices->at(i);
			btVector3 vert(v.xyzw[0], v.xyzw[1], v.xyzw[2]);
			btVector3 vt = visualTransform * vert;
			v.xyzw[0] = vt[0];
			v.xyzw[1] = vt[1];
			v.xyzw[2] = vt[2];
			btVector3 triNormal(v.normal[0], v.normal[1], v.normal[2]);
			triNormal = visualTransform.getBasis() * triNormal;
			v.normal[0] = triNormal[0];
			v.normal[1] = triNormal[1];
			v.normal[2] = triNormal[2];
			verticesOut.push_back(v);
		}
	}
	delete glmesh;
}

int BulletURDFImporter::convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
{
	int graphicsIndex = -1;
	btAlignedObjectArray<GLInstanceVertex> vertices;
	btAlignedObjectArray<int> indices;
	btTransform startTrans;
	startTrans.setIdentity();
	btAlignedObjectArray<BulletURDFTexture> textures;

	const UrdfModel& model = m_data->m_urdfParser.getModel();
	UrdfLink* const* linkPtr = model.m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{
		const UrdfLink* link = *linkPtr;

		for (int v = 0; v < link->m_visualArray.size(); v++)
		{
			const UrdfVisual& vis = link->m_visualArray[v];
			btTransform childTrans = vis.m_linkLocalFrame;
			btHashString matName(vis.m_materialName.c_str());
			UrdfMaterial* const* matPtr = model.m_materials[matName];
			b3ImportMeshData meshData;

			convertURDFToVisualShapeInternal(&vis, pathPrefix, localInertiaFrame.inverse() * childTrans, vertices, indices, textures,meshData);

			bool mtlOverridesUrdfColor = false;
			if ((meshData.m_flags & B3_IMPORT_MESH_HAS_RGBA_COLOR) &&
					(meshData.m_flags & B3_IMPORT_MESH_HAS_SPECULAR_COLOR))
			{
				mtlOverridesUrdfColor = (m_data->m_flags & CUF_USE_MATERIAL_COLORS_FROM_MTL) != 0;
				UrdfMaterialColor matCol;
				if (m_data->m_flags&CUF_USE_MATERIAL_TRANSPARANCY_FROM_MTL)
				{
					matCol.m_rgbaColor.setValue(meshData.m_rgbaColor[0],
								meshData.m_rgbaColor[1],
								meshData.m_rgbaColor[2],
								meshData.m_rgbaColor[3]);
				} else
				{
					matCol.m_rgbaColor.setValue(meshData.m_rgbaColor[0],
								meshData.m_rgbaColor[1],
								meshData.m_rgbaColor[2],
								1);
				}

				matCol.m_specularColor.setValue(meshData.m_specularColor[0],
					meshData.m_specularColor[1],
					meshData.m_specularColor[2]);
				m_data->m_linkColors.insert(linkIndex, matCol);
			}
			if (matPtr && !mtlOverridesUrdfColor)
			{
				UrdfMaterial* const mat = *matPtr;
				//printf("UrdfMaterial %s, rgba = %f,%f,%f,%f\n",mat->m_name.c_str(),mat->m_rgbaColor[0],mat->m_rgbaColor[1],mat->m_rgbaColor[2],mat->m_rgbaColor[3]);
				UrdfMaterialColor matCol;
				matCol.m_rgbaColor = mat->m_matColor.m_rgbaColor;
				matCol.m_specularColor = mat->m_matColor.m_specularColor;
				m_data->m_linkColors.insert(linkIndex, matCol);
			}
		}
	}
	if (vertices.size() && indices.size())
	{
		//		graphicsIndex  = m_data->m_guiHelper->registerGraphicsShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size());
		//graphicsIndex  = m_data->m_guiHelper->registerGraphicsShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size());

		//CommonRenderInterface* renderer = m_data->m_guiHelper->getRenderInterface();

		if (1)
		{
			int textureIndex = -1;
			if (textures.size())
			{
				textureIndex = m_data->m_guiHelper->registerTexture(textures[0].textureData1, textures[0].m_width, textures[0].m_height);
				if (textureIndex >= 0)
				{
					m_data->m_allocatedTextures.push_back(textureIndex);
				}
			}
			{
				B3_PROFILE("registerGraphicsShape");
				graphicsIndex = m_data->m_guiHelper->registerGraphicsShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size(), B3_GL_TRIANGLES, textureIndex);
			}
		}
	}

	//delete textures
	for (int i = 0; i < textures.size(); i++)
	{
		B3_PROFILE("free textureData");
		if (!textures[i].m_isCached)
		{
			free(textures[i].textureData1);
		}
	}
	return graphicsIndex;
}

bool BulletURDFImporter::getLinkColor(int linkIndex, btVector4& colorRGBA) const
{
	const UrdfMaterialColor* matColPtr = m_data->m_linkColors[linkIndex];
	if (matColPtr)
	{
		colorRGBA = matColPtr->m_rgbaColor;
		return true;
	}
	return false;
}

bool BulletURDFImporter::getLinkColor2(int linkIndex, UrdfMaterialColor& matCol) const
{
	UrdfMaterialColor* matColPtr = m_data->m_linkColors[linkIndex];
	if (matColPtr)
	{
		matCol = *matColPtr;
		return true;
	}
	return false;
}

void BulletURDFImporter::setLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const
{
	m_data->m_linkColors.insert(linkIndex, matCol);
}

bool BulletURDFImporter::getLinkContactInfo(int urdflinkIndex, URDFLinkContactInfo& contactInfo) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdflinkIndex);
	if (linkPtr)
	{
		const UrdfLink* link = *linkPtr;
		contactInfo = link->m_contactInfo;
		return true;
	}
	return false;
}

bool BulletURDFImporter::getLinkAudioSource(int linkIndex, SDFAudioSource& audioSource) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{
		const UrdfLink* link = *linkPtr;
		if (link->m_audioSource.m_flags & SDFAudioSource::SDFAudioSourceValid)
		{
			audioSource = link->m_audioSource;
			return true;
		}
	}
	return false;
}

void BulletURDFImporter::setEnableTinyRenderer(bool enable)
{
	m_data->m_enableTinyRenderer = enable;
}

void BulletURDFImporter::convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& localInertiaFrame, class btCollisionObject* colObj, int bodyUniqueId) const
{
	if (m_data->m_enableTinyRenderer && m_data->m_customVisualShapesConverter)
	{
		const UrdfModel& model = m_data->m_urdfParser.getModel();
		UrdfLink* const* linkPtr = model.m_links.getAtIndex(urdfIndex);
		if (linkPtr)
		{
			m_data->m_customVisualShapesConverter->setFlags(m_data->m_flags);
			
			int uid3 = m_data->m_customVisualShapesConverter->convertVisualShapes(linkIndex, pathPrefix, localInertiaFrame, *linkPtr, &model, colObj->getBroadphaseHandle()->getUid(), bodyUniqueId, m_data->m_fileIO);
			colObj->setUserIndex3(uid3);
		}
	}
}

int BulletURDFImporter::getNumAllocatedCollisionShapes() const
{
	return m_data->m_allocatedCollisionShapes.size();
}

btCollisionShape* BulletURDFImporter::getAllocatedCollisionShape(int index)
{
	return m_data->m_allocatedCollisionShapes[index];
}

int BulletURDFImporter::getNumAllocatedMeshInterfaces() const
{
	return m_data->m_allocatedMeshInterfaces.size();
}

btStridingMeshInterface* BulletURDFImporter::getAllocatedMeshInterface(int index)
{
	return m_data->m_allocatedMeshInterfaces[index];
}

int BulletURDFImporter::getNumAllocatedTextures() const
{
	return m_data->m_allocatedTextures.size();
}

int BulletURDFImporter::getAllocatedTexture(int index) const
{
	return m_data->m_allocatedTextures[index];
}

int BulletURDFImporter::getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const
{
	int result = 0;
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		for (int v = 0; v < link->m_collisionArray.size(); v++)
		{
			const UrdfCollision& col = link->m_collisionArray[v];
			if (col.m_flags & URDF_HAS_COLLISION_GROUP)
			{
				colGroup = col.m_collisionGroup;
				result |= URDF_HAS_COLLISION_GROUP;
			}
			if (col.m_flags & URDF_HAS_COLLISION_MASK)
			{
				colMask = col.m_collisionMask;
				result |= URDF_HAS_COLLISION_MASK;
			}
		}
	}
	return result;
}

class btCompoundShape* BulletURDFImporter::convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
{
	btCompoundShape* compoundShape = new btCompoundShape();
	m_data->m_allocatedCollisionShapes.push_back(compoundShape);

	compoundShape->setMargin(gUrdfDefaultCollisionMargin);
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;

		for (int v = 0; v < link->m_collisionArray.size(); v++)
		{
			const UrdfCollision& col = link->m_collisionArray[v];
			btCollisionShape* childShape = convertURDFToCollisionShape(&col, pathPrefix);
			if (childShape)
			{
				m_data->m_allocatedCollisionShapes.push_back(childShape);
				if (childShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
				{
					btCompoundShape* compound = (btCompoundShape*)childShape;
					for (int i = 0; i < compound->getNumChildShapes(); i++)
					{
						m_data->m_allocatedCollisionShapes.push_back(compound->getChildShape(i));
					}
				}

				btTransform childTrans = col.m_linkLocalFrame;

				compoundShape->addChildShape(localInertiaFrame.inverse() * childTrans, childShape);
			}
		}
	}

	return compoundShape;
}

const struct UrdfModel* BulletURDFImporter::getUrdfModel() const {
	return &m_data->m_urdfParser.getModel();
};

const struct UrdfDeformable& BulletURDFImporter::getDeformableModel() const
{
	return m_data->m_urdfParser.getDeformable();
}

const struct UrdfReducedDeformable& BulletURDFImporter::getReducedDeformableModel() const
{
	return m_data->m_urdfParser.getReducedDeformable();
}
