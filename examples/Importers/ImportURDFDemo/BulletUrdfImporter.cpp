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
#include"../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include "URDFImporterInterface.h"
#include "btBulletCollisionCommon.h"
#include "../ImportObjDemo/LoadMeshFromObj.h"
#include "../ImportSTLDemo/LoadMeshFromSTL.h"
#include "../ImportColladaDemo/LoadMeshFromCollada.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"//to create a tesselation of a generic btConvexShape
#include "../../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3FileUtils.h"
#include <string>
#include "../../Utils/b3ResourcePath.h"

#include "../ImportMeshUtility/b3ImportMeshUtility.h"

static btScalar gUrdfDefaultCollisionMargin = 0.001;

#include <iostream>
#include <fstream>
#include <list>
#include "UrdfParser.h"

struct MyTexture
{
	int m_width;
	int m_height;
	unsigned char* textureData;
};


ATTRIBUTE_ALIGNED16(struct) BulletURDFInternalData
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	UrdfParser m_urdfParser;
	struct GUIHelperInterface* m_guiHelper;
	std::string m_sourceFile;
	char m_pathPrefix[1024];
	int m_bodyId;
	btHashMap<btHashInt,UrdfMaterialColor> m_linkColors;
    btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;
	mutable btAlignedObjectArray<btTriangleMesh*> m_allocatedMeshInterfaces;
	
	LinkVisualShapesConverter* m_customVisualShapesConverter;

	void setSourceFile(const std::string& relativeFileName, const std::string& prefix)
	{
		m_sourceFile = relativeFileName;
		m_urdfParser.setSourceFile(relativeFileName);
		strncpy(m_pathPrefix, prefix.c_str(), sizeof(m_pathPrefix));
		m_pathPrefix[sizeof(m_pathPrefix)-1] = 0; // required, strncpy doesn't write zero on overflow
	}

	BulletURDFInternalData()
	{
		m_pathPrefix[0] = 0;
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

BulletURDFImporter::BulletURDFImporter(struct GUIHelperInterface* helper, LinkVisualShapesConverter* customConverter, double globalScaling)
{
	m_data = new BulletURDFInternalData;
	m_data->setGlobalScaling(globalScaling);
	m_data->m_guiHelper = helper;
	m_data->m_customVisualShapesConverter = customConverter;

  
}

struct BulletErrorLogger : public ErrorLogger
{
	int m_numErrors;
	int m_numWarnings;
	
	BulletErrorLogger()
	:m_numErrors(0),
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
	if (strlen(fileName)==0)
        return false;

//int argc=0;
	char relativeFileName[1024];
	
	b3FileUtils fu;
	
	//bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
  	bool fileFound = (b3ResourcePath::findResourcePath(fileName,relativeFileName,1024))>0;
	
	std::string xml_string;

	if (!fileFound){
		b3Warning("URDF file '%s' not found\n", fileName);
		return false;
	} else
	{
		
		char path[1024];
		fu.extractPath(relativeFileName, path, sizeof(path));
		m_data->setSourceFile(relativeFileName, path);

        std::fstream xml_file(relativeFileName, std::fstream::in);
        while ( xml_file.good())
        {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
    }

	BulletErrorLogger loggie;
    m_data->m_urdfParser.setParseSDF(false);
	bool result = m_data->m_urdfParser.loadUrdf(xml_string.c_str(), &loggie, forceFixedBase);

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
    bool fileFound = (b3ResourcePath::findResourcePath(fileName,relativeFileName,1024))>0;
    
    std::string xml_string;
    
    if (!fileFound){
        b3Warning("SDF file '%s' not found\n", fileName);
        return false;
    } else
    {
        
        char path[1024];
        fu.extractPath(relativeFileName, path, sizeof(path));
        m_data->setSourceFile(relativeFileName, path);
        
        std::fstream xml_file(relativeFileName, std::fstream::in);
        while ( xml_file.good() )
        {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
    }
    
    BulletErrorLogger loggie;
    //todo: quick test to see if we can re-use the URDF parser for SDF or not
    m_data->m_urdfParser.setParseSDF(true);
    bool result = m_data->m_urdfParser.loadSDF(xml_string.c_str(), &loggie);
    
    return result;
}


const char* BulletURDFImporter::getPathPrefix()
{
	return m_data->m_pathPrefix;
}

    
void BulletURDFImporter::setBodyUniqueId(int bodyId)
{
    m_data->m_bodyId =bodyId;
}


int BulletURDFImporter::getBodyUniqueId() const
{
    return  m_data->m_bodyId;
}


BulletURDFImporter::~BulletURDFImporter()
{
	delete m_data;
}

    
int BulletURDFImporter::getRootLinkIndex() const
{
	if (m_data->m_urdfParser.getModel().m_rootLinks.size()==1)
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
		
		for (int i=0;i<link->m_childLinks.size();i++)
		{
			int childIndex =link->m_childLinks[i]->m_linkIndex;
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
    

void  BulletURDFImporter::getMassAndInertia(int linkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const
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
		if (link->m_parentJoint==0 && m_data->m_urdfParser.getModel().m_overrideFixedBase)
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
		inertialFrame.setBasis(link->m_inertia.m_linkLocalFrame.getBasis()*linkInertiaBasis);
	}
	else
	{
		mass = 1.f;
		localInertiaDiagonal.setValue(1,1,1);
		inertialFrame.setIdentity();
	}
}
    
bool BulletURDFImporter::getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const 
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
			return true;
		} else
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
	return getJointInfo2(urdfLinkIndex, parent2joint, linkTransformInWorld, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction,jointMaxForce,jointMaxVelocity); 
	
}

void BulletURDFImporter::setRootTransformInWorld(const btTransform& rootTransformInWorld)
{
    m_data->m_urdfParser.getModel().m_rootTransformInWorld = rootTransformInWorld ;
}



bool BulletURDFImporter::getRootTransformInWorld(btTransform& rootTransformInWorld) const
{
    rootTransformInWorld = m_data->m_urdfParser.getModel().m_rootTransformInWorld;
    return true;
}

static btCollisionShape* createConvexHullFromShapes(std::vector<tinyobj::shape_t>& shapes, const btVector3& geomScale)
{
	B3_PROFILE("createConvexHullFromShapes");
	btCompoundShape* compound = new btCompoundShape();
	compound->setMargin(gUrdfDefaultCollisionMargin);

	btTransform identity;
	identity.setIdentity();

	for (int s = 0; s<(int)shapes.size(); s++)
	{
		btConvexHullShape* convexHull = new btConvexHullShape();
		convexHull->setMargin(gUrdfDefaultCollisionMargin);
		tinyobj::shape_t& shape = shapes[s];
		int faceCount = shape.mesh.indices.size();

		for (int f = 0; f<faceCount; f += 3)
		{

			btVector3 pt;
			pt.setValue(shape.mesh.positions[shape.mesh.indices[f] * 3 + 0],
				shape.mesh.positions[shape.mesh.indices[f] * 3 + 1],
				shape.mesh.positions[shape.mesh.indices[f] * 3 + 2]);
			
			convexHull->addPoint(pt*geomScale,false);

			pt.setValue(shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 0],
						shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 1],
						shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 2]);
			convexHull->addPoint(pt*geomScale, false);

			pt.setValue(shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 0],
						shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 1],
						shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 2]);
			convexHull->addPoint(pt*geomScale, false);
		}

		convexHull->recalcLocalAabb();
		convexHull->optimizeConvexHull();
		compound->addChildShape(identity,convexHull);
	}

	return compound;
}

bool findExistingMeshFile(
	const std::string& urdf_path, std::string fn,
	const std::string& error_message_prefix,
	std::string* out_found_filename, int* out_type)
{
	if (fn.size() <= 4)
	{
		b3Warning("%s: invalid mesh filename '%s'\n", error_message_prefix.c_str(), fn.c_str());
		return false;
	}

	std::string ext;
	std::string ext_ = fn.substr(fn.size()-4);
	for (std::string::iterator i=ext_.begin(); i!=ext_.end(); ++i)
	{
		ext += char(tolower(*i));
	}

	if (ext==".dae")
	{
		*out_type = UrdfGeometry::FILE_COLLADA;
	}
	else if (ext==".stl")
	{
		*out_type = UrdfGeometry::FILE_STL;
	}
	else if (ext==".obj")
	{
		*out_type = UrdfGeometry::FILE_OBJ;
	}
	else
	{
		b3Warning("%s: invalid mesh filename extension '%s'\n", error_message_prefix.c_str(), ext.c_str());
		return false;
	}

	std::string drop_it = "package://";
	if (fn.substr(0, drop_it.length())==drop_it)
		fn = fn.substr(drop_it.length());

	std::list<std::string> shorter;
	shorter.push_back("../..");
	shorter.push_back("..");
	shorter.push_back(".");
	int cnt = urdf_path.size();
	for (int i=0; i<cnt; ++i)
	{
		if (urdf_path[i]=='/' || urdf_path[i]=='\\')
		{
			shorter.push_back(urdf_path.substr(0, i));
		}
	}
	shorter.reverse();

	std::string existing_file;

	{
		std::string attempt = fn;
		FILE* f = fopen(attempt.c_str(), "rb");
		if (f)
		{
			existing_file = attempt;
			fclose(f);
		}
	}
	if (existing_file.empty())
	{
		for (std::list<std::string>::iterator x=shorter.begin(); x!=shorter.end(); ++x)
		{
			std::string attempt = *x + "/" + fn;
			FILE* f = fopen(attempt.c_str(), "rb");
			if (!f)
			{
				//b3Printf("%s: tried '%s'", error_message_prefix.c_str(), attempt.c_str());
				continue;
			}
			fclose(f);
			existing_file = attempt;
			//b3Printf("%s: found '%s'", error_message_prefix.c_str(), attempt.c_str());
			break;
		}
	}

	if (existing_file.empty())
	{
		b3Warning("%s: cannot find '%s' in any directory in urdf path\n", error_message_prefix.c_str(), fn.c_str());
		return false;
	}
	else
	{
		*out_found_filename = existing_file;
		return true;
	}
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
			btScalar planeConstant = 0;//not available?
			btStaticPlaneShape* plane = new btStaticPlaneShape(planeNormal,planeConstant);
			shape = plane;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_CAPSULE:
        {
			btScalar radius = collision->m_geometry.m_capsuleRadius;
			btScalar height = collision->m_geometry.m_capsuleHeight;
			btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(radius,height);
			shape = capsuleShape;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}

        case URDF_GEOM_CYLINDER:
        {
			btScalar cylRadius = collision->m_geometry.m_capsuleRadius;
			btScalar cylLength = collision->m_geometry.m_capsuleHeight;
			
            btAlignedObjectArray<btVector3> vertices;
            //int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
            int numSteps = 32;
            for (int i=0;i<numSteps;i++)
            {

                btVector3 vert(cylRadius*btSin(SIMD_2_PI*(float(i)/numSteps)),cylRadius*btCos(SIMD_2_PI*(float(i)/numSteps)),cylLength/2.);
                vertices.push_back(vert);
                vert[2] = -cylLength/2.;
                vertices.push_back(vert);

            }
            btConvexHullShape* convexHull = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
            convexHull->setMargin(gUrdfDefaultCollisionMargin);
			convexHull->initializePolyhedralFeatures();
			convexHull->optimizeConvexHull();
			
			//btConvexShape* cylZShape = new btConeShapeZ(cyl->radius,cyl->length);//(vexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
            
            //btVector3 halfExtents(cyl->radius,cyl->radius,cyl->length/2.);
            //btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
            

            shape = convexHull;
            break;
        }
        case URDF_GEOM_BOX:
        {
			btVector3 extents = collision->m_geometry.m_boxSize;
			btBoxShape* boxShape = new btBoxShape(extents*0.5f);
			//btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
            shape = boxShape;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
            break;
        }
        case URDF_GEOM_SPHERE:
        {
			btScalar radius = collision->m_geometry.m_sphereRadius;
			btSphereShape* sphereShape = new btSphereShape(radius);
            shape = sphereShape;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
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
				if (b3ResourcePath::findResourcePath(collision->m_geometry.m_meshFileName.c_str(), relativeFileName, 1024))
				{

					b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
				}
				glmesh = LoadMeshFromObj(collision->m_geometry.m_meshFileName.c_str(), pathPrefix);
			}
			else
			{
				std::vector<tinyobj::shape_t> shapes;
				std::string err = tinyobj::LoadObj(shapes, collision->m_geometry.m_meshFileName.c_str());
				//create a convex hull for each shape, and store it in a btCompoundShape

				shape = createConvexHullFromShapes(shapes, collision->m_geometry.m_meshScale);
				return shape;
			}
			break;

		case UrdfGeometry::FILE_STL:
			glmesh = LoadMeshFromSTL(collision->m_geometry.m_meshFileName.c_str());
			break;

		case UrdfGeometry::FILE_COLLADA:
			{
				btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
				btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
				btTransform upAxisTrans;upAxisTrans.setIdentity();
				float unitMeterScaling = 1;
				LoadMeshFromCollada(collision->m_geometry.m_meshFileName.c_str(), visualShapes, visualShapeInstances, upAxisTrans, unitMeterScaling, 2);

				glmesh = new GLInstanceGraphicsShape;
				glmesh->m_indices = new b3AlignedObjectArray<int>();
				glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

				for (int i=0;i<visualShapeInstances.size();i++)
				{
					ColladaGraphicsInstance* instance = &visualShapeInstances[i];
					GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];

					b3AlignedObjectArray<GLInstanceVertex> verts;
					verts.resize(gfxShape->m_vertices->size());

					int baseIndex = glmesh->m_vertices->size();

					for (int i=0;i<gfxShape->m_vertices->size();i++)
					{
						verts[i].normal[0] = 	gfxShape->m_vertices->at(i).normal[0];
						verts[i].normal[1] = 	gfxShape->m_vertices->at(i).normal[1];
						verts[i].normal[2] = 	gfxShape->m_vertices->at(i).normal[2];
						verts[i].uv[0] = gfxShape->m_vertices->at(i).uv[0];
						verts[i].uv[1] = gfxShape->m_vertices->at(i).uv[1];
						verts[i].xyzw[0] = gfxShape->m_vertices->at(i).xyzw[0];
						verts[i].xyzw[1] = gfxShape->m_vertices->at(i).xyzw[1];
						verts[i].xyzw[2] = gfxShape->m_vertices->at(i).xyzw[2];
						verts[i].xyzw[3] = gfxShape->m_vertices->at(i).xyzw[3];

					}

					int curNumIndices = glmesh->m_indices->size();
					int additionalIndices = gfxShape->m_indices->size();
					glmesh->m_indices->resize(curNumIndices+additionalIndices);
					for (int k=0;k<additionalIndices;k++)
					{
						glmesh->m_indices->at(curNumIndices+k)=gfxShape->m_indices->at(k)+baseIndex;
					}

					//compensate upAxisTrans and unitMeterScaling here
					btMatrix4x4 upAxisMat;
upAxisMat.setIdentity();
					//upAxisMat.setPureRotation(upAxisTrans.getRotation());
					btMatrix4x4 unitMeterScalingMat;
					unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling,unitMeterScaling,unitMeterScaling));
					btMatrix4x4 worldMat = unitMeterScalingMat*instance->m_worldTransform*upAxisMat;
					//btMatrix4x4 worldMat = instance->m_worldTransform;
					int curNumVertices = glmesh->m_vertices->size();
					int additionalVertices = verts.size();
					glmesh->m_vertices->reserve(curNumVertices+additionalVertices);

					for(int v=0;v<verts.size();v++)
					{
						btVector3 pos(verts[v].xyzw[0],verts[v].xyzw[1],verts[v].xyzw[2]);
						pos = worldMat*pos;
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

		if (!glmesh || glmesh->m_numvertices<=0)
		{
			b3Warning("%s: cannot extract mesh from '%s'\n", urdfPathPrefix, collision->m_geometry.m_meshFileName.c_str());
			delete glmesh;
			break;
		}

		btAlignedObjectArray<btVector3> convertedVerts;
		convertedVerts.reserve(glmesh->m_numvertices);
		for (int i=0; i<glmesh->m_numvertices; i++)
		{
			convertedVerts.push_back(btVector3(
				glmesh->m_vertices->at(i).xyzw[0]*collision->m_geometry.m_meshScale[0],
				glmesh->m_vertices->at(i).xyzw[1]*collision->m_geometry.m_meshScale[1],
				glmesh->m_vertices->at(i).xyzw[2]*collision->m_geometry.m_meshScale[2]));
		}

		if (collision->m_flags & URDF_FORCE_CONCAVE_TRIMESH)
		{
			BT_PROFILE("convert trimesh");
			btTriangleMesh* meshInterface = new btTriangleMesh();
			m_data->m_allocatedMeshInterfaces.push_back(meshInterface);
			{
				BT_PROFILE("convert vertices");

				for (int i=0; i<glmesh->m_numIndices/3; i++)
				{
					const btVector3& v0 = convertedVerts[glmesh->m_indices->at(i*3)];
					const btVector3& v1 = convertedVerts[glmesh->m_indices->at(i*3+1)];
					const btVector3& v2 = convertedVerts[glmesh->m_indices->at(i*3+2)];
					meshInterface->addTriangle(v0,v1,v2);
				}
			}
			{
				BT_PROFILE("create btBvhTriangleMeshShape");
				btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface,true,true);
				//trimesh->setLocalScaling(collision->m_geometry.m_meshScale);
				shape = trimesh;
			}

		} else
		{
			BT_PROFILE("convert btConvexHullShape");
			btConvexHullShape* convexHull = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
			convexHull->optimizeConvexHull();
			//convexHull->initializePolyhedralFeatures();
			convexHull->setMargin(gUrdfDefaultCollisionMargin);
			//convexHull->setLocalScaling(collision->m_geometry.m_meshScale);
			shape = convexHull;
		}

		delete glmesh;
		break;
	} // mesh case

        default:
		b3Warning("Error: unknown collision geometry type %i\n", collision->m_geometry.m_type);
		
	}
	return shape;
}


static void convertURDFToVisualShapeInternal(const UrdfVisual* visual, const char* urdfPathPrefix, const btTransform& visualTransform, btAlignedObjectArray<GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<MyTexture>& texturesOut)
{
	BT_PROFILE("convertURDFToVisualShapeInternal");

	
	GLInstanceGraphicsShape* glmesh = 0;

	btConvexShape* convexColShape = 0;

	switch (visual->m_geometry.m_type)
	{
		case URDF_GEOM_CYLINDER:
		{
			btAlignedObjectArray<btVector3> vertices;
		
			//int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
			int numSteps = 32;
			for (int i = 0; i<numSteps; i++)
			{

				btScalar cylRadius = visual->m_geometry.m_capsuleRadius;
				btScalar cylLength = visual->m_geometry.m_capsuleHeight;
				
				btVector3 vert(cylRadius*btSin(SIMD_2_PI*(float(i) / numSteps)), cylRadius*btCos(SIMD_2_PI*(float(i) / numSteps)), cylLength / 2.);
				vertices.push_back(vert);
				vert[2] = -cylLength / 2.;
				vertices.push_back(vert);
			}

			btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
			cylZShape->setMargin(gUrdfDefaultCollisionMargin);
			convexColShape = cylZShape;
			break;
		}

		case URDF_GEOM_BOX:
		{
			btVector3 extents = visual->m_geometry.m_boxSize;
			btBoxShape* boxShape = new btBoxShape(extents*0.5f);
			//btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
			convexColShape = boxShape;
			convexColShape->setMargin(gUrdfDefaultCollisionMargin);
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
			case UrdfGeometry::FILE_OBJ:
				{
					b3ImportMeshData meshData;
					if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(visual->m_geometry.m_meshFileName, meshData))
					{

						if (meshData.m_textureImage)
						{
							MyTexture texData;
							texData.m_width = meshData.m_textureWidth;
							texData.m_height = meshData.m_textureHeight;
							texData.textureData = meshData.m_textureImage;
							texturesOut.push_back(texData);
						}
						glmesh = meshData.m_gfxShape;
					}
					break;
				}

			case UrdfGeometry::FILE_STL:
				{
					glmesh = LoadMeshFromSTL(visual->m_geometry.m_meshFileName.c_str());
					break;
				}

			case UrdfGeometry::FILE_COLLADA:
				{
					btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
					btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
					btTransform upAxisTrans; upAxisTrans.setIdentity();
					float unitMeterScaling = 1;
					int upAxis = 2;

					LoadMeshFromCollada(visual->m_geometry.m_meshFileName.c_str(),
						visualShapes,
						visualShapeInstances,
						upAxisTrans,
						unitMeterScaling,
						upAxis);

					glmesh = new GLInstanceGraphicsShape;
			//		int index = 0;
					glmesh->m_indices = new b3AlignedObjectArray<int>();
					glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

					for (int i = 0; i<visualShapeInstances.size(); i++)
					{
						ColladaGraphicsInstance* instance = &visualShapeInstances[i];
						GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];

						b3AlignedObjectArray<GLInstanceVertex> verts;
						verts.resize(gfxShape->m_vertices->size());

						int baseIndex = glmesh->m_vertices->size();

						for (int i = 0; i<gfxShape->m_vertices->size(); i++)
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
						for (int k = 0; k<additionalIndices; k++)
						{
							glmesh->m_indices->at(curNumIndices + k) = gfxShape->m_indices->at(k) + baseIndex;
						}

						//compensate upAxisTrans and unitMeterScaling here
						btMatrix4x4 upAxisMat;
						upAxisMat.setIdentity();
	//								upAxisMat.setPureRotation(upAxisTrans.getRotation());
						btMatrix4x4 unitMeterScalingMat;
						unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling, unitMeterScaling, unitMeterScaling));
						btMatrix4x4 worldMat = unitMeterScalingMat*upAxisMat*instance->m_worldTransform;
						//btMatrix4x4 worldMat = instance->m_worldTransform;
						int curNumVertices = glmesh->m_vertices->size();
						int additionalVertices = verts.size();
						glmesh->m_vertices->reserve(curNumVertices + additionalVertices);

						for (int v = 0; v<verts.size(); v++)
						{
							btVector3 pos(verts[v].xyzw[0], verts[v].xyzw[1], verts[v].xyzw[2]);
							pos = worldMat*pos;
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
			} // switch file type

			if (!glmesh || !glmesh->m_vertices || glmesh->m_numvertices<=0)
			{
				b3Warning("%s: cannot extract anything useful from mesh '%s'\n", urdfPathPrefix, visual->m_geometry.m_meshFileName.c_str());
				break;
			}

			//apply the geometry scaling
			for (int i=0;i<glmesh->m_vertices->size();i++)
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
	if ((glmesh==0) && convexColShape)
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
				vtx.uv[0] = 0.5f;
				vtx.uv[1] = 0.5f;
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
	
	if (glmesh && glmesh->m_numIndices>0 && glmesh->m_numvertices >0)
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
			btVector3 vert(v.xyzw[0],v.xyzw[1],v.xyzw[2]);
			btVector3 vt = visualTransform*vert;
			v.xyzw[0] = vt[0];
			v.xyzw[1] = vt[1];
			v.xyzw[2] = vt[2];
			btVector3 triNormal(v.normal[0],v.normal[1],v.normal[2]);
			triNormal = visualTransform.getBasis()*triNormal;
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
	btTransform startTrans; startTrans.setIdentity();
	btAlignedObjectArray<MyTexture> textures;
	
    const UrdfModel& model = m_data->m_urdfParser.getModel();
	UrdfLink* const* linkPtr = model.m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{

		const UrdfLink* link = *linkPtr;
	
		for (int v = 0; v < link->m_visualArray.size();v++)
		{
			const UrdfVisual& vis = link->m_visualArray[v];
			btTransform childTrans = vis.m_linkLocalFrame;
			btHashString matName(vis.m_materialName.c_str());
			UrdfMaterial *const * matPtr = model.m_materials[matName];
			if (matPtr)
			{
				UrdfMaterial *const  mat = *matPtr;
				//printf("UrdfMaterial %s, rgba = %f,%f,%f,%f\n",mat->m_name.c_str(),mat->m_rgbaColor[0],mat->m_rgbaColor[1],mat->m_rgbaColor[2],mat->m_rgbaColor[3]);
				UrdfMaterialColor matCol;
				matCol.m_rgbaColor = mat->m_matColor.m_rgbaColor;
				matCol.m_specularColor = mat->m_matColor.m_specularColor;
				m_data->m_linkColors.insert(linkIndex,matCol);
			}
			convertURDFToVisualShapeInternal(&vis, pathPrefix, localInertiaFrame.inverse()*childTrans, vertices, indices,textures);
		
		
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
				
				textureIndex = m_data->m_guiHelper->registerTexture(textures[0].textureData,textures[0].m_width,textures[0].m_height);
			}
			graphicsIndex = m_data->m_guiHelper->registerGraphicsShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size(),B3_GL_TRIANGLES,textureIndex);
			
		}
	}
	
	//delete textures
	for (int i=0;i<textures.size();i++)
	{
		free( textures[i].textureData);
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

bool BulletURDFImporter::getLinkContactInfo(int urdflinkIndex, URDFLinkContactInfo& contactInfo ) const
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


void BulletURDFImporter::convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& localInertiaFrame, class btCollisionObject* colObj, int bodyUniqueId) const
{
  	if (m_data->m_customVisualShapesConverter)
	{
		const UrdfModel& model = m_data->m_urdfParser.getModel();
		UrdfLink*const* linkPtr = model.m_links.getAtIndex(urdfIndex);
		if (linkPtr)
		{
			m_data->m_customVisualShapesConverter->convertVisualShapes(linkIndex,pathPrefix,localInertiaFrame, *linkPtr, &model, colObj, bodyUniqueId);
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
		
			
		for (int v=0;v<link->m_collisionArray.size();v++)
		{
			const UrdfCollision& col = link->m_collisionArray[v];
			btCollisionShape* childShape = convertURDFToCollisionShape(&col ,pathPrefix);
			m_data->m_allocatedCollisionShapes.push_back(childShape);
			if (childShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
			{
				btCompoundShape* compound = (btCompoundShape*) childShape;
				for (int i=0;i<compound->getNumChildShapes();i++)
				{
					m_data->m_allocatedCollisionShapes.push_back(compound->getChildShape(i));
				}
			}
			
			if (childShape)
			{
				btTransform childTrans = col.m_linkLocalFrame;
				
				compoundShape->addChildShape(localInertiaFrame.inverse()*childTrans,childShape);
			}
		}
	}
	
    return compoundShape;
}
