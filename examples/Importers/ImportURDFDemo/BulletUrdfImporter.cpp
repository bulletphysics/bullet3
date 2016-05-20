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



#include <iostream>
#include <fstream>
#include "UrdfParser.h"

struct BulletURDFInternalData
{
	UrdfParser m_urdfParser;
	struct GUIHelperInterface* m_guiHelper;
	char m_pathPrefix[1024];
	
    btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;
	
	LinkVisualShapesConverter* m_customVisualShapesConverter;
};


void BulletURDFImporter::printTree()
{
//	btAssert(0);
}


enum MyFileType
{
	FILE_STL=1,
	FILE_COLLADA=2,
    FILE_OBJ=3,
};


    
BulletURDFImporter::BulletURDFImporter(struct GUIHelperInterface* helper, LinkVisualShapesConverter* customConverter)
{
	m_data = new BulletURDFInternalData;
	
	m_data->m_guiHelper = helper;
	m_data->m_pathPrefix[0]=0;
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

	

//int argc=0;
	char relativeFileName[1024];
	
	b3FileUtils fu;
	
	//bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
  	bool fileFound = b3ResourcePath::findResourcePath(fileName,relativeFileName,1024);
	
	std::string xml_string;
	m_data->m_pathPrefix[0] = 0;
    
    if (!fileFound){
        std::cerr << "URDF file not found" << std::endl;
		return false;
    } else
    {
		
		int maxPathLen = 1024;
		fu.extractPath(relativeFileName,m_data->m_pathPrefix,maxPathLen);


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
    bool fileFound = b3ResourcePath::findResourcePath(fileName,relativeFileName,1024);
    
    std::string xml_string;
    m_data->m_pathPrefix[0] = 0;
    
    if (!fileFound){
        std::cerr << "URDF file not found" << std::endl;
        return false;
    } else
    {
        
        int maxPathLen = 1024;
        fu.extractPath(relativeFileName,m_data->m_pathPrefix,maxPathLen);
        
        
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

bool BulletURDFImporter::getLinkColor(int linkIndex, btVector4& colorRGBA) const
{
	if (m_data->m_customVisualShapesConverter)
	{
		return m_data->m_customVisualShapesConverter->getLinkColor(linkIndex, colorRGBA);
	}
	return false;
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
	//todo(erwincoumans)
	//the link->m_inertia is NOT necessarily aligned with the inertial frame
	//so an additional transform might need to be computed
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		mass = link->m_inertia.m_mass;
		inertialFrame = link->m_inertia.m_linkLocalFrame;
		localInertiaDiagonal.setValue(link->m_inertia.m_ixx,link->m_inertia.m_iyy,
									  link->m_inertia.m_izz);
	}
	else
    {
        mass = 1.f;
        localInertiaDiagonal.setValue(1,1,1);
        inertialFrame.setIdentity();
    }
}
    
bool BulletURDFImporter::getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const
{
    jointLowerLimit = 0.f;
    jointUpperLimit = 0.f;
	jointDamping = 0.f;
	jointFriction = 0.f;

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

			return true;
		} else
		{
			parent2joint.setIdentity();
			return false;
		}
	}
	
	return false;
	
}

bool BulletURDFImporter::getRootTransformInWorld(btTransform& rootTransformInWorld) const
{
    rootTransformInWorld = m_data->m_urdfParser.getModel().m_rootTransformInWorld;
    return true;
}



btCollisionShape* convertURDFToCollisionShape(const UrdfCollision* collision, const char* urdfPathPrefix)
{
	btCollisionShape* shape = 0;

    switch (collision->m_geometry.m_type)
    {
        case URDF_GEOM_CYLINDER:
        {
			btScalar cylRadius = collision->m_geometry.m_cylinderRadius;
			btScalar cylLength = collision->m_geometry.m_cylinderLength;
			
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
            btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
            cylZShape->setMargin(0.001);
			cylZShape->initializePolyhedralFeatures();
			//btConvexShape* cylZShape = new btConeShapeZ(cyl->radius,cyl->length);//(vexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
            
            //btVector3 halfExtents(cyl->radius,cyl->radius,cyl->length/2.);
            //btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
            

            shape = cylZShape;
            break;
        }
        case URDF_GEOM_BOX:
        {
			btVector3 extents = collision->m_geometry.m_boxSize;
			btBoxShape* boxShape = new btBoxShape(extents*0.5f);
			//btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
            shape = boxShape;
			shape ->setMargin(0.001);
            break;
        }
        case URDF_GEOM_SPHERE:
        {
            
			btScalar radius = collision->m_geometry.m_sphereRadius;
			btSphereShape* sphereShape = new btSphereShape(radius);
            shape = sphereShape;
			shape ->setMargin(0.001);
            break;

            break;
        }
        case URDF_GEOM_MESH:
        {
			if (collision->m_name.length())
			{
				//b3Printf("collision->name=%s\n",collision->m_name.c_str());
			}
			if (1)
			{
				if (collision->m_geometry.m_meshFileName.length())
				{
					const char* filename = collision->m_geometry.m_meshFileName.c_str();
					//b3Printf("mesh->filename=%s\n",filename);
					char fullPath[1024];
					int fileType = 0;
					sprintf(fullPath,"%s%s",urdfPathPrefix,filename);
					b3FileUtils::toLower(fullPath);
                    char tmpPathPrefix[1024];
                    int maxPathLen = 1024;
                    b3FileUtils::extractPath(filename,tmpPathPrefix,maxPathLen);
                    
                    char collisionPathPrefix[1024];
                    sprintf(collisionPathPrefix,"%s%s",urdfPathPrefix,tmpPathPrefix);
                    
                    
                    
					if (strstr(fullPath,".dae"))
					{
						fileType = FILE_COLLADA;
					}
					if (strstr(fullPath,".stl"))
					{
						fileType = FILE_STL;
					}
                    if (strstr(fullPath,".obj"))
                   {
                       fileType = FILE_OBJ;
                   }

					sprintf(fullPath,"%s%s",urdfPathPrefix,filename);
					FILE* f = fopen(fullPath,"rb");
					if (f)
					{
						fclose(f);
						GLInstanceGraphicsShape* glmesh = 0;
						
						
						switch (fileType)
						{
                            case FILE_OBJ:
                            {
                                glmesh = LoadMeshFromObj(fullPath,collisionPathPrefix);
                                break;
                            }
						case FILE_STL:
							{
								glmesh = LoadMeshFromSTL(fullPath);
							break;
							}
						case FILE_COLLADA:
							{
								
								btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
								btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
								btTransform upAxisTrans;upAxisTrans.setIdentity();
								float unitMeterScaling=1;
								int upAxis = 2;
								LoadMeshFromCollada(fullPath,
													visualShapes, 
													visualShapeInstances,
													upAxisTrans,
													unitMeterScaling,
													upAxis );
								
								glmesh = new GLInstanceGraphicsShape;
						//		int index = 0;
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
								//glmesh = LoadMeshFromCollada(fullPath);

								break;
							}
						default:
							{
                                b3Warning("Unsupported file type in Collision: %s\n",fullPath);
                                btAssert(0);
							}
						}
					

						if (glmesh && (glmesh->m_numvertices>0))
						{
							//b3Printf("extracted %d verticed from STL file %s\n", glmesh->m_numvertices,fullPath);
							//int shapeId = m_glApp->m_instancingRenderer->registerShape(&gvertices[0].pos[0],gvertices.size(),&indices[0],indices.size());
							//convex->setUserIndex(shapeId);
							btAlignedObjectArray<btVector3> convertedVerts;
							convertedVerts.reserve(glmesh->m_numvertices);
							for (int i=0;i<glmesh->m_numvertices;i++)
							{
								convertedVerts.push_back(btVector3(
                                           glmesh->m_vertices->at(i).xyzw[0]*collision->m_geometry.m_meshScale[0],
                                           glmesh->m_vertices->at(i).xyzw[1]*collision->m_geometry.m_meshScale[1],
                                           glmesh->m_vertices->at(i).xyzw[2]*collision->m_geometry.m_meshScale[2]));
							}
							//btConvexHullShape* cylZShape = new btConvexHullShape(&glmesh->m_vertices->at(0).xyzw[0], glmesh->m_numvertices, sizeof(GLInstanceVertex));
							btConvexHullShape* cylZShape = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
							//cylZShape->initializePolyhedralFeatures();
							//btVector3 halfExtents(cyl->radius,cyl->radius,cyl->length/2.);
							//btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
							cylZShape->setMargin(0.001);
							shape = cylZShape;
						} else
						{
							b3Warning("issue extracting mesh from STL file %s\n", fullPath);
						}

                        delete glmesh;
                       
					} else
					{
						b3Warning("mesh geometry not found %s\n",fullPath);
					}
							
				}
			}

					
            break;
        }
        default:
        {
            b3Warning("Error: unknown visual geometry type\n");
        }
    }
	return shape;
}

int BulletURDFImporter::convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, class btCollisionShape* colShape) const
{
	int graphicsIndex = -1;

  	if (m_data->m_customVisualShapesConverter)
	{
		const UrdfModel& model = m_data->m_urdfParser.getModel();
		graphicsIndex  = m_data->m_customVisualShapesConverter->convertVisualShapes(linkIndex,pathPrefix,localInertiaFrame, model, colShape);
	}
    return graphicsIndex;

}

int BulletURDFImporter::getNumAllocatedCollisionShapes() const
{
    return m_data->m_allocatedCollisionShapes.size();
}


btCollisionShape* BulletURDFImporter::getAllocatedCollisionShape(int index)
{
    return m_data->m_allocatedCollisionShapes[index];
}

 class btCompoundShape* BulletURDFImporter::convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
{
        
    btCompoundShape* compoundShape = new btCompoundShape();
    m_data->m_allocatedCollisionShapes.push_back(compoundShape);
    
    compoundShape->setMargin(0.001);
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		
		UrdfLink* link = *linkPtr;
		
			
		for (int v=0;v<link->m_collisionArray.size();v++)
		{
			const UrdfCollision& col = link->m_collisionArray[v];
			btCollisionShape* childShape = convertURDFToCollisionShape(&col ,pathPrefix);
			
			if (childShape)
			{
				btTransform childTrans = col.m_linkLocalFrame;
				
				compoundShape->addChildShape(localInertiaFrame.inverse()*childTrans,childShape);
           		}
		}
	}
	
    return compoundShape;
}
