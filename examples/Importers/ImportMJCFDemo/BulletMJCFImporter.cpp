#include "BulletMJCFImporter.h"
#include "../../ThirdPartyLibs/tinyxml/tinyxml.h"
#include "Bullet3Common/b3FileUtils.h"
#include <string>
#include "../../Utils/b3ResourcePath.h"
#include <iostream>
#include <fstream>
#include "../ImportURDFDemo/UrdfParser.h"
#include "../ImportURDFDemo/urdfStringSplit.h"
#include "../ImportURDFDemo/urdfLexicalCast.h"
#include "../ImportObjDemo/LoadMeshFromObj.h"
#include "../ImportSTLDemo/LoadMeshFromSTL.h"
#include"../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include "../ImportMeshUtility/b3ImportMeshUtility.h"

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMesh.h"





#include <vector>

enum eMJCF_FILE_TYPE_ENUMS
{
	MJCF_FILE_STL = 1,
	MJCF_FILE_OBJ = 2
};

enum ePARENT_LINK_ENUMS
{
	BASE_LINK_INDEX=-1,

	INVALID_LINK_INDEX=-2
};

static int gUid=0;

static bool parseVector4(btVector4& vec4, const std::string& vector_str)
{
	vec4.setZero();
	btArray<std::string> pieces;
	btArray<float> rgba;
	btAlignedObjectArray<std::string> strArray;
	urdfIsAnyOf(" ", strArray);
	urdfStringSplit(pieces, vector_str, strArray);
	for (int i = 0; i < pieces.size(); ++i)
	{
		if (!pieces[i].empty())
		{
			rgba.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
		}
	}
	if (rgba.size() != 4)
	{
		return false;
	}
	vec4.setValue(rgba[0],rgba[1],rgba[2],rgba[3]);
	return true;
}

static bool parseVector3(btVector3& vec3, const std::string& vector_str, MJCFErrorLogger* logger, bool lastThree = false)
{
	vec3.setZero();
	btArray<std::string> pieces;
	btArray<float> rgba;
	btAlignedObjectArray<std::string> strArray;
	urdfIsAnyOf(" ", strArray);
	urdfStringSplit(pieces, vector_str, strArray);
	for (int i = 0; i < pieces.size(); ++i)
	{
		if (!pieces[i].empty())
		{
			rgba.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
		}
	}
	if (rgba.size() < 3)
	{
		logger->reportWarning("Couldn't parse vector3");
		return false;
	}
    if (lastThree) {
        vec3.setValue(rgba[rgba.size()-3], rgba[rgba.size()-2], rgba[rgba.size()-1]);
    }
    else
    {
        vec3.setValue(rgba[0],rgba[1],rgba[2]);

    }
	return true;
}


static bool parseVector6(btVector3& v0, btVector3& v1, const std::string& vector_str, MJCFErrorLogger* logger)
{
	v0.setZero();
	v1.setZero();

	btArray<std::string> pieces;
	btArray<float> values;
	btAlignedObjectArray<std::string> strArray;
	urdfIsAnyOf(" ", strArray);
	urdfStringSplit(pieces, vector_str, strArray);
	for (int i = 0; i < pieces.size(); ++i)
	{
		if (!pieces[i].empty())
		{
			values.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
		}
	}
	if (values.size() < 6)
	{
		logger->reportWarning("Couldn't parse 6 floats");
		return false;
	}
	v0.setValue(values[0],values[1],values[2]);
	v1.setValue(values[3],values[4],values[5]);

	return true;
}


struct MyMJCFAsset
{
	std::string  m_fileName;
};

struct BulletMJCFImporterInternalData
{
	GUIHelperInterface* m_guiHelper;
	char m_pathPrefix[1024];

	std::string m_fileModelName;
	btHashMap<btHashString,MyMJCFAsset> m_assets;

	btAlignedObjectArray<UrdfModel*>	m_models;

	//<compiler angle="radian" meshdir="mesh/" texturedir="texture/"/>
	std::string m_meshDir;
	std::string m_textureDir;


	int m_activeModel;
	//todo: for full MJCF compatibility, we would need a stack of default values
	int m_defaultCollisionGroup;
	int m_defaultCollisionMask;
	btScalar m_defaultCollisionMargin;

	//those collision shapes are deleted by caller (todo: make sure this happens!)
	btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;

	BulletMJCFImporterInternalData()
		:m_activeModel(-1),
		m_defaultCollisionGroup(1),
		m_defaultCollisionMask(1),
		m_defaultCollisionMargin(0.001)//assume unit meters, margin is 1mm
	{
		m_pathPrefix[0] = 0;
	}
	
	const UrdfLink* getLink(int modelIndex, int linkIndex) const
	{
		if (modelIndex>=0 && modelIndex<m_models.size())
		{
			UrdfLink** linkPtrPtr = m_models[modelIndex]->m_links.getAtIndex(linkIndex);
			if (linkPtrPtr && *linkPtrPtr)
			{
				UrdfLink* linkPtr = *linkPtrPtr;
				return linkPtr;
			}
		}
		return 0;
	}

	void parseCompiler(TiXmlElement* root_xml, MJCFErrorLogger* logger)
	{

		const char* meshDirStr = root_xml->Attribute("meshdir");
		if (meshDirStr)
		{
			m_meshDir = meshDirStr;
		}
		const char* textureDirStr = root_xml->Attribute("texturedir");
		if (textureDirStr)
		{
			m_textureDir = textureDirStr;
		}
#if 0
		for (TiXmlElement* child_xml = root_xml->FirstChildElement() ; child_xml ; child_xml = child_xml->NextSiblingElement())
		{
			std::string n = child_xml->Value();
		}
#endif
	}
	
	void parseAssets(TiXmlElement* root_xml, MJCFErrorLogger* logger)
	{
		//		<mesh name="index0" 	file="index0.stl"/>
		for (TiXmlElement* child_xml = root_xml->FirstChildElement() ; child_xml ; child_xml = child_xml->NextSiblingElement())
		{
			std::string n = child_xml->Value();
			if (n=="mesh")
			{
				const char* assetNameStr = child_xml->Attribute("name");
				const char* fileNameStr = child_xml->Attribute("file");
				if (assetNameStr && fileNameStr)
				{
					btHashString assetName = assetNameStr;
					MyMJCFAsset asset;
					asset.m_fileName = m_meshDir + fileNameStr;
					m_assets.insert(assetName,asset);
				}
			}
			
		}
	}
	bool parseDefaults(TiXmlElement* root_xml, MJCFErrorLogger* logger)
	{
		bool handled= false;
		//rudimentary 'default' support, would need more work for better feature coverage
		for (TiXmlElement* child_xml = root_xml->FirstChildElement() ; child_xml ; child_xml = child_xml->NextSiblingElement())
		{
			std::string n = child_xml->Value();
		
			if (n=="inertial")
			{
			}
			if (n=="asset")
			{
				parseAssets(child_xml,logger);
			}
			if (n=="geom")
			{
				//contype, conaffinity 
				const char* conTypeStr = child_xml->Attribute("contype");
				if (conTypeStr)
				{
					m_defaultCollisionGroup = urdfLexicalCast<int>(conTypeStr);
				}
				const char* conAffinityStr = child_xml->Attribute("conaffinity");
				if (conAffinityStr)
				{
					m_defaultCollisionMask = urdfLexicalCast<int>(conAffinityStr);
				}
			}
		}
		handled=true;
		return handled;
	}
	bool parseRootLevel(TiXmlElement* root_xml,MJCFErrorLogger* logger)
	{
		for (TiXmlElement* rootxml = root_xml->FirstChildElement() ; rootxml ; rootxml = rootxml->NextSiblingElement())
		{
			bool handled = false;
			std::string n = rootxml->Value();

			
			if (n=="body")
			{
				int modelIndex = m_models.size();
				UrdfModel* model = new UrdfModel();
				m_models.push_back(model);
				parseBody(rootxml,modelIndex, INVALID_LINK_INDEX,logger);
				initTreeAndRoot(*model,logger);
				handled = true;
			}

			if (n=="geom")
			{
				int modelIndex = m_models.size();
				UrdfModel* modelPtr = new UrdfModel();
				m_models.push_back(modelPtr);

				UrdfLink* linkPtr = new UrdfLink();
				linkPtr->m_name = "anonymous";
				const char* namePtr = rootxml->Attribute("name");
				if (namePtr)
				{
					linkPtr->m_name = namePtr;
				}
				int linkIndex = modelPtr->m_links.size();
				linkPtr->m_linkIndex = linkIndex;
				modelPtr->m_links.insert(linkPtr->m_name.c_str(),linkPtr);

				//don't parse geom transform here, it will be inside 'parseGeom'
				linkPtr->m_linkTransformInWorld.setIdentity();
				
//				modelPtr->m_rootLinks.push_back(linkPtr);

				btVector3 inertialShift(0,0,0);
				parseGeom(rootxml,modelIndex, linkIndex,logger,inertialShift);
				initTreeAndRoot(*modelPtr,logger);

				handled = true;
			}


			if (n=="site")
			{
				handled = true;
			}
			if (!handled)
			{
				logger->reportWarning("Unhandled root element");
				logger->reportWarning(n.c_str());
			}
		}
		return true;
	}

	bool parseJoint(TiXmlElement* link_xml, int modelIndex, int parentLinkIndex, int linkIndex, MJCFErrorLogger* logger, const btTransform& parentToLinkTrans, btTransform& jointTransOut)
	{
		const char* jType = link_xml->Attribute("type");
		const char* limitedStr = link_xml->Attribute("limited");
		const char* axisStr = link_xml->Attribute("axis");
		const char* posStr = link_xml->Attribute("pos");
		const char* ornStr = link_xml->Attribute("quat");
		const char* nameStr = link_xml->Attribute("name");
		const char* rangeStr = link_xml->Attribute("range");

		btTransform jointTrans;
		jointTrans.setIdentity();
		if (posStr)
		{
			btVector3 pos;
			std::string p=posStr;
			if (parseVector3(pos,p,logger))
			{
				jointTrans.setOrigin(pos);
			}
		}
		if (ornStr)
		{
			std::string o = ornStr;
			btVector4 o4;
			if (parseVector4(o4,o))
			{
				btQuaternion orn(o4[3],o4[0],o4[1],o4[2]);
				jointTrans.setRotation(orn);
			}
		}
		btVector3 jointAxis(1,0,0);

		if (axisStr)
		{
			std::string ax = axisStr;
			parseVector3(jointAxis,ax,logger);
		} else
		{
			logger->reportWarning("joint without axis attribute");
		}
		bool isLimited = false;
		double range[2] = {1,0};

		if (limitedStr)
		{
			std::string lim = limitedStr;
			if (lim=="true")
			{
				isLimited = true;
				//parse the 'range' field
				btArray<std::string> pieces;
				btArray<float> sizes;
				btAlignedObjectArray<std::string> strArray;
				urdfIsAnyOf(" ", strArray);
				urdfStringSplit(pieces, rangeStr, strArray);
				for (int i = 0; i < pieces.size(); ++i)
				{
					if (!pieces[i].empty())
					{
						sizes.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
					}
				}
				if (sizes.size()==2)
				{
					range[0] = sizes[0];
					range[1] = sizes[1];
				} else
				{
					logger->reportWarning("Expected range[2] in joint with limits");
				}

			}
		} else
		{
//			logger->reportWarning("joint without limited field");
		}

		bool jointHandled = false;
		const UrdfLink* linkPtr = getLink(modelIndex,linkIndex);
		
		btTransform parentLinkToJointTransform;
		parentLinkToJointTransform.setIdentity();
		parentLinkToJointTransform = parentToLinkTrans*jointTrans;

		jointTransOut = jointTrans;
		UrdfJointTypes ejtype;
		if (jType)
		{
			std::string jointType = jType; 
			if (jointType == "fixed")
			{
				ejtype = URDFFixedJoint;
				jointHandled = true;
			}
			if (jointType == "slide")
			{
				ejtype = URDFPrismaticJoint;
				jointHandled = true;
			}
			if (jointType == "hinge")
			{
				if (isLimited)
				{
					ejtype = URDFRevoluteJoint;
				} else
				{
					ejtype = URDFContinuousJoint;
				}
				jointHandled = true;
			}
		} else
		{
			logger->reportWarning("Expected 'type' attribute for joint");
		}

		if (jointHandled)
		{
			UrdfJoint* jointPtr = new UrdfJoint();
			jointPtr->m_childLinkName=linkPtr->m_name;
			const UrdfLink* parentLink = getLink(modelIndex,parentLinkIndex);
			jointPtr->m_parentLinkName =parentLink->m_name;
			jointPtr->m_localJointAxis=jointAxis;
			jointPtr->m_parentLinkToJointTransform = parentLinkToJointTransform;
			jointPtr->m_type = ejtype;
			int numJoints = m_models[modelIndex]->m_joints.size();
			
			//range
			jointPtr->m_lowerLimit = range[0];
			jointPtr->m_upperLimit = range[1];
			
			if (nameStr)
			{
				jointPtr->m_name =nameStr; 
			} else
			{
				char jointName[1024];
				sprintf(jointName,"joint%d_%d_%d",gUid++,linkIndex,numJoints);
				jointPtr->m_name =jointName; 
			}
			m_models[modelIndex]->m_joints.insert(jointPtr->m_name.c_str(),jointPtr);
			return true;
		}
		/*
		URDFRevoluteJoint=1,
		URDFPrismaticJoint,
		URDFContinuousJoint,
		URDFFloatingJoint,
		URDFPlanarJoint,
		URDFFixedJoint,
		*/
		return false;
	}
	bool parseGeom(TiXmlElement* link_xml, int modelIndex, int linkIndex, MJCFErrorLogger* logger, btVector3& inertialShift)
	{
		UrdfLink** linkPtrPtr = m_models[modelIndex]->m_links.getAtIndex(linkIndex);
		if (linkPtrPtr==0)
		{
			logger->reportWarning("Invalide linkindex");
			return false;
		}
		UrdfLink* linkPtr = *linkPtrPtr;
		
		btTransform linkLocalFrame;
		linkLocalFrame.setIdentity();


		bool handledGeomType = false;
		UrdfGeometry geom;

		

//		const char* rgba = link_xml->Attribute("rgba");
		const char* gType = link_xml->Attribute("type");
		const char* sz = link_xml->Attribute("size");
		const char* posS = link_xml->Attribute("pos");
		if (posS)
		{
			btVector3 pos(0,0,0);
			std::string p = posS;
			if (parseVector3(pos,p,logger))
			{
				linkLocalFrame.setOrigin(pos);
			}
		}
		const char* ornS = link_xml->Attribute("quat");
		if (ornS)
		{
			std::string ornStr = ornS;
			btQuaternion orn(0,0,0,1);
			btVector4 o4;
			if (parseVector4(o4,ornStr))
			{
				orn.setValue(o4[1],o4[2],o4[3],o4[0]);
				linkLocalFrame.setRotation(orn);
			}
		}
		if (gType)
		{
			std::string geomType = gType;

					
			if (geomType == "plane")
			{
				geom.m_type = URDF_GEOM_PLANE;
				geom.m_planeNormal.setValue(0,0,1);
				btVector3 size(1,1,1);
				if (sz)
				{
					std::string sizeStr = sz;
					bool lastThree = false;
					parseVector3(size,sizeStr,logger,lastThree);
				}
				geom.m_boxSize = size;
				handledGeomType = true;
			}
			if (geomType == "box")
			{
				btVector3 size(1,1,1);
				if (sz)
				{
					std::string sizeStr = sz;
					bool lastThree = false;
					parseVector3(size,sizeStr,logger,lastThree);
				}
				geom.m_type = URDF_GEOM_BOX;
				geom.m_boxSize = size;
				handledGeomType = true;
			}

			if (geomType == "sphere")
			{
				geom.m_type = URDF_GEOM_SPHERE;
				if (sz)
				{
					geom.m_sphereRadius = urdfLexicalCast<double>(sz);
				} else
				{
					logger->reportWarning("Expected size field (scalar) in sphere geom");
				}
				handledGeomType = true;
			}

			//todo: capsule, cylinder, meshes or heightfields etc
			if (geomType == "capsule")
			{
				geom.m_type = URDF_GEOM_CAPSULE;

				btArray<std::string> pieces;
				btArray<float> sizes;
				btAlignedObjectArray<std::string> strArray;
				urdfIsAnyOf(" ", strArray);
				urdfStringSplit(pieces, sz, strArray);
				for (int i = 0; i < pieces.size(); ++i)
				{
					if (!pieces[i].empty())
					{
						sizes.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
					}
				}

				geom.m_capsuleRadius = 0;
				geom.m_capsuleHalfHeight = 0.f;

				if (sizes.size()>0)
				{
					geom.m_capsuleRadius = sizes[0];
					if (sizes.size()>1)
					{
						geom.m_capsuleHalfHeight = sizes[1];
					}
				} else
				{
					logger->reportWarning("couldn't convert 'size' attribute of capsule geom");
				}
				const char* fromtoStr = link_xml->Attribute("fromto");
				geom.m_hasFromTo = false;

				if (fromtoStr)
				{
					geom.m_hasFromTo = true;
					std::string fromto = fromtoStr;
					parseVector6(geom.m_capsuleFrom,geom.m_capsuleTo,fromto,logger);
					inertialShift=0.5*(geom.m_capsuleFrom+geom.m_capsuleTo);
					handledGeomType = true;
				} else
				{
					if (sizes.size()<2)
					{
						logger->reportWarning("capsule without fromto attribute requires 2 sizes (radius and halfheight)");
					} else
					{
						handledGeomType = true;
					}
				}
			}
			if (geomType=="mesh")
			{
				const char* meshStr = link_xml->Attribute("mesh");
				if (meshStr)
				{
					MyMJCFAsset* assetPtr = m_assets[meshStr];
					if (assetPtr)
					{
						handledGeomType = true;
						geom.m_type = URDF_GEOM_MESH;
						geom.m_meshFileName = assetPtr->m_fileName;
						geom.m_meshScale.setValue(1,1,1);
						//todo: parse mesh scale
						if (sz)
						{
						}
					}
				}
			}
			#if 0
			if (geomType == "cylinder")
			{
				geom.m_type = URDF_GEOM_CYLINDER;
				handledGeomType = true;
			}
#endif
			if (handledGeomType)
			{
						
				UrdfCollision col;
				col.m_flags |= URDF_HAS_COLLISION_GROUP;
				col.m_collisionGroup = m_defaultCollisionGroup;
				
				col.m_flags |= URDF_HAS_COLLISION_MASK;
				col.m_collisionMask = m_defaultCollisionMask;
				
				//contype, conaffinity 
				const char* conTypeStr = link_xml->Attribute("contype");
				if (conTypeStr)
				{
					col.m_flags |= URDF_HAS_COLLISION_GROUP;
					col.m_collisionGroup = urdfLexicalCast<int>(conTypeStr);
				}
				const char* conAffinityStr = link_xml->Attribute("conaffinity");
				if (conAffinityStr)
				{
					col.m_flags |= URDF_HAS_COLLISION_MASK;
					col.m_collisionMask = urdfLexicalCast<int>(conAffinityStr);
				}

				col.m_geometry = geom;
				col.m_linkLocalFrame = linkLocalFrame;
				linkPtr->m_collisionArray.push_back(col);

			} else
			{
				char warn[1024];
				sprintf(warn,"Unknown/unhandled geom type: %s", geomType.c_str());
				logger->reportWarning(warn);
			}
		} else
		{
			logger->reportWarning("geom requires type");
		}

		return handledGeomType;
	}

	btTransform parseTransform(TiXmlElement* link_xml, MJCFErrorLogger* logger)
	{
		btTransform tr;
		tr.setIdentity();

		const char* p = link_xml->Attribute("pos");
		if (p)
		{
			btVector3 pos(0,0,0);
			std::string pstr = p;
			if (parseVector3(pos,pstr,logger))
			{
				tr.setOrigin(pos);
			}
			
		} else
		{
//			logger->reportWarning("body should have pos attribute");
		}
		const char* o = link_xml->Attribute("quat");
		if (o)
		{
			std::string ornstr = o;
			btVector4 o4;
			btQuaternion orn(0,0,0,1);
			if (parseVector4(o4,ornstr))
			{
				orn.setValue(o4[1],o4[2],o4[3],o4[0]);//MuJoCo quats are [w,x,y,z], Bullet uses [x,y,z,w]
				tr.setRotation(orn);
			}
		} else
		{
//			logger->reportWarning("body doesn't have quat (orientation) attribute");
		}
		return tr;
	}

	double computeVolume(const UrdfLink* linkPtr,MJCFErrorLogger* logger) const
	{
		double totalVolume = 0;

		for (int i=0;i<linkPtr->m_collisionArray.size();i++)
		{
			const UrdfCollision* col = &linkPtr->m_collisionArray[i];
			switch (col->m_geometry.m_type)
			{
			case URDF_GEOM_SPHERE:
			{
				double r = col->m_geometry.m_sphereRadius;
				totalVolume += 4./3.*SIMD_PI*r*r*r;
				break;
			}
			case URDF_GEOM_BOX:
			{
				totalVolume += 8. * col->m_geometry.m_boxSize[0]*
					col->m_geometry.m_boxSize[1]*
					col->m_geometry.m_boxSize[2];
				break;
			}

			case URDF_GEOM_CYLINDER:
			{
				//todo
				break;
			}
			case URDF_GEOM_MESH:
			{
				//todo (based on mesh bounding box?)
				break;
			}
			case URDF_GEOM_PLANE:
			{
				//todo
				break;
			}
			case URDF_GEOM_CAPSULE:
			{
				//one sphere 
				double r = col->m_geometry.m_capsuleRadius;
				totalVolume += 4./3.*SIMD_PI*r*r*r;
				//and one cylinder of 'height'
				btScalar h = (col->m_geometry.m_capsuleFrom-col->m_geometry.m_capsuleTo).length();
				totalVolume += SIMD_PI*r*r*h;
				
				break;
			}
			default:
			{
			}
			}
		}

		return totalVolume;
	}
	UrdfLink* getLink(int modelIndex, int linkIndex)
	{
		UrdfLink** linkPtrPtr = m_models[modelIndex]->m_links.getAtIndex(linkIndex);
		if (linkPtrPtr && *linkPtrPtr)
		{
			return *linkPtrPtr;
		}
		btAssert(0);
		return 0;
	}

	int createBody(int modelIndex, const char* namePtr)
	{
		UrdfModel* modelPtr = m_models[modelIndex];
		int orgChildLinkIndex = modelPtr->m_links.size();
		UrdfLink* linkPtr = new UrdfLink();
		char uniqueLinkName[1024];
		sprintf(uniqueLinkName,"link%d",orgChildLinkIndex );
		linkPtr->m_name = uniqueLinkName;
		if (namePtr)
		{
			linkPtr->m_name = namePtr;
		}
		linkPtr->m_linkIndex = orgChildLinkIndex ;
		modelPtr->m_links.insert(linkPtr->m_name.c_str(),linkPtr);

		return orgChildLinkIndex;
	}
	bool parseBody(TiXmlElement* link_xml, int modelIndex, int orgParentLinkIndex, MJCFErrorLogger* logger)
	{
		int newParentLinkIndex = orgParentLinkIndex;

		const char* bodyName = link_xml->Attribute("name");
		int orgChildLinkIndex = createBody(modelIndex,bodyName);
		btTransform localInertialFrame;
		localInertialFrame.setIdentity();

//		int curChildLinkIndex = orgChildLinkIndex;
		std::string bodyN;
		
		if (bodyName)
		{
			bodyN = bodyName;
		} else
		{
			char anon[1024];
			sprintf(anon,"anon%d",gUid++);
			bodyN = anon;		
		}
		

//		btTransform orgLinkTransform = parseTransform(link_xml,logger);

		btTransform linkTransform = parseTransform(link_xml,logger);
		UrdfLink* linkPtr = getLink(modelIndex,orgChildLinkIndex);

		

		bool massDefined = false;
		
		
		btScalar mass = 0.f;
		btVector3 localInertiaDiag(0,0,0);
	//	int thisLinkIndex = -2;
		bool hasJoint = false;
		btTransform jointTrans;
		jointTrans.setIdentity();
		bool skipFixedJoint = false;
		
		for (TiXmlElement* xml = link_xml->FirstChildElement() ; xml ; xml = xml->NextSiblingElement())
		{
			bool handled = false;
			std::string n = xml->Value();
			if (n=="inertial")
			{
				//   <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="297.821" diaginertia="109.36 69.9714 69.9714" />
				
				const char* p = xml->Attribute("pos");
				if (p)
				{
					std::string posStr = p;
					btVector3 inertialPos(0,0,0);
					if (parseVector3(inertialPos,posStr,logger))
					{
						localInertialFrame.setOrigin(inertialPos);
					}
				}
				const char* o = xml->Attribute("quat");
				{
					std::string ornStr = o;
					btQuaternion orn(0,0,0,1);
					btVector4 o4;
					if (parseVector4(o4,ornStr))
					{
						orn.setValue(o4[1],o4[2],o4[3],o4[0]);
						localInertialFrame.setRotation(orn);
					}
				}
				const char* m = xml->Attribute("mass");
				if (m)
				{
					mass = urdfLexicalCast<double>(m);
				}
				const char* i = xml->Attribute("diaginertia");
				if (i)
				{
					std::string istr = i;
					parseVector3(localInertiaDiag,istr,logger);
				}
				
				massDefined = true;

				handled = true;
			}

			if (n=="joint")
			{
				if (!hasJoint)
				{
					const char* jType = xml->Attribute("type");
					std::string jointType = jType? jType:"";

					if (newParentLinkIndex!=INVALID_LINK_INDEX || jointType!="free")
					{
						if (newParentLinkIndex==INVALID_LINK_INDEX)
						{
							int newRootLinkIndex = createBody(modelIndex,0);
							UrdfLink* rootLink = getLink(modelIndex,newRootLinkIndex);
							rootLink->m_inertia.m_mass = 0;
							rootLink->m_linkTransformInWorld.setIdentity();
							newParentLinkIndex = newRootLinkIndex;
						}

						int newLinkIndex = createBody(modelIndex,0);
						parseJoint(xml,modelIndex,newParentLinkIndex, newLinkIndex,logger,linkTransform,jointTrans);
						
						//getLink(modelIndex,newLinkIndex)->m_linkTransformInWorld = jointTrans*linkTransform;
						
						linkTransform = jointTrans.inverse();
						newParentLinkIndex = newLinkIndex;
						//newParentLinkIndex, curChildLinkIndex
						hasJoint = true;
						handled = true;
					}
				} else
				{
					int newLinkIndex = createBody(modelIndex,0);
					btTransform joint2nextjoint =  jointTrans.inverse();
					btTransform unused;
					parseJoint(xml,modelIndex,newParentLinkIndex, newLinkIndex,logger,joint2nextjoint,unused);
					newParentLinkIndex = newLinkIndex;
					//todo: compute relative joint transforms (if any) and append to linkTransform
					hasJoint = true;
					handled = true;
				}
				
			}
			if (n == "geom")
			{
				btVector3 inertialShift(0,0,0);
				parseGeom(xml,modelIndex, orgChildLinkIndex , logger,inertialShift);
				if (!massDefined)
				{
					localInertialFrame.setOrigin(inertialShift);
				}
				handled = true;
			}

			//recursive
			if (n=="body")
			{
				parseBody(xml,modelIndex,orgChildLinkIndex,logger);
				handled = true;
			}

			if (n=="light")
			{
				handled = true;
			}
			if (!handled)
			{
				char warn[1024];
				std::string n = xml->Value();
				sprintf(warn,"Unknown/unhandled field: %s", n.c_str());
				logger->reportWarning(warn);
			}
		}

		linkPtr->m_linkTransformInWorld = linkTransform;
		if (bodyN == "cart1")//front_left_leg")
		{
			printf("found!\n");
		}
		if ((newParentLinkIndex != INVALID_LINK_INDEX) && !skipFixedJoint)
		{
			//linkPtr->m_linkTransformInWorld.setIdentity();
			//default to 'fixed' joint
			UrdfJoint* jointPtr = new UrdfJoint();
			jointPtr->m_childLinkName=linkPtr->m_name;
			const UrdfLink* parentLink = getLink(modelIndex,newParentLinkIndex);
			jointPtr->m_parentLinkName =parentLink->m_name;
			jointPtr->m_localJointAxis.setValue(1,0,0);
			jointPtr->m_parentLinkToJointTransform = linkTransform;
			jointPtr->m_type = URDFFixedJoint;
			char jointName[1024];
			sprintf(jointName,"jointfix_%d_%d",gUid++,newParentLinkIndex);
			jointPtr->m_name =jointName; 
			m_models[modelIndex]->m_joints.insert(jointPtr->m_name.c_str(),jointPtr);
		}

		//check mass/inertia
		if (!massDefined)
		{
			double density = 1000;
			double volume = computeVolume(linkPtr,logger);
			mass = density * volume;
		}
		linkPtr->m_inertia.m_linkLocalFrame = localInertialFrame;// = jointTrans.inverse();
		linkPtr->m_inertia.m_mass = mass;
		return true;
	}

	void recurseAddChildLinks(UrdfModel* model, UrdfLink* link)
	{
		for (int i=0;i<link->m_childLinks.size();i++)
		{
			int linkIndex = model->m_links.size();
			link->m_childLinks[i]->m_linkIndex = linkIndex;
			const char* linkName = link->m_childLinks[i]->m_name.c_str();
			model->m_links.insert(linkName,link->m_childLinks[i]);
		}
		for (int i=0;i<link->m_childLinks.size();i++)
		{
			recurseAddChildLinks(model,link->m_childLinks[i]);
		}
	}

	bool initTreeAndRoot(UrdfModel& model, MJCFErrorLogger* logger)
	{
		// every link has children links and joints, but no parents, so we create a
		// local convenience data structure for keeping child->parent relations
		btHashMap<btHashString,btHashString> parentLinkTree;
	
		// loop through all joints, for every link, assign children links and children joints
		for (int i=0;i<model.m_joints.size();i++)
		{
			UrdfJoint** jointPtr = model.m_joints.getAtIndex(i);
			if (jointPtr)
			{
				UrdfJoint* joint = *jointPtr;
				std::string parent_link_name = joint->m_parentLinkName;
				std::string child_link_name = joint->m_childLinkName;
				if (parent_link_name.empty() || child_link_name.empty())
				{
					logger->reportError("parent link or child link is empty for joint");
					logger->reportError(joint->m_name.c_str());
					return false;
				}
			
				UrdfLink** childLinkPtr = model.m_links.find(joint->m_childLinkName.c_str());
				if (!childLinkPtr)
				{
					logger->reportError("Cannot find child link for joint ");
					logger->reportError(joint->m_name.c_str());

					return false;
				}
				UrdfLink* childLink = *childLinkPtr;
			
				UrdfLink** parentLinkPtr = model.m_links.find(joint->m_parentLinkName.c_str());
				if (!parentLinkPtr)
				{
					logger->reportError("Cannot find parent link for a joint");
					logger->reportError(joint->m_name.c_str());
					return false;
				}
				UrdfLink* parentLink = *parentLinkPtr;
			
				childLink->m_parentLink  = parentLink;
			
				childLink->m_parentJoint = joint;
				parentLink->m_childJoints.push_back(joint);
				parentLink->m_childLinks.push_back(childLink);
				parentLinkTree.insert(childLink->m_name.c_str(),parentLink->m_name.c_str());
			}
		}

		//search for children that have no parent, those are 'root'
		for (int i=0;i<model.m_links.size();i++)
		{
			UrdfLink** linkPtr = model.m_links.getAtIndex(i);
			btAssert(linkPtr);
			if (linkPtr)
			{
				UrdfLink* link = *linkPtr;
				link->m_linkIndex = i;
			
				if (!link->m_parentLink)
				{
					model.m_rootLinks.push_back(link);
				}
			}
		
		}
	
		if (model.m_rootLinks.size()>1)
		{
			logger->reportWarning("URDF file with multiple root links found");
		}
	
		if (model.m_rootLinks.size()==0)
		{
			logger->reportError("URDF without root link found");
			return false;
		}

		//re-index the link indices so parent indices are always smaller than child indices
		btAlignedObjectArray<UrdfLink*> links;
		links.resize(model.m_links.size());
		for (int i=0;i<model.m_links.size();i++)
		{
			links[i] = *model.m_links.getAtIndex(i);
		}
		model.m_links.clear();
		for (int i=0;i<model.m_rootLinks.size();i++)
		{
			UrdfLink* rootLink = model.m_rootLinks[i];
			int linkIndex = model.m_links.size();
			rootLink->m_linkIndex = linkIndex;
			model.m_links.insert(rootLink->m_name.c_str(),rootLink);
			recurseAddChildLinks(&model, rootLink);
		}
		return true;
	
	}

};

BulletMJCFImporter::BulletMJCFImporter(struct GUIHelperInterface* helper)
{
	m_data = new BulletMJCFImporterInternalData();
	m_data->m_guiHelper = helper;
}

BulletMJCFImporter::~BulletMJCFImporter()
{
	delete m_data;
}

bool BulletMJCFImporter::loadMJCF(const char* fileName, MJCFErrorLogger* logger, bool forceFixedBase)
{
	if (strlen(fileName)==0)
        return false;

//int argc=0;
	char relativeFileName[1024];
	
	b3FileUtils fu;
	
	//bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
  	bool fileFound = (b3ResourcePath::findResourcePath(fileName,relativeFileName,1024)>0);
	
	std::string xml_string;
	m_data->m_pathPrefix[0] = 0;
    
    if (!fileFound){
        std::cerr << "MJCF file not found" << std::endl;
		return false;
    } else
    {
		
		int maxPathLen = 1024;
		fu.extractPath(relativeFileName,m_data->m_pathPrefix,maxPathLen);


        std::fstream xml_file(relativeFileName, std::fstream::in);
        while ( xml_file.good())
        {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();

		if (parseMJCFString(xml_string.c_str(), logger))
		{
			return true;
		}
    }

	

	return false;
}

bool BulletMJCFImporter::parseMJCFString(const char* xmlText, MJCFErrorLogger* logger)
{
	TiXmlDocument xml_doc;
	xml_doc.Parse(xmlText);
	if (xml_doc.Error())
	{
		logger->reportError(xml_doc.ErrorDesc());
		xml_doc.ClearError();
		return false;
	}

	TiXmlElement *mujoco_xml = xml_doc.FirstChildElement("mujoco");
	if (!mujoco_xml)
	{
		logger->reportWarning("Cannot find <mujoco> root element");
		return false;
	}

	const char* modelName = mujoco_xml->Attribute("model");
	if (modelName)
	{
		m_data->m_fileModelName = modelName;
	}

	//<compiler>,<option>,<size>,<default>,<body>,<keyframe>,<contactpair>,
	//<light>, <camera>,<constraint>,<tendon>,<actuator>,<customfield>,<textfield>

	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("default"); link_xml; link_xml = link_xml->NextSiblingElement("default"))
	{
		m_data->parseDefaults(link_xml,logger);
	}

	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("compiler"); link_xml; link_xml = link_xml->NextSiblingElement("compiler"))
	{
		m_data->parseCompiler(link_xml,logger);
	}
	

	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("asset"); link_xml; link_xml = link_xml->NextSiblingElement("asset"))
	{
		m_data->parseAssets(link_xml,logger);
	}
	
	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("body"); link_xml; link_xml = link_xml->NextSiblingElement("body"))
	{
		m_data->parseRootLevel(link_xml,logger);
	}

	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("worldbody"); link_xml; link_xml = link_xml->NextSiblingElement("worldbody"))
	{
		m_data->parseRootLevel(link_xml,logger);
	}



	return true;
}

const char* BulletMJCFImporter::getPathPrefix()
{
	return m_data->m_pathPrefix;
}

int BulletMJCFImporter::getRootLinkIndex() const
{
	if (m_data->m_activeModel>=0 && m_data->m_activeModel<m_data->m_models.size())
	{
		if (m_data->m_models[m_data->m_activeModel]->m_rootLinks.size())
		{
			return 0;
		}
	}
	return -1;
}


 std::string BulletMJCFImporter::getLinkName(int linkIndex) const
 {
	 const UrdfLink* link = m_data->getLink(m_data->m_activeModel,linkIndex);
	 if (link)
	 {
		 return link->m_name;
	 }
	return "";
 }

bool BulletMJCFImporter::getLinkColor(int linkIndex, btVector4& colorRGBA) const
{
//	UrdfLink* link = m_data->getLink(linkIndex);
	return false;
}

//todo: placeholder implementation
//MuJoCo type/affinity is different from Bullet group/mask, so we should implement a custom collision filter instead
//(contype1 & conaffinity2) || (contype2 & conaffinity1)
int BulletMJCFImporter::getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const 
{
	int flags = 0;
	
	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,linkIndex);
	if (link)
	{
		for (int i=0;i<link->m_collisionArray.size();i++)
		{
			const UrdfCollision& col = link->m_collisionArray[i];
			colGroup = col.m_collisionGroup;
			flags |= URDF_HAS_COLLISION_GROUP;
			colMask = col.m_collisionMask;
			flags |= URDF_HAS_COLLISION_MASK;

		}
	}
	
	return flags;
}
	

std::string BulletMJCFImporter::getJointName(int linkIndex) const
{
	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,linkIndex);
	if (link)
	{
		if (link->m_parentJoint)
		{
			return link->m_parentJoint->m_name;
		}
		return link->m_name;
	}
	return "";
}

    //fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
void  BulletMJCFImporter::getMassAndInertia(int urdfLinkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const
{
	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,urdfLinkIndex);
	if (link)
	{
		mass = link->m_inertia.m_mass;
		localInertiaDiagonal.setValue(link->m_inertia.m_ixx,
			link->m_inertia.m_iyy,
			link->m_inertia.m_izz);
		inertialFrame.setIdentity();
		inertialFrame = link->m_inertia.m_linkLocalFrame;
	} else
	{
		mass = 0;
		localInertiaDiagonal.setZero();
		inertialFrame.setIdentity();
	}
}
    
///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
void BulletMJCFImporter::getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const
{
	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,urdfLinkIndex);
	if (link)
	{
		for (int i=0;i<link->m_childLinks.size();i++)
		{
			int childIndex = link->m_childLinks[i]->m_linkIndex;
			childLinkIndices.push_back(childIndex);
		}
	}
}
    
bool BulletMJCFImporter::getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const
{
    jointLowerLimit = 0.f;
    jointUpperLimit = 0.f;
	jointDamping = 0.f;
	jointFriction = 0.f;

	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,urdfLinkIndex);
	if (link)
	{
		
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
    
bool BulletMJCFImporter::getRootTransformInWorld(btTransform& rootTransformInWorld) const
{
	rootTransformInWorld.setIdentity();
	/*
	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,0);
	if (link)
	{
		rootTransformInWorld = link->m_linkTransformInWorld;
	}
	*/
	return true;
}

int BulletMJCFImporter::convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame) const
{
	return -1;
}

bool BulletMJCFImporter::getLinkContactInfo(int linkIndex, URDFLinkContactInfo& contactInfo ) const
{
	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,linkIndex);
	if (link)
	{
		contactInfo = link->m_contactInfo;
		return true;
	}
	return false;
}


void BulletMJCFImporter::convertLinkVisualShapes2(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame, class btCollisionObject* colObj, int objectIndex) const
{
}
void BulletMJCFImporter::setBodyUniqueId(int bodyId)
{

}

int BulletMJCFImporter::getBodyUniqueId() const 
{ 
	return 0;
}


static btCollisionShape* MjcfCreateConvexHullFromShapes(std::vector<tinyobj::shape_t>& shapes, const btVector3& geomScale, btScalar collisionMargin)
{
	btCompoundShape* compound = new btCompoundShape();
	compound->setMargin(collisionMargin);

	btTransform identity;
	identity.setIdentity();

	for (int s = 0; s<(int)shapes.size(); s++)
	{
		btConvexHullShape* convexHull = new btConvexHullShape();
		convexHull->setMargin(collisionMargin);
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

    
class btCompoundShape* BulletMJCFImporter::convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
{
	btCompoundShape* compound = new btCompoundShape();
	m_data->m_allocatedCollisionShapes.push_back(compound);

	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,linkIndex);
	if (link)
	{
		for (int i=0;i<	link->m_collisionArray.size();i++)
		{
			const UrdfCollision* col = &link->m_collisionArray[i];
			btCollisionShape* childShape = 0;

			switch (col->m_geometry.m_type)
			{
			case URDF_GEOM_PLANE:
			{
				childShape = new btStaticPlaneShape(col->m_geometry.m_planeNormal,0);
				break;
			}
			case URDF_GEOM_SPHERE:
			{
				childShape = new btSphereShape(col->m_geometry.m_sphereRadius);
				break;
			}
			case URDF_GEOM_BOX:
			{
				childShape = new btBoxShape(col->m_geometry.m_boxSize);
				break;
			}
			case URDF_GEOM_CYLINDER:
			{
//				childShape = new btCylinderShape(col->m_geometry...);
				break;
			}
			case URDF_GEOM_MESH:
			{
				//////////////////////
				if (1)
			{
				if (col->m_geometry.m_meshFileName.length())
				{
					const char* filename = col->m_geometry.m_meshFileName.c_str();
					//b3Printf("mesh->filename=%s\n",filename);
					char fullPath[1024];
					int fileType = 0;
					sprintf(fullPath,"%s%s",pathPrefix,filename);
					b3FileUtils::toLower(fullPath);
                    char tmpPathPrefix[1024];
                    int maxPathLen = 1024;
                    b3FileUtils::extractPath(filename,tmpPathPrefix,maxPathLen);
                    
                    char collisionPathPrefix[1024];
                    sprintf(collisionPathPrefix,"%s%s",pathPrefix,tmpPathPrefix);
                    
					if (strstr(fullPath,".stl"))
					{
						fileType = MJCF_FILE_STL;
					}
                    if (strstr(fullPath,".obj"))
                   {
                       fileType = MJCF_FILE_OBJ;
                   }

					sprintf(fullPath,"%s%s",pathPrefix,filename);
					FILE* f = fopen(fullPath,"rb");
					if (f)
					{
						fclose(f);
						GLInstanceGraphicsShape* glmesh = 0;
						
						
						switch (fileType)
						{
                            case MJCF_FILE_OBJ:
                            {
								if (col->m_flags & URDF_FORCE_CONCAVE_TRIMESH)
								{
									glmesh = LoadMeshFromObj(fullPath, collisionPathPrefix);
								}
								else
								{
									std::vector<tinyobj::shape_t> shapes;
									std::string err = tinyobj::LoadObj(shapes, fullPath, collisionPathPrefix);
									//create a convex hull for each shape, and store it in a btCompoundShape

									childShape = MjcfCreateConvexHullFromShapes(shapes, col->m_geometry.m_meshScale, m_data->m_defaultCollisionMargin);
									
								}
                                break;
                            }
						case MJCF_FILE_STL:
							{
								glmesh = LoadMeshFromSTL(fullPath);
							break;
							}
						
						default:
							{
                                b3Warning("Unsupported file type in Collision: %s\n",fullPath);
                               
							}
						}
					

						if (!childShape && glmesh && (glmesh->m_numvertices>0))
						{
							//b3Printf("extracted %d verticed from STL file %s\n", glmesh->m_numvertices,fullPath);
							//int shapeId = m_glApp->m_instancingRenderer->registerShape(&gvertices[0].pos[0],gvertices.size(),&indices[0],indices.size());
							//convex->setUserIndex(shapeId);
							btAlignedObjectArray<btVector3> convertedVerts;
							convertedVerts.reserve(glmesh->m_numvertices);
							for (int i=0;i<glmesh->m_numvertices;i++)
							{
								convertedVerts.push_back(btVector3(
                                           glmesh->m_vertices->at(i).xyzw[0]*col->m_geometry.m_meshScale[0],
                                           glmesh->m_vertices->at(i).xyzw[1]*col->m_geometry.m_meshScale[1],
                                           glmesh->m_vertices->at(i).xyzw[2]*col->m_geometry.m_meshScale[2]));
							}
							
							if (col->m_flags & URDF_FORCE_CONCAVE_TRIMESH)
							{
								
								btTriangleMesh* meshInterface = new btTriangleMesh();
								for (int i=0;i<glmesh->m_numIndices/3;i++)
								{
									float* v0 = glmesh->m_vertices->at(glmesh->m_indices->at(i*3)).xyzw;
									float* v1 = glmesh->m_vertices->at(glmesh->m_indices->at(i*3+1)).xyzw;
									float* v2 = glmesh->m_vertices->at(glmesh->m_indices->at(i*3+2)).xyzw;
									meshInterface->addTriangle(btVector3(v0[0],v0[1],v0[2]),
																btVector3(v1[0],v1[1],v1[2]),
															btVector3(v2[0],v2[1],v2[2]));
								}
								
								btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface,true,true);
								childShape = trimesh;
							} else
							{
								btConvexHullShape* convexHull = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
								convexHull->optimizeConvexHull();
								//convexHull->initializePolyhedralFeatures();
								convexHull->setMargin(m_data->m_defaultCollisionMargin);
								childShape = convexHull;
							}
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

				//////////////////////
				break;
			}
			
			case URDF_GEOM_CAPSULE:
			{
				//todo: convert fromto to btCapsuleShape + local btTransform
				if (col->m_geometry.m_hasFromTo)
				{
					btVector3 f = col->m_geometry.m_capsuleFrom;
					btVector3 t = col->m_geometry.m_capsuleTo;
					//MuJoCo seems to take the average of the spheres as center?
					//btVector3 c = (f+t)*0.5;
					//f-=c;
					//t-=c;
					btVector3 fromto[2] = {f,t};
					btScalar radii[2] = {btScalar(col->m_geometry.m_capsuleRadius)
										,btScalar(col->m_geometry.m_capsuleRadius)};
			
					btMultiSphereShape* ms = new btMultiSphereShape(fromto,radii,2);
					childShape = ms;
				} else
				{
					btCapsuleShapeZ* cap = new btCapsuleShapeZ(col->m_geometry.m_capsuleRadius,
						2.*col->m_geometry.m_capsuleHalfHeight);
					childShape = cap;
				}
				break;
			}
			default:
			{

			}
			}
			if (childShape)
			{
				m_data->m_allocatedCollisionShapes.push_back(childShape);
				compound->addChildShape(localInertiaFrame.inverse()*col->m_linkLocalFrame,childShape);
			}
		}
	}
	return compound;
}

int BulletMJCFImporter::getNumAllocatedCollisionShapes() const
{
	return m_data->m_allocatedCollisionShapes.size();
}
class btCollisionShape* BulletMJCFImporter::getAllocatedCollisionShape(int index)
{
	return m_data->m_allocatedCollisionShapes[index];
}


int BulletMJCFImporter::getNumModels() const
{
	return m_data->m_models.size();
}

void BulletMJCFImporter::activateModel(int modelIndex)
{
	if ((modelIndex>=0) && (modelIndex<m_data->m_models.size()))
	{
		m_data->m_activeModel = modelIndex;
	}
}

