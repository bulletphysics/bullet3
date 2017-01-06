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

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"



enum ePARENT_LINK_ENUMS
{
	INVALID_LINK_INDEX=-2
};



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



struct BulletMJCFImporterInternalData
{
	GUIHelperInterface* m_guiHelper;
	char m_pathPrefix[1024];

	std::string m_fileModelName;

	btAlignedObjectArray<UrdfModel*>	m_models;
	int m_activeModel;

	//those collision shapes are deleted by caller (todo: make sure this happens!)
	btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;

	BulletMJCFImporterInternalData()
		:m_activeModel(-1)
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

	bool parseRootLevel(TiXmlElement* root_xml,MJCFErrorLogger* logger)
	{
		for (TiXmlElement* xml = root_xml->FirstChildElement() ; xml ; xml = xml->NextSiblingElement())
		{
			bool handled = false;
			std::string n = xml->Value();
			if (n=="body")
			{
				int modelIndex = m_models.size();
				UrdfModel* model = new UrdfModel();
				m_models.push_back(model);
				parseBody(xml,modelIndex, INVALID_LINK_INDEX,logger);
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
				const char* namePtr = xml->Attribute("name");
				if (namePtr)
				{
					linkPtr->m_name = namePtr;
				}
				int linkIndex = modelPtr->m_links.size();
				linkPtr->m_linkIndex = linkIndex;
				modelPtr->m_links.insert(linkPtr->m_name.c_str(),linkPtr);

				btTransform linkTransform = parseTransform(xml,logger);
				linkPtr->m_linkTransformInWorld = linkTransform;

//				modelPtr->m_rootLinks.push_back(linkPtr);

				parseGeom(xml,modelIndex, linkIndex,logger);
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

	bool parseJoint(TiXmlElement* link_xml, int modelIndex, int parentLinkIndex, int linkIndex, MJCFErrorLogger* logger, btTransform& jointTransOut)
	{
		const char* jType = link_xml->Attribute("type");
		const char* limitedStr = link_xml->Attribute("limited");
		const char* axisStr = link_xml->Attribute("axis");
		const char* posStr = link_xml->Attribute("pos");
		const char* ornStr = link_xml->Attribute("quat");
		const char* nameStr = link_xml->Attribute("name");
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
		if (limitedStr)
		{
			std::string lim = limitedStr;
			if (lim=="true")
			{
				isLimited = true;
			}
		} else
		{
//			logger->reportWarning("joint without limited field");
		}

		bool jointHandled = false;
		const UrdfLink* linkPtr = getLink(modelIndex,linkIndex);
		
		btTransform parentLinkToJointTransform;
		parentLinkToJointTransform.setIdentity();
		parentLinkToJointTransform = jointTrans*linkPtr->m_linkTransformInWorld;
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
			//default to 'fixed' joint
			UrdfJoint* jointPtr = new UrdfJoint();
			jointPtr->m_childLinkName=linkPtr->m_name;
			const UrdfLink* parentLink = getLink(modelIndex,parentLinkIndex);
			jointPtr->m_parentLinkName =parentLink->m_name;
			jointPtr->m_localJointAxis=jointAxis;
			jointPtr->m_parentLinkToJointTransform = parentLinkToJointTransform;
			jointPtr->m_type = ejtype;
			int numJoints = m_models[modelIndex]->m_joints.size();
			if (nameStr)
			{
				jointPtr->m_name =nameStr; 
			} else
			{
				char jointName[1024];
				sprintf(jointName,"joint%d_%d",linkIndex,numJoints);
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
	bool parseGeom(TiXmlElement* link_xml, int modelIndex, int linkIndex, MJCFErrorLogger* logger)
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
		const char* rgba = link_xml->Attribute("rgba");
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
				orn.setValue(o4[3],o4[0],o4[1],o4[2]);
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
				geom.m_capsuleRadius = urdfLexicalCast<double>(sz);
				const char* fromtoStr = link_xml->Attribute("fromto");
				if (fromtoStr)
				{
					std::string fromto = fromtoStr;
					parseVector6(geom.m_capsuleFrom,geom.m_capsuleTo,fromto,logger);
					handledGeomType = true;
				} else
				{
					logger->reportWarning("capsule without fromto attribute");
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
				orn.setValue(o4[3],o4[0],o4[1],o4[2]);//MuJoCo quats are [w,x,y,z], Bullet uses [x,y,z,w]
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

	bool parseBody(TiXmlElement* link_xml, int modelIndex, int parentLinkIndex, MJCFErrorLogger* logger)
	{
		UrdfModel* modelPtr = m_models[modelIndex];
		int linkIndex = modelPtr->m_links.size();
		UrdfLink* linkPtr = new UrdfLink();
		char uniqueLinkName[1024];
		sprintf(uniqueLinkName,"link%d",linkIndex);
		linkPtr->m_name = uniqueLinkName;
		const char* namePtr = link_xml->Attribute("name");
		if (namePtr)
		{
			linkPtr->m_name = namePtr;
		}
		
		linkPtr->m_linkIndex = linkIndex;
		modelPtr->m_links.insert(linkPtr->m_name.c_str(),linkPtr);

		
		btTransform linkTransform = parseTransform(link_xml,logger);

		linkPtr->m_linkTransformInWorld = linkTransform;
		//body/geom links with no parent are root links
		if (parentLinkIndex==INVALID_LINK_INDEX)
		{
//			modelPtr->m_rootLinks.push_back(linkPtr);
		}

		bool massDefined = false;
		btVector3 inertialPos(0,0,0);
		btQuaternion inertialOrn(0,0,0,1);
		btScalar mass = 0.f;
		btVector3 localInertiaDiag(0,0,0);
		int thisLinkIndex = -2;
		bool hasJoint = false;
		btTransform jointTrans;
		jointTrans.setIdentity();

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
					parseVector3(inertialPos,posStr,logger);
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
				//skip joints at the root for now 
				//MuJoCo supports more than just 'free' or 'fixed',
				//so we will need to emulate this with extra root links+joints

				//for now, only convert 1st joint
				if (!hasJoint)
				{
					if (parentLinkIndex!=INVALID_LINK_INDEX)
					{
						parseJoint(xml,modelIndex,parentLinkIndex, linkIndex,logger,jointTrans);
					}
				}
				hasJoint = true;
				handled = true;
			}
			if (n == "geom")
			{
				parseGeom(xml,modelIndex, linkIndex, logger);
				handled = true;
			}

			//recursive
			if (n=="body")
			{
				parseBody(xml,modelIndex,linkIndex,logger);
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

		if ((!hasJoint) && (parentLinkIndex != INVALID_LINK_INDEX))
		{
			//default to 'fixed' joint
			UrdfJoint* jointPtr = new UrdfJoint();
			jointPtr->m_childLinkName=linkPtr->m_name;
			const UrdfLink* parentLink = getLink(modelIndex,parentLinkIndex);
			jointPtr->m_parentLinkName =parentLink->m_name;
			jointPtr->m_localJointAxis.setValue(1,0,0);
			jointPtr->m_parentLinkToJointTransform = linkTransform;
			jointPtr->m_type = URDFFixedJoint;
			char jointName[1024];
			sprintf(jointName,"joint%d",linkIndex);
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
		linkPtr->m_inertia.m_linkLocalFrame = jointTrans.inverse();
		linkPtr->m_inertia.m_mass = mass;
		return true;
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
  	bool fileFound = b3ResourcePath::findResourcePath(fileName,relativeFileName,1024);
	
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

	
	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("body"); link_xml; link_xml = link_xml->NextSiblingElement("body"))
	{
		m_data->parseRootLevel(link_xml,logger);
	}

	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("worldbody"); link_xml; link_xml = link_xml->NextSiblingElement("worldbody"))
	{
		m_data->parseRootLevel(link_xml,logger);
	}

	//<compiler>,<option>,<size>,<default>,<body>,<keyframe>,<contactpair>,
	//<light>, <camera>,<constraint>,<tendon>,<actuator>,<customfield>,<textfield>

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

std::string BulletMJCFImporter::getJointName(int linkIndex) const
{
	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,linkIndex);
	return link->m_name;
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
			childLinkIndices.push_back(link->m_childLinks[i]->m_linkIndex);
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

	const UrdfLink* link = m_data->getLink(m_data->m_activeModel,0);
	if (link)
	{
		rootTransformInWorld = link->m_linkTransformInWorld;
	}
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
				break;
			}
			
			case URDF_GEOM_CAPSULE:
			{
				//todo: convert fromto to btCapsuleShape + local btTransform
				
				btVector3 f = col->m_geometry.m_capsuleFrom;
				btVector3 t = col->m_geometry.m_capsuleTo;
				//MuJoCo seems to take the average of the spheres as center?
				btVector3 c = (f+t)*0.5;
				//f-=c;
				//t-=c;
				btVector3 fromto[2] = {f,t};
				btScalar radii[2] = {col->m_geometry.m_capsuleRadius,col->m_geometry.m_capsuleRadius};
			
				btMultiSphereShape* ms = new btMultiSphereShape(fromto,radii,2);
				childShape = ms;
				break;
			}
			default:
			{
			}
			}
			if (childShape)
			{
				m_data->m_allocatedCollisionShapes.push_back(childShape);
				compound->addChildShape(col->m_linkLocalFrame,childShape);
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

