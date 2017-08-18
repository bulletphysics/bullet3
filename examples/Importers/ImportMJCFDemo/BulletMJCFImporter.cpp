#include "BulletMJCFImporter.h"
#include "../../ThirdPartyLibs/tinyxml/tinyxml.h"
#include "Bullet3Common/b3FileUtils.h"
#include "Bullet3Common/b3HashMap.h"

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
		logger->reportWarning( ("Couldn't parse vector3 '" + vector_str + "'").c_str() );
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
		logger->reportWarning( ("Couldn't parse 6 floats '" + vector_str + "'").c_str() );

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

struct MyMJCFDefaults
{
	int m_defaultCollisionGroup;
	int m_defaultCollisionMask;
	btScalar m_defaultCollisionMargin;

	// joint defaults
	std::string m_defaultJointLimited;

	// geom defaults
	std::string m_defaultGeomRgba;
	int m_defaultConDim;
	double m_defaultLateralFriction;
	double m_defaultSpinningFriction;
	double m_defaultRollingFriction;

	MyMJCFDefaults()
		:m_defaultCollisionGroup(1),
		m_defaultCollisionMask(1),
		m_defaultCollisionMargin(0.001),//assume unit meters, margin is 1mm
		m_defaultConDim(3),
		m_defaultLateralFriction(0.5),
		m_defaultSpinningFriction(0),
		m_defaultRollingFriction(0)
	{
	}

};

struct BulletMJCFImporterInternalData
{
	GUIHelperInterface* m_guiHelper;
	struct LinkVisualShapesConverter* m_customVisualShapesConverter;
	char m_pathPrefix[1024];

	std::string m_sourceFileName; // with path
	std::string m_fileModelName;  // without path
	btHashMap<btHashString,MyMJCFAsset> m_assets;

	btAlignedObjectArray<UrdfModel*>	m_models;

	//<compiler angle="radian" meshdir="mesh/" texturedir="texture/"/>
	std::string m_meshDir;
	std::string m_textureDir;
	std::string m_angleUnits;


	int m_activeModel;
	int m_activeBodyUniqueId;

	//todo: for better MJCF compatibility, we would need a stack of default values
	MyMJCFDefaults m_globalDefaults;
	b3HashMap<b3HashString, MyMJCFDefaults> m_classDefaults;

	//those collision shapes are deleted by caller (todo: make sure this happens!)
	btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;

	BulletMJCFImporterInternalData()
		:m_activeModel(-1),
		m_activeBodyUniqueId(-1)
	{
		m_pathPrefix[0] = 0;
	}

	~BulletMJCFImporterInternalData()
	{
		for (int i=0;i<m_models.size();i++)
		{
			delete m_models[i];
		}
	}

	std::string sourceFileLocation(TiXmlElement* e)
	{
#if 0
	//no C++11 snprintf etc
		char buf[1024];
		snprintf(buf, sizeof(buf), "%s:%i", m_sourceFileName.c_str(), e->Row());
		return buf;
#else
		char row[1024];
		sprintf(row,"%d",e->Row());
		std::string str = m_sourceFileName.c_str() + std::string(":") + std::string(row);
		return str;
#endif
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
		const char* angle = root_xml->Attribute("angle");
		m_angleUnits = angle ? angle : "degree";  // degrees by default, http://www.mujoco.org/book/modeling.html#compiler
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
	
	
	bool parseDefaults(MyMJCFDefaults& defaults, TiXmlElement* root_xml, MJCFErrorLogger* logger)
	{
		bool handled= false;
		//rudimentary 'default' support, would need more work for better feature coverage
		for (TiXmlElement* child_xml = root_xml->FirstChildElement() ; child_xml ; child_xml = child_xml->NextSiblingElement())
		{
			std::string n = child_xml->Value();
		
			if (n.find("default")!=std::string::npos)
			{
				const char* className = child_xml->Attribute("class");

				if (className)
				{
					MyMJCFDefaults* curDefaultsPtr = m_classDefaults[className];
					if (!curDefaultsPtr)
					{
						MyMJCFDefaults def;
						m_classDefaults.insert(className,def);
						curDefaultsPtr = m_classDefaults[className];
					}
					if (curDefaultsPtr)
					{
						MyMJCFDefaults& curDefaults = *curDefaultsPtr; 
						parseDefaults(curDefaults, child_xml, logger);
					}
				}
			}

			if (n=="inertial")
			{
			}
			if (n=="asset")
			{
				parseAssets(child_xml,logger);
			}
			if (n=="joint")
			{
				// Other attributes here:
				// armature="1"
				// damping="1"
				// limited="true"
				if (const char* conTypeStr = child_xml->Attribute("limited"))
				{
					defaults.m_defaultJointLimited = child_xml->Attribute("limited");
				}
			}
			if (n=="geom")
			{
				//contype, conaffinity 
				const char* conTypeStr = child_xml->Attribute("contype");
				if (conTypeStr)
				{
					defaults.m_defaultCollisionGroup = urdfLexicalCast<int>(conTypeStr);
				}
				const char* conAffinityStr = child_xml->Attribute("conaffinity");
				if (conAffinityStr)
				{
					defaults.m_defaultCollisionMask = urdfLexicalCast<int>(conAffinityStr);
				}
				const char* rgba = child_xml->Attribute("rgba");
				if (rgba)
				{
					defaults.m_defaultGeomRgba = rgba;
				}

				const char* conDimS = child_xml->Attribute("condim");
				if (conDimS)
				{
					defaults.m_defaultConDim=urdfLexicalCast<int>(conDimS);
				}
				int conDim = defaults.m_defaultConDim;

				const char* frictionS = child_xml->Attribute("friction");
				if (frictionS)
				{
					btArray<std::string> pieces;
					btArray<float> frictions;
					btAlignedObjectArray<std::string> strArray;
					urdfIsAnyOf(" ", strArray);
					urdfStringSplit(pieces, frictionS, strArray);
					for (int i = 0; i < pieces.size(); ++i)
					{
						if (!pieces[i].empty())
						{
							frictions.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
						}
					}
					if (frictions.size()>0)
					{
						defaults.m_defaultLateralFriction = frictions[0];
					}
					if (frictions.size()>1)
					{
						defaults.m_defaultSpinningFriction = frictions[1];
					}
					if (frictions.size()>2)
					{
						defaults.m_defaultRollingFriction = frictions[2];
					}
				}
			}
		}
		handled=true;
		return handled;
	}
	bool parseRootLevel(MyMJCFDefaults& defaults,  TiXmlElement* root_xml,MJCFErrorLogger* logger)
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
				parseBody(defaults, rootxml,modelIndex, INVALID_LINK_INDEX,logger);
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
				parseGeom(defaults, rootxml,modelIndex, linkIndex,logger,inertialShift);
				initTreeAndRoot(*modelPtr,logger);

				handled = true;
			}


			if (n=="site")
			{
				handled = true;
			}
			if (!handled)
			{
				logger->reportWarning( (sourceFileLocation(rootxml) + ": unhandled root element '" + n + "'").c_str() );
			}
		}
		return true;
	}

	bool parseJoint(MyMJCFDefaults& defaults, TiXmlElement* link_xml, int modelIndex, int parentLinkIndex, int linkIndex, MJCFErrorLogger* logger, const btTransform& parentToLinkTrans, btTransform& jointTransOut)
	{
		bool jointHandled = false;
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
			logger->reportWarning( (sourceFileLocation(link_xml) + ": joint without axis attribute").c_str() );
		}

		double range[2] = {1,0};
		std::string lim = m_globalDefaults.m_defaultJointLimited;
		if (limitedStr)
		{
			lim = limitedStr;
		}
		bool isLimited = lim=="true";

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
			logger->reportWarning( (sourceFileLocation(link_xml) + ": expected 'type' attribute for joint").c_str() );
		}

		if (isLimited)
		{
			//parse the 'range' field
			btArray<std::string> pieces;
			btArray<float> limits;
			btAlignedObjectArray<std::string> strArray;
			urdfIsAnyOf(" ", strArray);
			urdfStringSplit(pieces, rangeStr, strArray);
			for (int i = 0; i < pieces.size(); ++i)
			{
				if (!pieces[i].empty())
				{
					limits.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
				}
			}
			if (limits.size()==2)
			{
				range[0] = limits[0];
				range[1] = limits[1];
				if (m_angleUnits=="degree" && ejtype==URDFRevoluteJoint)
				{
					range[0] = limits[0] * B3_PI / 180;
					range[1] = limits[1] * B3_PI / 180;
				}
			}
			else
			{
				logger->reportWarning( (sourceFileLocation(link_xml) + ": cannot parse 'range' attribute (units='" + m_angleUnits + "'')").c_str() );
			}
		}

		// TODO armature : real, "0" Armature inertia (or rotor inertia) of all
		// degrees of freedom created by this joint. These are constants added to the
		// diagonal of the inertia matrix in generalized coordinates. They make the
		// simulation more stable, and often increase physical realism. This is because
		// when a motor is attached to the system with a transmission that amplifies
		// the motor force by c, the inertia of the rotor (i.e. the moving part of the
		// motor) is amplified by c*c. The same holds for gears in the early stages of
		// planetary gear boxes. These extra inertias often dominate the inertias of
		// the robot parts that are represented explicitly in the model, and the
		// armature attribute is the way to model them.

		// TODO damping : real, "0" Damping applied to all degrees of
		// freedom created by this joint. Unlike friction loss
		// which is computed by the constraint solver, damping is
		// simply a force linear in velocity. It is included in
		// the passive forces. Despite this simplicity, larger
		// damping values can make numerical integrators unstable,
		// which is why our Euler integrator handles damping
		// implicitly. See Integration in the Computation chapter.

		const UrdfLink* linkPtr = getLink(modelIndex,linkIndex);
		
		btTransform parentLinkToJointTransform;
		parentLinkToJointTransform.setIdentity();
		parentLinkToJointTransform = parentToLinkTrans*jointTrans;
		jointTransOut = jointTrans;

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
	bool parseGeom(MyMJCFDefaults& defaults, TiXmlElement* link_xml, int modelIndex, int linkIndex, MJCFErrorLogger* logger, btVector3& inertialShift)
	{
		UrdfLink** linkPtrPtr = m_models[modelIndex]->m_links.getAtIndex(linkIndex);
		if (linkPtrPtr==0)
		{
			// XXX: should it be assert?
			logger->reportWarning("Invalide linkindex");
			return false;
		}
		UrdfLink* linkPtr = *linkPtrPtr;
		
		btTransform linkLocalFrame;
		linkLocalFrame.setIdentity();


		bool handledGeomType = false;
		UrdfGeometry geom;

		const char* sz = link_xml->Attribute("size");
		int conDim = defaults.m_defaultConDim;

		const char* conDimS = link_xml->Attribute("condim");
		{
			if (conDimS)
			{
				conDim = urdfLexicalCast<int>(conDimS);
			} 
		}

		double lateralFriction = defaults.m_defaultLateralFriction;
		double spinningFriction = defaults.m_defaultSpinningFriction;
		double rollingFriction = defaults.m_defaultRollingFriction;

		const char* frictionS = link_xml->Attribute("friction");
		if (frictionS)
		{
			btArray<std::string> pieces;
			btArray<float> frictions;
			btAlignedObjectArray<std::string> strArray;
			urdfIsAnyOf(" ", strArray);
			urdfStringSplit(pieces, frictionS, strArray);
			for (int i = 0; i < pieces.size(); ++i)
			{
				if (!pieces[i].empty())
				{
					frictions.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
				}
			}
			if (frictions.size()>0)
			{
				lateralFriction = frictions[0];
			}
			if (frictions.size()>1 && conDim>3)
			{
				spinningFriction = frictions[1];
			}
			if (frictions.size()>2 && conDim>4)
			{
				rollingFriction = frictions[2];
			}

		}

		linkPtr->m_contactInfo.m_lateralFriction=lateralFriction;
		linkPtr->m_contactInfo.m_spinningFriction=spinningFriction;
		linkPtr->m_contactInfo.m_rollingFriction=rollingFriction;

		if (conDim>3)
		{
			linkPtr->m_contactInfo.m_spinningFriction=defaults.m_defaultSpinningFriction;
			linkPtr->m_contactInfo.m_flags |= URDF_CONTACT_HAS_SPINNING_FRICTION;
		}
		if (conDim>4)
		{
			linkPtr->m_contactInfo.m_rollingFriction=defaults.m_defaultRollingFriction;
			linkPtr->m_contactInfo.m_flags |= URDF_CONTACT_HAS_ROLLING_FRICTION;
		}

		std::string rgba = defaults.m_defaultGeomRgba;
		if (const char* rgbattr = link_xml->Attribute("rgba"))
		{
			rgba = rgbattr;
		}
		if (!rgba.empty())
		{
			// "0 0.7 0.7 1"
			parseVector4(geom.m_localMaterial.m_matColor.m_rgbaColor, rgba);
			geom.m_hasLocalMaterial = true;
			geom.m_localMaterial.m_name = rgba;
		}

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
			btQuaternion orn(0,0,0,1);
			btVector4 o4;
			if (parseVector4(o4, ornS))
			{
				orn.setValue(o4[1],o4[2],o4[3],o4[0]);
				linkLocalFrame.setRotation(orn);
			}
		}

		const char* axis_and_angle = link_xml->Attribute("axisangle");
		if (axis_and_angle)
		{
			btQuaternion orn(0,0,0,1);
			btVector4 o4;
			if (parseVector4(o4, axis_and_angle))
			{
				orn.setRotation(btVector3(o4[0],o4[1],o4[2]), o4[3]);
				linkLocalFrame.setRotation(orn);
			}
		}

		const char* gType = link_xml->Attribute("type");
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
					logger->reportWarning( (sourceFileLocation(link_xml) + ": no size field (scalar) in sphere geom").c_str() );
				}
				handledGeomType = true;
			}

			if (geomType == "capsule" || geomType == "cylinder")
			{
				// <geom conaffinity="0" contype="0" fromto="0 0 0 0 0 0.02" name="root" rgba="0.9 0.4 0.6 1" size=".011" type="cylinder"/>
				geom.m_type = geomType=="cylinder" ? URDF_GEOM_CYLINDER : URDF_GEOM_CAPSULE;

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

				geom.m_capsuleRadius = 2.00f; // 2 to make it visible if something is wrong
				geom.m_capsuleHeight = 2.00f;

				if (sizes.size()>0)
				{
					geom.m_capsuleRadius = sizes[0];
					if (sizes.size()>1)
					{
						geom.m_capsuleHeight = 2*sizes[1];
					}
				} else
				{
					logger->reportWarning( (sourceFileLocation(link_xml) + ": couldn't convert 'size' attribute of capsule geom").c_str() );
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
						logger->reportWarning( (sourceFileLocation(link_xml) + ": capsule without fromto attribute requires 2 sizes (radius and halfheight)").c_str() );
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
						geom.m_type = URDF_GEOM_MESH;
						geom.m_meshFileName = assetPtr->m_fileName;
						bool exists = findExistingMeshFile(
							m_sourceFileName, assetPtr->m_fileName, sourceFileLocation(link_xml),
							&geom.m_meshFileName,
							&geom.m_meshFileType);
						handledGeomType = exists;

						geom.m_meshScale.setValue(1,1,1);
						//todo: parse mesh scale
						if (sz)
						{
						}
					}
				}
			}
			if (handledGeomType)
			{
						
				UrdfCollision col;
				col.m_flags |= URDF_HAS_COLLISION_GROUP;
				col.m_collisionGroup = defaults.m_defaultCollisionGroup;
				
				col.m_flags |= URDF_HAS_COLLISION_MASK;
				col.m_collisionMask = defaults.m_defaultCollisionMask;
				
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
				col.m_sourceFileLocation = sourceFileLocation(link_xml);
				linkPtr->m_collisionArray.push_back(col);

			} else
			{
				logger->reportWarning( (sourceFileLocation(link_xml) + ": unhandled geom type '" + geomType + "'").c_str() );
			}
		} else
		{
			logger->reportWarning( (sourceFileLocation(link_xml) + ": geom requires type").c_str() );
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
			case URDF_GEOM_CYLINDER:
			case URDF_GEOM_CAPSULE:
			{
				//one sphere 
				double r = col->m_geometry.m_capsuleRadius;
				if (col->m_geometry.m_type==URDF_GEOM_CAPSULE)
				{
					totalVolume += 4./3.*SIMD_PI*r*r*r;
				}
				btScalar h(0);
				if (col->m_geometry.m_hasFromTo)
				{
					//and one cylinder of 'height'
					h = (col->m_geometry.m_capsuleFrom-col->m_geometry.m_capsuleTo).length();
				} else
				{
					h = col->m_geometry.m_capsuleHeight;
				}
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
		char linkn[1024];
		sprintf(linkn, "link%d_%d", modelIndex, orgChildLinkIndex);
		linkPtr->m_name = linkn;
		if (namePtr)
		{
			linkPtr->m_name = namePtr;
		}
		linkPtr->m_linkIndex = orgChildLinkIndex ;
		modelPtr->m_links.insert(linkPtr->m_name.c_str(),linkPtr);

		return orgChildLinkIndex;
	}

	bool parseBody(MyMJCFDefaults& defaults, TiXmlElement* link_xml, int modelIndex, int orgParentLinkIndex, MJCFErrorLogger* logger)
	{
		MyMJCFDefaults curDefaults = defaults;

		int newParentLinkIndex = orgParentLinkIndex;

		const char* childClassName = link_xml->Attribute("childclass");
		if (childClassName)
		{
			MyMJCFDefaults* classDefaults = m_classDefaults[childClassName];
			if (classDefaults)
			{
				curDefaults = *classDefaults;
			}
		}
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
				if (o)
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
						parseJoint(curDefaults,xml,modelIndex,newParentLinkIndex, newLinkIndex,logger,linkTransform,jointTrans);
						
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
					parseJoint(curDefaults, xml,modelIndex,newParentLinkIndex, newLinkIndex,logger,joint2nextjoint,unused);
					newParentLinkIndex = newLinkIndex;
					//todo: compute relative joint transforms (if any) and append to linkTransform
					hasJoint = true;
					handled = true;
				}
				
			}
			if (n == "geom")
			{
				btVector3 inertialShift(0,0,0);
				parseGeom(curDefaults, xml,modelIndex, orgChildLinkIndex , logger,inertialShift);
				if (!massDefined)
				{
					localInertialFrame.setOrigin(inertialShift);
				}
				handled = true;
			}

			//recursive
			if (n=="body")
			{
				parseBody(curDefaults, xml,modelIndex,orgChildLinkIndex,logger);
				handled = true;
			}

			if (n=="light")
			{
				handled = true;
			}
			if (n=="site")
			{
				handled = true;
			}

			if (!handled)
			{
				logger->reportWarning( (sourceFileLocation(xml) + ": unknown field '" + n + "'").c_str() );
			}
		}

		linkPtr->m_linkTransformInWorld = linkTransform;

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

BulletMJCFImporter::BulletMJCFImporter(struct GUIHelperInterface* helper, LinkVisualShapesConverter* customConverter)
{
	m_data = new BulletMJCFImporterInternalData();
	m_data->m_guiHelper = helper;
	m_data->m_customVisualShapesConverter = customConverter;
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
	m_data->m_sourceFileName = relativeFileName;
	
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
		m_data->parseDefaults(m_data->m_globalDefaults,link_xml,logger);
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
		m_data->parseRootLevel(m_data->m_globalDefaults, link_xml,logger);
	}

	for (TiXmlElement* link_xml = mujoco_xml->FirstChildElement("worldbody"); link_xml; link_xml = link_xml->NextSiblingElement("worldbody"))
	{
		m_data->parseRootLevel(m_data->m_globalDefaults, link_xml,logger);
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

std::string BulletMJCFImporter::getBodyName() const
{
	return m_data->m_fileModelName;
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
    
bool BulletMJCFImporter::getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const
{
	jointLowerLimit = 0.f;
    jointUpperLimit = 0.f;
	jointDamping = 0.f;
	jointFriction = 0.f;
	jointMaxForce = 0;
	jointMaxVelocity = 0;

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
}

bool BulletMJCFImporter::getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const
{
	//backwards compatibility for custom file importers
	btScalar jointMaxForce = 0;
	btScalar jointMaxVelocity = 0;
	return getJointInfo2(urdfLinkIndex, parent2joint, linkTransformInWorld, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction,jointMaxForce, jointMaxVelocity);
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

void BulletMJCFImporter::convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& inertialFrame, class btCollisionObject* colObj, int objectIndex) const
{
	if (m_data->m_customVisualShapesConverter)
	{
		const UrdfLink* link = m_data->getLink(m_data->m_activeModel, urdfIndex);
		m_data->m_customVisualShapesConverter->convertVisualShapes(linkIndex,pathPrefix,inertialFrame, link, 0, colObj, objectIndex);
	}
}

void BulletMJCFImporter::setBodyUniqueId(int bodyId)
{
	m_data->m_activeBodyUniqueId = bodyId;
}

int BulletMJCFImporter::getBodyUniqueId() const 
{ 
	b3Assert(m_data->m_activeBodyUniqueId != -1);
	return m_data->m_activeBodyUniqueId;
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

    
class btCompoundShape* BulletMJCFImporter::convertLinkCollisionShapes( int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
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
				if (col->m_geometry.m_hasFromTo)
				{
					btVector3 f = col->m_geometry.m_capsuleFrom;
					btVector3 t = col->m_geometry.m_capsuleTo;
					
					//compute the local 'fromto' transform
					btVector3 localPosition = btScalar(0.5)*(t+f);
					btQuaternion localOrn;
					localOrn = btQuaternion::getIdentity();

					btVector3 diff = t-f;
					btScalar lenSqr = diff.length2();
					btScalar height = 0.f;

					if (lenSqr > SIMD_EPSILON)
					{
						height = btSqrt(lenSqr);
						btVector3 ax = diff / height;

						btVector3 zAxis(0,0,1);
						localOrn = shortestArcQuat(zAxis,ax);
					}
					btCylinderShapeZ* cyl = new btCylinderShapeZ(btVector3(col->m_geometry.m_capsuleRadius,col->m_geometry.m_capsuleRadius,btScalar(0.5)*height));

					btCompoundShape* compound = new btCompoundShape();
					btTransform localTransform(localOrn,localPosition);
					compound->addChildShape(localTransform,cyl);
					childShape = compound;
				} else
				{
					btCylinderShapeZ* cap = new btCylinderShapeZ(btVector3(col->m_geometry.m_capsuleRadius,
						col->m_geometry.m_capsuleRadius,btScalar(0.5)*col->m_geometry.m_capsuleHeight));
					childShape = cap;
				}
				break;
			}
			case URDF_GEOM_MESH:
			{
				GLInstanceGraphicsShape* glmesh = 0;
				switch (col->m_geometry.m_meshFileType)
				{
				case UrdfGeometry::FILE_OBJ:
					{
						if (col->m_flags & URDF_FORCE_CONCAVE_TRIMESH)
						{
							glmesh = LoadMeshFromObj(col->m_geometry.m_meshFileName.c_str(), 0);
						}
						else
						{
							std::vector<tinyobj::shape_t> shapes;
							std::string err = tinyobj::LoadObj(shapes, col->m_geometry.m_meshFileName.c_str());
							//create a convex hull for each shape, and store it in a btCompoundShape

							childShape = MjcfCreateConvexHullFromShapes( shapes, col->m_geometry.m_meshScale, m_data->m_globalDefaults.m_defaultCollisionMargin);

						}
						break;
					}
				case UrdfGeometry::FILE_STL:
					{
						glmesh = LoadMeshFromSTL(col->m_geometry.m_meshFileName.c_str());
						break;
					}
				default:
					b3Warning("%s: Unsupported file type in Collision: %s (maybe .dae?)\n", col->m_sourceFileLocation.c_str(), col->m_geometry.m_meshFileType);
				}

				if (childShape)
				{
					// okay!
				}
				else if (!glmesh || glmesh->m_numvertices<=0)
				{
					b3Warning("%s: cannot extract anything useful from mesh '%s'\n", col->m_sourceFileLocation.c_str(), col->m_geometry.m_meshFileName.c_str());
				}
				else
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
						convexHull->setMargin(m_data->m_globalDefaults.m_defaultCollisionMargin);
						childShape = convexHull;
					}
				}

				delete glmesh;
				break;
			}
			case URDF_GEOM_CAPSULE:
			{
				if (col->m_geometry.m_hasFromTo)
				{
					btVector3 f = col->m_geometry.m_capsuleFrom;
					btVector3 t = col->m_geometry.m_capsuleTo;
					btVector3 fromto[2] = {f,t};
					btScalar radii[2] = {btScalar(col->m_geometry.m_capsuleRadius)
										,btScalar(col->m_geometry.m_capsuleRadius)};

					btMultiSphereShape* ms = new btMultiSphereShape(fromto,radii,2);
					childShape = ms;
				} else
				{
					btCapsuleShapeZ* cap = new btCapsuleShapeZ(col->m_geometry.m_capsuleRadius,
						col->m_geometry.m_capsuleHeight);
					childShape = cap;
				}
				break;
			}
			case URDF_GEOM_UNKNOWN:
                        {
				break;
			}

			} // switch geom

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

