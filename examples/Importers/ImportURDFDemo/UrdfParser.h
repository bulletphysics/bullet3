#ifndef URDF_PARSER_H
#define URDF_PARSER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"
#include "URDFJointTypes.h"
#include "SDFAudioTypes.h"

#define btArray btAlignedObjectArray
#include <string>

struct ErrorLogger
{
	virtual void reportError(const char* error)=0;
	virtual void reportWarning(const char* warning)=0;
	virtual void printMessage(const char* msg)=0;
};



struct UrdfMaterial
{
	std::string m_name;
	std::string m_textureFilename;
	UrdfMaterialColor m_matColor;

	UrdfMaterial()
	{
	}
};

struct UrdfInertia
{
	btTransform m_linkLocalFrame;
	bool m_hasLinkLocalFrame;

	double m_mass;
	double m_ixx,m_ixy,m_ixz,m_iyy,m_iyz,m_izz;
	
	UrdfInertia()
	{
		m_hasLinkLocalFrame = false;
		m_linkLocalFrame.setIdentity();
		m_mass = 0.f;
		m_ixx=m_ixy=m_ixz=m_iyy=m_iyz=m_izz=0.f;
	}
};

enum UrdfGeomTypes
{
	URDF_GEOM_SPHERE=2,
	URDF_GEOM_BOX,
	URDF_GEOM_CYLINDER,
	URDF_GEOM_MESH,
	URDF_GEOM_PLANE,
	URDF_GEOM_CAPSULE, //non-standard URDF?
	URDF_GEOM_UNKNOWN, 
};


struct UrdfGeometry
{
	UrdfGeomTypes m_type;
	
	double m_sphereRadius;
	
	btVector3 m_boxSize;
	
	double m_capsuleRadius;
	double m_capsuleHeight;
	int m_hasFromTo;
	btVector3 m_capsuleFrom;
	btVector3 m_capsuleTo;

	btVector3 m_planeNormal;
    
	enum {
		FILE_STL     =1,
		FILE_COLLADA =2,
		FILE_OBJ     =3,
	};
	int         m_meshFileType;
	std::string m_meshFileName;
	btVector3   m_meshScale;

	UrdfMaterial m_localMaterial;
	bool m_hasLocalMaterial;

	UrdfGeometry()
	:m_type(URDF_GEOM_UNKNOWN),
		m_sphereRadius(1),
		m_boxSize(1,1,1),
		m_capsuleRadius(1),
		m_capsuleHeight(1),
		m_hasFromTo(0),
		m_capsuleFrom(0,1,0),
		m_capsuleTo(1,0,0),
		m_planeNormal(0,0,1),
		m_meshFileType(0),
		m_meshScale(1,1,1),
	m_hasLocalMaterial(false)
	{
	}

};

bool findExistingMeshFile(const std::string& urdf_path, std::string fn,
	const std::string& error_message_prefix,
	std::string* out_found_filename, int* out_type); // intended to fill UrdfGeometry::m_meshFileName and Type, but can be used elsewhere

struct UrdfShape
{
	std::string m_sourceFileLocation;
	btTransform m_linkLocalFrame;
	UrdfGeometry m_geometry;
	std::string m_name;
};

struct UrdfVisual: UrdfShape
{
	std::string m_materialName;
};

struct UrdfCollision: UrdfShape
{
	int m_flags;
	int m_collisionGroup;
	int m_collisionMask;
	UrdfCollision()
		:m_flags(0)
	{
	}
};

struct UrdfJoint;

struct UrdfLink
{
	std::string	m_name;
	UrdfInertia	m_inertia;
    btTransform m_linkTransformInWorld;
	btArray<UrdfVisual> m_visualArray;
	btArray<UrdfCollision> m_collisionArray;
	UrdfLink* m_parentLink;
	UrdfJoint* m_parentJoint;
	
	btArray<UrdfJoint*> m_childJoints;
	btArray<UrdfLink*> m_childLinks;
	
	int m_linkIndex;
	
	URDFLinkContactInfo m_contactInfo;

	SDFAudioSource m_audioSource;

	UrdfLink()
		:m_parentLink(0),
		m_parentJoint(0),
		m_linkIndex(-2)
	{
	}
	
};
struct UrdfJoint
{
	std::string m_name;
	UrdfJointTypes m_type;
	btTransform m_parentLinkToJointTransform;
	std::string m_parentLinkName;
	std::string m_childLinkName;
	btVector3 m_localJointAxis;
	
	double m_lowerLimit;
	double m_upperLimit;
	
	double m_effortLimit;
	double m_velocityLimit;

	double m_jointDamping;
	double m_jointFriction;
	UrdfJoint()
		:m_lowerLimit(0),
		m_upperLimit(-1),
		m_effortLimit(0),
		m_velocityLimit(0),
		m_jointDamping(0),
		m_jointFriction(0)
	{
	}
};

struct UrdfModel
{
	std::string m_name;
	std::string m_sourceFile;
    btTransform m_rootTransformInWorld;
	btHashMap<btHashString, UrdfMaterial*> m_materials;
	btHashMap<btHashString, UrdfLink*> m_links;
	btHashMap<btHashString, UrdfJoint*> m_joints;
	
	btArray<UrdfLink*> m_rootLinks;
	bool m_overrideFixedBase;

	UrdfModel()
		:m_overrideFixedBase(false)
	{
		m_rootTransformInWorld.setIdentity();
	}

	~UrdfModel()
	{
		for (int i = 0; i < m_materials.size(); i++)
		{
			UrdfMaterial** ptr = m_materials.getAtIndex(i);
			if (ptr)
			{
				UrdfMaterial* t = *ptr;
				delete t;
			}
		}
		for (int i = 0; i < m_links.size(); i++)
		{
			UrdfLink** ptr = m_links.getAtIndex(i);
			if (ptr)
			{
				UrdfLink* t = *ptr;
				delete t;
			}
		}
		for (int i = 0; i < m_joints.size(); i++)
		{
			UrdfJoint** ptr = m_joints.getAtIndex(i);
			if (ptr)
			{
				UrdfJoint* t = *ptr;
				delete t;
			}
		}
	}
};

class UrdfParser
{
protected:
    
    UrdfModel m_urdf2Model;
    btAlignedObjectArray<UrdfModel*> m_sdfModels;
    btAlignedObjectArray<UrdfModel*> m_tmpModels;
    
    bool m_parseSDF;
    int m_activeSdfModel;

	btScalar m_urdfScaling;
    bool parseTransform(btTransform& tr, class TiXmlElement* xml, ErrorLogger* logger, bool parseSDF = false);
	bool parseInertia(UrdfInertia& inertia, class TiXmlElement* config, ErrorLogger* logger);
	bool parseGeometry(UrdfGeometry& geom, class TiXmlElement* g, ErrorLogger* logger);
	bool parseVisual(UrdfModel& model, UrdfVisual& visual, class TiXmlElement* config, ErrorLogger* logger);
	bool parseCollision(UrdfCollision& collision, class TiXmlElement* config, ErrorLogger* logger);
	bool initTreeAndRoot(UrdfModel& model, ErrorLogger* logger);
	bool parseMaterial(UrdfMaterial& material, class TiXmlElement *config, ErrorLogger* logger);
	bool parseJointLimits(UrdfJoint& joint, TiXmlElement* config, ErrorLogger* logger);
    bool parseJointDynamics(UrdfJoint& joint, TiXmlElement* config, ErrorLogger* logger);
	bool parseJoint(UrdfJoint& link, TiXmlElement *config, ErrorLogger* logger);
	bool parseLink(UrdfModel& model, UrdfLink& link, TiXmlElement *config, ErrorLogger* logger);
	
    
public:
	
	UrdfParser();
	virtual ~UrdfParser();

    void setParseSDF(bool useSDF)
    {
        m_parseSDF = useSDF;
    }
    bool getParseSDF() const
    {
        return m_parseSDF;
    }
	void setGlobalScaling(btScalar scaling)
	{
		m_urdfScaling = scaling;
	}

    bool loadUrdf(const char* urdfText, ErrorLogger* logger, bool forceFixedBase);
    bool loadSDF(const char* sdfText, ErrorLogger* logger);
    
    int getNumModels() const
    {
        //user should have loaded an SDF when calling this method
        if (m_parseSDF)
        {
            return m_sdfModels.size();
        }
		return 1;
    }
    
    void activateModel(int modelIndex);
    
    UrdfModel& getModelByIndex(int index)
    {
        //user should have loaded an SDF when calling this method
        btAssert(m_parseSDF);

        return *m_sdfModels[index];
    }

    const UrdfModel& getModelByIndex(int index) const
    {
        //user should have loaded an SDF when calling this method
        btAssert(m_parseSDF);

        return *m_sdfModels[index];
    }
    
    const UrdfModel& getModel() const
	{
        if (m_parseSDF)
        {
            return *m_sdfModels[m_activeSdfModel];
        }

		return m_urdf2Model;
	}
	
	UrdfModel& getModel()
 	{
        if (m_parseSDF)
        {
            return *m_sdfModels[m_activeSdfModel];
        }
		return m_urdf2Model;
	}

	std::string sourceFileLocation(TiXmlElement* e);

	void setSourceFile(const std::string& sourceFile)
	{
		m_urdf2Model.m_sourceFile = sourceFile;
	}
};

#endif

