#ifndef URDF_PARSER_H
#define URDF_PARSER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"
#include "URDFJointTypes.h"

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
	btVector4 m_rgbaColor;
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
	URDF_GEOM_CAPSULE//non-standard URDF?
    
};


struct UrdfGeometry
{
	UrdfGeomTypes m_type;
	
	double m_sphereRadius;
	
	btVector3 m_boxSize;
	
	double m_capsuleRadius;
	double m_capsuleHalfHeight;
	int m_hasFromTo;
	btVector3 m_capsuleFrom;
	btVector3 m_capsuleTo;

	double m_cylinderRadius;
	double m_cylinderLength;

    btVector3 m_planeNormal;
    
	std::string m_meshFileName;
	btVector3 m_meshScale;
};

struct UrdfVisual
{
	btTransform m_linkLocalFrame;
	UrdfGeometry m_geometry;
	std::string m_name;
	std::string m_materialName;
	bool m_hasLocalMaterial;
	UrdfMaterial m_localMaterial;
};




struct UrdfCollision
{
	btTransform m_linkLocalFrame;
	UrdfGeometry m_geometry;
	std::string m_name;
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

	UrdfLink()
		:m_parentLink(0),
		m_parentJoint(0)
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
	
};

class UrdfParser
{
protected:
    
    UrdfModel m_urdf2Model;
    btAlignedObjectArray<UrdfModel*> m_sdfModels;
    btAlignedObjectArray<UrdfModel*> m_tmpModels;
    
    bool m_parseSDF;
    int m_activeSdfModel;

    
    void cleanModel(UrdfModel* model);
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
    bool loadUrdf(const char* urdfText, ErrorLogger* logger, bool forceFixedBase);
    bool loadSDF(const char* sdfText, ErrorLogger* logger);
    
    int getNumModels() const
    {
        //user should have loaded an SDF when calling this method
        btAssert(m_parseSDF);
        if (m_parseSDF)
        {
            return m_sdfModels.size();
        }
		return 0;
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
};

#endif

