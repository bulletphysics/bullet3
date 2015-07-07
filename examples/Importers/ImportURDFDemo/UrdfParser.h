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
	double m_mass;
	double m_ixx,m_ixy,m_ixz,m_iyy,m_iyz,m_izz;
	
	UrdfInertia()
	{
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
	URDF_GEOM_MESH
};


struct UrdfGeometry
{
	UrdfGeomTypes m_type;
	
	double m_sphereRadius;
	
	btVector3 m_boxSize;
	
	double m_cylinderRadius;
	double m_cylinderLength;

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
};

struct UrdfJoint;

struct UrdfLink
{
	std::string	m_name;
	UrdfInertia	m_inertia;
	btArray<UrdfVisual> m_visualArray;
	btArray<UrdfCollision> m_collisionArray;
	UrdfLink* m_parentLink;
	UrdfJoint* m_parentJoint;
	
	btArray<UrdfJoint*> m_childJoints;
	btArray<UrdfLink*> m_childLinks;
	
	int m_linkIndex;
	
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
};

struct UrdfModel
{
	std::string m_name;
	btHashMap<btHashString, UrdfMaterial*> m_materials;
	btHashMap<btHashString, UrdfLink*> m_links;
	btHashMap<btHashString, UrdfJoint*> m_joints;
	
	btArray<UrdfLink*> m_rootLinks;
	
};

class UrdfParser
{
protected:
	bool parseInertia(UrdfInertia& inertia, class TiXmlElement* config, ErrorLogger* logger);
	bool parseGeometry(UrdfGeometry& geom, class TiXmlElement* g, ErrorLogger* logger);
	bool parseVisual(UrdfVisual& visual, class TiXmlElement* config, ErrorLogger* logger);
	bool parseCollision(UrdfCollision& collision, class TiXmlElement* config, ErrorLogger* logger);
	bool initTreeAndRoot(ErrorLogger* logger);
	bool parseMaterial(UrdfMaterial& material, class TiXmlElement *config, ErrorLogger* logger);
	bool parseJointLimits(UrdfJoint& joint, TiXmlElement* config, ErrorLogger* logger);
	bool parseJoint(UrdfJoint& link, TiXmlElement *config, ErrorLogger* logger);
	bool parseLink(UrdfLink& link, TiXmlElement *config, ErrorLogger* logger);
	UrdfModel m_model;
	
public:
	
	UrdfParser();
	virtual ~UrdfParser();
	
	bool loadUrdf(const char* urdfText, ErrorLogger* logger, bool forceFixedBase);

	const UrdfModel& getModel() const
	{
		return m_model;
	}
	
	UrdfModel& getModel()
 	{
		return m_model;
	}
};

#endif

