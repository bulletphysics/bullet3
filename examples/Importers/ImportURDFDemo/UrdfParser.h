#ifndef URDF_PARSER_H
#define URDF_PARSER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"
#include "URDFJointTypes.h"
#include "SDFAudioTypes.h"
#include <string>

#define btArray btAlignedObjectArray

struct ErrorLogger
{
	virtual ~ErrorLogger() {}
	virtual void reportError(const char* error) = 0;
	virtual void reportWarning(const char* warning) = 0;
	virtual void printMessage(const char* msg) = 0;
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
	double m_ixx, m_ixy, m_ixz, m_iyy, m_iyz, m_izz;

	UrdfInertia()
	{
		m_hasLinkLocalFrame = false;
		m_linkLocalFrame.setIdentity();
		m_mass = 0.f;
		m_ixx = m_ixy = m_ixz = m_iyy = m_iyz = m_izz = 0.f;
	}
};

enum UrdfGeomTypes
{
	URDF_GEOM_SPHERE = 2,
	URDF_GEOM_BOX,
	URDF_GEOM_CYLINDER,
	URDF_GEOM_MESH,
	URDF_GEOM_PLANE,
	URDF_GEOM_CAPSULE,  //non-standard URDF
	URDF_GEOM_CDF,      //signed-distance-field, non-standard URDF
	URDF_GEOM_HEIGHTFIELD,   //heightfield, non-standard URDF
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

	enum
	{
		FILE_STL = 1,
		FILE_COLLADA = 2,
		FILE_OBJ = 3,
		FILE_CDF = 4,
		MEMORY_VERTICES = 5,
	        FILE_VTK = 6,

	};
	int m_meshFileType;
	std::string m_meshFileName;
	btVector3 m_meshScale;

	btArray<btVector3> m_vertices;
	btArray<btVector3> m_uvs;
	btArray<btVector3> m_normals;
	btArray<int> m_indices;


	UrdfMaterial m_localMaterial;
	bool m_hasLocalMaterial;

	UrdfGeometry()
		: m_type(URDF_GEOM_UNKNOWN),
		  m_sphereRadius(1),
		  m_boxSize(1, 1, 1),
		  m_capsuleRadius(1),
		  m_capsuleHeight(1),
		  m_hasFromTo(0),
		  m_capsuleFrom(0, 1, 0),
		  m_capsuleTo(1, 0, 0),
		  m_planeNormal(0, 0, 1),
		  m_meshFileType(0),
		  m_meshScale(1, 1, 1),
		  m_hasLocalMaterial(false)
	{
	}
};


struct UrdfShape
{
	std::string m_sourceFileLocation;
	btTransform m_linkLocalFrame;
	UrdfGeometry m_geometry;
	std::string m_name;
};

struct UrdfVisual : UrdfShape
{
	std::string m_materialName;
	// Maps user data keys to user data values.
	btHashMap<btHashString, std::string> m_userData;
};

struct UrdfCollision : UrdfShape
{
	int m_flags;
	int m_collisionGroup;
	int m_collisionMask;
	UrdfCollision()
		: m_flags(0)
	{
	}
};

struct UrdfJoint;

struct UrdfLink
{
	std::string m_name;
	UrdfInertia m_inertia;
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
	// Maps user data keys to user data values.
	btHashMap<btHashString, std::string> m_userData;

	UrdfLink()
		: m_parentLink(0),
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
		: m_lowerLimit(0),
		  m_upperLimit(-1),
		  m_effortLimit(0),
		  m_velocityLimit(0),
		  m_jointDamping(0),
		  m_jointFriction(0)
	{
	}
};

struct SpringCoeffcients
{
	double elastic_stiffness;
	double damping_stiffness;
	double bending_stiffness;
	int damp_all_directions;
	int bending_stride;
	SpringCoeffcients() : elastic_stiffness(0.),
						  damping_stiffness(0.),
						  bending_stiffness(0.),
						  damp_all_directions(0),
						  bending_stride(2) {}
};

struct LameCoefficients
{
	double mu;
	double lambda;
	double damping;
	LameCoefficients() : mu(0.), lambda(0.), damping(0.) {}
};

struct UrdfDeformable
{
	std::string m_name;
	double m_mass;
	double m_collisionMargin;
	double m_friction;
	double m_repulsionStiffness;
	double m_gravFactor;
	bool m_cache_barycenter;

	SpringCoeffcients m_springCoefficients;
	LameCoefficients m_corotatedCoefficients;
	LameCoefficients m_neohookeanCoefficients;

	std::string m_visualFileName;
	std::string m_simFileName;
	btHashMap<btHashString, std::string> m_userData;

	UrdfDeformable() : m_mass(1.), m_collisionMargin(0.02), m_friction(1.), m_repulsionStiffness(0.5), m_gravFactor(1.), m_cache_barycenter(false), m_visualFileName(""), m_simFileName("")
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
	UrdfDeformable m_deformable;
	// Maps user data keys to user data values.
	btHashMap<btHashString, std::string> m_userData;

	btArray<UrdfLink*> m_rootLinks;
	bool m_overrideFixedBase;

	UrdfModel()
		: m_overrideFixedBase(false)
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

namespace tinyxml2
{
class XMLElement;
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

	struct CommonFileIOInterface* m_fileIO;

	bool parseTransform(btTransform& tr, tinyxml2::XMLElement* xml, ErrorLogger* logger, bool parseSDF = false);
	bool parseInertia(UrdfInertia& inertia, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseGeometry(UrdfGeometry& geom, tinyxml2::XMLElement* g, ErrorLogger* logger);
	bool parseVisual(UrdfModel& model, UrdfVisual& visual, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseCollision(UrdfCollision& collision, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool initTreeAndRoot(UrdfModel& model, ErrorLogger* logger);
	bool parseMaterial(UrdfMaterial& material, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseJointLimits(UrdfJoint& joint, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseJointDynamics(UrdfJoint& joint, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseJoint(UrdfJoint& joint, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseLink(UrdfModel& model, UrdfLink& link, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseSensor(UrdfModel& model, UrdfLink& link, UrdfJoint& joint, tinyxml2::XMLElement* config, ErrorLogger* logger);
  bool parseLameCoefficients(LameCoefficients& lameCoefficients, tinyxml2::XMLElement* config, ErrorLogger* logger);
	bool parseDeformable(UrdfModel& model, tinyxml2::XMLElement* config, ErrorLogger* logger);


public:
	UrdfParser(struct CommonFileIOInterface* fileIO);
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

	bool loadUrdf(const char* urdfText, ErrorLogger* logger, bool forceFixedBase, bool parseSensors);

	bool loadUrdf(const char* urdfText, ErrorLogger* logger, bool forceFixedBase)
	{
		return loadUrdf(urdfText, logger, forceFixedBase, false);
	}

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

	const UrdfDeformable& getDeformable() const
	{
		return m_urdf2Model.m_deformable;
	}

	bool mergeFixedLinks(UrdfModel& model, UrdfLink* link, ErrorLogger* logger, bool forceFixedBase, int level);
	bool printTree(UrdfLink* link, ErrorLogger* logger, int level);
	bool recreateModel(UrdfModel& model, UrdfLink* link, ErrorLogger* logger);

	std::string sourceFileLocation(tinyxml2::XMLElement* e);

	void setSourceFile(const std::string& sourceFile)
	{
		m_urdf2Model.m_sourceFile = sourceFile;
	}
};

#endif
