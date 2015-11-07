#include "UrdfParser.h"

#include "tinyxml/tinyxml.h"
#include "urdfStringSplit.h"
#include "urdfLexicalCast.h"

UrdfParser::UrdfParser()
{
}
UrdfParser::~UrdfParser()
{
	//todo(erwincoumans) delete memory
}

static bool parseVector4(btVector4& vec4, const std::string& vector_str)
{
	vec4.setZero();
	btArray<std::string> pieces;
	btArray<float> rgba;
	urdfStringSplit(pieces, vector_str, urdfIsAnyOf(" "));
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

static bool parseVector3(btVector3& vec3, const std::string& vector_str, ErrorLogger* logger)
{
	vec3.setZero();
	btArray<std::string> pieces;
	btArray<float> rgba;
	urdfStringSplit(pieces, vector_str, urdfIsAnyOf(" "));
	for (int i = 0; i < pieces.size(); ++i)
	{
		if (!pieces[i].empty())
		{
			rgba.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
		}
	}
	if (rgba.size() != 3)
	{
		logger->reportWarning("Couldn't parse vector3");
		return false;
	}
	vec3.setValue(rgba[0],rgba[1],rgba[2]);
	return true;
}


bool UrdfParser::parseMaterial(UrdfMaterial& material, TiXmlElement *config, ErrorLogger* logger)
{
		
	if (!config->Attribute("name"))
	{
	  logger->reportError("Material must contain a name attribute");
	  return false;
	}

	material.m_name = config->Attribute("name");
		
	// texture
	TiXmlElement *t = config->FirstChildElement("texture");
	if (t)
	{
	  if (t->Attribute("filename"))
	  {
		  material.m_textureFilename = t->Attribute("filename");
	  }
	}

	if (material.m_textureFilename.length()==0)
	{
		//logger->reportWarning("material has no texture file name");
	}
		
	// color
	TiXmlElement *c = config->FirstChildElement("color");
	if (c)
	{
	  if (c->Attribute("rgba")) 
	  {
		  if (!parseVector4(material.m_rgbaColor,c->Attribute("rgba")))
		  {
			  std::string msg = material.m_name+" has no rgba";
			  logger->reportWarning(msg.c_str());
		  }
	  }
	}
	return true;

}

bool parseTransform(btTransform& tr, TiXmlElement* xml, ErrorLogger* logger)
{
	tr.setIdentity();
	
	{
		const char* xyz_str = xml->Attribute("xyz");
		if (xyz_str)
		{
			parseVector3(tr.getOrigin(),std::string(xyz_str),logger);
		}
	}
	
	{
		const char* rpy_str = xml->Attribute("rpy");
		if (rpy_str != NULL)
		{
			btVector3 rpy;
			if (parseVector3(rpy,std::string(rpy_str),logger))
			{
				double phi, the, psi;
				double roll = rpy[0];
				double pitch = rpy[1];
				double yaw = rpy[2];
				
				phi = roll / 2.0;
				the = pitch / 2.0;
				psi = yaw / 2.0;
			  
				btQuaternion orn(
								sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
								cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
								cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
								 cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi));
			  
				orn.normalize();
				tr.setRotation(orn);
			}
		}
	}
	return true;
}
bool UrdfParser::parseInertia(UrdfInertia& inertia, TiXmlElement* config, ErrorLogger* logger)
{
	inertia.m_linkLocalFrame.setIdentity();
	inertia.m_mass = 0.f;
	
		
	// Origin
	TiXmlElement *o = config->FirstChildElement("origin");
	if (o)
	{
	  if (!parseTransform(inertia.m_linkLocalFrame,o,logger))
	  {
		  return false;
	  }
	}
		
	TiXmlElement *mass_xml = config->FirstChildElement("mass");
	if (!mass_xml)
	{
	  logger->reportError("Inertial element must have a mass element");
	  return false;
	}
	if (!mass_xml->Attribute("value"))
	{
	  logger->reportError("Inertial: mass element must have value attribute");
	  return false;
	}

	inertia.m_mass = urdfLexicalCast<double>(mass_xml->Attribute("value"));
	
		
	TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
	if (!inertia_xml)
	{
	  logger->reportError("Inertial element must have inertia element");
	  return false;
	}
	if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
		inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
		inertia_xml->Attribute("izz")))
	{
	  logger->reportError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
	  return false;
	}
	inertia.m_ixx  = urdfLexicalCast<double>(inertia_xml->Attribute("ixx"));
	inertia.m_ixy  = urdfLexicalCast<double>(inertia_xml->Attribute("ixy"));
	inertia.m_ixz  = urdfLexicalCast<double>(inertia_xml->Attribute("ixz"));
	inertia.m_iyy  = urdfLexicalCast<double>(inertia_xml->Attribute("iyy"));
	inertia.m_iyz  = urdfLexicalCast<double>(inertia_xml->Attribute("iyz"));
	inertia.m_izz  = urdfLexicalCast<double>(inertia_xml->Attribute("izz"));
	
	return true;
}

bool UrdfParser::parseGeometry(UrdfGeometry& geom, TiXmlElement* g, ErrorLogger* logger)
{
	btAssert(g);
		
	TiXmlElement *shape = g->FirstChildElement();
	if (!shape)
	{
		logger->reportError("Geometry tag contains no child element.");
		return false;
	}
		
	const std::string type_name = shape->ValueTStr().c_str();
	if (type_name == "sphere")
	{
		geom.m_type = URDF_GEOM_SPHERE;
		if (!shape->Attribute("radius"))
		{
			logger->reportError("Sphere shape must have a radius attribute");
			return false;
		} else
		{
			geom.m_sphereRadius = urdfLexicalCast<double>(shape->Attribute("radius"));
		}
	}	
	else if (type_name == "box")
	{
		geom.m_type = URDF_GEOM_BOX;
	  if (!shape->Attribute("size"))
	  {
		  logger->reportError("box requires a size attribute");
	  } else
	  {
		  parseVector3(geom.m_boxSize,shape->Attribute("size"),logger);
	  }
	}
	else if (type_name == "cylinder")
	{
		geom.m_type = URDF_GEOM_CYLINDER;
		if (!shape->Attribute("length") ||
			!shape->Attribute("radius"))
	  {
		  logger->reportError("Cylinder shape must have both length and radius attributes");
		  return false;
	  }
		geom.m_cylinderRadius = urdfLexicalCast<double>(shape->Attribute("radius"));
		geom.m_cylinderLength = urdfLexicalCast<double>(shape->Attribute("length"));
		
	}
	
  else if (type_name == "mesh")
  {
	  geom.m_type = URDF_GEOM_MESH;
	  if (!shape->Attribute("filename")) {
		logger->reportError("Mesh must contain a filename attribute");
		return false;
	  }
	  
	  geom.m_meshFileName = shape->Attribute("filename");
	  
	  if (shape->Attribute("scale")) 
	  {
		  parseVector3(geom.m_meshScale,shape->Attribute("scale"),logger);
	  } else
	  {
		  geom.m_meshScale.setValue(1,1,1);
	  }

  }
  else
  {
	  logger->reportError("Unknown geometry type:");
	  logger->reportError(type_name.c_str());
	  return false;
  }
  
	return true;
}


bool UrdfParser::parseCollision(UrdfCollision& collision, TiXmlElement* config, ErrorLogger* logger)
{

	collision.m_linkLocalFrame.setIdentity();
	
	// Origin
	TiXmlElement *o = config->FirstChildElement("origin");
	if (o) 
	{
		if (!parseTransform(collision.m_linkLocalFrame, o,logger))
			return false;
	}
 // Geometry
	TiXmlElement *geom = config->FirstChildElement("geometry");
	if (!parseGeometry(collision.m_geometry,geom,logger))
	{
		return false;
	}
	
	
	const char *name_char = config->Attribute("name");
	if (name_char)
		collision.m_name = name_char;
	
	
	return true;
}

bool UrdfParser::parseVisual(UrdfVisual& visual, TiXmlElement* config, ErrorLogger* logger)
{
	visual.m_linkLocalFrame.setIdentity();
		
  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) 
  {
	  if (!parseTransform(visual.m_linkLocalFrame, o,logger))
		  return false;
  }
 // Geometry
	TiXmlElement *geom = config->FirstChildElement("geometry");
	if (!parseGeometry(visual.m_geometry,geom,logger))
	{
		return false;
	}
	
 		
  const char *name_char = config->Attribute("name");
  if (name_char)
	  visual.m_name = name_char;

	visual.m_hasLocalMaterial = false;
	
  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (mat) 
  {
	  // get material name
	  if (!mat->Attribute("name")) 
	  {
		  logger->reportError("Visual material must contain a name attribute");
		  return false;
	  }
	  visual.m_materialName = mat->Attribute("name");
	  
	  // try to parse material element in place
	  
	  TiXmlElement *t = mat->FirstChildElement("texture");
	  TiXmlElement *c = mat->FirstChildElement("color");
	  if (t||c)
	  {
		  if (parseMaterial(visual.m_localMaterial, mat,logger))
		  {
			  UrdfMaterial* matPtr = new UrdfMaterial(visual.m_localMaterial);
			  m_model.m_materials.insert(matPtr->m_name.c_str(),matPtr);
			  visual.m_hasLocalMaterial = true;
		  }
	  }
  }
  
  return true;
}

bool UrdfParser::parseLink(UrdfLink& link, TiXmlElement *config, ErrorLogger* logger)
{
	const char* linkName  = config->Attribute("name");
	if (!linkName)
	{
		logger->reportError("Link with no name");
		return false;
	}
	link.m_name = linkName;


  // Inertial (optional)
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
	  if (!parseInertia(link.m_inertia, i,logger))
	  {
		  logger->reportError("Could not parse inertial element for Link:");
		  logger->reportError(link.m_name.c_str());
		  return false;
	  }
  } else
  {
	  logger->reportWarning("No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frame");
	  link.m_inertia.m_mass = 1.f;
	  link.m_inertia.m_linkLocalFrame.setIdentity();
	  link.m_inertia.m_ixx = 1.f;
	  link.m_inertia.m_iyy = 1.f;
	  link.m_inertia.m_izz= 1.f;

	  logger->reportWarning(link.m_name.c_str());
  }
		
  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {
	  UrdfVisual visual;
	  
	  if (parseVisual(visual, vis_xml,logger))
	  {
		  link.m_visualArray.push_back(visual);
	  }
	  else
	  {
		  logger->reportError("Could not parse visual element for Link:");
		  logger->reportError(link.m_name.c_str());
		  return false;
	  }
	  
  }
		
  
  // Multiple Collisions (optional)
  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
  {
	  UrdfCollision col;
	  if (parseCollision(col, col_xml,logger))
	  {      
		  link.m_collisionArray.push_back(col);
	  }
	  else
	  {
		  logger->reportError("Could not parse collision element for Link:");
		  logger->reportError(link.m_name.c_str());
		  return false;
	  }
  }
	return true;
}

bool UrdfParser::parseJointLimits(UrdfJoint& joint, TiXmlElement* config, ErrorLogger* logger)
{
	joint.m_lowerLimit = 0.f;
	joint.m_upperLimit = 0.f;
	joint.m_effortLimit = 0.f;
	joint.m_velocityLimit = 0.f;
	
	const char* lower_str = config->Attribute("lower");
	if (lower_str)
	{
	joint.m_lowerLimit = urdfLexicalCast<double>(lower_str);
	}

	const char* upper_str = config->Attribute("upper");
	if (upper_str)
	{
	  joint.m_upperLimit = urdfLexicalCast<double>(upper_str);
	}
	
		
  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str)
  {
	  joint.m_effortLimit = urdfLexicalCast<double>(effort_str);
  }
		
  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str)
  {
	  joint.m_velocityLimit = urdfLexicalCast<double>(velocity_str);
  }
		
	return true;
}
bool UrdfParser::parseJoint(UrdfJoint& joint, TiXmlElement *config, ErrorLogger* logger)
{
	
		
	// Get Joint Name
	const char *name = config->Attribute("name");
	if (!name)
	{
	  logger->reportError("unnamed joint found");
	  return false;
	}
	joint.m_name = name;
	joint.m_parentLinkToJointTransform.setIdentity();
	
	// Get transform from Parent Link to Joint Frame
	TiXmlElement *origin_xml = config->FirstChildElement("origin");
	if (origin_xml)
	{
	  if (!parseTransform(joint.m_parentLinkToJointTransform, origin_xml,logger))
	  {
		  logger->reportError("Malformed parent origin element for joint:");
		  logger->reportError(joint.m_name.c_str());
		  return false;
	  }
  }
		
  // Get Parent Link
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
	  const char *pname = parent_xml->Attribute("link");
	  if (!pname)
	  {
		  logger->reportError("no parent link name specified for Joint link. this might be the root?");
		  logger->reportError(joint.m_name.c_str());
		  return false;
	  }
	  else
	  {
		  joint.m_parentLinkName = std::string(pname);
	  }
  }
		
  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
	  const char *pname = child_xml->Attribute("link");
	  if (!pname)
	  {
		  logger->reportError("no child link name specified for Joint link [%s].");
		  logger->reportError(joint.m_name.c_str());
		  return false;
	  }
	  else
	  {
		  joint.m_childLinkName = std::string(pname);
	  }
  }
		
  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
	  logger->reportError("joint [%s] has no type, check to see if it's a reference.");
	  logger->reportError(joint.m_name.c_str());
	  return false;
  }
  
  std::string type_str = type_char;
  if (type_str == "planar")
	  joint.m_type = URDFPlanarJoint;
  else if (type_str == "floating")
	  joint.m_type = URDFFloatingJoint;
  else if (type_str == "revolute")
	  joint.m_type = URDFRevoluteJoint;
  else if (type_str == "continuous")
	  joint.m_type = URDFContinuousJoint;
  else if (type_str == "prismatic")
	  joint.m_type = URDFPrismaticJoint;
  else if (type_str == "fixed")
	  joint.m_type = URDFFixedJoint;
  else
  {
	  logger->reportError("Joint ");
	  logger->reportError(joint.m_name.c_str());
	  logger->reportError("has unknown type:");
	  logger->reportError(type_str.c_str());
	  return false;
  }
		
  // Get Joint Axis
  if (joint.m_type != URDFFloatingJoint && joint.m_type != URDFFixedJoint)
  {
	  // axis
	  TiXmlElement *axis_xml = config->FirstChildElement("axis");
	  if (!axis_xml){
		  logger->reportWarning("urdfdom: no axis elemement for Joint, defaulting to (1,0,0) axis");
		  logger->reportWarning(joint.m_name.c_str());
		  joint.m_localJointAxis.setValue(1,0,0);
	  }
	  else{
		  if (axis_xml->Attribute("xyz"))
		  {
			  if (!parseVector3(joint.m_localJointAxis,axis_xml->Attribute("xyz"),logger))
			  {
				  logger->reportError("Malformed axis element:");
				  logger->reportError(joint.m_name.c_str());
				  logger->reportError(" for joint:");
				  logger->reportError(axis_xml->Attribute("xyz"));
				  return false;
			  }
		  }
	  }
  }
		
  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
	  if (!parseJointLimits(joint, limit_xml,logger))
	  {
		  logger->reportError("Could not parse limit element for joint:");
		  logger->reportError(joint.m_name.c_str());
		  return false;
	  }
  }
  else if (joint.m_type == URDFRevoluteJoint)
  {
	  logger->reportError("Joint is of type REVOLUTE but it does not specify limits");
	  logger->reportError(joint.m_name.c_str());
	  return false;
  }
  else if (joint.m_type == URDFPrismaticJoint)
  {
	  logger->reportError("Joint is of type PRISMATIC without limits");
	  logger->reportError( joint.m_name.c_str()); 
	  return false;
  }
	
	joint.m_jointDamping = 0;
	joint.m_jointFriction = 0;
	
	// Get Dynamics
	TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
	if (prop_xml)
	{
	
		// Get joint damping
		const char* damping_str = prop_xml->Attribute("damping");
		if (damping_str)
		{
			joint.m_jointDamping = urdfLexicalCast<double>(damping_str);
		}
			
		// Get joint friction
		const char* friction_str = prop_xml->Attribute("friction");
		if (friction_str)
		{
		  joint.m_jointFriction = urdfLexicalCast<double>(friction_str);
		}
			
		if (damping_str == NULL && friction_str == NULL)
		{
		  logger->reportError("joint dynamics element specified with no damping and no friction");
		  return false;
		}
	}
	
	return true;
}


bool UrdfParser::initTreeAndRoot(ErrorLogger* logger)
{
	// every link has children links and joints, but no parents, so we create a
	// local convenience data structure for keeping child->parent relations
	btHashMap<btHashString,btHashString> parentLinkTree;
	
	// loop through all joints, for every link, assign children links and children joints
	for (int i=0;i<m_model.m_joints.size();i++)
	{
		UrdfJoint** jointPtr = m_model.m_joints.getAtIndex(i);
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
			
			UrdfLink** childLinkPtr = m_model.m_links.find(joint->m_childLinkName.c_str());
			if (!childLinkPtr)
			{
				logger->reportError("Cannot find child link for joint ");
				logger->reportError(joint->m_name.c_str());

				return false;
			}
			UrdfLink* childLink = *childLinkPtr;
			
			UrdfLink** parentLinkPtr = m_model.m_links.find(joint->m_parentLinkName.c_str());
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
	for (int i=0;i<m_model.m_links.size();i++)
	{
		UrdfLink** linkPtr = m_model.m_links.getAtIndex(i);
		btAssert(linkPtr);
		if (linkPtr)
		{
			UrdfLink* link = *linkPtr;
			link->m_linkIndex = i;
			
			if (!link->m_parentLink)
			{
				m_model.m_rootLinks.push_back(link);
			}
		}
		
	}
	
	if (m_model.m_rootLinks.size()>1)
	{
		logger->reportWarning("URDF file with multiple root links found");
	}
	
	if (m_model.m_rootLinks.size()==0)
	{
		logger->reportError("URDF without root link found");
		return false;
	}
	return true;
	
}

bool UrdfParser::loadUrdf(const char* urdfText, ErrorLogger* logger, bool forceFixedBase)
{
	
	TiXmlDocument xml_doc;
	xml_doc.Parse(urdfText);
	if (xml_doc.Error())
	{
		logger->reportError(xml_doc.ErrorDesc());
		xml_doc.ClearError();
		return false;
	}

	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		logger->reportError("expected a robot element");
		return false;
	}
	
	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name)
	{
		logger->reportError("Expected a name for robot");
		return false;
	}
	m_model.m_name = name;
	
	
	
	// Get all Material elements
	for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
	{
		UrdfMaterial* material = new UrdfMaterial;
		
		parseMaterial(*material, material_xml, logger);


		UrdfMaterial** mat =m_model.m_materials.find(material->m_name.c_str());
		if (mat)
		{
			logger->reportWarning("Duplicate material");
		} else
		{
			m_model.m_materials.insert(material->m_name.c_str(),material);
		}
	}

	char msg[1024];
	sprintf(msg,"Num materials=%d", m_model.m_materials.size());
	logger->printMessage(msg);

	
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
	{
		UrdfLink* link = new UrdfLink;
		
		if (parseLink(*link, link_xml,logger))
		{
			if (m_model.m_links.find(link->m_name.c_str()))
			{
				logger->reportError("Link name is not unique, link names in the same model have to be unique");
				logger->reportError(link->m_name.c_str());
				return false;
			} else
			{
				//copy model material into link material, if link has no local material
				for (int i=0;i<link->m_visualArray.size();i++)
				{
					UrdfVisual& vis = link->m_visualArray.at(i);
					if (!vis.m_hasLocalMaterial && vis.m_materialName.c_str())
					{
						UrdfMaterial** mat = m_model.m_materials.find(vis.m_materialName.c_str());
						if (mat && *mat)
						{
							vis.m_localMaterial = **mat;
						} else
						{
							//logger->reportError("Cannot find material with name:");
							//logger->reportError(vis.m_materialName.c_str());
						}
					}
				}
				
				m_model.m_links.insert(link->m_name.c_str(),link);
			}
		} else
		{
			logger->reportError("failed to parse link");
			delete link;
			return false;
		}
		
	}
	if (m_model.m_links.size() == 0)
	{
		logger->reportWarning("No links found in URDF file");
		return false;
	}
	
	// Get all Joint elements
	for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
	{
		UrdfJoint* joint = new UrdfJoint;
		
		if (parseJoint(*joint, joint_xml,logger))
		{
			if (m_model.m_joints.find(joint->m_name.c_str()))
			{
				logger->reportError("joint '%s' is not unique.");
				logger->reportError(joint->m_name.c_str());
				return false;
			}
			else
			{
				m_model.m_joints.insert(joint->m_name.c_str(),joint);
			}
		}
		else
		{
			logger->reportError("joint xml is not initialized correctly");
			return false;
		}
	}

	bool ok(initTreeAndRoot(logger));
	if (!ok)
	{
		return false;
	}
	
	if (forceFixedBase)
	{
		for (int i=0;i<m_model.m_rootLinks.size();i++)
		{
			UrdfLink* link(m_model.m_rootLinks.at(i));
			link->m_inertia.m_mass = 0.0;
			link->m_inertia.m_ixx = 0.0;
			link->m_inertia.m_ixy = 0.0;
			link->m_inertia.m_ixz = 0.0;
			link->m_inertia.m_iyy = 0.0;
			link->m_inertia.m_iyz = 0.0;
			link->m_inertia.m_izz = 0.0;
		}
	}
	
	return true;
}
