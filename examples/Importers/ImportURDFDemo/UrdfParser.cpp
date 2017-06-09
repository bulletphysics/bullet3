#include "UrdfParser.h"

#include "../../ThirdPartyLibs/tinyxml/tinyxml.h"
#include "urdfStringSplit.h"
#include "urdfLexicalCast.h"

UrdfParser::UrdfParser()
:m_parseSDF(false),
m_activeSdfModel(-1)
{
	m_urdf2Model.m_sourceFile = "IN_MEMORY_STRING"; // if loadUrdf() called later, source file name will be replaced with real
}

UrdfParser::~UrdfParser()
{
	for (int i=0;i<m_tmpModels.size();i++)
	{
		delete m_tmpModels[i];
	}
}

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

static bool parseVector3(btVector3& vec3, const std::string& vector_str, ErrorLogger* logger, bool lastThree = false)
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
	{
		TiXmlElement *c = config->FirstChildElement("color");
		if (c)
		{
		  if (c->Attribute("rgba")) 
		  {
			  if (!parseVector4(material.m_matColor.m_rgbaColor,c->Attribute("rgba")))
			  {
				  std::string msg = material.m_name+" has no rgba";
				  logger->reportWarning(msg.c_str());
			  }
		  }
		}
	}

	{
		// specular (non-standard)
		TiXmlElement *s = config->FirstChildElement("specular");
		if (s)
		{
		  if (s->Attribute("rgb")) 
		  {
			  if (!parseVector3(material.m_matColor.m_specularColor,s->Attribute("rgb"),logger))
			  {
			  }
		  }
		}
	}
	return true;

}

bool parseTransform(btTransform& tr, TiXmlElement* xml, ErrorLogger* logger, bool parseSDF = false)
{
    tr.setIdentity();
    
    if (parseSDF)
    {
        parseVector3(tr.getOrigin(),std::string(xml->GetText()),logger);
    }
    else
    {
        const char* xyz_str = xml->Attribute("xyz");
        if (xyz_str)
        {
            parseVector3(tr.getOrigin(),std::string(xyz_str),logger);
        }
    }
    
    if (parseSDF)
    {
        btVector3 rpy;
        if (parseVector3(rpy,std::string(xml->GetText()),logger,true))
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
    else
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
	if(m_parseSDF)
	{
		TiXmlElement* pose = config->FirstChildElement("pose");
		if (pose)
		{
			parseTransform(inertia.m_linkLocalFrame, pose,logger,m_parseSDF);
		}
	}

	
		
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
    if (m_parseSDF)
    {
        inertia.m_mass = urdfLexicalCast<double>(mass_xml->GetText());
    } else
    {
        if (!mass_xml->Attribute("value"))
        {
          logger->reportError("Inertial: mass element must have value attribute");
          return false;
        }

        inertia.m_mass = urdfLexicalCast<double>(mass_xml->Attribute("value"));
    }
		
	TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
	if (!inertia_xml)
	{
	  logger->reportError("Inertial element must have inertia element");
	  return false;
	}
    if (m_parseSDF)
    {
        TiXmlElement* ixx = inertia_xml->FirstChildElement("ixx");
        TiXmlElement* ixy = inertia_xml->FirstChildElement("ixy");
        TiXmlElement* ixz = inertia_xml->FirstChildElement("ixz");
        TiXmlElement* iyy = inertia_xml->FirstChildElement("iyy");
        TiXmlElement* iyz = inertia_xml->FirstChildElement("iyz");
        TiXmlElement* izz = inertia_xml->FirstChildElement("izz");
        if (ixx && ixy && ixz && iyy && iyz && izz)
        {
            inertia.m_ixx  = urdfLexicalCast<double>(ixx->GetText());
            inertia.m_ixy  = urdfLexicalCast<double>(ixy->GetText());
            inertia.m_ixz  = urdfLexicalCast<double>(ixz->GetText());
            inertia.m_iyy  = urdfLexicalCast<double>(iyy->GetText());
            inertia.m_iyz  = urdfLexicalCast<double>(iyz->GetText());
            inertia.m_izz  = urdfLexicalCast<double>(izz->GetText());
        } else
        {
			if (ixx && iyy && izz)
			{
				inertia.m_ixx  = urdfLexicalCast<double>(ixx->GetText());
				inertia.m_ixy  = 0;
				inertia.m_ixz  = 0;
				inertia.m_iyy  = urdfLexicalCast<double>(iyy->GetText());
				inertia.m_iyz  = 0;
				inertia.m_izz  = urdfLexicalCast<double>(izz->GetText());
			} else
			{
	            logger->reportError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz child elements");
		        return false;
			}
        }
    } else
    {
        if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
            inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
            inertia_xml->Attribute("izz")))
        {
			if ((inertia_xml->Attribute("ixx") && inertia_xml->Attribute("iyy") &&
            inertia_xml->Attribute("izz")))
			{
				inertia.m_ixx  = urdfLexicalCast<double>(inertia_xml->Attribute("ixx"));
				inertia.m_ixy  = 0;
				inertia.m_ixz  = 0;
				inertia.m_iyy  = urdfLexicalCast<double>(inertia_xml->Attribute("iyy"));
				inertia.m_iyz  = 0;
				inertia.m_izz  = urdfLexicalCast<double>(inertia_xml->Attribute("izz"));
			} else
			{
	          logger->reportError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
		      return false;
			}
        } else
		{
			inertia.m_ixx  = urdfLexicalCast<double>(inertia_xml->Attribute("ixx"));
			inertia.m_ixy  = urdfLexicalCast<double>(inertia_xml->Attribute("ixy"));
			inertia.m_ixz  = urdfLexicalCast<double>(inertia_xml->Attribute("ixz"));
			inertia.m_iyy  = urdfLexicalCast<double>(inertia_xml->Attribute("iyy"));
			inertia.m_iyz  = urdfLexicalCast<double>(inertia_xml->Attribute("iyz"));
			inertia.m_izz  = urdfLexicalCast<double>(inertia_xml->Attribute("izz"));
		}
    }
	return true;
    
}

bool UrdfParser::parseGeometry(UrdfGeometry& geom, TiXmlElement* g, ErrorLogger* logger)
{
//	btAssert(g);
	if (g==0)
		return false;

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
        if (m_parseSDF)
        {
            TiXmlElement* size = shape->FirstChildElement("size");
            if (0==size)
            {
                logger->reportError("box requires a size child element");
                return false;
            }
            parseVector3(geom.m_boxSize,size->GetText(),logger);
        }
        else
        {
              if (!shape->Attribute("size"))
              {
                  logger->reportError("box requires a size attribute");
                  return false;
              } else
              {
                  parseVector3(geom.m_boxSize,shape->Attribute("size"),logger);
              }
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
		geom.m_hasFromTo = false;
		geom.m_capsuleRadius = urdfLexicalCast<double>(shape->Attribute("radius"));
		geom.m_capsuleHeight = urdfLexicalCast<double>(shape->Attribute("length"));
		
	}
	else if (type_name == "capsule")
	{
		geom.m_type = URDF_GEOM_CAPSULE;
		if (!shape->Attribute("length") ||
			!shape->Attribute("radius"))
		{
			logger->reportError("Capsule shape must have both length and radius attributes");
			return false;
		}
		geom.m_hasFromTo = false;
		geom.m_capsuleRadius = urdfLexicalCast<double>(shape->Attribute("radius"));
		geom.m_capsuleHeight = urdfLexicalCast<double>(shape->Attribute("length"));
	}
	else if (type_name == "mesh")
	{
		geom.m_type = URDF_GEOM_MESH;
		geom.m_meshScale.setValue(1,1,1);
		std::string fn;

		if (m_parseSDF)
		{
			if (TiXmlElement* scale = shape->FirstChildElement("scale"))
			{
				parseVector3(geom.m_meshScale,scale->GetText(),logger);
			}
			if (TiXmlElement* filename = shape->FirstChildElement("uri"))
			{
				fn = filename->GetText();
			}
		}
		else
		{
			// URDF
			if (shape->Attribute("filename"))
			{
				fn = shape->Attribute("filename");
			}
			if (shape->Attribute("scale"))
			{
				if (!parseVector3(geom.m_meshScale, shape->Attribute("scale"), logger))
				{
					logger->reportWarning("Scale should be a vector3, not single scalar. Workaround activated.\n");
					std::string scalar_str = shape->Attribute("scale");
					double scaleFactor = urdfLexicalCast<double>(scalar_str.c_str());
					if (scaleFactor)
					{
						geom.m_meshScale.setValue(scaleFactor, scaleFactor, scaleFactor);
					}
				}
			}
		}

		if (fn.empty())
		{
			logger->reportError("Mesh filename is empty");
			return false;
		}

		geom.m_meshFileName = fn;
		bool success = findExistingMeshFile(
			m_urdf2Model.m_sourceFile, fn, sourceFileLocation(shape),
			&geom.m_meshFileName, &geom.m_meshFileType);
		if (!success)
		{
			// warning already printed
			return false;
		}
  }
  else
  {
      if (this->m_parseSDF)
      {
          if (type_name == "plane")
          {
              geom.m_type = URDF_GEOM_PLANE;
             
              TiXmlElement *n = shape->FirstChildElement("normal");
              TiXmlElement *s = shape->FirstChildElement("size");

              if ((0==n)||(0==s))
              {
                  logger->reportError("Plane shape must have both normal and size attributes");
                  return false;
              }
            
              parseVector3(geom.m_planeNormal,n->GetText(),logger);
          }
      } else
      {
          logger->reportError("Unknown geometry type:");
          logger->reportError(type_name.c_str());
          return false;
      }
  }
  
	return true;
}


bool UrdfParser::parseCollision(UrdfCollision& collision, TiXmlElement* config, ErrorLogger* logger)
{

	collision.m_linkLocalFrame.setIdentity();
	
	if(m_parseSDF)
	{
		TiXmlElement* pose = config->FirstChildElement("pose");
		if (pose)
		{
			parseTransform(collision.m_linkLocalFrame, pose,logger,m_parseSDF);
		}
	}

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
	
	const char *concave_char = config->Attribute("concave");
	if (concave_char)
		collision.m_flags |= URDF_FORCE_CONCAVE_TRIMESH;
	
	return true;
}

bool UrdfParser::parseVisual(UrdfModel& model, UrdfVisual& visual, TiXmlElement* config, ErrorLogger* logger)
{
	visual.m_linkLocalFrame.setIdentity();
	if(m_parseSDF)
	{
		TiXmlElement* pose = config->FirstChildElement("pose");
		if (pose)
		{
			parseTransform(visual.m_linkLocalFrame, pose,logger,m_parseSDF);
		}
	}

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

	visual.m_geometry.m_hasLocalMaterial = false;
	
  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
//todo(erwincoumans) skip materials in SDF for now (due to complexity)
  if (mat)
  {
    if (m_parseSDF)
    {
        UrdfMaterial* matPtr = new UrdfMaterial;
        matPtr->m_name = "mat";
		if (name_char)
			matPtr->m_name = name_char;
		model.m_materials.insert(matPtr->m_name.c_str(),matPtr);
		{
			TiXmlElement *diffuse = mat->FirstChildElement("diffuse");
			if (diffuse) {
				std::string diffuseText = diffuse->GetText();
				btVector4 rgba(1,0,0,1);
				parseVector4(rgba,diffuseText);
				matPtr->m_matColor.m_rgbaColor = rgba;
            
				visual.m_materialName = matPtr->m_name;
				visual.m_geometry.m_hasLocalMaterial = true;
			}
		}
		{
			TiXmlElement *specular = mat->FirstChildElement("specular");
			if (specular) {
				std::string specularText = specular->GetText();
				btVector3 rgba(1,1,1);
				parseVector3(rgba,specularText,logger);
				matPtr->m_matColor.m_specularColor = rgba;
				visual.m_materialName = matPtr->m_name;
				visual.m_geometry.m_hasLocalMaterial = true;
			}
		}
    } 
    else
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
		  TiXmlElement *s = mat->FirstChildElement("specular");
		  if (t||c||s)
          {
              if (parseMaterial(visual.m_geometry.m_localMaterial, mat,logger))
              {
                  UrdfMaterial* matPtr = new UrdfMaterial(visual.m_geometry.m_localMaterial);
				  
				  UrdfMaterial** oldMatPtrPtr = model.m_materials[matPtr->m_name.c_str()];
				  if (oldMatPtrPtr)
				  {
					  UrdfMaterial* oldMatPtr = *oldMatPtrPtr;
					  model.m_materials.remove(matPtr->m_name.c_str());
					  if (oldMatPtr)
						  delete oldMatPtr;
				  }
                  model.m_materials.insert(matPtr->m_name.c_str(),matPtr);
                  visual.m_geometry.m_hasLocalMaterial = true;
              }
          }
      }
  }
  
  return true;
}

bool UrdfParser::parseLink(UrdfModel& model, UrdfLink& link, TiXmlElement *config, ErrorLogger* logger)
{
	const char* linkName  = config->Attribute("name");
	if (!linkName)
	{
		logger->reportError("Link with no name");
		return false;
	}
	link.m_name = linkName;
    
    if (m_parseSDF) {


        TiXmlElement* pose = config->FirstChildElement("pose");
        if (0==pose)
        {
            link.m_linkTransformInWorld.setIdentity();
        }
        else
        {
            parseTransform(link.m_linkTransformInWorld, pose,logger,m_parseSDF);
        }
    }

	{
		//optional 'audio_source' parameters
		//modified version of SDF audio_source specification in //http://sdformat.org/spec?ver=1.6&elem=link
		#if 0
		<audio_source>
          <uri>file://media/audio/cheer.mp3</uri>
          <pitch>2.0</pitch>
          <gain>1.0</gain>
          <loop>false</loop>
          <contact>
            <collision>collision</collision>
          </contact>
        </audio_source>
		#endif
		TiXmlElement* ci = config->FirstChildElement("audio_source");
		if (ci)
		{
			link.m_audioSource.m_flags |= SDFAudioSource::SDFAudioSourceValid;
			
			const char* fn = ci->Attribute("filename");
			if (fn)
			{
				link.m_audioSource.m_uri = fn;
			} else
			{
				if (TiXmlElement* filename_xml = ci->FirstChildElement("uri"))
				{
					link.m_audioSource.m_uri = filename_xml->GetText();
				}
			}
			if (TiXmlElement* pitch_xml = ci->FirstChildElement("pitch"))
			{
				link.m_audioSource.m_pitch = urdfLexicalCast<double>(pitch_xml->GetText());
			}
			if (TiXmlElement* gain_xml = ci->FirstChildElement("gain"))
			{
				link.m_audioSource.m_gain = urdfLexicalCast<double>(gain_xml->GetText());
			}

			if (TiXmlElement* attack_rate_xml = ci->FirstChildElement("attack_rate"))
			{
				link.m_audioSource.m_attackRate = urdfLexicalCast<double>(attack_rate_xml->GetText());
			}

			if (TiXmlElement* decay_rate_xml = ci->FirstChildElement("decay_rate"))
			{
				link.m_audioSource.m_decayRate = urdfLexicalCast<double>(decay_rate_xml->GetText());
			}

			if (TiXmlElement* sustain_level_xml = ci->FirstChildElement("sustain_level"))
			{
				link.m_audioSource.m_sustainLevel = urdfLexicalCast<double>(sustain_level_xml->GetText());
			}

			if (TiXmlElement* release_rate_xml = ci->FirstChildElement("release_rate"))
			{
				link.m_audioSource.m_releaseRate = urdfLexicalCast<double>(release_rate_xml->GetText());
			}

			if (TiXmlElement* loop_xml = ci->FirstChildElement("loop"))
			{
				std::string looptxt = loop_xml->GetText();
				if (looptxt == "true")
				{
					link.m_audioSource.m_flags |= SDFAudioSource::SDFAudioSourceLooping;
				}
			}
			if (TiXmlElement* forceThreshold_xml = ci->FirstChildElement("collision_force_threshold"))
			{
				link.m_audioSource.m_collisionForceThreshold= urdfLexicalCast<double>(forceThreshold_xml->GetText());
			}
			//todo: see other audio_source children
			//pitch, gain, loop, contact
			
		}
	}
	{
		//optional 'contact' parameters
	 TiXmlElement* ci = config->FirstChildElement("contact");
	  if (ci)
	  {
        
          TiXmlElement *damping_xml = ci->FirstChildElement("inertia_scaling");
          if (damping_xml)
          {
              if (m_parseSDF)
              {
                  link.m_contactInfo.m_inertiaScaling = urdfLexicalCast<double>(damping_xml->GetText());
                  link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_INERTIA_SCALING;
              } else
              {
                  if (!damping_xml->Attribute("value"))
                  {
                      logger->reportError("Link/contact: damping element must have value attribute");
                      return false;
                  }
                  
                  link.m_contactInfo.m_inertiaScaling = urdfLexicalCast<double>(damping_xml->Attribute("value"));
                  link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_INERTIA_SCALING;

              }
          }
          {
		TiXmlElement *friction_xml = ci->FirstChildElement("lateral_friction");
		if (friction_xml)
		{
			if (m_parseSDF)
			{
				link.m_contactInfo.m_lateralFriction = urdfLexicalCast<double>(friction_xml->GetText());
			} else
			{
				if (!friction_xml->Attribute("value"))
				{
				  logger->reportError("Link/contact: lateral_friction element must have value attribute");
				  return false;
				}

				link.m_contactInfo.m_lateralFriction = urdfLexicalCast<double>(friction_xml->Attribute("value"));
			}
		}
          }
          
          {
              TiXmlElement *rolling_xml = ci->FirstChildElement("rolling_friction");
              if (rolling_xml)
              {
                  if (m_parseSDF)
                  {
                      link.m_contactInfo.m_rollingFriction = urdfLexicalCast<double>(rolling_xml->GetText());
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_ROLLING_FRICTION;
                  } else
                  {
                      if (!rolling_xml->Attribute("value"))
                      {
                          logger->reportError("Link/contact: rolling friction element must have value attribute");
                          return false;
                      }
                      
                      link.m_contactInfo.m_rollingFriction = urdfLexicalCast<double>(rolling_xml->Attribute("value"));
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_ROLLING_FRICTION;
                      
                  }
              }
          }

		  {
			  TiXmlElement *restitution_xml = ci->FirstChildElement("restitution");
			  if (restitution_xml)
			  {
				  if (m_parseSDF)
				  {
					  link.m_contactInfo.m_restitution = urdfLexicalCast<double>(restitution_xml->GetText());
					  link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_RESTITUTION;
				  }
				  else
				  {
					  if (!restitution_xml->Attribute("value"))
					  {
						  logger->reportError("Link/contact: restitution element must have value attribute");
						  return false;
					  }

					  link.m_contactInfo.m_restitution = urdfLexicalCast<double>(restitution_xml->Attribute("value"));
					  link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_RESTITUTION;

				  }
			  }
		  }

          {
              TiXmlElement *spinning_xml = ci->FirstChildElement("spinning_friction");
              if (spinning_xml)
              {
                  if (m_parseSDF)
                  {
                      link.m_contactInfo.m_spinningFriction = urdfLexicalCast<double>(spinning_xml->GetText());
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_SPINNING_FRICTION;
                  } else
                  {
                      if (!spinning_xml->Attribute("value"))
                      {
                          logger->reportError("Link/contact: spinning friction element must have value attribute");
                          return false;
                      }
                      
                      link.m_contactInfo.m_spinningFriction = urdfLexicalCast<double>(spinning_xml->Attribute("value"));
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_SPINNING_FRICTION;
                      
                  }
              }
          }
          {
          	TiXmlElement *friction_anchor = ci->FirstChildElement("friction_anchor");
            if (friction_anchor)
            {
            	link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_FRICTION_ANCHOR;
            }
          }
           {
           
              TiXmlElement *stiffness_xml = ci->FirstChildElement("stiffness");
              if (stiffness_xml)
              {
                  if (m_parseSDF)
                  {
                      link.m_contactInfo.m_contactStiffness = urdfLexicalCast<double>(stiffness_xml->GetText());
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
                  } else
                  {
                      if (!stiffness_xml->Attribute("value"))
                      {
                          logger->reportError("Link/contact: stiffness element must have value attribute");
                          return false;
                      }
                      
                      link.m_contactInfo.m_contactStiffness = urdfLexicalCast<double>(stiffness_xml->Attribute("value"));
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
                      
                  }
              }
            }
            {
           
              TiXmlElement *damping_xml = ci->FirstChildElement("damping");
              if (damping_xml)
              {
                  if (m_parseSDF)
                  {
                      link.m_contactInfo.m_contactDamping = urdfLexicalCast<double>(damping_xml->GetText());
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
                  } else
                  {
                      if (!damping_xml->Attribute("value"))
                      {
                          logger->reportError("Link/contact: damping element must have value attribute");
                          return false;
                      }
                      
                      link.m_contactInfo.m_contactDamping = urdfLexicalCast<double>(damping_xml->Attribute("value"));
                      link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
                      
                  }
              }
            }
	  }
	}

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
      
      if ((strlen(linkName)==5) && (strncmp(linkName, "world", 5))==0)
      {
          link.m_inertia.m_mass = 0.f;
          link.m_inertia.m_linkLocalFrame.setIdentity();
          link.m_inertia.m_ixx = 0.f;
          link.m_inertia.m_iyy = 0.f;
          link.m_inertia.m_izz= 0.f;
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
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {
	  UrdfVisual visual;
	  visual.m_sourceFileLocation = sourceFileLocation(vis_xml);

	  if (parseVisual(model, visual, vis_xml,logger))
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
	  col.m_sourceFileLocation = sourceFileLocation(col_xml);

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
	joint.m_upperLimit = -1.f;
	joint.m_effortLimit = 0.f;
	joint.m_velocityLimit = 0.f;
	joint.m_jointDamping = 0.f;
	joint.m_jointFriction = 0.f;
	
    if (m_parseSDF)
    {
        TiXmlElement *lower_xml = config->FirstChildElement("lower");
        if (lower_xml) {
            joint.m_lowerLimit = urdfLexicalCast<double>(lower_xml->GetText());
        }
        
        TiXmlElement *upper_xml = config->FirstChildElement("upper");
        if (upper_xml) {
            joint.m_upperLimit = urdfLexicalCast<double>(upper_xml->GetText());
        }
        
        TiXmlElement *effort_xml = config->FirstChildElement("effort");
        if (effort_xml) {
            joint.m_effortLimit = urdfLexicalCast<double>(effort_xml->GetText());
        }
        
        TiXmlElement *velocity_xml = config->FirstChildElement("velocity");
        if (velocity_xml) {
            joint.m_velocityLimit = urdfLexicalCast<double>(velocity_xml->GetText());
        }
    }
    else
    {
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
    }
		
	return true;
}

bool UrdfParser::parseJointDynamics(UrdfJoint& joint, TiXmlElement* config, ErrorLogger* logger)
{
    joint.m_jointDamping = 0;
    joint.m_jointFriction = 0;
    
    if (m_parseSDF) {
        TiXmlElement *damping_xml = config->FirstChildElement("damping");
        if (damping_xml) {
            joint.m_jointDamping = urdfLexicalCast<double>(damping_xml->GetText());
        }
        
        TiXmlElement *friction_xml = config->FirstChildElement("friction");
        if (friction_xml) {
            joint.m_jointFriction = urdfLexicalCast<double>(friction_xml->GetText());
        }
        
        if (damping_xml == NULL && friction_xml == NULL)
        {
            logger->reportError("joint dynamics element specified with no damping and no friction");
            return false;
        }
    }
    else
    {
        // Get joint damping
        const char* damping_str = config->Attribute("damping");
        if (damping_str)
        {
            joint.m_jointDamping = urdfLexicalCast<double>(damping_str);
        }
        
        // Get joint friction
        const char* friction_str = config->Attribute("friction");
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
      if (m_parseSDF)
      {
          joint.m_parentLinkName = std::string(parent_xml->GetText());
      }
      else
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
  }
 		
  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
      if (m_parseSDF)
      {
          joint.m_childLinkName = std::string(child_xml->GetText());
      }
      else
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
		
    if (m_parseSDF)
    {
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
                TiXmlElement *xyz_xml = axis_xml->FirstChildElement("xyz");
                if (xyz_xml) {
                    if (!parseVector3(joint.m_localJointAxis,std::string(xyz_xml->GetText()),logger))
                    {
                        logger->reportError("Malformed axis element:");
                        logger->reportError(joint.m_name.c_str());
                        logger->reportError(" for joint:");
                        logger->reportError(xyz_xml->GetText());
                        return false;
                    }
                }
                
                TiXmlElement *limit_xml = axis_xml->FirstChildElement("limit");
                if (limit_xml)
                {
					if (joint.m_type != URDFContinuousJoint)
					{
						if (!parseJointLimits(joint, limit_xml,logger))
						{
							logger->reportError("Could not parse limit element for joint:");
							logger->reportError(joint.m_name.c_str());
							return false;
						}
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
                
                TiXmlElement *prop_xml = axis_xml->FirstChildElement("dynamics");
                if (prop_xml)
                {
                    if (!parseJointDynamics(joint, prop_xml,logger))
                    {
                        logger->reportError("Could not parse dynamics element for joint:");
                        logger->reportError(joint.m_name.c_str());
                        return false;
                    }
                }
            }
        }
    }
    else
    {
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
    }
	
	return true;
}


bool UrdfParser::initTreeAndRoot(UrdfModel& model, ErrorLogger* logger)
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
	m_urdf2Model.m_name = name;
	
	
	
	// Get all Material elements
	for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
	{
		UrdfMaterial* material = new UrdfMaterial;
		
		parseMaterial(*material, material_xml, logger);


		UrdfMaterial** mat =m_urdf2Model.m_materials.find(material->m_name.c_str());
		if (mat)
		{
			delete material;
			logger->reportWarning("Duplicate material");
		} else
		{
			m_urdf2Model.m_materials.insert(material->m_name.c_str(),material);
		}
	}


//	char msg[1024];
//	sprintf(msg,"Num materials=%d", m_model.m_materials.size());
//	logger->printMessage(msg);

	
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
	{
		UrdfLink* link = new UrdfLink;
		
		if (parseLink(m_urdf2Model,*link, link_xml,logger))
		{
			if (m_urdf2Model.m_links.find(link->m_name.c_str()))
			{
				logger->reportError("Link name is not unique, link names in the same model have to be unique");
				logger->reportError(link->m_name.c_str());
				delete link;
				return false;
			} else
			{
				//copy model material into link material, if link has no local material
				for (int i=0;i<link->m_visualArray.size();i++)
				{
					UrdfVisual& vis = link->m_visualArray.at(i);
					if (!vis.m_geometry.m_hasLocalMaterial && vis.m_materialName.c_str())
					{
						UrdfMaterial** mat = m_urdf2Model.m_materials.find(vis.m_materialName.c_str());
						if (mat && *mat)
						{
							vis.m_geometry.m_localMaterial = **mat;
						} else
						{
							//logger->reportError("Cannot find material with name:");
							//logger->reportError(vis.m_materialName.c_str());
						}
					}
				}
				
				m_urdf2Model.m_links.insert(link->m_name.c_str(),link);
			}
		} else
		{
			logger->reportError("failed to parse link");
			delete link;
			return false;
		}
		
	}
	if (m_urdf2Model.m_links.size() == 0)
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
			if (m_urdf2Model.m_joints.find(joint->m_name.c_str()))
			{
				logger->reportError("joint '%s' is not unique.");
				logger->reportError(joint->m_name.c_str());
				delete joint;
				return false;
			}
			else
			{
				m_urdf2Model.m_joints.insert(joint->m_name.c_str(),joint);
			}
		}
		else
		{
			logger->reportError("joint xml is not initialized correctly");
			delete joint;
			return false;
		}
	}

	bool ok(initTreeAndRoot(m_urdf2Model,logger));
	if (!ok)
	{
		return false;
	}
	
	if (forceFixedBase)
	{
		for (int i=0;i<m_urdf2Model.m_rootLinks.size();i++)
		{
			UrdfLink* link(m_urdf2Model.m_rootLinks.at(i));
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

void UrdfParser::activateModel(int modelIndex)
{
    m_activeSdfModel = modelIndex;
}


bool UrdfParser::loadSDF(const char* sdfText, ErrorLogger* logger)
{
    
    TiXmlDocument xml_doc;
    xml_doc.Parse(sdfText);
    if (xml_doc.Error())
    {
        logger->reportError(xml_doc.ErrorDesc());
        xml_doc.ClearError();
        return false;
    }

    TiXmlElement *sdf_xml = xml_doc.FirstChildElement("sdf");
    if (!sdf_xml)
    {
        logger->reportError("expected an sdf element");
        return false;
    }
    
	//apparently, SDF doesn't require a "world" element, optional? URDF does.
	TiXmlElement *world_xml = sdf_xml->FirstChildElement("world");
	
	TiXmlElement* robot_xml = 0;

	if (!world_xml)
	{
		logger->reportWarning("expected a world element, continuing without it.");
		robot_xml = sdf_xml->FirstChildElement("model");
	} else
	{
		robot_xml = world_xml->FirstChildElement("model");
	}

    // Get all model (robot) elements
    for (; robot_xml; robot_xml = robot_xml->NextSiblingElement("model"))
    {
        UrdfModel* localModel = new UrdfModel;
        m_tmpModels.push_back(localModel);
        
		TiXmlElement* stat = robot_xml->FirstChildElement("static");
        if (0!=stat)
        {
			int val = int(atof(stat->GetText()));
			if (val==1)
			{
				localModel->m_overrideFixedBase = true;
			}
		}

        
        // Get robot name
        const char *name = robot_xml->Attribute("name");
        if (!name)
        {
            logger->reportError("Expected a name for robot");
            return false;
        }
        localModel->m_name = name;
        
        TiXmlElement* pose_xml = robot_xml->FirstChildElement("pose");
        if (0==pose_xml)
        {
            localModel->m_rootTransformInWorld.setIdentity();
        }
        else
        {
            parseTransform(localModel->m_rootTransformInWorld,pose_xml,logger,m_parseSDF);
        }

        // Get all Material elements
        for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
        {
            UrdfMaterial* material = new UrdfMaterial;
            
            parseMaterial(*material, material_xml, logger);
            
            
            UrdfMaterial** mat =localModel->m_materials.find(material->m_name.c_str());
            if (mat)
            {
		logger->reportWarning("Duplicate material");
		delete material;
            } else
            {
                localModel->m_materials.insert(material->m_name.c_str(),material);
            }
        }
        
        
        //	char msg[1024];
        //	sprintf(msg,"Num materials=%d", m_model.m_materials.size());
        //	logger->printMessage(msg);
        
        
        for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
        {
            UrdfLink* link = new UrdfLink;
            
            if (parseLink(*localModel, *link, link_xml,logger))
            {
                if (localModel->m_links.find(link->m_name.c_str()))
                {
                    logger->reportError("Link name is not unique, link names in the same model have to be unique");
                    logger->reportError(link->m_name.c_str());
                    delete link;
                    return false;
                } else
                {
                    //copy model material into link material, if link has no local material
                    for (int i=0;i<link->m_visualArray.size();i++)
                    {
                        UrdfVisual& vis = link->m_visualArray.at(i);
                        if (!vis.m_geometry.m_hasLocalMaterial && vis.m_materialName.c_str())
                        {
                            UrdfMaterial** mat = localModel->m_materials.find(vis.m_materialName.c_str());
                            if (mat && *mat)
                            {
                                vis.m_geometry.m_localMaterial = **mat;
                            } else
                            {
                                //logger->reportError("Cannot find material with name:");
                                //logger->reportError(vis.m_materialName.c_str());
                            }
                        }
                    }
                    
                    localModel->m_links.insert(link->m_name.c_str(),link);
                }
            } else
            {
                logger->reportError("failed to parse link");
                delete link;
                return false;
            }
            
        }
        if (localModel->m_links.size() == 0)
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
                if (localModel->m_joints.find(joint->m_name.c_str()))
                {
                    logger->reportError("joint '%s' is not unique.");
                    logger->reportError(joint->m_name.c_str());
                    delete joint;
                    return false;
                }
                else
                {
                    localModel->m_joints.insert(joint->m_name.c_str(),joint);
                }
            }
            else
            {
                logger->reportError("joint xml is not initialized correctly");
                delete joint;
                return false;
            }
        }
        
        bool ok(initTreeAndRoot(*localModel,logger));
        if (!ok)
        {
            return false;
        }
        m_sdfModels.push_back(localModel);
    }
    
    return true;
}

std::string UrdfParser::sourceFileLocation(TiXmlElement* e)
{
#if 0
	//no C++11 etc, no snprintf

	char buf[1024];
	snprintf(buf, sizeof(buf), "%s:%i", m_urdf2Model.m_sourceFile.c_str(), e->Row());
	return buf;
#else
	char row[1024];
        sprintf(row,"%d",e->Row());
        std::string str = m_urdf2Model.m_sourceFile.c_str() + std::string(":") + std::string(row);
        return str;
#endif

}
