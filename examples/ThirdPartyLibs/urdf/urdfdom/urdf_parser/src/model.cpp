/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */
//#include <boost/algorithm/string.hpp>
#include <vector>
#include "urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h"
#ifdef URDF_USE_CONSOLE_BRIDGE
	#include <console_bridge/console.h>
#else
	#include "urdf/boost_replacement/printf_console.h"
#endif

namespace urdf{

bool parseMaterial(Material &material, TiXmlElement *config, bool only_name_is_ok);
bool parseLink(Link &link, TiXmlElement *config);
bool parseJoint(Joint &joint, TiXmlElement *config);


my_shared_ptr<ModelInterface>  parseURDF(const std::string &xml_string)
{
  my_shared_ptr<ModelInterface> model(new ModelInterface);
  model->clear();

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  if (xml_doc.Error())
  {
    logError(xml_doc.ErrorDesc());
    xml_doc.ClearError();
    model.reset(0);
    return model;
  }

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    logError("Could not find the 'robot' element in the xml file");
    model.reset(0);
    return model;
  }

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    logError("No name given for the robot.");
    model.reset(0);
    return model;
  }
  model->name_ = std::string(name);

  // Get all Material elements
  for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
  {
    my_shared_ptr<Material> material;
    material.reset(new Material);

    try {
      parseMaterial(*material, material_xml, false); // material needs to be fully defined here
      if (model->getMaterial(material->name))
      {
        logError("material '%s' is not unique.", material->name.c_str());
        material.reset(0);
        model.reset(0);
        return model;
      }
      else
      {
        model->materials_.insert(make_pair(material->name,material));
        logDebug("urdfdom: successfully added a new material '%s'", material->name.c_str());
      }
    }
    catch (ParseError &e) {
      logError("material xml is not initialized correctly");
      material.reset(0);
      model.reset(0);
      return model;
    }
  }

  // Get all Link elements
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    my_shared_ptr<Link> link;
    link.reset(new Link);
	model->m_numLinks++;

    try {
      parseLink(*link, link_xml);
      if (model->getLink(link->name))
      {
        logError("link '%s' is not unique.", link->name.c_str());
        model.reset(0);
        return model;
      }
      else
      {
        // set link visual material
        logDebug("urdfdom: setting link '%s' material", link->name.c_str());
        if (link->visual)
        {
          if (!link->visual->material_name.empty())
          {
            if (model->getMaterial(link->visual->material_name))
            {
              logDebug("urdfdom: setting link '%s' material to '%s'", link->name.c_str(),link->visual->material_name.c_str());
              link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
            }
            else
            {
              if (link->visual->material)
              {
                logDebug("urdfdom: link '%s' material '%s' defined in Visual.", link->name.c_str(),link->visual->material_name.c_str());
                model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
              }
              else
              {
                logError("link '%s' material '%s' undefined.", link->name.c_str(),link->visual->material_name.c_str());
                model.reset(0);
                return model;
              }
            }
          }
        }

        model->links_.insert(make_pair(link->name,link));
        logDebug("urdfdom: successfully added a new link '%s'", link->name.c_str());
      }
    }
    catch (ParseError &e) {
      logError("link xml is not initialized correctly");
      model.reset(0);
      return model;
    }
  }
  if (model->links_.empty()){
    logError("No link elements found in urdf file");
    model.reset(0);
    return model;
  }

  // Get all Joint elements
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    my_shared_ptr<Joint> joint;
    joint.reset(new Joint);
	model->m_numJoints++;

    if (parseJoint(*joint, joint_xml))
    {
      if (model->getJoint(joint->name))
      {
        logError("joint '%s' is not unique.", joint->name.c_str());
        model.reset(0);
        return model;
      }
      else
      {
        model->joints_.insert(make_pair(joint->name,joint));
        logDebug("urdfdom: successfully added a new joint '%s'", joint->name.c_str());
      }
    }
    else
    {
      logError("joint xml is not initialized correctly");
      model.reset(0);
      return model;
    }
  }


  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  try 
  {
    model->initTree(parent_link_tree);
  }
  catch(ParseError &e)
  {
    logError("Failed to build tree: %s", e.what());
    model.reset(0);
    return model;
  }

  // find the root link
  try
  {
    model->initRoot(parent_link_tree);
  }
  catch(ParseError &e)
  {
    logError("Failed to find root link: %s", e.what());
    model.reset(0);
    return model;
  }
  
  return model;
}



}

