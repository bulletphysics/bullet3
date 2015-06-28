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


#include <urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h>
//#include <fstream>
//#include <sstream>
#ifdef URDF_USE_BOOST
#include <boost/lexical_cast.hpp>
#else
#include <urdf/boost_replacement/lexical_cast.h>
#endif

#include <algorithm>
#include <tinyxml/tinyxml.h>
#ifdef URDF_USE_CONSOLE_BRIDGE
#include <console_bridge/console.h>
#else
#include "urdf/boost_replacement/printf_console.h"
#endif



namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseMaterial(Material &material, TiXmlElement *config, bool only_name_is_ok)
{
  bool has_rgb = false;
  bool has_filename = false;

  material.clear();

  if (!config->Attribute("name"))
  {
    logError("Material must contain a name attribute");
    return false;
  }
  
  material.name = config->Attribute("name");

  // texture
  TiXmlElement *t = config->FirstChildElement("texture");
	
  if (t)
  {
    if (t->Attribute("filename"))
    {
      material.texture_filename = t->Attribute("filename");
      has_filename = true;
    }
  }

  // color
  TiXmlElement *c = config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba")) {

      try {
        material.color.init(c->Attribute("rgba"));
        has_rgb = true;
      }
      catch (ParseError &e) {  
        material.color.clear();
        logError(std::string("Material [" + material.name + "] has malformed color rgba values: " + e.what()).c_str());
      }
    }
  }

  if (!has_rgb && !has_filename) {
    if (!only_name_is_ok) // no need for an error if only name is ok
    {
      if (!has_rgb) logError(std::string("Material ["+material.name+"] color has no rgba").c_str());
      if (!has_filename) logError(std::string("Material ["+material.name+"] not defined in file").c_str());
    }
    return false;
  }
  return true;
}


bool parseSphere(Sphere &s, TiXmlElement *c)
{
  s.clear();

  s.type = Geometry::SPHERE;
  if (!c->Attribute("radius"))
  {
    logError("Sphere shape must have a radius attribute");
    return false;
  }

  try
  {
    s.radius = boost::lexical_cast<double>(c->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
   // std::stringstream stm;
   // stm << "radius [" << c->Attribute("radius") << "] is not a valid float: " << e.what();
   // logError(stm.str().c_str());
      logError("radius issue");
    return false;
  }
  
  return true;
}

bool parseBox(Box &b, TiXmlElement *c)
{
  b.clear();
  
  b.type = Geometry::BOX;
  if (!c->Attribute("size"))
  {
    logError("Box shape has no size attribute");
    return false;
  }
  try
  {
    b.dim.init(c->Attribute("size"));
  }
  catch (ParseError &e)
  {
    b.dim.clear();
    logError(e.what());
    return false;
  }
  return true;
}

bool parseCylinder(Cylinder &y, TiXmlElement *c)
{
  y.clear();

  y.type = Geometry::CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    logError("Cylinder shape must have both length and radius attributes");
    return false;
  }

  try
  {
    y.length = boost::lexical_cast<double>(c->Attribute("length"));
  }
  catch (boost::bad_lexical_cast &e)
  {
  //  std::stringstream stm;
   // stm << "length [" << c->Attribute("length") << "] is not a valid float";
    //logError(stm.str().c_str());
      logError("length");
      return false;
  }

  try
  {
    y.radius = boost::lexical_cast<double>(c->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
 //   std::stringstream stm;
   // stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
    //logError(stm.str().c_str());
      logError("radius");
      return false;
  }
  return true;
}


bool parseMesh(Mesh &m, TiXmlElement *c)
{
  m.clear();

  m.type = Geometry::MESH;
  if (!c->Attribute("filename")) {
    logError("Mesh must contain a filename attribute");
    return false;
  }

  m.filename = c->Attribute("filename");

  if (c->Attribute("scale")) {
    try {
      m.scale.init(c->Attribute("scale"));
    }
    catch (ParseError &e) {
      m.scale.clear();
      logError("Mesh scale was specified, but could not be parsed: %s", e.what());
      return false;
    }
  }
  else
  {
    m.scale.x = m.scale.y = m.scale.z = 1;
  }
  return true;
}

my_shared_ptr<Geometry> parseGeometry(TiXmlElement *g)
{
  my_shared_ptr<Geometry> geom;
  if (!g) return geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    logError("Geometry tag contains no child element.");
    return geom;
  }

  const std::string type_name = shape->ValueTStr().c_str();
  if (type_name == "sphere")
  {
    Sphere *s = new Sphere();
    geom.reset(s);
    if (parseSphere(*s, shape))
      return geom;
  }
  else if (type_name == "box")
  {
    Box *b = new Box();
    geom.reset(b);
    if (parseBox(*b, shape))
      return geom;
  }
  else if (type_name == "cylinder")
  {
    Cylinder *c = new Cylinder();
    geom.reset(c);
    if (parseCylinder(*c, shape))
      return geom;
  }
  else if (type_name == "mesh")
  {
    Mesh *m = new Mesh();
    geom.reset(m);
    if (parseMesh(*m, shape))
      return geom;    
  }
  else
  {
    logError("Unknown geometry type '%s'", type_name.c_str());
    return geom;
  }
  
  return my_shared_ptr<Geometry>();
}

bool parseInertial(Inertial &i, TiXmlElement *config)
{
  i.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    if (!parsePose(i.origin, o))
      return false;
  }

  TiXmlElement *mass_xml = config->FirstChildElement("mass");
  if (!mass_xml)
  {
    logError("Inertial element must have a mass element");
    return false;
  }
  if (!mass_xml->Attribute("value"))
  {
    logError("Inertial: mass element must have value attribute");
    return false;
  }

  try
  {
    i.mass = boost::lexical_cast<double>(mass_xml->Attribute("value"));
  }
  catch (boost::bad_lexical_cast &e)
  {
  //  std::stringstream stm;
   // stm << "Inertial: mass [" << mass_xml->Attribute("value")
    //    << "] is not a float";
    //logError(stm.str().c_str());
      logError("Inertial mass issue");
      return false;
  }

  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    logError("Inertial element must have inertia element");
    return false;
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    logError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
    return false;
  }
  try
  {
    i.ixx  = boost::lexical_cast<double>(inertia_xml->Attribute("ixx"));
    i.ixy  = boost::lexical_cast<double>(inertia_xml->Attribute("ixy"));
    i.ixz  = boost::lexical_cast<double>(inertia_xml->Attribute("ixz"));
    i.iyy  = boost::lexical_cast<double>(inertia_xml->Attribute("iyy"));
    i.iyz  = boost::lexical_cast<double>(inertia_xml->Attribute("iyz"));
    i.izz  = boost::lexical_cast<double>(inertia_xml->Attribute("izz"));
  }
  catch (boost::bad_lexical_cast &e)
  {
   /* std::stringstream stm;
    stm << "Inertial: one of the inertia elements is not a valid double:"
        << " ixx [" << inertia_xml->Attribute("ixx") << "]"
        << " ixy [" << inertia_xml->Attribute("ixy") << "]"
        << " ixz [" << inertia_xml->Attribute("ixz") << "]"
        << " iyy [" << inertia_xml->Attribute("iyy") << "]"
        << " iyz [" << inertia_xml->Attribute("iyz") << "]"
        << " izz [" << inertia_xml->Attribute("izz") << "]";
    logError(stm.str().c_str());
    */
      logError("Inertia error");
      
    return false;
  }
  return true;
}

bool parseVisual(Visual &vis, TiXmlElement *config)
{
  vis.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    if (!parsePose(vis.origin, o))
      return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  vis.geometry = parseGeometry(geom);
  if (!vis.geometry)
    return false;

  const char *name_char = config->Attribute("name");
  if (name_char)
    vis.name = name_char;

  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (mat) {
    // get material name
    if (!mat->Attribute("name")) {
      logError("Visual material must contain a name attribute");
      return false;
    }
    vis.material_name = mat->Attribute("name");
    
    // try to parse material element in place
    vis.material.reset(new Material());
    if (!parseMaterial(*vis.material, mat, true))
    {
      logDebug("urdfdom: material has only name, actual material definition may be in the model");
    }
  }
  
  return true;
}

bool parseCollision(Collision &col, TiXmlElement* config)
{  
  col.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    if (!parsePose(col.origin, o))
      return false;
  }
  
  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  col.geometry = parseGeometry(geom);
  if (!col.geometry)
    return false;

  const char *name_char = config->Attribute("name");
  if (name_char)
    col.name = name_char;

  return true;
}

bool parseLink(Link &link, TiXmlElement* config)
{
  
  link.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    logError("No name given for the link.");
    return false;
  }
  link.name = std::string(name_char);

  // Inertial (optional)
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    link.inertial.reset(new Inertial());
    if (!parseInertial(*link.inertial, i))
    {
      logError("Could not parse inertial element for Link [%s]", link.name.c_str());
      return false;
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {

    my_shared_ptr<Visual> vis;
    vis.reset(new Visual());
    if (parseVisual(*vis, vis_xml))
    {
      link.visual_array.push_back(vis);
    }
    else
    {
      vis.reset(0);
      logError("Could not parse visual element for Link [%s]", link.name.c_str());
      return false;
    }
  }

  // Visual (optional)
  // Assign the first visual to the .visual ptr, if it exists
  if (!link.visual_array.empty())
    link.visual = link.visual_array[0];
  
  // Multiple Collisions (optional)
  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
  {
    my_shared_ptr<Collision> col;
    col.reset(new Collision());
    if (parseCollision(*col, col_xml))
    {      
      link.collision_array.push_back(col);
    }
    else
    {
      col.reset(0);
      logError("Could not parse collision element for Link [%s]",  link.name.c_str());
      return false;
    }
  }
  
  // Collision (optional)  
  // Assign the first collision to the .collision ptr, if it exists
  if (!link.collision_array.empty())
    link.collision = link.collision_array[0];
  return true;

}

}
