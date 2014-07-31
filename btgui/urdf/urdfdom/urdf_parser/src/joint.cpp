/*********************************************************************
* Software Ligcense Agreement (BSD License)
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

/* Author: John Hsu */

#include <sstream>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h>
#ifdef URDF_USE_BOOST
#include <boost/lexical_cast.hpp>
#else
#include <urdf/boost_replacement/lexical_cast.h>
#endif
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h>

#ifdef URDF_USE_CONSOLE_BRIDGE
	#include <console_bridge/console.h>
#else
	#include "urdf/boost_replacement/printf_console.h"
#endif

#include <tinyxml/tinyxml.h>
#include <urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h>

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseJointDynamics(JointDynamics &jd, TiXmlElement* config)
{
  jd.clear();

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str == NULL){
    logDebug("urdfdom.joint_dynamics: no damping, defaults to 0");
    jd.damping = 0;
  }
  else
  {
    try
    {
      jd.damping = boost::lexical_cast<double>(damping_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("damping value (%s) is not a float: %s",damping_str, e.what());
      return false;
    }
  }

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str == NULL){
    logDebug("urdfdom.joint_dynamics: no friction, defaults to 0");
    jd.friction = 0;
  }
  else
  {
    try
    {
      jd.friction = boost::lexical_cast<double>(friction_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("friction value (%s) is not a float: %s",friction_str, e.what());
      return false;
    }
  }

  if (damping_str == NULL && friction_str == NULL)
  {
    logError("joint dynamics element specified with no damping and no friction");
    return false;
  }
  else{
    logDebug("urdfdom.joint_dynamics: damping %f and friction %f", jd.damping, jd.friction);
    return true;
  }
}

bool parseJointLimits(JointLimits &jl, TiXmlElement* config)
{
  jl.clear();

  // Get lower joint limit
  const char* lower_str = config->Attribute("lower");
  if (lower_str == NULL){
    logDebug("urdfdom.joint_limit: no lower, defaults to 0");
    jl.lower = 0;
  }
  else
  {
    try
    {
      jl.lower = boost::lexical_cast<double>(lower_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("lower value (%s) is not a float: %s", lower_str, e.what());
      return false;
    }
  }

  // Get upper joint limit
  const char* upper_str = config->Attribute("upper");
  if (upper_str == NULL){
    logDebug("urdfdom.joint_limit: no upper, , defaults to 0");
    jl.upper = 0;
  }
  else
  {
    try
    {
      jl.upper = boost::lexical_cast<double>(upper_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("upper value (%s) is not a float: %s",upper_str, e.what());
      return false;
    }
  }

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL){
    logError("joint limit: no effort");
    return false;
  }
  else
  {
    try
    {
      jl.effort = boost::lexical_cast<double>(effort_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("effort value (%s) is not a float: %s",effort_str, e.what());
      return false;
    }
  }

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL){
    logError("joint limit: no velocity");
    return false;
  }
  else
  {
    try
    {
      jl.velocity = boost::lexical_cast<double>(velocity_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("velocity value (%s) is not a float: %s",velocity_str, e.what());
      return false;
    }
  }

  return true;
}

bool parseJointSafety(JointSafety &js, TiXmlElement* config)
{
  js.clear();

  // Get soft_lower_limit joint limit
  const char* soft_lower_limit_str = config->Attribute("soft_lower_limit");
  if (soft_lower_limit_str == NULL)
  {
    logDebug("urdfdom.joint_safety: no soft_lower_limit, using default value");
    js.soft_lower_limit = 0;
  }
  else
  {
    try
    {
      js.soft_lower_limit = boost::lexical_cast<double>(soft_lower_limit_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("soft_lower_limit value (%s) is not a float: %s",soft_lower_limit_str, e.what());
      return false;
    }
  }

  // Get soft_upper_limit joint limit
  const char* soft_upper_limit_str = config->Attribute("soft_upper_limit");
  if (soft_upper_limit_str == NULL)
  {
    logDebug("urdfdom.joint_safety: no soft_upper_limit, using default value");
    js.soft_upper_limit = 0;
  }
  else
  {
    try
    {
      js.soft_upper_limit = boost::lexical_cast<double>(soft_upper_limit_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("soft_upper_limit value (%s) is not a float: %s",soft_upper_limit_str, e.what());
      return false;
    }
  }

  // Get k_position_ safety "position" gain - not exactly position gain
  const char* k_position_str = config->Attribute("k_position");
  if (k_position_str == NULL)
  {
    logDebug("urdfdom.joint_safety: no k_position, using default value");
    js.k_position = 0;
  }
  else
  {
    try
    {
      js.k_position = boost::lexical_cast<double>(k_position_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("k_position value (%s) is not a float: %s",k_position_str, e.what());
      return false;
    }
  }
  // Get k_velocity_ safety velocity gain
  const char* k_velocity_str = config->Attribute("k_velocity");
  if (k_velocity_str == NULL)
  {
    logError("joint safety: no k_velocity");
    return false;
  }
  else
  {
    try
    {
      js.k_velocity = boost::lexical_cast<double>(k_velocity_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("k_velocity value (%s) is not a float: %s",k_velocity_str, e.what());
      return false;
    }
  }

  return true;
}

bool parseJointCalibration(JointCalibration &jc, TiXmlElement* config)
{
  jc.clear();

  // Get rising edge position
  const char* rising_position_str = config->Attribute("rising");
  if (rising_position_str == NULL)
  {
    logDebug("urdfdom.joint_calibration: no rising, using default value");
    jc.rising.reset(0);
  }
  else
  {
    try
    {
      jc.rising.reset(new double(boost::lexical_cast<double>(rising_position_str)));
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("risingvalue (%s) is not a float: %s",rising_position_str, e.what());
      return false;
    }
  }

  // Get falling edge position
  const char* falling_position_str = config->Attribute("falling");
  if (falling_position_str == NULL)
  {
    logDebug("urdfdom.joint_calibration: no falling, using default value");
    jc.falling.reset(0);
  }
  else
  {
    try
    {
      jc.falling.reset(new double(boost::lexical_cast<double>(falling_position_str)));
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("fallingvalue (%s) is not a float: %s",falling_position_str, e.what());
      return false;
    }
  }

  return true;
}

bool parseJointMimic(JointMimic &jm, TiXmlElement* config)
{
  jm.clear();

  // Get name of joint to mimic
  const char* joint_name_str = config->Attribute("joint");

  if (joint_name_str == NULL)
  {
    logError("joint mimic: no mimic joint specified");
    return false;
  }
  else
    jm.joint_name = joint_name_str;
  
  // Get mimic multiplier
  const char* multiplier_str = config->Attribute("multiplier");

  if (multiplier_str == NULL)
  {
    logDebug("urdfdom.joint_mimic: no multiplier, using default value of 1");
    jm.multiplier = 1;    
  }
  else
  {
    try
    {
      jm.multiplier = boost::lexical_cast<double>(multiplier_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("multiplier value (%s) is not a float: %s",multiplier_str, e.what());
      return false;
    }
  }

  
  // Get mimic offset
  const char* offset_str = config->Attribute("offset");
  if (offset_str == NULL)
  {
    logDebug("urdfdom.joint_mimic: no offset, using default value of 0");
    jm.offset = 0;
  }
  else
  {
    try
    {
      jm.offset = boost::lexical_cast<double>(offset_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      logError("offset value (%s) is not a float: %s",offset_str, e.what());
      return false;
    }
  }

  return true;
}

bool parseJoint(Joint &joint, TiXmlElement* config)
{
  joint.clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    logError("unnamed joint found");
    return false;
  }
  joint.name = name;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    logDebug("urdfdom: Joint [%s] missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform).", joint.name.c_str());
    joint.parent_to_joint_origin_transform.clear();
  }
  else
  {
    if (!parsePose(joint.parent_to_joint_origin_transform, origin_xml))
    {
      joint.parent_to_joint_origin_transform.clear();
      logError("Malformed parent origin element for joint [%s]", joint.name.c_str());
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
      logInform("no parent link name specified for Joint link [%s]. this might be the root?", joint.name.c_str());
    }
    else
    {
      joint.parent_link_name = std::string(pname);
    }
  }

  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
    {
      logInform("no child link name specified for Joint link [%s].", joint.name.c_str());
    }
    else
    {
      joint.child_link_name = std::string(pname);
    }
  }

  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    logError("joint [%s] has no type, check to see if it's a reference.", joint.name.c_str());
    return false;
  }
  
  std::string type_str = type_char;
  if (type_str == "planar")
    joint.type = Joint::PLANAR;
  else if (type_str == "floating")
    joint.type = Joint::FLOATING;
  else if (type_str == "revolute")
    joint.type = Joint::REVOLUTE;
  else if (type_str == "continuous")
    joint.type = Joint::CONTINUOUS;
  else if (type_str == "prismatic")
    joint.type = Joint::PRISMATIC;
  else if (type_str == "fixed")
    joint.type = Joint::FIXED;
  else
  {
    logError("Joint [%s] has no known type [%s]", joint.name.c_str(), type_str.c_str());
    return false;
  }

  // Get Joint Axis
  if (joint.type != Joint::FLOATING && joint.type != Joint::FIXED)
  {
    // axis
    TiXmlElement *axis_xml = config->FirstChildElement("axis");
    if (!axis_xml){
      logDebug("urdfdom: no axis elemement for Joint link [%s], defaulting to (1,0,0) axis", joint.name.c_str());
      joint.axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (axis_xml->Attribute("xyz")){
        try {
          joint.axis.init(axis_xml->Attribute("xyz"));
        }
        catch (ParseError &e) {
          joint.axis.clear();
          logError("Malformed axis element for joint [%s]: %s", joint.name.c_str(), e.what());
          return false;
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    joint.limits.reset(new JointLimits());
    if (!parseJointLimits(*joint.limits, limit_xml))
    {
      logError("Could not parse limit element for joint [%s]", joint.name.c_str());
      joint.limits.reset(0);
      return false;
    }
  }
  else if (joint.type == Joint::REVOLUTE)
  {
    logError("Joint [%s] is of type REVOLUTE but it does not specify limits", joint.name.c_str());
    return false;
  }
  else if (joint.type == Joint::PRISMATIC)
  {
    logError("Joint [%s] is of type PRISMATIC without limits", joint.name.c_str()); 
    return false;
  }

  // Get safety
  TiXmlElement *safety_xml = config->FirstChildElement("safety_controller");
  if (safety_xml)
  {
    joint.safety.reset(new JointSafety());
    if (!parseJointSafety(*joint.safety, safety_xml))
    {
      logError("Could not parse safety element for joint [%s]", joint.name.c_str());
      joint.safety.reset(0);
      return false;
    }
  }

  // Get calibration
  TiXmlElement *calibration_xml = config->FirstChildElement("calibration");
  if (calibration_xml)
  {
    joint.calibration.reset(new JointCalibration());
    if (!parseJointCalibration(*joint.calibration, calibration_xml))
    {
      logError("Could not parse calibration element for joint  [%s]", joint.name.c_str());
      joint.calibration.reset(0);
      return false;
    }
  }

  // Get Joint Mimic
  TiXmlElement *mimic_xml = config->FirstChildElement("mimic");
  if (mimic_xml)
  {
    joint.mimic.reset(new JointMimic());
    if (!parseJointMimic(*joint.mimic, mimic_xml))
    {
      logError("Could not parse mimic element for joint  [%s]", joint.name.c_str());
      joint.mimic.reset(0);
      return false;
    }
  }

  // Get Dynamics
  TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml)
  {
    joint.dynamics.reset(new JointDynamics());
    if (!parseJointDynamics(*joint.dynamics, prop_xml))
    {
      logError("Could not parse joint_dynamics element for joint [%s]", joint.name.c_str());
      joint.dynamics.reset(0);
      return false;
    }
  }

  return true;
}




}
