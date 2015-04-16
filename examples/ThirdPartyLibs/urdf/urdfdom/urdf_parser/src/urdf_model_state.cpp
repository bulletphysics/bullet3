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

/* Author: John Hsu */


#include <urdf/urdfdom_headers/urdf_model_state/include/urdf_model_state/model_state.h>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <tinyxml.h>
#include <console_bridge/console.h>

namespace urdf{

bool parseModelState(ModelState &ms, TiXmlElement* config)
{
  ms.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    logError("No name given for the model_state.");
    return false;
  }
  ms.name = std::string(name_char);

  const char *time_stamp_char = config->Attribute("time_stamp");
  if (time_stamp_char)
  {
    try {
      double sec = boost::lexical_cast<double>(time_stamp_char);
      ms.time_stamp.set(sec);
    }
    catch (boost::bad_lexical_cast &e) {
      logError("Parsing time stamp [%s] failed: %s", time_stamp_char, e.what());
      return false;
    }
  }

  TiXmlElement *joint_state_elem = config->FirstChildElement("joint_state");
  if (joint_state_elem)
  {
    boost::shared_ptr<JointState> joint_state;
    joint_state.reset(new JointState());

    const char *joint_char = joint_state_elem->Attribute("joint");
    if (joint_char)
      joint_state->joint = std::string(joint_char);
    else
    {
      logError("No joint name given for the model_state.");
      return false;
    }
    
    // parse position
    const char *position_char = joint_state_elem->Attribute("position");
    if (position_char)
    {

      std::vector<std::string> pieces;
      boost::split( pieces, position_char, boost::is_any_of(" "));
      for (unsigned int i = 0; i < pieces.size(); ++i){
        if (pieces[i] != ""){
          try {
            joint_state->position.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
          }
          catch (boost::bad_lexical_cast &e) {
            throw ParseError("position element ("+ pieces[i] +") is not a valid float");
          }
        }
      }
    }

    // parse velocity
    const char *velocity_char = joint_state_elem->Attribute("velocity");
    if (velocity_char)
    {

      std::vector<std::string> pieces;
      boost::split( pieces, velocity_char, boost::is_any_of(" "));
      for (unsigned int i = 0; i < pieces.size(); ++i){
        if (pieces[i] != ""){
          try {
            joint_state->velocity.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
          }
          catch (boost::bad_lexical_cast &e) {
            throw ParseError("velocity element ("+ pieces[i] +") is not a valid float");
          }
        }
      }
    }

    // parse effort
    const char *effort_char = joint_state_elem->Attribute("effort");
    if (effort_char)
    {

      std::vector<std::string> pieces;
      boost::split( pieces, effort_char, boost::is_any_of(" "));
      for (unsigned int i = 0; i < pieces.size(); ++i){
        if (pieces[i] != ""){
          try {
            joint_state->effort.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
          }
          catch (boost::bad_lexical_cast &e) {
            throw ParseError("effort element ("+ pieces[i] +") is not a valid float");
          }
        }
      }
    }

    // add to vector
    ms.joint_states.push_back(joint_state);
  }
};



}


