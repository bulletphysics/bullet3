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

/* encapsulates components in a world
   see http://ros.org/wiki/usdf/XML/urdf_world and
   for details
*/
/* example world XML

<world name="pr2_with_table">
  <!-- include the models by including
       either the complete urdf or
       referencing the file name.  -->
  <model name="pr2">
    ...
  </model>
  <include filename="table.urdf" model_name="table_model"/>

  <!-- models in the world -->
  <entity model="pr2" name="prj">
    <origin xyz="0 1 0" rpy="0 0 0"/>
    <twist linear="0 0 0" angular="0 0 0"/>
  </entity>
  <entity model="pr2" name="prk">
    <origin xyz="0 2 0" rpy="0 0 0"/>
    <twist linear="0 0 0" angular="0 0 0"/>
  </entity>
  <entity model="table_model">
    <origin xyz="0 3 0" rpy="0 0 0"/>
    <twist linear="0 0 0" angular="0 0 0"/>
  </entity>

</world>

*/

#ifndef USDF_STATE_H
#define USDF_STATE_H

#include <string>
#include <vector>
#include <map>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "urdf_model/model.h"
#include "urdf_model/pose.h"
#include "urdf_model/twist.h"

namespace urdf{

class Entity
{
public:
  boost::shared_ptr<ModelInterface> model;
  Pose origin;
  Twist twist;
};

class World
{
public:
  World() { this->clear(); };

  /// world name must be unique
  std::string name;

  std::vector<Entity> models;

  void initXml(TiXmlElement* config);

  void clear()
  {
    this->name.clear();
  };
};
}

#endif

