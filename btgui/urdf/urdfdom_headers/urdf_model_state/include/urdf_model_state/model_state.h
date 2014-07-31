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

#ifndef URDF_MODEL_STATE_H
#define URDF_MODEL_STATE_H

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "urdf_model/pose.h"
#include <urdf_model/twist.h>


namespace urdf{

class Time
{
public:
  Time() { this->clear(); };

  void set(double _seconds)
  {
    this->sec = (int32_t)(floor(_seconds));
    this->nsec = (int32_t)(round((_seconds - this->sec) * 1e9));
    this->Correct();
  };

  operator double ()
  {
    return (static_cast<double>(this->sec) +
            static_cast<double>(this->nsec)*1e-9);
  };

  int32_t sec;
  int32_t nsec;

  void clear()
  {
    this->sec = 0;
    this->nsec = 0;
  };
private:
  void Correct()
  {
    // Make any corrections
    if (this->nsec >= 1e9)
    {
      this->sec++;
      this->nsec = (int32_t)(this->nsec - 1e9);
    }
    else if (this->nsec < 0)
    {
      this->sec--;
      this->nsec = (int32_t)(this->nsec + 1e9);
    }
  };
};


class JointState
{
public:
  JointState() { this->clear(); };

  /// joint name
  std::string joint;

  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;

  void clear()
  {
    this->joint.clear();
    this->position.clear();
    this->velocity.clear();
    this->effort.clear();
  }
};

class ModelState
{
public:
  ModelState() { this->clear(); };

  /// state name must be unique
  std::string name;

  Time time_stamp;

  void clear()
  {
    this->name.clear();
    this->time_stamp.set(0);
    this->joint_states.clear();
  };

  std::vector<boost::shared_ptr<JointState> > joint_states;

};

}

#endif

