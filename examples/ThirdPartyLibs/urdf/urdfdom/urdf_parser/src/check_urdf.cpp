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

#include "urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>

using namespace urdf;

void printTree(my_shared_ptr<const Link> link,int level = 0)
{
  level+=2;
  int count = 0;
  for (std::vector<my_shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
    }
  }

}


#define MSTRINGIFY(A) #A


const char* urdf_char = MSTRINGIFY(
<robot name="test_robot">
<link name="link1" />
<link name="link2" />
<link name="link3" />
<link name="link4" />

<joint name="joint1" type="continuous">
<parent link="link1"/>
<child link="link2"/>
</joint>

<joint name="joint2" type="continuous">
<parent link="link1"/>
<child link="link3"/>
</joint>

<joint name="joint3" type="continuous">
<parent link="link3"/>
<child link="link4"/>
</joint>
</robot>);


int main(int argc, char** argv)
{
    
  std::string xml_string;
    
  if (argc < 2){
    std::cerr << "No URDF file name provided, using a dummy test URDF" << std::endl;

      xml_string = std::string(urdf_char);
      
  } else
  {

      
      std::fstream xml_file(argv[1], std::fstream::in);
      while ( xml_file.good() )
      {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
  }
    
  my_shared_ptr<ModelInterface> robot = parseURDF(xml_string);
  if (!robot){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }
  std::cout << "robot name is: " << robot->getName() << std::endl;

  // get info from parser
  std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
  // get root link
  my_shared_ptr<const Link> root_link=robot->getRoot();
  if (!root_link) return -1;

  std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;


  // print entire tree
  printTree(root_link);
  return 0;
}

