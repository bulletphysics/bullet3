#include "ImportURDFSetup.h"

ImportUrdfDemo::ImportUrdfDemo()
{
    
}

ImportUrdfDemo::~ImportUrdfDemo()
{
    
}




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

const char* urdf_char2 = MSTRINGIFY(
                                   <?xml version="1.0"?>
                                   <robot name="myfirst">
                                   <link name="base_link">
                                   <visual>
                                   <geometry>
                                   <cylinder length="0.6" radius="0.2"/>
                                   </geometry>
                                   </visual>
                                   </link>
                                   </robot>
                                   );

int main2(int argc, char** argv)
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


void ImportUrdfDemo::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
    main2(0,0);
}