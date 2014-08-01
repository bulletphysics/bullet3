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


const char* urdf_char2 = MSTRINGIFY(
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

const char* urdf_char1 = MSTRINGIFY(
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

const char* urdf_char3 = MSTRINGIFY(<?xml version="1.0"?>
                                   <robot name="multipleshapes">
                                   <link name="base_link">
                                   <visual>
                                   <geometry>
                                   <cylinder length="0.6" radius="0.2"/>
                                   </geometry>
                                   </visual>
                                   </link>
                                   
                                   <link name="right_leg">
                                   <visual>
                                   <geometry>
                                   <box size="0.6 .2 .1"/>
                                   </geometry>
                                   </visual>
                                   </link>
                                   
                                   <joint name="base_to_right_leg" type="fixed">
                                   <parent link="base_link"/>
                                   <child link="right_leg"/>
                                   </joint>
                                   
                                   </robot>);

const char* urdf_char = MSTRINGIFY(<?xml version="1.0"?>
                                   <robot name="materials">
                                   <link name="base_link">
                                   <visual>
                                   <geometry>
                                   <cylinder length="0.6" radius="0.2"/>
                                   </geometry>
                                   <material name="blue">
                                   <color rgba="0 0 .8 1"/>
                                   </material>
                                   </visual>
                                   </link>
                                   
                                   <link name="right_leg">
                                   <visual>
                                   <geometry>
                                   <box size="0.6 .2 .1"/>
                                   </geometry>
                                   <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
                                   <material name="white">
                                   <color rgba="1 1 1 1"/>
                                   </material>
                                   </visual>
                                   </link>
                                   
                                   <joint name="base_to_right_leg" type="fixed">
                                   <parent link="base_link"/>
                                   <child link="right_leg"/>
                                   <origin xyz="0.22 0 .25"/>
                                   </joint>
                                   
                                   <link name="left_leg">
                                   <visual>
                                   <geometry>
                                   <box size="0.6 .2 .1"/>
                                   </geometry>
                                   <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
                                   <material name="white"/>
                                   </visual>
                                   </link>
                                   
                                   <joint name="base_to_left_leg" type="fixed">
                                   <parent link="base_link"/>
                                   <child link="left_leg"/>
                                   <origin xyz="-0.22 0 .25"/>
                                   </joint>
                                   
                                   </robot>);


#include "BulletCollision/CollisionShapes/btCylinderShape.h"

void URDFvisual2BulletCollisionShape(my_shared_ptr<const Link> link, GraphicsPhysicsBridge& gfxBridge)
{
    btCollisionShape* shape = 0;

    {
        printf("converting link %s",link->name.c_str());
        
        for (int v=0;v<link->visual_array.size();v++)
        {
            const Visual* visual = link->visual_array[v].get();
            
            
            switch (visual->geometry->type)
            {
                    //            , BOX, CYLINDER, MESH:
                case Geometry::CYLINDER:
                {
                    printf("processing a cylinder\n");
                    urdf::Cylinder* cyl = (urdf::Cylinder*)visual->geometry.get();
                    btVector3 halfExtents(cyl->radius,cyl->radius,cyl->length);
                    btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
                    shape = cylZShape;
                    break;
                }
                case Geometry::BOX:
                {
                    printf("processing a box\n");
                    urdf::Box* box = (urdf::Box*)visual->geometry.get();
                    btVector3 halfExtents(box->dim.x,box->dim.y,box->dim.z);
                    btBoxShape* boxShape = new btBoxShape(halfExtents);
                    shape = boxShape;
                    break;
                }
                case Geometry::SPHERE:
                {
                    break;
                }
                case Geometry::MESH:
                {
                    break;
                }
                default:
                {
                    printf("Error: unknown visual geometry type\n");
                }
            }
            
            if (shape)
            {
                gfxBridge.createCollisionShapeGraphicsObject(shape);
                
                btVector3 color(0,0,1);
                if (visual->material.get())
                {
                    color.setValue(visual->material->color.r,visual->material->color.g,visual->material->color.b);//,visual->material->color.a);
                }
                //            btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia=btVector3(0,0,0)):
                btScalar mass = 0.f;
                btVector3 localInertia(0,0,0);
                if (mass)
                {
                    shape->calculateLocalInertia(mass,localInertia);
                }
                btRigidBody::btRigidBodyConstructionInfo rbci(mass,0,shape,localInertia);
                btRigidBody* body = new btRigidBody(rbci);
                gfxBridge.createRigidBodyGraphicsObject(body,color);
            }
        }
    }
    
    for (std::vector<my_shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        if (*child)
        {
            URDFvisual2BulletCollisionShape(*child,gfxBridge);
            
        }
        else
        {
            std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }
    
  

 
}
void ImportUrdfDemo::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
    int argc=0;
    char* filename="somefile.urdf";
    
    std::string xml_string;
    
    if (argc < 2){
        std::cerr << "No URDF file name provided, using a dummy test URDF" << std::endl;
        
        xml_string = std::string(urdf_char);
        
    } else
    {
        
        
        std::fstream xml_file(filename, std::fstream::in);
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
    
    {
        URDFvisual2BulletCollisionShape(root_link, gfxBridge);
    }
    
  
}