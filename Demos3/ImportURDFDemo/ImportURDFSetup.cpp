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

const char* urdf_char4 = MSTRINGIFY(

								   <?xml version="1.0"?>
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
								   
								   <link name="head">
								   <visual>
								   <geometry>
								   <sphere radius="0.2"/>
								   </geometry>
								   <material name="white"/>
								   </visual>
								   </link>
								   
								   <joint name="head_swivel" type="fixed">
								   <parent link="base_link"/>
								   <child link="head"/>
								   <origin xyz="0 0 0.3"/>
								   </joint>

								   <link name="box">
								   <visual>
								   <geometry>
								   <box size=".08 .08 .08"/>
								   </geometry>
								   <material name="blue"/>
								   </visual>
								   </link>
								   
								   <joint name="tobox" type="fixed">
								   <parent link="head"/>
								   <child link="box"/>
								   <origin xyz="0 0.1414 0.1414"/>
								   </joint>
								   
								   </robot>


);

const char* urdf_char_r2d2 = MSTRINGIFY(

								   <?xml version="1.0"?>
								   <robot name="visual">
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
								   
								   <link name="right_base">
								   <visual>
								   <geometry>
								   <box size=".1 0.4 .1"/>
								   </geometry>
								   <material name="white"/>
								   </visual>
								   </link>
								   
								   <joint name="right_base_joint" type="fixed">
								   <parent link="right_leg"/>
								   <child link="right_base"/>
								   <origin xyz="0 0 -0.6"/>
								   </joint>
								   
								   <link name="right_front_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black">
								   <color rgba="0 0 0 1"/>
								   </material>
								   </visual>
								   </link>
								   
								   <joint name="right_front_wheel_joint" type="fixed">
								   <parent link="right_base"/>
								   <child link="right_front_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 0.133333333333 -0.085"/>
								   </joint>
								   
								   <link name="right_back_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black"/>
								   </visual>
								   </link>
								   
								   <joint name="right_back_wheel_joint" type="fixed">
								   <parent link="right_base"/>
								   <child link="right_back_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 -0.133333333333 -0.085"/>
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
								   
								   <link name="left_base">
								   <visual>
								   <geometry>
								   <box size=".1 0.4 .1"/>
								   </geometry>
								   <material name="white"/>
								   </visual>
								   </link>
								   
								   <joint name="left_base_joint" type="fixed">
								   <parent link="left_leg"/>
								   <child link="left_base"/>
								   <origin xyz="0 0 -0.6"/>
								   </joint>
								   
								   <link name="left_front_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black"/>
								   </visual>
								   </link>
								   
								   <joint name="left_front_wheel_joint" type="fixed">
								   <parent link="left_base"/>
								   <child link="left_front_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 0.133333333333 -0.085"/>
								   </joint>
								   
								   <link name="left_back_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black"/>
								   </visual>
								   </link>
								   
								   <joint name="left_back_wheel_joint" type="fixed">
								   <parent link="left_base"/>
								   <child link="left_back_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 -0.133333333333 -0.085"/>
								   </joint>
								   
								   <joint name="gripper_extension" type="fixed">
								   <parent link="base_link"/>
								   <child link="gripper_pole"/>
								   <origin rpy="0 0 1.57075" xyz="0 0.19 .2"/>
								   </joint>
								   
								   <link name="gripper_pole">
								   <visual>
								   <geometry>
								   <cylinder length="0.2" radius=".01"/>
								   </geometry>
								   <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
								   <material name="Gray">
								   <color rgba=".7 .7 .7 1"/>
								   </material>
								   </visual>
								   </link>
								   
								   <joint name="left_gripper_joint" type="fixed">
								   <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
								   <parent link="gripper_pole"/>
								   <child link="left_gripper"/>
								   </joint>
								   
								   <link name="left_gripper">
								   <visual>
								   <origin rpy="0.0 0 0" xyz="0 0 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <joint name="left_tip_joint" type="fixed">
								   <parent link="left_gripper"/>
								   <child link="left_tip"/>
								   </joint>
								   
								   <link name="left_tip">
								   <visual>
								   <origin rpy="0.0 0 0" xyz="0.09137 0.00495 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <joint name="right_gripper_joint" type="fixed">
								   <origin rpy="0 0 0" xyz="0.2 -0.01 0"/>
								   <parent link="gripper_pole"/>
								   <child link="right_gripper"/>
								   </joint>
								   
								   <link name="right_gripper">
								   <visual>
								   <origin rpy="-3.1415 0 0" xyz="0 0 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <joint name="right_tip_joint" type="fixed">
								   <parent link="right_gripper"/>
								   <child link="right_tip"/>
								   </joint>
								   
								   <link name="right_tip">
								   <visual>
								   <origin rpy="-3.1415 0 0" xyz="0.09137 0.00495 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <link name="head">
								   <visual>
								   <geometry>
								   <sphere radius="0.2"/>
								   </geometry>
								   <material name="white"/>
								   </visual>
								   </link>
								   
								   <joint name="head_swivel" type="fixed">
								   <parent link="base_link"/>
								   <child link="head"/>
								   <origin xyz="0 0 0.3"/>
								   </joint>
								   
								   <link name="box">
								   <visual>
								   <geometry>
								   <box size=".08 .08 .08"/>
								   </geometry>
								   <material name="blue"/>
								   </visual>
								   </link>
								   
								   <joint name="tobox" type="fixed">
								   <parent link="head"/>
								   <child link="box"/>
								   <origin xyz="0 0.1414 0.1414"/>
								   </joint>
								   
								   </robot>
);

const char* urdf_char = MSTRINGIFY(

								   <?xml version="1.0"?>
								   <robot name="flexible">
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
								   
								   <link name="right_base">
								   <visual>
								   <geometry>
								   <box size=".1 0.4 .1"/>
								   </geometry>
								   <material name="white"/>
								   </visual>
								   </link>
								   
								   <joint name="right_base_joint" type="fixed">
								   <parent link="right_leg"/>
								   <child link="right_base"/>
								   <origin xyz="0 0 -0.6"/>
								   </joint>
								   
								   <link name="right_front_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black">
								   <color rgba="0 0 0 1"/>
								   </material>
								   </visual>
								   </link>
								   
								   <joint name="right_front_wheel_joint" type="continuous">
								   <axis xyz="0 0 1"/>
								   <parent link="right_base"/>
								   <child link="right_front_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 0.133333333333 -0.085"/>
								   <limit effort="100" velocity="100"/>
								   <joint_properties damping="0.0" friction="0.0"/>
								   </joint>
								   
								   <link name="right_back_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black"/>
								   </visual>
								   </link>
								   
								   <joint name="right_back_wheel_joint" type="continuous">
								   <axis xyz="0 0 1"/>
								   <parent link="right_base"/>
								   <child link="right_back_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 -0.133333333333 -0.085"/>
								   <limit effort="100" velocity="100"/>
								   <joint_properties damping="0.0" friction="0.0"/>
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
								   
								   <link name="left_base">
								   <visual>
								   <geometry>
								   <box size=".1 0.4 .1"/>
								   </geometry>
								   <material name="white"/>
								   </visual>
								   </link>
								   
								   <joint name="left_base_joint" type="fixed">
								   <parent link="left_leg"/>
								   <child link="left_base"/>
								   <origin xyz="0 0 -0.6"/>
								   </joint>
								   
								   <link name="left_front_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black"/>
								   </visual>
								   </link>
								   
								   <joint name="left_front_wheel_joint" type="continuous">
								   <axis xyz="0 0 1"/>
								   <parent link="left_base"/>
								   <child link="left_front_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 0.133333333333 -0.085"/>
								   <limit effort="100" velocity="100"/>
								   <joint_properties damping="0.0" friction="0.0"/>
								   </joint>
								   
								   <link name="left_back_wheel">
								   <visual>
								   <geometry>
								   <cylinder length=".1" radius="0.035"/>
								   </geometry>
								   <material name="black"/>
								   </visual>
								   </link>
								   
								   <joint name="left_back_wheel_joint" type="continuous">
								   <axis xyz="0 0 1"/>
								   <parent link="left_base"/>
								   <child link="left_back_wheel"/>
								   <origin rpy="0 1.57075 0" xyz="0 -0.133333333333 -0.085"/>
								   <limit effort="100" velocity="100"/>
								   <joint_properties damping="0.0" friction="0.0"/>
								   </joint>
								   <joint name="gripper_extension" type="prismatic">
								   <parent link="base_link"/>
								   <child link="gripper_pole"/>
								   <limit effort="1000.0" lower="-0.38" upper="0" velocity="0.5"/>
								   <origin rpy="0 0 1.57075" xyz="0 0.19 .2"/>
								   </joint>
								   
								   <link name="gripper_pole">
								   <visual>
								   <geometry>
								   <cylinder length="0.2" radius=".01"/>
								   </geometry>
								   <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
								   <material name="Gray">
								   <color rgba=".7 .7 .7 1"/>
								   </material>
								   </visual>
								   </link>
								   
								   <joint name="left_gripper_joint" type="revolute">
								   <axis xyz="0 0 1"/>
								   <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
								   <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
								   <parent link="gripper_pole"/>
								   <child link="left_gripper"/>
								   </joint>
								   
								   <link name="left_gripper">
								   <visual>
								   <origin rpy="0.0 0 0" xyz="0 0 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <joint name="left_tip_joint" type="fixed">
								   <parent link="left_gripper"/>
								   <child link="left_tip"/>
								   </joint>
								   
								   <link name="left_tip">
								   <visual>
								   <origin rpy="0.0 0 0" xyz="0.09137 0.00495 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <joint name="right_gripper_joint" type="revolute">
								   <axis xyz="0 0 -1"/>
								   <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
								   <origin rpy="0 0 0" xyz="0.2 -0.01 0"/>
								   <parent link="gripper_pole"/>
								   <child link="right_gripper"/>
								   </joint>
								   
								   <link name="right_gripper">
								   <visual>
								   <origin rpy="-3.1415 0 0" xyz="0 0 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <joint name="right_tip_joint" type="fixed">
								   <parent link="right_gripper"/>
								   <child link="right_tip"/>
								   </joint>
								   
								   <link name="right_tip">
								   <visual>
								   <origin rpy="-3.1415 0 0" xyz="0.09137 0.00495 0"/>
								   <geometry>
								   <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"/>
								   </geometry>
								   </visual>
								   </link>
								   
								   <link name="head">
								   <visual>
								   <geometry>
								   <sphere radius="0.2"/>
								   </geometry>
								   <material name="white"/>
								   </visual>
								   </link>
								   
								   <joint name="head_swivel" type="continuous">
								   <parent link="base_link"/>
								   <child link="head"/>
								   <axis xyz="0 0 1"/>
								   <origin xyz="0 0 0.3"/>
								   </joint>
								   
								   <link name="box">
								   <visual>
								   <geometry>
								   <box size=".08 .08 .08"/>
								   </geometry>
								   <material name="blue"/>
								   </visual>
								   </link>
								   
								   <joint name="tobox" type="fixed">
								   <parent link="head"/>
								   <child link="box"/>
								   <origin xyz="0 0.1414 0.1414"/>
								   </joint>
								   
								   </robot>

);

#include "BulletCollision/CollisionShapes/btCylinderShape.h"

void URDFvisual2BulletCollisionShape(my_shared_ptr<const Link> link, GraphicsPhysicsBridge& gfxBridge, const btTransform& parentTransformInWorldSpace, btDiscreteDynamicsWorld* world)
{
    btCollisionShape* shape = 0;
	
	btTransform linkTransformInWorldSpace;
	linkTransformInWorldSpace.setIdentity();
	
	btScalar mass = 0;
	btTransform inertialFrame;
	inertialFrame.setIdentity();
	
	if ((*link).inertial)
	{
		mass = (*link).inertial->mass;
		inertialFrame.setOrigin(btVector3((*link).inertial->origin.position.x,(*link).inertial->origin.position.y,(*link).inertial->origin.position.z));
		inertialFrame.setRotation(btQuaternion((*link).inertial->origin.rotation.x,(*link).inertial->origin.rotation.y,(*link).inertial->origin.rotation.z,(*link).inertial->origin.rotation.w));
	}
	
	if ((*link).parent_joint)
	{
		btTransform p2j;
		const urdf::Vector3 pos = (*link).parent_joint->parent_to_joint_origin_transform.position;
		const urdf::Rotation orn = (*link).parent_joint->parent_to_joint_origin_transform.rotation;
		btTransform parent2joint;
		parent2joint.setOrigin(btVector3(pos.x,pos.y,pos.z));
		parent2joint.setRotation(btQuaternion(orn.x,orn.y,orn.z,orn.w));
		linkTransformInWorldSpace =parentTransformInWorldSpace*parent2joint;
	} else
	{
		linkTransformInWorldSpace = parentTransformInWorldSpace;
	}
	
	//btTransform linkCenterOfMass =inertialFrame*linkTransformInWorldSpace;
	
    {
        printf("converting link %s",link->name.c_str());
        for (int v=0;v<link->visual_array.size();v++)
        {
            const Visual* visual = link->visual_array[v].get();
            
            
            switch (visual->geometry->type)
            {
                case Geometry::CYLINDER:
                {
                    printf("processing a cylinder\n");
                    urdf::Cylinder* cyl = (urdf::Cylinder*)visual->geometry.get();
                    btVector3 halfExtents(cyl->radius,cyl->radius,cyl->length/2.);
                    btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
                    shape = cylZShape;
                    break;
                }
                case Geometry::BOX:
                {
                    printf("processing a box\n");
                    urdf::Box* box = (urdf::Box*)visual->geometry.get();
                    btVector3 extents(box->dim.x,box->dim.y,box->dim.z);
                    btBoxShape* boxShape = new btBoxShape(extents*0.5f);
                    shape = boxShape;
                    break;
                }
                case Geometry::SPHERE:
                {
					printf("processing a sphere\n");
                    urdf::Sphere* sphere = (urdf::Sphere*)visual->geometry.get();
                    btScalar radius = sphere->radius;
					btSphereShape* sphereShape = new btSphereShape(radius);
                    shape = sphereShape;
                    break;

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
               
                btVector3 localInertia(0,0,0);
                if (mass)
                {
                    shape->calculateLocalInertia(mass,localInertia);
                }
                btRigidBody::btRigidBodyConstructionInfo rbci(mass,0,shape,localInertia);
				
				
				btVector3 visual_pos(visual->origin.position.x,visual->origin.position.y,visual->origin.position.z);
				btQuaternion visual_orn(visual->origin.rotation.x,visual->origin.rotation.y,visual->origin.rotation.z,visual->origin.rotation.w);
				btTransform visual_frame;
				visual_frame.setOrigin(visual_pos);
				visual_frame.setRotation(visual_orn);
				
				btTransform visualFrameInWorldSpace =linkTransformInWorldSpace*visual_frame;
				rbci.m_startWorldTransform = visualFrameInWorldSpace;//linkCenterOfMass;
				
				
                btRigidBody* body = new btRigidBody(rbci);
				world->addRigidBody(body);
                gfxBridge.createRigidBodyGraphicsObject(body,color);
            }
        }
    }
    
    for (std::vector<my_shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        if (*child)
        {
            URDFvisual2BulletCollisionShape(*child,gfxBridge, linkTransformInWorldSpace, world);
            
        }
        else
        {
            std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }
    
  

 
}
void ImportUrdfDemo::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	
	int upAxis = 2;
	gfxBridge.setUpAxis(upAxis);
	
	this->createEmptyDynamicsWorld();
/*	btVector3 groundHalfExtents(50,50,50);
	groundHalfExtents[upAxis]=1.f;
	btBoxShape* box = new btBoxShape(groundHalfExtents);
	gfxBridge.createCollisionShapeGraphicsObject(box);
	btTransform start; start.setIdentity();
	btVector3 groundOrigin(0,0,0);
	groundOrigin[upAxis]=-5;
	start.setOrigin(groundOrigin);	
	btRigidBody* body =  createRigidBody(0,start,box);
	btVector3 color(0,1,0);
	gfxBridge.createRigidBodyGraphicsObject(body,color);
*/
	btVector3 gravity(0,0,0);
	gravity[upAxis]=-9.8;
	
	m_dynamicsWorld->setGravity(gravity);
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
        return ;
    }
    std::cout << "robot name is: " << robot->getName() << std::endl;
    
    // get info from parser
    std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
    // get root link
    my_shared_ptr<const Link> root_link=robot->getRoot();
    if (!root_link) return ;
    
    std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;
    
    // print entire tree
    printTree(root_link);
    btTransform worldTrans;
	worldTrans.setIdentity();
	
    {
        URDFvisual2BulletCollisionShape(root_link, gfxBridge, worldTrans,m_dynamicsWorld);
    }
    
  
}
