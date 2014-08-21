
#include "ImportURDFSetup.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

static int bodyCollisionFilterGroup=btBroadphaseProxy::CharacterFilter;
static int bodyCollisionFilterMask=btBroadphaseProxy::AllFilter&(~btBroadphaseProxy::CharacterFilter);
static bool enableConstraints = true;//false;


ImportUrdfDemo::ImportUrdfDemo()
{

}

ImportUrdfDemo::~ImportUrdfDemo()
{

}




#include "urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h"

#include "urdf_samples.h"

//#include "BulletCollision/CollisionShapes/btCylinderShape.h"
//#define USE_BARREL_VERTICES
//#include "OpenGLWindow/ShapeData.h"

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


struct URDF_LinkInformation
{
    const Link* m_thisLink;

    btTransform m_localInertialFrame;
    btTransform m_localVisualFrame;

    btRigidBody* m_bulletRigidBody;
	virtual ~URDF_LinkInformation()
	{
printf("~\n");
	}
};

struct URDF_JointInformation
{

};


struct URDF2BulletMappings
{
    btHashMap<btHashPtr /*to Link*/, URDF_LinkInformation*> m_link2rigidbody;
    btHashMap<btHashPtr /*to Joint*/, btTypedConstraint*> m_joint2Constraint;
};

void URDFvisual2BulletCollisionShape(my_shared_ptr<const Link> link, GraphicsPhysicsBridge& gfxBridge, const btTransform& parentTransformInWorldSpace, btDiscreteDynamicsWorld* world, URDF2BulletMappings& mappings)
{
    btCollisionShape* shape = 0;

	btTransform linkTransformInWorldSpace;
	linkTransformInWorldSpace.setIdentity();

	btScalar mass = 1;
	btTransform inertialFrame;
	inertialFrame.setIdentity();
    const Link* parentLink = (*link).getParent();
	URDF_LinkInformation* pp = 0;

    {
		URDF_LinkInformation** ppRigidBody = mappings.m_link2rigidbody.find(parentLink);
		if (ppRigidBody)
		{
		pp = (*ppRigidBody);
			btRigidBody* parentRigidBody = pp->m_bulletRigidBody;
			btTransform tr = parentRigidBody->getWorldTransform();
			printf("rigidbody origin (COM) of link(%s) parent(%s): %f,%f,%f\n",(*link).name.c_str(), parentLink->name.c_str(), tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z());
		}
	}
	if ((*link).inertial)
	{
		mass = (*link).inertial->mass;
		inertialFrame.setOrigin(btVector3((*link).inertial->origin.position.x,(*link).inertial->origin.position.y,(*link).inertial->origin.position.z));
		inertialFrame.setRotation(btQuaternion((*link).inertial->origin.rotation.x,(*link).inertial->origin.rotation.y,(*link).inertial->origin.rotation.z,(*link).inertial->origin.rotation.w));
	}

    btTransform parent2joint;

	if ((*link).parent_joint)
	{
		btTransform p2j;
		const urdf::Vector3 pos = (*link).parent_joint->parent_to_joint_origin_transform.position;
		const urdf::Rotation orn = (*link).parent_joint->parent_to_joint_origin_transform.rotation;

		parent2joint.setOrigin(btVector3(pos.x,pos.y,pos.z));
		parent2joint.setRotation(btQuaternion(orn.x,orn.y,orn.z,orn.w));
		linkTransformInWorldSpace =parentTransformInWorldSpace*parent2joint;
	} else
	{
		linkTransformInWorldSpace = parentTransformInWorldSpace;
	}


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

                    btAlignedObjectArray<btVector3> vertices;
                    //int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
                    int numSteps = 32;
                    for (int i=0;i<numSteps;i++)
                    {

                        btVector3 vert(cyl->radius*btSin(SIMD_2_PI*(float(i)/numSteps)),cyl->radius*btCos(SIMD_2_PI*(float(i)/numSteps)),cyl->length/2.);
                        vertices.push_back(vert);
                        vert[2] = -cyl->length/2.;
                        vertices.push_back(vert);

                    }
                    btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
                    cylZShape->initializePolyhedralFeatures();
                    //btVector3 halfExtents(cyl->radius,cyl->radius,cyl->length/2.);
                    //btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
                    cylZShape->setMargin(0.001);

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
                    btScalar radius = sphere->radius*0.8;
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

				world->addRigidBody(body,bodyCollisionFilterGroup,bodyCollisionFilterMask);
    //            body->setFriction(0);

                gfxBridge.createRigidBodyGraphicsObject(body,color);
                URDF_LinkInformation* linkInfo = new URDF_LinkInformation;
                linkInfo->m_bulletRigidBody = body;
                linkInfo->m_localVisualFrame =visual_frame;
                linkInfo->m_localInertialFrame =inertialFrame;
                linkInfo->m_thisLink = link.get();
                const Link* p = link.get();
                mappings.m_link2rigidbody.insert(p, linkInfo);

                //create a joint if necessary
                if ((*link).parent_joint)
                {
                    btRigidBody* parentBody =pp->m_bulletRigidBody;

                    const Joint* pj = (*link).parent_joint.get();
                    btTransform offsetInA,offsetInB;
                    btTransform p2j; p2j.setIdentity();
                    btVector3 p2jPos; p2jPos.setValue(pj->parent_to_joint_origin_transform.position.x,
                                                      pj->parent_to_joint_origin_transform.position.y,
                                                      pj->parent_to_joint_origin_transform.position.z);
                    btQuaternion p2jOrn;p2jOrn.setValue(pj->parent_to_joint_origin_transform.rotation.x,
                                                        pj->parent_to_joint_origin_transform.rotation.y,
                                                        pj->parent_to_joint_origin_transform.rotation.z,
                                                        pj->parent_to_joint_origin_transform.rotation.w);

                    p2j.setOrigin(p2jPos);
                    p2j.setRotation(p2jOrn);

                    offsetInA.setIdentity();

                    offsetInA = pp->m_localVisualFrame.inverse()*p2j;
                    offsetInB.setIdentity();
                    offsetInB = visual_frame.inverse();

                    switch (pj->type)
                    {
                        case Joint::FIXED:
                        {
                            printf("Fixed joint\n");
                            btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentBody, *body,offsetInA,offsetInB);
//                            btVector3 bulletAxis(pj->axis.x,pj->axis.y,pj->axis.z);
                          dof6->setLinearLowerLimit(btVector3(0,0,0));
                            dof6->setLinearUpperLimit(btVector3(0,0,0));

                            dof6->setAngularLowerLimit(btVector3(0,0,0));
                           dof6->setAngularUpperLimit(btVector3(0,0,0));

                            if (enableConstraints)
                                world->addConstraint(dof6,true);

//                            btFixedConstraint* fixed = new btFixedConstraint(*parentBody, *body,offsetInA,offsetInB);
  //                          world->addConstraint(fixed,true);
                            break;
                        }
                        case Joint::CONTINUOUS:
                        case Joint::REVOLUTE:
                        {
                            btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentBody, *body,offsetInA,offsetInB);
//                            btVector3 bulletAxis(pj->axis.x,pj->axis.y,pj->axis.z);
                          dof6->setLinearLowerLimit(btVector3(0,0,0));
                            dof6->setLinearUpperLimit(btVector3(0,0,0));

                            dof6->setAngularLowerLimit(btVector3(0,0,1000));
                           dof6->setAngularUpperLimit(btVector3(0,0,-1000));

                            if (enableConstraints)
                                world->addConstraint(dof6,true);

                            printf("Revolute/Continuous joint\n");
                            break;
                        }
                        case Joint::PRISMATIC:
                        {
                            btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentBody, *body,offsetInA,offsetInB);

                            dof6->setLinearLowerLimit(btVector3(pj->limits->lower,0,0));
                            dof6->setLinearUpperLimit(btVector3(pj->limits->upper,0,0));

                            dof6->setAngularLowerLimit(btVector3(0,0,0));
                            dof6->setAngularUpperLimit(btVector3(0,0,0));

                            if (enableConstraints)
                                world->addConstraint(dof6,true);

                            printf("Prismatic\n");
                            break;
                        }
                        default:
                        {
                            printf("Error: unsupported joint type in URDF (%d)\n", pj->type);
                        }
                    }

                }
            }
        }
    }

    for (std::vector<my_shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        if (*child)
        {
            URDFvisual2BulletCollisionShape(*child,gfxBridge, linkTransformInWorldSpace, world,mappings);

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
	gfxBridge.setUpAxis(2);

	this->createEmptyDynamicsWorld();
    gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
    //btIDebugDraw::DBG_DrawConstraints
    +btIDebugDraw::DBG_DrawContactPoints
    //+btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);



	btVector3 gravity(0,0,0);
	gravity[upAxis]=-9.8;

	m_dynamicsWorld->setGravity(gravity);
    int argc=0;
    const char* filename="somefile.urdf";

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
        URDF2BulletMappings mappings;
        URDFvisual2BulletCollisionShape(root_link, gfxBridge, worldTrans,m_dynamicsWorld,mappings);
    }

    {
        btVector3 groundHalfExtents(20,20,20);
        groundHalfExtents[upAxis]=1.f;
        btBoxShape* box = new btBoxShape(groundHalfExtents);
        box->initializePolyhedralFeatures();

        gfxBridge.createCollisionShapeGraphicsObject(box);
        btTransform start; start.setIdentity();
        btVector3 groundOrigin(0,0,0);
        groundOrigin[upAxis]=-1.5;
        start.setOrigin(groundOrigin);
        btRigidBody* body =  createRigidBody(0,start,box);
        btVector3 color(0.5,0.5,0.5);
        gfxBridge.createRigidBodyGraphicsObject(body,color);
    }


}
