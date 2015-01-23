
#include "ImportURDFSetup.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../ImportSTLDemo/LoadMeshFromSTL.h"
#include "../ImportColladaDemo/LoadMeshFromCollada.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"

static int bodyCollisionFilterGroup=btBroadphaseProxy::CharacterFilter;
static int bodyCollisionFilterMask=btBroadphaseProxy::AllFilter&(~btBroadphaseProxy::CharacterFilter);
static bool enableConstraints = true;//false;


ImportUrdfSetup::ImportUrdfSetup()
{
    sprintf(m_fileName,"r2d2.urdf");//sphere2.urdf");//
}

ImportUrdfSetup::~ImportUrdfSetup()
{

}

static btVector4 colors[4] =
{
	btVector4(1,0,0,1),
	btVector4(0,1,0,1),
	btVector4(0,1,1,1),
	btVector4(1,1,0,1),
};


btVector3 selectColor()
{

	static int curColor = 0;
	btVector4 color = colors[curColor];
	curColor++;
	curColor&=3;
	return color;
}

void ImportUrdfSetup::setFileName(const char* urdfFileName)
{
    memcpy(m_fileName,urdfFileName,strlen(urdfFileName)+1);
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
	int m_linkIndex;
	int m_parentIndex;

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

	btAlignedObjectArray<btTransform>		m_linkLocalInertiaTransforms;//Body transform is in center of mass, aligned with Principal Moment Of Inertia;
	btAlignedObjectArray<btScalar>			m_linkMasses;
	btAlignedObjectArray<btVector3>			m_linkLocalDiagonalInertiaTensors;
	btAlignedObjectArray<btTransform>		m_jointTransforms;//for root, it is identity
	btAlignedObjectArray<int>				m_parentIndices;//for root, it is identity
	btAlignedObjectArray<btVector3>			m_jointAxisArray;
	btAlignedObjectArray<btTransform>			m_jointOffsetInParent;
	btAlignedObjectArray<btTransform>			m_jointOffsetInChild;
	btAlignedObjectArray<int>				m_jointTypeArray;

};
enum MyFileType
{
	FILE_STL=1,
	FILE_COLLADA=2
};

btCollisionShape* convertVisualToCollisionShape(const Collision* visual, const char* pathPrefix)
{
	btCollisionShape* shape = 0;

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
			shape ->setMargin(0.001);
            break;
        }
        case Geometry::SPHERE:
        {
			printf("processing a sphere\n");
            urdf::Sphere* sphere = (urdf::Sphere*)visual->geometry.get();
            btScalar radius = sphere->radius;
			btSphereShape* sphereShape = new btSphereShape(radius);
            shape = sphereShape;
			shape ->setMargin(0.001);
            break;

            break;
        }
        case Geometry::MESH:
        {
			if (visual->name.length())
			{
				printf("visual->name=%s\n",visual->name.c_str());
			}
			if (visual->geometry)
			{
				const urdf::Mesh* mesh = (const urdf::Mesh*) visual->geometry.get();
				if (mesh->filename.length())
				{
					const char* filename = mesh->filename.c_str();
					printf("mesh->filename=%s\n",filename);
					char fullPath[1024];
					int fileType = 0;
					sprintf(fullPath,"%s%s",pathPrefix,filename);
					b3FileUtils::toLower(fullPath);
					if (strstr(fullPath,".dae"))
					{
						fileType = FILE_COLLADA;
					}
					if (strstr(fullPath,".stl"))
					{
						fileType = FILE_STL;
					}
					

					sprintf(fullPath,"%s%s",pathPrefix,filename);
					FILE* f = fopen(fullPath,"rb");
					if (f)
					{
						fclose(f);
						GLInstanceGraphicsShape* glmesh = 0;
						
						
						switch (fileType)
						{
						case FILE_STL:
							{
								glmesh = LoadMeshFromSTL(fullPath);
							break;
							}
						case FILE_COLLADA:
							{
								
								btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
								btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
								btTransform upAxisTrans;upAxisTrans.setIdentity();
								float unitMeterScaling=1;

								LoadMeshFromCollada(fullPath,
													visualShapes, 
													visualShapeInstances,
													upAxisTrans,
													unitMeterScaling);
								
								glmesh = new GLInstanceGraphicsShape;
								int index = 0;
								glmesh->m_indices = new b3AlignedObjectArray<int>();
								glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

								for (int i=0;i<visualShapeInstances.size();i++)
								{
									ColladaGraphicsInstance* instance = &visualShapeInstances[i];
									GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];
		
									b3AlignedObjectArray<GLInstanceVertex> verts;
									verts.resize(gfxShape->m_vertices->size());

									int baseIndex = glmesh->m_vertices->size();

									for (int i=0;i<gfxShape->m_vertices->size();i++)
									{
										verts[i].normal[0] = 	gfxShape->m_vertices->at(i).normal[0];
										verts[i].normal[1] = 	gfxShape->m_vertices->at(i).normal[1];
										verts[i].normal[2] = 	gfxShape->m_vertices->at(i).normal[2];
										verts[i].uv[0] = gfxShape->m_vertices->at(i).uv[0];
										verts[i].uv[1] = gfxShape->m_vertices->at(i).uv[1];
										verts[i].xyzw[0] = gfxShape->m_vertices->at(i).xyzw[0];
										verts[i].xyzw[1] = gfxShape->m_vertices->at(i).xyzw[1];
										verts[i].xyzw[2] = gfxShape->m_vertices->at(i).xyzw[2];
										verts[i].xyzw[3] = gfxShape->m_vertices->at(i).xyzw[3];

									}

									int curNumIndices = glmesh->m_indices->size();
									int additionalIndices = gfxShape->m_indices->size();
									glmesh->m_indices->resize(curNumIndices+additionalIndices);
									for (int k=0;k<additionalIndices;k++)
									{
										glmesh->m_indices->at(curNumIndices+k)=gfxShape->m_indices->at(k)+baseIndex;
									}
			
									//compensate upAxisTrans and unitMeterScaling here
									btMatrix4x4 upAxisMat;
									upAxisMat.setPureRotation(upAxisTrans.getRotation());
									btMatrix4x4 unitMeterScalingMat;
									unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling,unitMeterScaling,unitMeterScaling));
									btMatrix4x4 worldMat = unitMeterScalingMat*upAxisMat*instance->m_worldTransform;
									//btMatrix4x4 worldMat = instance->m_worldTransform;
									int curNumVertices = glmesh->m_vertices->size();
									int additionalVertices = verts.size();
									glmesh->m_vertices->reserve(curNumVertices+additionalVertices);
									
									for(int v=0;v<verts.size();v++)
									{
										btVector3 pos(verts[v].xyzw[0],verts[v].xyzw[1],verts[v].xyzw[2]);
										pos = worldMat*pos;
										verts[v].xyzw[0] = float(pos[0]);
										verts[v].xyzw[1] = float(pos[1]);
										verts[v].xyzw[2] = float(pos[2]);
										glmesh->m_vertices->push_back(verts[v]);
									}
								}
								glmesh->m_numIndices = glmesh->m_indices->size();
								glmesh->m_numvertices = glmesh->m_vertices->size();
								//glmesh = LoadMeshFromCollada(fullPath);

								break;
							}
						default:
							{
							}
						}
					

						if (glmesh && (glmesh->m_numvertices>0))
						{
							printf("extracted %d verticed from STL file %s\n", glmesh->m_numvertices,fullPath);
							//int shapeId = m_glApp->m_instancingRenderer->registerShape(&gvertices[0].pos[0],gvertices.size(),&indices[0],indices.size());
							//convex->setUserIndex(shapeId);
							btAlignedObjectArray<btVector3> convertedVerts;
							convertedVerts.reserve(glmesh->m_numvertices);
							for (int i=0;i<glmesh->m_numvertices;i++)
							{
								convertedVerts.push_back(btVector3(glmesh->m_vertices->at(i).xyzw[0],glmesh->m_vertices->at(i).xyzw[1],glmesh->m_vertices->at(i).xyzw[2]));
							}
							//btConvexHullShape* cylZShape = new btConvexHullShape(&glmesh->m_vertices->at(0).xyzw[0], glmesh->m_numvertices, sizeof(GLInstanceVertex));
							btConvexHullShape* cylZShape = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
							//cylZShape->initializePolyhedralFeatures();
							//btVector3 halfExtents(cyl->radius,cyl->radius,cyl->length/2.);
							//btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
							cylZShape->setMargin(0.001);
							shape = cylZShape;
						} else
						{
							printf("issue extracting mesh from STL file %s\n", fullPath);
						}

					} else
					{
						printf("mesh geometry not found %s\n",fullPath);
					}
							
							
				}
			}

					
            break;
        }
        default:
        {
            printf("Error: unknown visual geometry type\n");
        }
    }
	return shape;
}
void URDFvisual2BulletCollisionShape(my_shared_ptr<const Link> link, GraphicsPhysicsBridge& gfxBridge, const btTransform& parentTransformInWorldSpace, btDiscreteDynamicsWorld* world1, URDF2BulletMappings& mappings, const char* pathPrefix)
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
	parent2joint.setIdentity();

	if ((*link).parent_joint)
	{
		
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
        printf("converting visuals of link %s",link->name.c_str());
        for (int v=0;v<(int)link->collision_array.size();v++)
        {
            const Collision* visual = link->collision_array[v].get();

			shape = convertVisualToCollisionShape(visual,pathPrefix);
			
            if (shape)
            {
                gfxBridge.createCollisionShapeGraphicsObject(shape);

                btVector3 color = selectColor();
/*                if (visual->material.get())
                {
                    color.setValue(visual->material->color.r,visual->material->color.g,visual->material->color.b);//,visual->material->color.a);
                }
				*/
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

				world1->addRigidBody(body,bodyCollisionFilterGroup,bodyCollisionFilterMask);
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
                if ((*link).parent_joint && pp)
                {
					btAssert(pp);

                    btRigidBody* parentBody =pp->m_bulletRigidBody;

                    const Joint* pj = (*link).parent_joint.get();
                    btTransform offsetInA,offsetInB;
                    
                    offsetInA.setIdentity();

                    offsetInA = pp->m_localVisualFrame.inverse()*parent2joint;
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
                                world1->addConstraint(dof6,true);

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
                                world1->addConstraint(dof6,true);

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
                                world1->addConstraint(dof6,true);

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
            URDFvisual2BulletCollisionShape(*child,gfxBridge, linkTransformInWorldSpace, world1,mappings,pathPrefix);

        }
        else
        {
            std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }




}



btMultiBody* URDF2BulletMultiBody(my_shared_ptr<const Link> link, GraphicsPhysicsBridge& gfxBridge, const btTransform& parentTransformInWorldSpace, btMultiBodyDynamicsWorld* world, URDF2BulletMappings& mappings, const char* pathPrefix, btMultiBody* mb, int totalNumJoints)
{

	btScalar mass = 0.f;
	btTransform localInertialTransform; localInertialTransform.setIdentity();
	btVector3 localInertiaDiagonal(0,0,0);

	{
		
		if ((*link).inertial)
		{
			mass = (*link).inertial->mass;
			btMatrix3x3 inertiaMat;
			inertiaMat.setIdentity();
			inertiaMat.setValue(
				(*link).inertial->ixx,(*link).inertial->ixy,(*link).inertial->ixz,
				(*link).inertial->ixy,(*link).inertial->iyy,(*link).inertial->iyz,
				(*link).inertial->ixz,(*link).inertial->iyz,(*link).inertial->izz);
			
			btScalar threshold = 0.00001f;
			int maxSteps=20;
			btMatrix3x3 inertia2PrincipalAxis;
			inertiaMat.diagonalize(inertia2PrincipalAxis,threshold,maxSteps);
			localInertiaDiagonal.setValue(inertiaMat[0][0],inertiaMat[1][1],inertiaMat[2][2]);
            
            btVector3 inertiaLocalCOM((*link).inertial->origin.position.x,(*link).inertial->origin.position.y,(*link).inertial->origin.position.z);
            localInertialTransform.setOrigin(inertiaLocalCOM);
            btQuaternion inertiaOrn((*link).inertial->origin.rotation.x,(*link).inertial->origin.rotation.y,(*link).inertial->origin.rotation.z,(*link).inertial->origin.rotation.w);
            btMatrix3x3 inertiaOrnMat(inertiaOrn);
            
            if (mass > 0 && (localInertiaDiagonal[0]==0.f || localInertiaDiagonal[1] == 0.f
				|| localInertiaDiagonal[2] == 0.f))
            {
				b3Warning("Error: inertia should not be zero if mass is positive\n");
                localInertiaDiagonal.setMax(btVector3(0.1,0.1,0.1));
                localInertialTransform.setIdentity();//.setBasis(inertiaOrnMat);
            }
            else
            {
                localInertialTransform.setBasis(inertiaOrnMat*inertia2PrincipalAxis);
            }
		}
	}
	btTransform linkTransformInWorldSpace;
	int parentIndex = -1;

	const Link* parentLink = (*link).getParent();
	if (parentLink)
	{
		parentIndex = parentLink->m_link_index;
		btAssert(parentIndex>=0);
	}
	int linkIndex =  mappings.m_linkMasses.size();

	btTransform parent2joint;

	if ((*link).parent_joint)
	{
		const urdf::Vector3 pos = (*link).parent_joint->parent_to_joint_origin_transform.position;
		const urdf::Rotation orn = (*link).parent_joint->parent_to_joint_origin_transform.rotation;
		parent2joint.setOrigin(btVector3(pos.x,pos.y,pos.z));
		parent2joint.setRotation(btQuaternion(orn.x,orn.y,orn.z,orn.w));
		linkTransformInWorldSpace =parentTransformInWorldSpace*parent2joint;
	} else
	{
		linkTransformInWorldSpace = parentTransformInWorldSpace;

		btAssert(mb==0);

		bool multiDof = true;
		bool canSleep = false;
		bool isFixedBase = (mass==0);//todo: figure out when base is fixed

		mb = new btMultiBody(totalNumJoints,mass, localInertiaDiagonal, isFixedBase, canSleep, multiDof);


	}

	btAssert(mb);
	
	(*link).m_link_index = linkIndex;

	//compute this links center of mass transform, aligned with the principal axis of inertia


	{
		//btTransform rigidBodyFrameInWorldSpace =linkTransformInWorldSpace*inertialFrame;
			
		mappings.m_linkMasses.push_back(mass);
		mappings.m_linkLocalDiagonalInertiaTensors.push_back(localInertiaDiagonal);
		mappings.m_linkLocalInertiaTransforms.push_back(localInertialTransform);
		
			
		
		if ((*link).parent_joint)
		{
			btTransform offsetInA,offsetInB;
        offsetInA.setIdentity();
		//offsetInA = mappings.m_linkLocalInertiaTransforms[parentIndex].inverse()*parent2joint;
		offsetInA = parent2joint;
        offsetInB.setIdentity();
        //offsetInB = localInertialTransform.inverse();

			const Joint* pj = (*link).parent_joint.get();
			//btVector3 jointAxis(0,0,1);//pj->axis.x,pj->axis.y,pj->axis.z);
			btVector3 jointAxis(pj->axis.x,pj->axis.y,pj->axis.z);
			mappings.m_jointAxisArray.push_back(jointAxis);
			mappings.m_jointOffsetInParent.push_back(offsetInA);
			mappings.m_jointOffsetInChild.push_back(offsetInB);
			mappings.m_jointTypeArray.push_back(pj->type);

			switch (pj->type)
            {
                case Joint::FIXED:
                {
                    printf("Fixed joint\n");
					mb->setupFixed(linkIndex-1,mass,localInertiaDiagonal,parentIndex-1,offsetInA.getRotation(),offsetInA.getOrigin(),offsetInB.getOrigin());
					
					break;
				}
				case Joint::CONTINUOUS:
                case Joint::REVOLUTE:
                {
                    printf("Revolute joint\n");
					mb->setupRevolute(linkIndex-1,mass,localInertiaDiagonal,parentIndex-1,offsetInA.getRotation(),jointAxis,offsetInA.getOrigin(),offsetInB.getOrigin(),true);
					mb->finalizeMultiDof();
					//mb->setJointVel(linkIndex-1,1);

					break;
				}
				case Joint::PRISMATIC:
				{
					mb->setupPrismatic(linkIndex-1,mass,localInertiaDiagonal,parentIndex-1,offsetInA.getRotation(),jointAxis,offsetInB.getOrigin(),true);
					printf("Prismatic joint\n");
					break;
				}
				default:
				{
					printf("Unknown joint\n");
					btAssert(0);
				}
			};
			
			

		
		} else
		{
			mappings.m_jointAxisArray.push_back(btVector3(0,0,0));
			btTransform ident;
			ident.setIdentity();
			mappings.m_jointOffsetInParent.push_back(ident);
			mappings.m_jointOffsetInChild.push_back(ident);
			mappings.m_jointTypeArray.push_back(-1);
			

		}
	}

	//btCompoundShape* compoundShape = new btCompoundShape();
	btCollisionShape* shape = 0;

	for (int v=0;v<(int)link->collision_array.size();v++)
	{
		const Collision* visual = link->collision_array[v].get();

		shape = convertVisualToCollisionShape(visual,pathPrefix);
			
		if (shape)//childShape)
		{
			gfxBridge.createCollisionShapeGraphicsObject(shape);//childShape);

			//btVector3 color = selectColor();
			/*
			if (visual->material.get())
			{
				color.setValue(visual->material->color.r,visual->material->color.g,visual->material->color.b);//,visual->material->color.a);
			}
			*/
			btVector3 localInertia(0,0,0);
			if (mass)
			{
				shape->calculateLocalInertia(mass,localInertia);
			}
			//btRigidBody::btRigidBodyConstructionInfo rbci(mass,0,shape,localInertia);


			btVector3 visual_pos(visual->origin.position.x,visual->origin.position.y,visual->origin.position.z);
			btQuaternion visual_orn(visual->origin.rotation.x,visual->origin.rotation.y,visual->origin.rotation.z,visual->origin.rotation.w);
			btTransform visual_frame;
			visual_frame.setOrigin(visual_pos);
			visual_frame.setRotation(visual_orn);
			btTransform childTransform;
			childTransform.setIdentity();//TODO(erwincoumans): compute relative visual/inertial transform
		//	compoundShape->addChildShape(childTransform,childShape);
		}
	}
	
	if (shape)//compoundShape->getNumChildShapes()>0)
	{
		btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(mb, linkIndex-1);
        col->setCollisionShape(shape);

        btTransform tr;
        tr.setIdentity();
		tr = linkTransformInWorldSpace;
		//if we don't set the initial pose of the btCollisionObject, the simulator will do this 
		//when syncing the btMultiBody link transforms to the btMultiBodyLinkCollider
                
		//tr.setOrigin(local_origin[0]);
        //tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
        col->setWorldTransform(tr);

		bool isDynamic = true;
		short collisionFilterGroup = isDynamic? short(btBroadphaseProxy::DefaultFilter) : short(btBroadphaseProxy::StaticFilter);
		short collisionFilterMask = isDynamic? 	short(btBroadphaseProxy::AllFilter) : 	short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
		
        world->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);

        btVector3 color = selectColor();//(0.0,0.0,0.5);
        gfxBridge.createCollisionObjectGraphicsObject(col,color);
		btScalar friction = 0.5f;

        col->setFriction(friction);
        
		if (parentIndex>=0)
		{
			mb->getLink(linkIndex-1).m_collider=col;
		} else
		{
			mb->setBaseCollider(col);
		}
	}

	for (std::vector<my_shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        if (*child)
        {
            URDF2BulletMultiBody(*child,gfxBridge, linkTransformInWorldSpace, world,mappings,pathPrefix,mb,totalNumJoints);

        }
        else
        {
            std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }
	return mb;
}


void ImportUrdfSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{

	int upAxis = 2;
	gfxBridge.setUpAxis(2);

	this->createEmptyDynamicsWorld();
    gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
    btIDebugDraw::DBG_DrawConstraints
    +btIDebugDraw::DBG_DrawContactPoints
    +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);



	btVector3 gravity(0,0,0);
	gravity[upAxis]=-9.8;

	m_dynamicsWorld->setGravity(gravity);
    //int argc=0;
	char relativeFileName[1024];
	
	b3FileUtils fu;
	printf("m_fileName=%s\n", m_fileName);
	bool fileFound = fu.findFile(m_fileName, relativeFileName, 1024);



	std::string xml_string;
	char pathPrefix[1024];
	pathPrefix[0] = 0;
	
    if (!fileFound){
        std::cerr << "URDF file not found, using a dummy test URDF" << std::endl;
        xml_string = std::string(urdf_char);

    } else
    {
		
		int maxPathLen = 1024;
		fu.extractPath(relativeFileName,pathPrefix,maxPathLen);


        std::fstream xml_file(relativeFileName, std::fstream::in);
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
    btTransform identityTrans;
	identityTrans.setIdentity();
	
	int numJoints = (*robot).m_numJoints;

	static bool useFeatherstone = false;
    if (!useFeatherstone)
    {
        URDF2BulletMappings mappings;
        URDFvisual2BulletCollisionShape(root_link, gfxBridge, identityTrans,m_dynamicsWorld,mappings,pathPrefix);
    }

	//the btMultiBody support is work-in-progress :-)
#if 1
	else
    {
        URDF2BulletMappings mappings;
		
		btMultiBody* mb = URDF2BulletMultiBody(root_link, gfxBridge, identityTrans,m_dynamicsWorld,mappings,pathPrefix, 0,numJoints);
		
		mb->setHasSelfCollision(false);
		mb->finalizeMultiDof();
		m_dynamicsWorld->addMultiBody(mb);
		//m_dynamicsWorld->integrateTransforms(0.f);

    }
#endif//
	useFeatherstone = !useFeatherstone;
	printf("numJoints/DOFS = %d\n", numJoints);

	if (0)
	{
        btVector3 halfExtents(1,1,1);
        btBoxShape* box = new btBoxShape(halfExtents);
        box->initializePolyhedralFeatures();

        gfxBridge.createCollisionShapeGraphicsObject(box);
        btTransform start; start.setIdentity();
        btVector3 origin(0,0,0);
        origin[upAxis]=5;
        start.setOrigin(origin);
        btRigidBody* body =  createRigidBody(1,start,box);
        btVector3 color(0.5,0.5,0.5);
        gfxBridge.createRigidBodyGraphicsObject(body,color);
    }

    {
        btVector3 groundHalfExtents(20,20,20);
        groundHalfExtents[upAxis]=1.f;
        btBoxShape* box = new btBoxShape(groundHalfExtents);
        box->initializePolyhedralFeatures();

        gfxBridge.createCollisionShapeGraphicsObject(box);
        btTransform start; start.setIdentity();
        btVector3 groundOrigin(0,0,0);
        groundOrigin[upAxis]=-2.5;
        start.setOrigin(groundOrigin);
        btRigidBody* body =  createRigidBody(0,start,box);
        //m_dynamicsWorld->removeRigidBody(body);
       // m_dynamicsWorld->addRigidBody(body,2,1);
        btVector3 color(0.5,0.5,0.5);
        gfxBridge.createRigidBodyGraphicsObject(body,color);
    }


}

void ImportUrdfSetup::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		//the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
		m_dynamicsWorld->stepSimulation(deltaTime,10,1./240.);
	}
}
