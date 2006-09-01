/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"

//#include "GL_LineSegmentShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/CylinderShape.h"
#include "CollisionShapes/ConeShape.h"
#include "CollisionShapes/StaticPlaneShape.h"
#include "CollisionShapes/ConvexHullShape.h"
#include "CollisionShapes/TriangleMesh.h"
#include "CollisionShapes/ConvexTriangleMeshShape.h"
#include "CollisionShapes/TriangleMeshShape.h"
#include "CollisionShapes/TriangleIndexVertexArray.h"
#include "CollisionShapes/CompoundShape.h"


extern SimdVector3 gCameraUp;
extern int	gForwardAxis;

#include "CollisionShapes/Simplex1to4Shape.h"
#include "CollisionShapes/EmptyShape.h"

#include "Dynamics/RigidBody.h"
#include "CollisionDispatch/CollisionDispatcher.h"
#include "BroadphaseCollision/SimpleBroadphase.h"
#include "BroadphaseCollision/AxisSweep3.h"
#include "ConstraintSolver/Point2PointConstraint.h"
#include "ConstraintSolver/HingeConstraint.h"

#include "quickprof.h"
#include "IDebugDraw.h"

#include "GLDebugDrawer.h"

//in future make it a bsp2dae
//#define QUAKE_BSP_IMPORTING 1

#ifdef QUAKE_BSP_IMPORTING
#include "BspLoader.h"
#include "BspConverter.h"
#endif //QUAKE_BSP_IMPORTING


//either FCollada or COLLADA_DOM

//COLLADA_DOM and LibXML source code are included in Extras/ folder.
//COLLADA_DOM should compile under all platforms, and is enabled by default.

//If you want to compile with FCollada (under windows), add the FCollada sourcecode
//in Extras/FCollada, and define USE_FOLLADE, and include the library.

//#define USE_FCOLLADA 1
#ifdef USE_FCOLLADA

//Collada Physics test
//#define NO_LIBXML //need LIBXML, because FCDocument/FCDPhysicsRigidBody.h needs FUDaeWriter, through FCDPhysicsParameter.hpp
#include "FUtils/FUtils.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDSceneNode.h"
#include "FUtils/FUFileManager.h"
#include "FUtils/FULogFile.h"
#include "FCDocument/FCDPhysicsSceneNode.h"
#include "FCDocument/FCDPhysicsModelInstance.h"
#include "FCDocument/FCDPhysicsRigidBodyInstance.h"
#include "FCDocument/FCDPhysicsRigidBody.h"
#include "FCDocument/FCDGeometryInstance.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDPhysicsParameter.h"
#include "FCDocument/FCDPhysicsShape.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FUtils/FUDaeSyntax.h"

#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDPhysicsAnalyticalGeometry.h"


#else
//Use Collada-dom

#include "ColladaConverter.h"

#endif





#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;
float bulletSpeed = 40.f;

#ifdef WIN32
#if _MSC_VER >= 1310
//only use SIMD Hull code under Win32
#define USE_HULL 1
#include "NarrowPhaseCollision/Hull.h"
#endif //_MSC_VER 
#endif //WIN32


#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif
//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"


extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;



int numObjects = 0;

const int maxNumObjects = 450;

SimdTransform startTransforms[maxNumObjects];

//quick test to export new position into a COLLADA .dae file
#ifndef USE_FCOLLADA

#endif //USE_FCOLLADA

DefaultMotionState ms[maxNumObjects];
CcdPhysicsController* physObjects[maxNumObjects] = {0,0,0,0};

CcdPhysicsEnvironment* physicsEnvironmentPtr = 0;


#define CUBE_HALF_EXTENTS 1
#define EXTRA_HEIGHT -20.f


CollisionShape* gShapePtr[maxNumObjects];//1 rigidbody has 1 shape (no re-use of shapes)

////////////////////////////////////

///Very basic import
CcdPhysicsController*  LocalCreatePhysicsObject(bool isDynamic, float mass, const SimdTransform& startTransform,CollisionShape* shape)
{

	startTransforms[numObjects] = startTransform;

	PHY_ShapeProps shapeProps;

	shapeProps.m_do_anisotropic = false;
	shapeProps.m_do_fh = false;
	shapeProps.m_do_rot_fh = false;
	shapeProps.m_friction_scaling[0] = 1.;
	shapeProps.m_friction_scaling[1] = 1.;
	shapeProps.m_friction_scaling[2] = 1.;

	shapeProps.m_inertia = 1.f;
	shapeProps.m_lin_drag = 0.2f;
	shapeProps.m_ang_drag = 0.1f;
	shapeProps.m_mass = 10.0f;

	PHY_MaterialProps materialProps;
	materialProps.m_friction = 10.5f;
	materialProps.m_restitution = 0.0f;

	CcdConstructionInfo ccdObjectCi;
	ccdObjectCi.m_friction = 0.5f;

	ccdObjectCi.m_linearDamping = shapeProps.m_lin_drag;
	ccdObjectCi.m_angularDamping = shapeProps.m_ang_drag;

	SimdTransform tr;
	tr.setIdentity();

	int i = numObjects;
	{
		gShapePtr[i] = shape;

		shapeProps.m_shape = gShapePtr[i];
		shapeProps.m_shape->SetMargin(0.05f);

		SimdQuaternion orn = startTransform.getRotation();


		ms[i].setWorldOrientation(orn[0],orn[1],orn[2],orn[3]);
		ms[i].setWorldPosition(startTransform.getOrigin().getX(),startTransform.getOrigin().getY(),startTransform.getOrigin().getZ());

		ccdObjectCi.m_MotionState = &ms[i];
		ccdObjectCi.m_gravity = SimdVector3(0,-9.8,0);
		ccdObjectCi.m_localInertiaTensor =SimdVector3(0,0,0);
		if (!isDynamic)
		{
			shapeProps.m_mass = 0.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
			ccdObjectCi.m_collisionFlags = CollisionObject::isStatic;
		}
		else
		{
			shapeProps.m_mass = mass;
			ccdObjectCi.m_mass = shapeProps.m_mass;
			ccdObjectCi.m_collisionFlags = 0;
		}


		SimdVector3 localInertia(0.f,0.f,0.f);

		if (isDynamic)
		{
			gShapePtr[i]->CalculateLocalInertia(shapeProps.m_mass,localInertia);
		}

		ccdObjectCi.m_localInertiaTensor = localInertia;
		ccdObjectCi.m_collisionShape = gShapePtr[i];


		physObjects[i]= new CcdPhysicsController( ccdObjectCi);

		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		physObjects[i]->GetRigidBody()->m_ccdSquareMotionTreshold = 0.f; 

		//Experimental: better estimation of CCD Time of Impact:
		//physObjects[i]->GetRigidBody()->m_ccdSweptShereRadius = 0.5*CUBE_HALF_EXTENTS;

		physicsEnvironmentPtr->addCcdPhysicsController( physObjects[i]);

	}

	//return newly created PhysicsController
	return physObjects[numObjects++];
}

class MyColladaConverter : public ColladaConverter
{
	public:
		///those 2 virtuals are called for each constraint/physics object
	virtual int			createUniversalD6Constraint(
		class PHY_IPhysicsController* ctrlRef,class PHY_IPhysicsController* ctrlOther,
			SimdTransform& localAttachmentFrameRef,
			SimdTransform& localAttachmentOther,
			const SimdVector3& linearMinLimits,
			const SimdVector3& linearMaxLimits,
			const SimdVector3& angularMinLimits,
			const SimdVector3& angularMaxLimits
			)
		{
			return physicsEnvironmentPtr->createUniversalD6Constraint(
					ctrlRef,ctrlOther,
					localAttachmentFrameRef,
					localAttachmentOther,
					linearMinLimits,
					linearMaxLimits,
					angularMinLimits,
					angularMaxLimits
				);
		}

	virtual CcdPhysicsController*  CreatePhysicsObject(bool isDynamic, 
		float mass, 
		const SimdTransform& startTransform,
		CollisionShape* shape)
	{
		CcdPhysicsController*  ctrl = LocalCreatePhysicsObject(isDynamic, mass, startTransform,shape);
		return ctrl;
	}


	virtual	void	SetGravity(const SimdVector3& grav)
	{
		physicsEnvironmentPtr->setGravity(grav.getX(),grav.getY(),grav.getZ());
	}
	virtual void	SetCameraInfo(const SimdVector3& camUp,int forwardAxis) 
	{
		gCameraUp = camUp;
		gForwardAxis = forwardAxis;
	}

};

MyColladaConverter* gColladaConverter = 0;


#ifdef QUAKE_BSP_IMPORTING

///BspToBulletConverter  extends the BspConverter to convert to Bullet datastructures
class BspToBulletConverter : public BspConverter
{
public:

		virtual void	AddConvexVerticesCollider(std::vector<SimdVector3>& vertices, bool isEntity, const SimdVector3& entityTargetLocation)
		{
			///perhaps we can do something special with entities (isEntity)
			///like adding a collision Triggering (as example)
			
			if (vertices.size() > 0)
			{
				bool isDynamic = false;
				float mass = 0.f;
				SimdTransform startTransform;
				//can use a shift
				startTransform.setIdentity();
				startTransform.setOrigin(SimdVector3(0,0,-10.f));
				//this create an internal copy of the vertices
				CollisionShape* shape = new ConvexHullShape(&vertices[0],vertices.size());

				LocalCreatePhysicsObject(isDynamic, mass, startTransform,shape);
			}
		}
};
#endif //QUAKE_BSP_IMPORTING

#ifdef USE_FCOLLADA


bool ConvertColladaPhysicsToBulletPhysics(const FCDPhysicsSceneNode* inputNode)
{

	assert(inputNode);

	/// FRSceneNodeList nodesToDelete;
	// FRMeshPhysicsController::StartCooking();

	FCDPhysicsModelInstanceList models = inputNode->GetInstances();

	//Go through all of the physics models 
	for (FCDPhysicsModelInstanceList::iterator itM=models.begin(); itM != models.end(); itM++) 
	{

		FCDEntityInstanceList& instanceList = (*itM)->GetInstances();
		//create one node per physics model. This node is pretty much only a middle man, 
		//but better describes the structure we get from the input file 
		//FRSceneNode* modelNode = new FRSceneNode(); 
		//modelNode->SetParent(outputNode); 
		//outputNode->AddChild(modelNode); 
		//Go through all of the rigid bodies and rigid constraints in that model 

		for (FCDEntityInstanceList::iterator itE=instanceList.begin(); itE!=instanceList.end(); itE++) 
		{
			if ((*itE)->GetType() == FCDEntityInstance::PHYSICS_RIGID_CONSTRAINT) 
			{
				//not yet, could add point to point / hinge support easily 
			}
			else 
				if ((*itE)->GetType() == FCDEntityInstance::PHYSICS_RIGID_BODY) 
				{

					printf("PHYSICS_RIGID_BODY\n");


					//Create a controller per rigid-body 

					physicsEnvironmentPtr->setGravity(inputNode->GetGravity().x,inputNode->GetGravity().y,inputNode->GetGravity().z);
					//FRMeshPhysicsController* controller = new FRMeshPhysicsController(inputNode->GetGravity(), inputNode->GetTimestep()); 
					FCDPhysicsRigidBodyInstance* rbInstance = (FCDPhysicsRigidBodyInstance*)(*itE);

					FCDSceneNode* targetNode = rbInstance->GetTargetNode();


					if (!targetNode) 
					{


						//DebugOut("FCTranslator: No target node defined in rigid body instance"); 

						//SAFE_DELETE(controller); 

						continue; 
					}


					//Transfer all the transforms in n into cNode, and bake
					//at the same time the scalings. It is necessary to re-translate the
					//transforms as they will get deleted when we delete the old node.
					//A better way to do this would be to steal the transforms from the old
					//nodes, and make sure they're not deleted later, but this is impractical
					//right now as we would also have to migrate all the animation curves.


					SimdVector3 localScaling(1.f,1.f,1.f);





					SimdTransform accumulatedWorldTransform;
					accumulatedWorldTransform.setIdentity();

					uint32 numTransforms = targetNode->GetTransformCount();

					for (uint32 i=0; i<numTransforms; i++)
					{

						if (targetNode->GetTransforms()[i]->GetType() == FCDTransform::SCALE)
						{

							FCDTScale* scaleTrans = (FCDTScale*)targetNode->GetTransforms()[i];
							const FMVector3& scaling = scaleTrans->GetScale();
							localScaling[0] = scaling.x;
							localScaling[1] = scaling.y;
							localScaling[2] = scaling.z;

						} else
						{

							FMMatrix44 mat = (targetNode->GetTransforms()[i])->ToMatrix();
							SimdVector3 pos(mat.GetTranslation().x,mat.GetTranslation().y,mat.GetTranslation().z);

							SimdMatrix3x3 rotMat(
								mat.m[0][0],mat.m[0][1],mat.m[0][2],
								mat.m[1][0],mat.m[1][1],mat.m[1][2],
								mat.m[2][0],mat.m[2][1],mat.m[2][2]);

							rotMat = rotMat.transpose();
							SimdTransform trans(rotMat,pos);

							//TODO: check pre or post multiply
							accumulatedWorldTransform = accumulatedWorldTransform * trans;
						}

					}

					//Then affect all of its geometry instances.
					//Collect all the entities inside the entity vector and inside the children nodes
					/*
					FREntityList childEntities = n->GetEntities();
					FRSceneNodeList childrenToParse = n->GetChildren();

					while (!childrenToParse.empty())
					{
					FRSceneNode* child = *childrenToParse.begin();
					const FREntityList& e = child->GetEntities();
					//add the entities of that child
					childEntities.insert(childEntities.end(), e.begin(), e.end());
					//queue the grand-children nodes
					childrenToParse.insert(childrenToParse.end(), child->GetChildren().begin(), child->GetChildren().end());
					childrenToParse.erase(childrenToParse.begin());

					}
					*/
					//now check which ones are geometry mesh (right now an entity is only derived by mesh
					//but do this to avoid problems in the future)
					/*
					for (FREntityList::iterator itT = childEntities.begin(); itT != childEntities.end(); itT++)
					{

					if ((*itT)->GetType() == FREntity::MESH || (*itT)->GetType() == FREntity::MESH_CONTROLLER)

					{

					FRMesh* cMesh = (FRMesh*)*itT;

					//while we're here, bake the scaling transforms into the meshes

					BakeScalingIntoMesh(cMesh, scaleTransforms);

					controller->AddBindMesh((FRMesh*)*itT);

					}

					}
					*/


					/////////////////////////////////////////////////////////////////////
					//We're done with the targets. Now take care of the physics shapes.
					FCDPhysicsRigidBody* rigidBody = rbInstance->FlattenRigidBody();
					FCDPhysicsMaterial* mat = rigidBody->GetPhysicsMaterial();
					FCDPhysicsShapeList shapes = rigidBody->GetPhysicsShapeList();

					//need to deal with compound shapes and single shapes
					//easiest is to always create a compound, and then add each shape
					//and at the end, if compound consists of just 1 objects (without local transform) simplify it
					CollisionShape* collisionShape = 0;
					


					FCDPhysicsParameter<bool>* dyn = (FCDPhysicsParameter<bool>*)rigidBody->FindParameterByReference(DAE_DYNAMIC_ELEMENT);

					bool isDynamic = true;

					if (dyn) 
					{
						isDynamic = *dyn->GetValue();
						printf("isDynamic %i\n",isDynamic);
					}

					for (uint32 i=0; i<shapes.size(); i++)
					{
						FCDPhysicsShape* OldShape = shapes[i];

						OldShape->GetType();//
						//controller->SetDensity(OldShape->GetDensity());


						if (OldShape->GetGeometryInstance())
						{
							printf("mesh/convex geometry\n");

							FCDGeometry* geoTemp = (FCDGeometry*)(OldShape->GetGeometryInstance()->GetEntity());

							FCDGeometryMesh* colladaMesh = geoTemp->GetMesh();


							if (colladaMesh)
							{

								if (1)
								{
									bool useConvexHull = false;

									//useConvexHull uses just the points. works, but there is no rendering at the moment
									//for convex hull shapes
									if (useConvexHull)
									{
										int count = colladaMesh->GetVertexSourceCount();
										for (int i=0;i<count;i++)
										{
											const FCDGeometrySource* geomSource = colladaMesh->GetVertexSource(i);

											if (geomSource->GetSourceType()==FUDaeGeometryInput::POSITION)
											{

												int numPoints = geomSource->GetSourceData().size()/3;
												SimdPoint3* points = new SimdPoint3[numPoints];
												for (int p=0;p<numPoints;p++)
												{
													points[p].setValue(geomSource->GetSourceData()[p*3],
														geomSource->GetSourceData()[p*3+1],
														geomSource->GetSourceData()[p*3+2]);
												}


												collisionShape = new ConvexHullShape(points,numPoints);

												delete points;

												break;
											}
										}

									}
									else
									{
										TriangleMesh* trimesh = new TriangleMesh();

										int polyCount = colladaMesh->GetPolygonsCount();
										for (uint32 j=0; j<polyCount; j++)
										{
											FCDGeometryPolygons* poly = colladaMesh->GetPolygons(j);
											poly->Triangulate();

											int numfaces = poly->GetFaceCount();
											int numfacevertex = poly->GetFaceVertexCount();

											std::vector<UInt32List> dataIndices;


											//for (FCDGeometryPolygonsInputList::iterator itI = poly->idxOwners.begin(); itI != idxOwners.end(); ++itI)
											//{
											//	UInt32List* indices = &(*itI)->indices;
											//	dataIndices.push_back(*indices);
											//}




											FCDGeometryPolygonsInput* inputs = poly->FindInput(FUDaeGeometryInput::POSITION);


											int startIndex = 0;
											for (int p=0;p<numfaces;p++)
											{
												int numfacevertices = poly->GetFaceVertexCounts()[p];

												switch (numfacevertices)
												{
												case 3:
													{

														//float value = inputs->source->GetSourceData()[index];

														int offset = poly->GetFaceVertexOffset(p);
														int index;
														index = inputs->indices[offset];

														SimdVector3 vertex0(
															inputs->GetSource()->GetSourceData()[index*3],
															inputs->GetSource()->GetSourceData()[index*3+1],
															inputs->GetSource()->GetSourceData()[index*3+2]);

														index = inputs->indices[offset+1];

														SimdVector3 vertex1(
															inputs->GetSource()->GetSourceData()[index*3],
															inputs->GetSource()->GetSourceData()[index*3+1],
															inputs->GetSource()->GetSourceData()[index*3+2]);

														index = inputs->indices[offset+2];

														SimdVector3 vertex2(
															inputs->GetSource()->GetSourceData()[index*3],
															inputs->GetSource()->GetSourceData()[index*3+1],
															inputs->GetSource()->GetSourceData()[index*3+2]);


														trimesh->AddTriangle(vertex0,vertex1,vertex2);
														break;
													}

												case 4:
													{
														int offset = poly->GetFaceVertexOffset(p);
														int index;
														index = inputs->indices[offset];

														SimdVector3 vertex0(
															inputs->GetSource()->GetSourceData()[index*3],
															inputs->GetSource()->GetSourceData()[index*3+1],
															inputs->GetSource()->GetSourceData()[index*3+2]);

														index = inputs->indices[offset+1];

														SimdVector3 vertex1(
															inputs->GetSource()->GetSourceData()[index*3],
															inputs->GetSource()->GetSourceData()[index*3+1],
															inputs->GetSource()->GetSourceData()[index*3+2]);

														index = inputs->indices[offset+2];

														SimdVector3 vertex2(
															inputs->GetSource()->GetSourceData()[index*3],
															inputs->GetSource()->GetSourceData()[index*3+1],
															inputs->GetSource()->GetSourceData()[index*3+2]);

														index = inputs->indices[offset+3];

														SimdVector3 vertex3(
															inputs->GetSource()->GetSourceData()[index*3],
															inputs->GetSource()->GetSourceData()[index*3+1],
															inputs->GetSource()->GetSourceData()[index*3+2]);

														trimesh->AddTriangle(vertex0,vertex1,vertex2);
														trimesh->AddTriangle(vertex0,vertex2,vertex3);

														break;
													}

												default:
													{
													}
												}
											}

											if (colladaMesh->IsConvex() || isDynamic)
											{
												collisionShape = new ConvexTriangleMeshShape(trimesh);
											} else
											{
												collisionShape = new TriangleMeshShape(trimesh);
											}


										}


									}


								} else
								{
									printf("static not yet?\n");

									//should be static triangle mesh!

									//FRMesh* cMesh = ToFREntityGeometry(geoTemp);
									//BakeScalingIntoMesh(cMesh, scaleTransforms);

									for (uint32 j=0; j<colladaMesh->GetPolygonsCount(); j++)
									{

										/*
										FRMeshPhysicsShape* NewShape = new FRMeshPhysicsShape(controller);

										if (!NewShape->CreateTriangleMesh(cMesh, j, true))

										{

										SAFE_DELETE(NewShape);

										continue;

										}
										if (mat)
										{
										NewShape->SetMaterial(mat->GetStaticFriction(), mat->GetDynamicFriction(), mat->GetRestitution());
										//FIXME
										//NewShape->material->setFrictionCombineMode();
										//NewShape->material->setSpring();
										}

										controller->AddShape(NewShape);

										*/
									}
								}
							}



						}

						else

						{

							//FRMeshPhysicsShape* NewShape = new FRMeshPhysicsShape(controller);

							FCDPhysicsAnalyticalGeometry* analGeom = OldShape->GetAnalyticalGeometry();

							//increse the following value for nicer shapes with more vertices

							uint16 superEllipsoidSubdivisionLevel = 2;

							if (!analGeom)

								continue;

							switch (analGeom->GetGeomType())

							{

							case FCDPhysicsAnalyticalGeometry::BOX:

								{

									FCDPASBox* box = (FCDPASBox*)analGeom;
									printf("Box\n");
									collisionShape = new BoxShape(SimdVector3(box->halfExtents.x,box->halfExtents.y,box->halfExtents.z));

									break;

								}

							case FCDPhysicsAnalyticalGeometry::PLANE:

								{

									FCDPASPlane* plane = (FCDPASPlane*)analGeom;
									printf("Plane\n");
									break;

								}

							case FCDPhysicsAnalyticalGeometry::SPHERE:

								{

									FCDPASSphere* sphere = (FCDPASSphere*)analGeom;
									collisionShape = new SphereShape(sphere->radius);
									printf("Sphere\n");
									break;

								}

							case FCDPhysicsAnalyticalGeometry::CYLINDER:

								{

									//FIXME: only using the first radius of the cylinder

									FCDPASCylinder* cylinder = (FCDPASCylinder*)analGeom;
									printf("Cylinder\n");
									//Blender exports Z cylinders
									//collisionShape = new CylinderShapeZ(SimdVector3(cylinder->radius,cylinder->radius,cylinder->height));
									collisionShape = new CylinderShape(SimdVector3(cylinder->radius,cylinder->height,cylinder->radius));

									break;

								}

							case FCDPhysicsAnalyticalGeometry::CAPSULE:

								{

									//FIXME: only using the first radius of the capsule

									FCDPASCapsule* capsule = (FCDPASCapsule*)analGeom;
									printf("Capsule\n");

									break;

								}

							case FCDPhysicsAnalyticalGeometry::TAPERED_CAPSULE:

								{

									//FIXME: only using the first radius of the capsule

									FCDPASTaperedCapsule* tcapsule = (FCDPASTaperedCapsule*)analGeom;
									printf("TaperedCapsule\n");
									break;

								}

							case FCDPhysicsAnalyticalGeometry::TAPERED_CYLINDER:

								{

									//FIXME: only using the first radius of the cylinder

									FCDPASTaperedCylinder* tcylinder = (FCDPASTaperedCylinder*)analGeom;
									printf("TaperedCylinder, creating a cone for now\n");
									if (!tcylinder->height)
									{
										printf("tapered_cylinder with height 0.0\n");
										tcylinder->height = 1.f;
									}
									//either use radius1 or radius2 for now
									collisionShape = new ConeShape(tcylinder->radius,tcylinder->height);

									break;

								}

							default:

								{

									break;

								}

							}

							//controller->AddShape(NewShape);
						}

					}//for all shapes



					FCDPhysicsParameter<float>* mass = (FCDPhysicsParameter<float>*)rigidBody->FindParameterByReference(DAE_MASS_ELEMENT);

					float mymass = 1.f;

					if (mass) 
					{
						mymass = *mass->GetValue();
						printf("RB mass:%f\n",mymass);
					}

					FCDPhysicsParameter<FMVector3>* inertia = (FCDPhysicsParameter<FMVector3>*)rigidBody->FindParameterByReference(DAE_INERTIA_ELEMENT);

					if (inertia) 
					{
						inertia->GetValue();//this should be calculated from shape
					}

					FCDPhysicsParameter<FMVector3>* velocity = (FCDPhysicsParameter<FMVector3>*)rigidBody->FindParameterByReference(DAE_VELOCITY_ELEMENT);

					if (velocity) 
					{
						velocity->GetValue();
					}

					FCDPhysicsParameter<FMVector3>* angularVelocity = (FCDPhysicsParameter<FMVector3>*)rigidBody->FindParameterByReference(DAE_ANGULAR_VELOCITY_ELEMENT);

					if (angularVelocity) 
					{
						angularVelocity->GetValue();
					}

					static int once = true;

					if (collisionShape)
					{
						once = false;
						printf("create Physics Object\n");
						//void	LocalCreatePhysicsObject(bool isDynamic, float mass, const SimdTransform& startTransform,CollisionShape* shape)

						collisionShape->setLocalScaling(localScaling);
						ms[numObjects].m_localScaling = localScaling;

						LocalCreatePhysicsObject(isDynamic, mymass, accumulatedWorldTransform,collisionShape);


					}
					//controller->SetGlobalPose(n->CalculateWorldTransformation());//??

					//SAFE_DELETE(rigidBody);

				}

		}

	}

	return true; 
}

#else
//Collada-dom

#endif



////////////////////////////////////



GLDebugDrawer debugDrawer;




int main(int argc,char** argv)
{

	/// Import Collada 1.4 Physics objects

	//char* filename = "analyticalGeomPhysicsTest.dae";//ColladaPhysics.dae";
	//char* filename = "colladaphysics_spherebox.dae";
	//char* filename = "friction.dae";
	char* filename = "jenga.dae";

	printf("argc=%i\n",argc);
	{
		for (int i=0;i<argc;i++)
		{
			printf("argv[%i]=%s\n",i,argv[i]);
		}
	}
	if (argc>1)
	{
		filename = argv[1];
	}

	gCameraUp = SimdVector3(0,0,1);
	gForwardAxis = 1;

	///Setup a Physics Simulation Environment
	CollisionDispatcher* dispatcher = new	CollisionDispatcher();
	SimdVector3 worldAabbMin(-10000,-10000,-10000);
	SimdVector3 worldAabbMax(10000,10000,10000);
	OverlappingPairCache* broadphase = new AxisSweep3(worldAabbMin,worldAabbMax);
	//BroadphaseInterface* broadphase = new SimpleBroadphase();
	physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	physicsEnvironmentPtr->setDeactivationTime(2.f);
	physicsEnvironmentPtr->setGravity(0,0,-10);
	physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);



#ifdef QUAKE_BSP_IMPORTING

	void* memoryBuffer = 0;
	char* bspfilename = "bsptest.bsp";
	FILE* file = fopen(bspfilename,"r");
	if (!file)
	{
		//try again other path, 
		//sight... visual studio leaves the current working directory in the projectfiles folder
		//instead of executable folder. who wants this default behaviour?!?
		bspfilename = "../../bsptest.bsp";
		file = fopen(bspfilename,"r");
	}
	if (file)
	{
		BspLoader bspLoader;
		int size=0;
		if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET)) {        /* File operations denied? ok, just close and return failure */
			printf("Error: cannot get filesize from %s\n", bspfilename);
		} else
		{
			//how to detect file size?
			memoryBuffer = malloc(size+1);
			fread(memoryBuffer,1,size,file);
			bspLoader.LoadBSPFile( memoryBuffer);

			BspToBulletConverter bsp2bullet;
			float bspScaling = 0.1f;
			bsp2bullet.convertBsp(bspLoader,bspScaling);

		}
		fclose(file);
	}

#endif




#ifdef USE_FCOLLADA

	FCDocument* document = new FCDocument();
	FUStatus status = document->LoadFromFile(filename);
	bool success = status.IsSuccessful();
	printf ("Collada import %i\n",success);

	if (success)
	{
		const FCDPhysicsSceneNode* physicsSceneRoot = document->GetPhysicsSceneRoot();
		if (ConvertColladaPhysicsToBulletPhysics( physicsSceneRoot ))
		{
			printf("ConvertColladaPhysicsToBulletPhysics successfull\n");
		} else
		{
			printf("ConvertColladaPhysicsToBulletPhysics failed\n");
		}
	}
#else

	MyColladaConverter* converter = new MyColladaConverter;

	bool result = converter->load(filename);

	if (result)
	{
		result = converter->convert();
	}
	if (result)
	{
		gColladaConverter = converter;
	} else
	{
		gColladaConverter = 0;
	}

#endif

	clientResetScene();

	setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet COLLADA Physics Viewer http://bullet.sourceforge.net");
}

//to be implemented by the demo
void renderme()
{
	debugDrawer.SetDebugMode(getDebugMode());



	float m[16];
	int i;


	if (getDebugMode() & IDebugDraw::DBG_DisableBulletLCP)
	{
		//don't use Bullet, use quickstep
		physicsEnvironmentPtr->setSolverType(0);
	} else
	{
		//Bullet LCP solver
		physicsEnvironmentPtr->setSolverType(1);
	}

	if (getDebugMode() & IDebugDraw::DBG_EnableCCD)
	{
		physicsEnvironmentPtr->setCcdMode(3);
	} else
	{
		physicsEnvironmentPtr->setCcdMode(0);
	}


	bool isSatEnabled = (getDebugMode() & IDebugDraw::DBG_EnableSatComparison);

	physicsEnvironmentPtr->EnableSatCollisionDetection(isSatEnabled);



	for (i=0;i<numObjects;i++)
	{
		SimdTransform transA;
		transA.setIdentity();

		float pos[3];
		float rot[4];

		ms[i].getWorldPosition(pos[0],pos[1],pos[2]);
		ms[i].getWorldOrientation(rot[0],rot[1],rot[2],rot[3]);

		SimdQuaternion q(rot[0],rot[1],rot[2],rot[3]);
		transA.setRotation(q);

		SimdPoint3 dpos;
		dpos.setValue(pos[0],pos[1],pos[2]);

		transA.setOrigin( dpos );
		transA.getOpenGLMatrix( m );


		SimdVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
		if (i & 1)
		{
			wireColor = SimdVector3(0.f,0.0f,1.f);
		}
		///color differently for active, sleeping, wantsdeactivation states
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 1) //active
		{
			if (i & 1)
			{
				wireColor += SimdVector3 (1.f,0.f,0.f);
			} else
			{			
				wireColor += SimdVector3 (.5f,0.f,0.f);
			}
		}
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 2) //ISLAND_SLEEPING
		{
			if (i & 1)
			{
				wireColor += SimdVector3 (0.f,1.f, 0.f);
			} else
			{
				wireColor += SimdVector3 (0.f,0.5f,0.f);
			}
		}

		char	extraDebug[125];

		sprintf(extraDebug,"islandId=%i, Body=%i, ShapeType=%s",physObjects[i]->GetRigidBody()->m_islandTag1,physObjects[i]->GetRigidBody()->m_debugBodyId,physObjects[i]->GetRigidBody()->GetCollisionShape()->GetName());
		physObjects[i]->GetRigidBody()->GetCollisionShape()->SetExtraDebugInfo(extraDebug);
		GL_ShapeDrawer::DrawOpenGL(m,physObjects[i]->GetRigidBody()->GetCollisionShape(),wireColor,getDebugMode());

		///this block is just experimental code to show some internal issues with replacing shapes on the fly.
		if (getDebugMode()!=0 && (i>0))
		{
			if (physObjects[i]->GetRigidBody()->GetCollisionShape()->GetShapeType() == EMPTY_SHAPE_PROXYTYPE)
			{
				physObjects[i]->GetRigidBody()->SetCollisionShape(gShapePtr[1]);

				//remove the persistent collision pairs that were created based on the previous shape

				BroadphaseProxy* bpproxy = physObjects[i]->GetRigidBody()->m_broadphaseHandle;

				physicsEnvironmentPtr->GetBroadphase()->CleanProxyFromPairs(bpproxy);

				SimdVector3 newinertia;
				SimdScalar newmass = 10.f;
				physObjects[i]->GetRigidBody()->GetCollisionShape()->CalculateLocalInertia(newmass,newinertia);
				physObjects[i]->GetRigidBody()->setMassProps(newmass,newinertia);
				physObjects[i]->GetRigidBody()->updateInertiaTensor();

			}

		}


	}

	if (!(getDebugMode() & IDebugDraw::DBG_NoHelpText))
	{

		float xOffset = 10.f;
		float yStart = 20.f;

		float yIncr = -2.f;

		SimdVector3 offset(xOffset,0,0);
		SimdVector3 up = gCameraUp;
		char buf[124];

		glColor3f(0, 0, 0);

#ifdef USE_QUICKPROF


		if ( getDebugMode() & IDebugDraw::DBG_ProfileTimings)
		{
			static int counter = 0;
			counter++;
			std::map<std::string, hidden::ProfileBlock*>::iterator iter;
			for (iter = Profiler::mProfileBlocks.begin(); iter != Profiler::mProfileBlocks.end(); ++iter)
			{
				char blockTime[128];
				sprintf(blockTime, "%s: %lf",&((*iter).first[0]),Profiler::getBlockTime((*iter).first, Profiler::BLOCK_CYCLE_SECONDS));//BLOCK_TOTAL_PERCENT));
				glRasterPos3f(xOffset,yStart,0);
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),blockTime);
				yStart += yIncr;

			}
		}
#endif //USE_QUICKPROF
		//profiling << Profiler::createStatsString(Profiler::BLOCK_TOTAL_PERCENT); 
		//<< std::endl;


		SimdVector3 textPos = offset + up*yStart;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"mouse to interact");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);

		yStart += yIncr;
		textPos = offset + up*yStart;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"space to reset");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"cursor keys and z,x to navigate");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"i to toggle simulation, s single step");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"q to quit");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"d to toggle deactivation");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"a to draw temporal AABBs");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"e to export COLLADA 1.4 physics snapshot");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"c to show contact points (wireframe more)");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;


		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());


		sprintf(buf,"h to toggle help text");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		bool useBulletLCP = !(getDebugMode() & IDebugDraw::DBG_DisableBulletLCP);

		bool useCCD = (getDebugMode() & IDebugDraw::DBG_EnableCCD);

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());


		sprintf(buf,"m Bullet GJK = %i",!isSatEnabled);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"n Bullet LCP = %i",useBulletLCP);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"1 CCD mode (adhoc) = %i",useCCD);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"+- shooting speed = %10.2f",bulletSpeed);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

	}

}

void clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);

	renderme();

	glFlush();
	glutSwapBuffers();

}



void clientDisplay(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	physicsEnvironmentPtr->UpdateAabbs(deltaTime);

	renderme();


	glFlush();
	glutSwapBuffers();
}



///make this positive to show stack falling from a distance
///this shows the penalty tresholds in action, springy/spungy look

void clientResetScene()
{
	for (int i=0;i<numObjects;i++)
	{
		ms[i].m_worldTransform = startTransforms[i];
		physObjects[i]->setPosition(startTransforms[i].getOrigin().getX(),startTransforms[i].getOrigin().getY(),startTransforms[i].getOrigin().getZ());
		physObjects[i]->SetLinearVelocity(0,0,0,0);
		physObjects[i]->SetAngularVelocity(0,0,0,0);
		SimdQuaternion orn;
		startTransforms[i].getBasis().getRotation(orn);
		physObjects[i]->setOrientation(orn.x(),orn.y(),orn.z(),orn[3]);

	}

	//delete and reload, or keep transforms ready?
}



void	shootBox(const SimdVector3& destination)
{

	bool isDynamic = true;
	float mass = 1.f;
	SimdTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(SimdVector3(eye[0],eye[1],eye[2]));
	CollisionShape* boxShape = new BoxShape(SimdVector3(1.f,1.f,1.f));

	LocalCreatePhysicsObject(isDynamic, mass, startTransform,boxShape);

	int i  = numObjects-1;



	SimdVector3 linVel(destination[0]-eye[0],destination[1]-eye[1],destination[2]-eye[2]);
	linVel.normalize();
	linVel*=bulletSpeed;

	physObjects[i]->setPosition(eye[0],eye[1],eye[2]);
	physObjects[i]->setOrientation(0,0,0,1);
	physObjects[i]->SetLinearVelocity(linVel[0],linVel[1],linVel[2],false);
	physObjects[i]->SetAngularVelocity(0,0,0,false);
}

void clientKeyboard(unsigned char key, int x, int y)
{

#ifndef USE_FCOLLADA
	if (key =='e')
	{
		//save a COLLADA .dae physics snapshot
		if (gColladaConverter)
			gColladaConverter->saveAs();
	}
#endif
	if (key == '.')
	{
		shootBox(SimdVector3(0,0,0));
	}

	if (key == '+')
	{
		bulletSpeed += 10.f;
	}
	if (key == '-')
	{
		bulletSpeed -= 10.f;
	}

	defaultKeyboard(key, x, y);
}

int gPickingConstraintId = 0;
SimdVector3 gOldPickingPos;
float gOldPickingDist  = 0.f;
RigidBody* pickedBody = 0;//for deactivation state



SimdVector3	GetRayTo(int x,int y)
{
	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = 2.0 * atanf (tanFov);

	SimdVector3	rayFrom(eye[0],eye[1],eye[2]);
	SimdVector3 rayForward = -rayFrom;
	rayForward.normalize();
	float farPlane = 600.f;
	rayForward*= farPlane;

	SimdVector3 rightOffset;
	SimdVector3 vertical = gCameraUp;

	SimdVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);
	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;
	SimdVector3 rayToCenter = rayFrom + rayForward;
	SimdVector3 dHor = hor * 1.f/float(glutScreenWidth);
	SimdVector3 dVert = vertical * 1.f/float(glutScreenHeight);
	SimdVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += x * dHor;
	rayTo -= y * dVert;
	return rayTo;
}
void clientMouseFunc(int button, int state, int x, int y)
{
	//printf("button %i, state %i, x=%i,y=%i\n",button,state,x,y);
	//button 0, state 0 means left mouse down

	SimdVector3 rayTo = GetRayTo(x,y);

	switch (button)
	{
	case 2:
		{
			if (state==0)
			{
				shootBox(rayTo);
			}
			break;
		};
	case 1:
		{
			if (state==0)
			{
				//apply an impulse
				if (physicsEnvironmentPtr)
				{
					float hit[3];
					float normal[3];
					PHY_IPhysicsController* hitObj = physicsEnvironmentPtr->rayTest(0,eye[0],eye[1],eye[2],rayTo.getX(),rayTo.getY(),rayTo.getZ(),hit[0],hit[1],hit[2],normal[0],normal[1],normal[2]);
					if (hitObj)
					{
						CcdPhysicsController* physCtrl = static_cast<CcdPhysicsController*>(hitObj);
						RigidBody* body = physCtrl->GetRigidBody();
						if (body)
						{
							body->SetActivationState(ACTIVE_TAG);
							SimdVector3 impulse = rayTo;
							impulse.normalize();
							float impulseStrength = 10.f;
							impulse *= impulseStrength;
							SimdVector3 relPos(
								hit[0] - body->getCenterOfMassPosition().getX(),						
								hit[1] - body->getCenterOfMassPosition().getY(),
								hit[2] - body->getCenterOfMassPosition().getZ());

							body->applyImpulse(impulse,relPos);
						}

					}

				}

			} else
			{

			}
			break;	
		}
	case 0:
		{
			if (state==0)
			{
				//add a point to point constraint for picking
				if (physicsEnvironmentPtr)
				{
					float hit[3];
					float normal[3];
					PHY_IPhysicsController* hitObj = physicsEnvironmentPtr->rayTest(0,eye[0],eye[1],eye[2],rayTo.getX(),rayTo.getY(),rayTo.getZ(),hit[0],hit[1],hit[2],normal[0],normal[1],normal[2]);
					if (hitObj)
					{

						CcdPhysicsController* physCtrl = static_cast<CcdPhysicsController*>(hitObj);
						RigidBody* body = physCtrl->GetRigidBody();

						if (body && !body->IsStatic())
						{
							pickedBody = body;
							pickedBody->SetActivationState(DISABLE_DEACTIVATION);

							SimdVector3 pickPos(hit[0],hit[1],hit[2]);

							SimdVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;

							gPickingConstraintId = physicsEnvironmentPtr->createConstraint(physCtrl,0,PHY_POINT2POINT_CONSTRAINT,
								localPivot.getX(),
								localPivot.getY(),
								localPivot.getZ(),
								0,0,0);
							//printf("created constraint %i",gPickingConstraintId);

							//save mouse position for dragging
							gOldPickingPos = rayTo;


							SimdVector3 eyePos(eye[0],eye[1],eye[2]);

							gOldPickingDist  = (pickPos-eyePos).length();

							Point2PointConstraint* p2p = static_cast<Point2PointConstraint*>(physicsEnvironmentPtr->getConstraintById(gPickingConstraintId));
							if (p2p)
							{
								//very weak constraint for picking
								p2p->m_setting.m_tau = 0.1f;
							}
						}
					}
				}
			} else
			{
				if (gPickingConstraintId && physicsEnvironmentPtr)
				{
					physicsEnvironmentPtr->removeConstraint(gPickingConstraintId);
					//printf("removed constraint %i",gPickingConstraintId);
					gPickingConstraintId = 0;
					pickedBody->ForceActivationState(ACTIVE_TAG);
					pickedBody->m_deactivationTime = 0.f;
					pickedBody = 0;


				}
			}

			break;

		}
	default:
		{
		}
	}

}

void	clientMotionFunc(int x,int y)
{

	if (gPickingConstraintId && physicsEnvironmentPtr)
	{

		//move the constraint pivot

		Point2PointConstraint* p2p = static_cast<Point2PointConstraint*>(physicsEnvironmentPtr->getConstraintById(gPickingConstraintId));
		if (p2p)
		{
			//keep it at the same picking distance

			SimdVector3 newRayTo = GetRayTo(x,y);
			SimdVector3 eyePos(eye[0],eye[1],eye[2]);
			SimdVector3 dir = newRayTo-eyePos;
			dir.normalize();
			dir *= gOldPickingDist;

			SimdVector3 newPos = eyePos + dir;
			p2p->SetPivotB(newPos);
		}

	}
}


