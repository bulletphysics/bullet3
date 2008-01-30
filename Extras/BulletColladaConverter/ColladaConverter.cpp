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

#include <string>
#include "ColladaConverter.h"
#include "btBulletDynamicsCommon.h"
#include "dae.h"
#include "dom/domCOLLADA.h"
#include "dae/domAny.h"
#include "dom/domConstants.h"

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMesh.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
//#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "LinearMath/btDefaultMotionState.h"



char* getLastFileName();
char* fixFileName(const char* lpCmdLine);
//todo: sort out this domInstance_rigid_bodyRef forward definition, put it in the headerfile and make it virtual (make code more re-usable)

struct	btRigidBodyInput
{
	domInstance_rigid_bodyRef	m_instanceRigidBodyRef;
	domRigid_bodyRef	m_rigidBodyRef2;

	char* m_bodyName;
};

struct	ConstraintInput
{
	domInstance_physics_modelRef	m_instance_physicsModelRef;
	domPhysics_modelRef	m_model;
};


struct	btRigidBodyOutput
{
	float	m_mass;
	bool	m_isDynamics;
	btCollisionShape*	m_colShape;
	btCompoundShape*	m_compoundShape;
};



domMatrix_Array emptyMatrixArray;

///This code is actually wrong: the order of transformations is lost, so we need to rewrite this!
btTransform	GetbtTransformFromCOLLADA_DOM(domMatrix_Array& matrixArray,
														domRotate_Array& rotateArray,
														domTranslate_Array& translateArray,
														float meterScaling
														)

{
	btTransform	startTransform;
	startTransform.setIdentity();
	
	unsigned int i;
	//either load the matrix (worldspace) or incrementally build the transform from 'translate'/'rotate'
	for (i=0;i<matrixArray.getCount();i++)
	{
		domMatrixRef matrixRef = matrixArray[i];
		domFloat4x4 fl16 = matrixRef->getValue();
		btVector3 origin(fl16.get(3),fl16.get(7),fl16.get(11));
		startTransform.setOrigin(origin*meterScaling);
		btMatrix3x3 basis(fl16.get(0),fl16.get(1),fl16.get(2),
							fl16.get(4),fl16.get(5),fl16.get(6),
							fl16.get(8),fl16.get(9),fl16.get(10));
		startTransform.setBasis(basis);
	}

	for (i=0;i<rotateArray.getCount();i++)
	{
		domRotateRef rotateRef = rotateArray[i];
		domFloat4 fl4 = rotateRef->getValue();
		float angleRad = SIMD_RADS_PER_DEG*fl4.get(3);
		btQuaternion rotQuat(btVector3(fl4.get(0),fl4.get(1),fl4.get(2)),angleRad);
		startTransform.getBasis() = startTransform.getBasis() * btMatrix3x3(rotQuat);
	}

	for (i=0;i<translateArray.getCount();i++)
	{
		domTranslateRef translateRef = translateArray[i];
		domFloat3 fl3 = translateRef->getValue();
		btVector3 orgTrans(fl3.get(0),fl3.get(1),fl3.get(2));
		startTransform.getOrigin() += orgTrans*meterScaling;
	}
	return startTransform;
}





ColladaConverter::ColladaConverter()
:m_collada(0),
m_dom(0),
m_filename(0),
m_unitMeterScaling(1.f)
{
}
	

bool	ColladaConverter::load(const char* orgfilename)
{

	const char* filename = fixFileName(orgfilename);

	//Collada-m_dom
	m_collada = new DAE;
	
	int res = m_collada->load(filename);//,docBuffer);

	if (res != DAE_OK)
	{
		//some platforms might require different path, try two additional locations
		char newname[256];
		sprintf(newname,"../../%s",orgfilename);
		filename = fixFileName(newname);
		res = m_collada->load(filename);
		if (res != DAE_OK)
		{
			printf("DAE/Collada-m_dom: Couldn't load %s\n",filename);
			return false;
		}
	}
	
	if (res == DAE_OK)
	{

		m_dom = m_collada->getDom(filename);
		if ( !m_dom )
		{
			printf("COLLADA File loaded to the m_dom, but query for the m_dom assets failed \n" );
			printf("COLLADA Load Aborted! \n" );
			delete m_collada;	
			return false;
		}
	}

	convert ();
	delete m_collada;

	return true;
}





bool ColladaConverter::convert()
{

	unsigned i;

//succesfully loaded file, now convert data

			if (m_dom->getAsset() && m_dom->getAsset()->getUnit())
			{
				domAsset::domUnitRef unit = m_dom->getAsset()->getUnit();
				domFloat meter = unit->getMeter();
				printf("asset unit meter=%f\n",meter);
				//m_unitMeterScaling = meter;
		

			}
			if ( m_dom->getAsset() && m_dom->getAsset()->getUp_axis() )
			{
				domAsset::domUp_axis * up = m_dom->getAsset()->getUp_axis();
				switch( up->getValue() )
				{
				case UPAXISTYPE_X_UP:
					printf("	X is Up Data and Hiearchies must be converted!\n" ); 
					printf("  Conversion to X axis Up isn't currently supported!\n" ); 
					printf("  COLLADA_RT defaulting to Y Up \n" ); 
					setGravity(btVector3(-10,0,0));
					setCameraInfo(btVector3(1,0,0),1);
					break; 
				case UPAXISTYPE_Y_UP:
					printf("	Y Axis is Up for this file \n" ); 
					printf("  COLLADA_RT set to Y Up \n" ); 
					setGravity(btVector3(0,-10,0));
					setCameraInfo(btVector3(0,1,0),0);

					break;
				case UPAXISTYPE_Z_UP:
					printf("	Z Axis is Up for this file \n" ); 
					printf("  All Geometry and Hiearchies must be converted!\n" ); 
					setGravity(btVector3(0,0,-10));
					break; 
				default:

					break; 
				}
			}


			//we don't handle visual objects, physics objects are rered as such
			for (unsigned int s=0;s<m_dom->getLibrary_visual_scenes_array().getCount();s++)
			{
				domLibrary_visual_scenesRef scenesRef = m_dom->getLibrary_visual_scenes_array()[s];
				for (unsigned int i=0;i<scenesRef->getVisual_scene_array().getCount();i++)
				{
					domVisual_sceneRef sceneRef = scenesRef->getVisual_scene_array()[i];
					for (unsigned int n=0;n<sceneRef->getNode_array().getCount();n++)
					{
						domNodeRef nodeRef = sceneRef->getNode_array()[n];
						nodeRef->getRotate_array();
						nodeRef->getTranslate_array();
						nodeRef->getScale_array();

					}
				}
			}




			// Load all the geometry libraries
			for (  i = 0; i < m_dom->getLibrary_geometries_array().getCount(); i++)
			{
				domLibrary_geometriesRef libgeom = m_dom->getLibrary_geometries_array()[i];

				printf(" CrtScene::Reading Geometry Library \n" );
				for ( unsigned int  i = 0; i < libgeom->getGeometry_array().getCount(); i++)
				{
					//ReadGeometry(  ); 
					domGeometryRef lib = libgeom->getGeometry_array()[i];

					domMesh			*meshElement		= lib->getMesh();
					if (meshElement)
					{
						// Find out how many groups we need to allocate space for 
						int	numTriangleGroups = (int)meshElement->getTriangles_array().getCount();
						int	numPolygonGroups  = (int)meshElement->getPolygons_array().getCount();
						int	totalGroups		  = numTriangleGroups + numPolygonGroups;
						if (totalGroups == 0) 
						{
							printf("No Triangles or Polygons found int Geometry %s \n", lib->getId() ); 
						} else
						{
							printf("Found mesh geometry (%s): numTriangleGroups:%i numPolygonGroups:%i\n",lib->getId(),numTriangleGroups,numPolygonGroups);
						}


					}
					domConvex_mesh	*convexMeshElement	= lib->getConvex_mesh();
					if (convexMeshElement)
					{
						printf("found convexmesh element\n");
						// Find out how many groups we need to allocate space for 
						int	numTriangleGroups = (int)convexMeshElement->getTriangles_array().getCount();
						int	numPolygonGroups  = (int)convexMeshElement->getPolygons_array().getCount();

						int	totalGroups		  = numTriangleGroups + numPolygonGroups;
						if (totalGroups == 0) 
						{
							printf("No Triangles or Polygons found in ConvexMesh Geometry %s \n", lib->getId() ); 
						}else
						{
							printf("Found convexmesh geometry: numTriangleGroups:%i numPolygonGroups:%i\n",numTriangleGroups,numPolygonGroups);
						}
					}//fi
				}//for each geometry

			}//for all geometry libraries


			//m_dom->getLibrary_physics_models_array()

			for (  i = 0; i < m_dom->getLibrary_physics_scenes_array().getCount(); i++)
			{
				domLibrary_physics_scenesRef physicsScenesRef = m_dom->getLibrary_physics_scenes_array()[i];
				for (unsigned int s=0;s<physicsScenesRef->getPhysics_scene_array().getCount();s++)
				{
					domPhysics_sceneRef physicsSceneRef = physicsScenesRef->getPhysics_scene_array()[s];

					if (physicsSceneRef->getTechnique_common())
					{
						if (physicsSceneRef->getTechnique_common()->getGravity())
						{
							const domFloat3 grav = physicsSceneRef->getTechnique_common()->getGravity()->getValue();
							printf("gravity set to %f,%f,%f\n",grav.get(0),grav.get(1),grav.get(2));

							setGravity(btVector3(grav.get(0),grav.get(1),grav.get(2)));
						}

					} 

					for (unsigned int ps=0;ps<physicsSceneRef->getInstance_physics_model_array().getCount();ps++)
					{
						domInstance_physics_modelRef instance_physicsModelRef = physicsSceneRef->getInstance_physics_model_array()[ps];

						daeElementRef ref = instance_physicsModelRef->getUrl().getElement();

						domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 


						unsigned int p,r;
						for ( p=0;p<model->getInstance_physics_model_array().getCount();p++)
						{
							domInstance_physics_modelRef	instancePhysicsModelRef = model->getInstance_physics_model_array()[p];

							daeElementRef ref = instancePhysicsModelRef->getUrl().getElement();
	
							domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 

							//todo: group some shared functionality in following 2 'blocks'.
							for (r=0;r<instancePhysicsModelRef->getInstance_rigid_body_array().getCount();r++)
							{
								domInstance_rigid_bodyRef instRigidbodyRef = instancePhysicsModelRef->getInstance_rigid_body_array()[r];

								btVector3 linearVelocity = btVector3(0.0, 0.0, 0.0);
								btVector3 angularVelocity = btVector3(0.0, 0.0, 0.0);
								float mass = 1.f;
								bool isDynamics = true;
								btCollisionShape* colShape = 0;
								btCompoundShape* compoundShape = 0;

								xsNCName bodyName = instRigidbodyRef->getBody();

								domInstance_rigid_body::domTechnique_commonRef techniqueRef = instRigidbodyRef->getTechnique_common();
								if (techniqueRef)
								{
									if (techniqueRef->getMass())
									{
										mass = techniqueRef->getMass()->getValue();
									}
									if (techniqueRef->getDynamic())
									{
										isDynamics = techniqueRef->getDynamic()->getValue();
									}
									if (techniqueRef->getVelocity())
									{
									}
									if (techniqueRef->getAngular_velocity())
									{
									}
								}

								printf("mass = %f, isDynamics %i\n",mass,isDynamics);

								if (bodyName && model)
								{
									//try to find the rigid body

									for (unsigned int r=0;r<model->getRigid_body_array().getCount();r++)
									{
										domRigid_bodyRef rigidBodyRef = model->getRigid_body_array()[r];
										if (rigidBodyRef->getSid() && !strcmp(rigidBodyRef->getSid(),bodyName))
										{


											btRigidBodyOutput output;
											output.m_colShape = colShape;
											output.m_compoundShape = compoundShape;
											output.m_mass = 1.f;
											output.m_isDynamics = true;

											btRigidBodyInput rbInput;
											rbInput.m_rigidBodyRef2 = rigidBodyRef;
											rbInput.m_instanceRigidBodyRef = instRigidbodyRef;
											ConvertRigidBodyRef( rbInput , output );

											mass = output.m_mass;
											isDynamics = output.m_isDynamics;
											colShape = output.m_colShape;
											compoundShape = output.m_compoundShape;

										}
									}

									//////////////////////
								}

								if (compoundShape)
									colShape = compoundShape;

								if (colShape)
								{
									btRigidBodyInput input;
									input.m_instanceRigidBodyRef = instRigidbodyRef;
									input.m_rigidBodyRef2 = 0;
									input.m_bodyName = (char*)bodyName;
									PreparePhysicsObject(input, isDynamics,mass,colShape, linearVelocity, angularVelocity);
								}
							}
						}


						for (r=0;r<instance_physicsModelRef->getInstance_rigid_body_array().getCount();r++)
						{

							domInstance_rigid_bodyRef instRigidbodyRef = instance_physicsModelRef->getInstance_rigid_body_array()[r];

							btVector3 linearVelocity = btVector3(0.0, 0.0, 0.0);
							btVector3 angularVelocity = btVector3(0.0, 0.0, 0.0);

							float mass = 1.f;
							bool isDynamics = true;
							btCollisionShape* colShape = 0;
							btCompoundShape* compoundShape = 0;

							xsNCName bodyName = instRigidbodyRef->getBody();

							domInstance_rigid_body::domTechnique_commonRef techniqueRef = instRigidbodyRef->getTechnique_common();
							if (techniqueRef)
							{
								if (techniqueRef->getMass())
								{
									mass = techniqueRef->getMass()->getValue();
								}
								if (techniqueRef->getDynamic())
								{
									isDynamics = techniqueRef->getDynamic()->getValue();
								}
								if (techniqueRef->getVelocity())
								{
									linearVelocity = btVector3(
												techniqueRef->getVelocity()->getValue()[0],
												techniqueRef->getVelocity()->getValue()[1],
												techniqueRef->getVelocity()->getValue()[2]);
								}
								if (techniqueRef->getAngular_velocity())
								{
									angularVelocity = btVector3(
												techniqueRef->getAngular_velocity()->getValue()[0],
												techniqueRef->getAngular_velocity()->getValue()[1],
												techniqueRef->getAngular_velocity()->getValue()[2]);
								}
							}

							printf("mass = %f, isDynamics %i\n",mass,isDynamics);

							if (bodyName && model)
							{
								//try to find the rigid body

								for (unsigned int r=0;r<model->getRigid_body_array().getCount();r++)
								{
									domRigid_bodyRef rigidBodyRef = model->getRigid_body_array()[r];
									if (rigidBodyRef->getSid() && !strcmp(rigidBodyRef->getSid(),bodyName))
									{


										btRigidBodyOutput output;
										output.m_colShape = colShape;
										output.m_compoundShape = compoundShape;
										output.m_mass = 1.f;
										output.m_isDynamics = true;

										btRigidBodyInput rbInput;
										rbInput.m_rigidBodyRef2 = rigidBodyRef;
										rbInput.m_instanceRigidBodyRef = instRigidbodyRef;
										ConvertRigidBodyRef( rbInput , output );

										mass = output.m_mass;
										isDynamics = output.m_isDynamics;
										colShape = output.m_colShape;
										compoundShape = output.m_compoundShape;

									}
								}

								//////////////////////
							}

							if (compoundShape)
								colShape = compoundShape;

							if (colShape)
							{
								btRigidBodyInput input;
								input.m_instanceRigidBodyRef = instRigidbodyRef;
								input.m_rigidBodyRef2 = 0;
								input.m_bodyName = (char*)bodyName;
								PreparePhysicsObject(input, isDynamics,mass,colShape, linearVelocity, angularVelocity);
							}

						} //for  each  instance_rigid_body

						
					} //for each physics model

					
					//handle constraints
					for (unsigned int ma=0;ma<physicsSceneRef->getInstance_physics_model_array().getCount();ma++)
					{
						domInstance_physics_modelRef instance_physicsModelRef = physicsSceneRef->getInstance_physics_model_array()[ma];

						daeElementRef ref = instance_physicsModelRef->getUrl().getElement();

						domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 

						{
							ConstraintInput cInput;
							cInput.m_instance_physicsModelRef = instance_physicsModelRef;
							cInput.m_model = model;
							prepareConstraints(cInput);
						}

						//also don't forget the model's 'instance_physics_models!
						for ( unsigned int p=0;p<model->getInstance_physics_model_array().getCount();p++)
						{
							domInstance_physics_modelRef	instancePhysicsModelRef = model->getInstance_physics_model_array()[p];

							daeElementRef ref = instancePhysicsModelRef->getUrl().getElement();
	
							domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 
							
							ConstraintInput cInput;
							cInput.m_instance_physicsModelRef = instancePhysicsModelRef;
							cInput.m_model = model;
							prepareConstraints(cInput);
						}

											
					} //2nd time, for each physics model

				}
			}

			return true;
}


void	ColladaConverter::prepareConstraints(ConstraintInput& input)
{
	domInstance_physics_modelRef instance_physicsModelRef = input.m_instance_physicsModelRef;
	domPhysics_modelRef model = input.m_model;

	for (unsigned int c=0;c<instance_physicsModelRef->getInstance_rigid_constraint_array().getCount();c++)
	{
		domInstance_rigid_constraintRef constraintRef = instance_physicsModelRef->getInstance_rigid_constraint_array().get(c);
		xsNCName constraintName = constraintRef->getConstraint();

		if (constraintName && model)
		{
			//try to find the rigid body
			int numConstraints= model->getRigid_constraint_array().getCount();

			for (int r=0;r<numConstraints;r++)
			{
				domRigid_constraintRef rigidConstraintRef = model->getRigid_constraint_array()[r];
				
				if (rigidConstraintRef->getSid() && !strcmp(rigidConstraintRef->getSid(),constraintName))
				{
					
					//two bodies
					const domRigid_constraint::domRef_attachmentRef attachRefBody = rigidConstraintRef->getRef_attachment();
					const domRigid_constraint::domAttachmentRef attachBody1 = rigidConstraintRef->getAttachment();

					daeString orgUri0 = attachRefBody ? attachRefBody->getRigid_body().getOriginalURI() : "";
					daeString orgUri1 = attachBody1 ? attachBody1->getRigid_body().getOriginalURI() : "";
					btRigidBody* body0=0,*body1=0;

					for (int i=0;i<getNumRigidBodies();i++)
					{
						btRigidBody* body = getRigidBody (i);
						if (body->getName())
						{
							char name[512];
							snprintf(&name[0], 512, "%s-RigidBody", body->getName());
							if (!strcmp(name,orgUri0))
							{
								body0=body;
							}
							if (!strcmp(name,orgUri1))
							{
								body1=body;
							}
						}
					}



					const domRigid_constraint::domAttachmentRef attachOtherBody = rigidConstraintRef->getAttachment();

					
					const domRigid_constraint::domTechnique_commonRef commonRef = rigidConstraintRef->getTechnique_common();
					
					domFloat3 flMin = commonRef->getLimits()->getLinear()->getMin()->getValue();
					btVector3 minLinearLimit(flMin.get(0),flMin.get(1),flMin.get(2));
					
					domFloat3 flMax = commonRef->getLimits()->getLinear()->getMax()->getValue();
					btVector3 maxLinearLimit(flMax.get(0),flMax.get(1),flMax.get(2));
														
					domFloat3 coneMinLimit = commonRef->getLimits()->getSwing_cone_and_twist()->getMin()->getValue();
					btVector3 angularMin(coneMinLimit.get(0),coneMinLimit.get(1),coneMinLimit.get(2));

					domFloat3 coneMaxLimit = commonRef->getLimits()->getSwing_cone_and_twist()->getMax()->getValue();
					btVector3 angularMax(coneMaxLimit.get(0),coneMaxLimit.get(1),coneMaxLimit.get(2));

					{
						
						btTransform attachFrameRef0;
						attachFrameRef0.setIdentity();

						if (attachRefBody)
						{
							attachFrameRef0 = 
								GetbtTransformFromCOLLADA_DOM
								(
								emptyMatrixArray,
								attachRefBody->getRotate_array(),
								attachRefBody->getTranslate_array(),
								m_unitMeterScaling);
						}

						btTransform attachFrameOther;
						attachFrameOther.setIdentity();
						if (attachBody1)
						{
							attachFrameOther =
								GetbtTransformFromCOLLADA_DOM
								(
								emptyMatrixArray,
								attachBody1->getRotate_array(),
								attachBody1->getTranslate_array(),
								m_unitMeterScaling
								);
						}

						domBool interpenetrate = false;
						if (commonRef->getInterpenetrate())
							interpenetrate = commonRef->getInterpenetrate()->getValue();
						bool disableCollisionsBetweenLinkedBodies = interpenetrate;
						//convert INF / -INF into lower > upper

						//currently there is a hack in the DOM to detect INF / -INF
						//see daeMetaAttribute.cpp
						//INF -> 999999.9
						//-INF -> -999999.9
						float linearCheckThreshold = 999999.0;
						float angularCheckThreshold = 180.0;//check this



						
						//free means upper < lower, 
						//locked means upper == lower
						//limited means upper > lower
						//limitIndex: first 3 are linear, next 3 are angular

						btVector3 linearLowerLimits = minLinearLimit;
						btVector3 linearUpperLimits = maxLinearLimit;
						btVector3 angularLowerLimits = angularMin;
						btVector3 angularUpperLimits = angularMax;
						{
							for (int i=0;i<3;i++)
							{
								if  ((linearLowerLimits[i] < -linearCheckThreshold) ||
									(linearUpperLimits[i] > linearCheckThreshold))
								{
									//disable limits
									linearLowerLimits[i] = 1;
									linearUpperLimits[i] = 0;
								}

								if  ((angularLowerLimits[i] < -angularCheckThreshold) ||
									(angularUpperLimits[i] > angularCheckThreshold))
								{
									//disable limits
									angularLowerLimits[i] = 1;
									angularUpperLimits[i] = 0;
								}
							}
						}


						if (body0 || body1)
						{
							//swap so that first body is non-zero
							btTypedConstraint* constraint = NULL;
							if (!body0)
							{
								constraint = createUniversalD6Constraint(
								body1,
								body0,
								attachFrameOther,
								attachFrameRef0,
								linearLowerLimits,
								linearUpperLimits,
								angularLowerLimits,
								angularUpperLimits,
								disableCollisionsBetweenLinkedBodies
									);
							} else
							{
								constraint = createUniversalD6Constraint(
								body0,
								body1,
								attachFrameRef0,
								attachFrameOther,
								linearLowerLimits,
								linearUpperLimits,
								angularLowerLimits,
								angularUpperLimits,
								disableCollisionsBetweenLinkedBodies
									);
							}

							// XXX: User must free this name before destroy the constraint
							constraint->setName (strdup(rigidConstraintRef->getSid()));
							printf("Added constraint %s to the world\n", rigidConstraintRef->getSid());
						} else
						{
							printf("Error: Cannot find Rigidbodies(%s,%s) for constraint %s\n",orgUri0,orgUri1,constraintName);
						}


					}



				}
			}
		}

	}

}

void	ColladaConverter::PreparePhysicsObject(struct btRigidBodyInput& input, bool isDynamics, float mass,btCollisionShape* colShape, btVector3 linearVelocity, btVector3 angularVelocity)
{
	btTransform startTransform;
	startTransform.setIdentity();
	btVector3 startScale(1.f,1.f,1.f);

	//The 'target' points to a graphics element/node, which contains the start (world) transform
	daeElementRef elem = input.m_instanceRigidBodyRef->getTarget().getElement();

	xsNCName bodyName;
	if (elem)
	{
		domNodeRef node = *(domNodeRef*)&elem;
		bodyName = node->getName();

		//find transform of the node that this rigidbody maps to


		startTransform = GetbtTransformFromCOLLADA_DOM(
							node->getMatrix_array(),
							node->getRotate_array(),
							node->getTranslate_array(),
							m_unitMeterScaling
							);

		unsigned int i;
		for (i=0;i<node->getScale_array().getCount();i++)
		{
			domScaleRef scaleRef = node->getScale_array()[i];
			domFloat3 fl3 = scaleRef->getValue();
			startScale = btVector3(fl3.get(0),fl3.get(1),fl3.get(2));
		}

	}

	if (startScale.length() < 0.001)
		printf("invalid scale\n");

	colShape->setLocalScaling(startScale);

	btRigidBody* body= createRigidBody(isDynamics,mass,startTransform,colShape);
	if (body)
	{
		if (body->getInvMass() != 0.0)
		{
			body->setLinearVelocity (linearVelocity);
			body->setAngularVelocity (angularVelocity);
		}
		body->setName (strdup(bodyName));
		printf("node = %s\n", body->getName());
		printf("shape = %s\n", colShape->getShapeName());
	}

}

void
ColladaConverter::addConvexHull (btCollisionShape* shape, const char* nodeName, domLibrary_geometries* geomLib)
{
	btConvexHullShape* hullShape = (btConvexHullShape*)shape;
	btShapeHull* triHull = new btShapeHull (hullShape);
	if (triHull->buildHull (0.0) == false)
	{
		printf("Failed to build triangle mesh of hull\n");
		return;
	}

	domGeometry *geo = daeSafeCast<domGeometry>( geomLib->createAndPlace( COLLADA_ELEMENT_GEOMETRY ) );
	if ( geo == NULL )
	{
		printf("Failed to create the geometry element\n");
		return;
	}
	//set it's id
	geo->setId( nodeName );
	geo->setName ( nodeName);

	domConvex_mesh *convexMesh = daeSafeCast<domConvex_mesh>( geo->createAndPlace( COLLADA_ELEMENT_CONVEX_MESH ) );
	if ( convexMesh == NULL )
	{
		printf("Failed to create the mesh element\n");
		return;
	}

	//we will need 3 sources for this mesh. positions, normals, and UVs
	domSource *positionSrc = daeSafeCast<domSource>( convexMesh->createAndPlace( COLLADA_ELEMENT_SOURCE ) );
	if (!positionSrc)
	{
		printf("Failed to create position source\n");
		return;
	}

	//create the positions source.
	std::string srcName = std::string(nodeName) + std::string("-positions");
	positionSrc->setId( srcName.c_str() );
	domFloat_array *fa = daeSafeCast<domFloat_array>( positionSrc->createAndPlace( COLLADA_ELEMENT_FLOAT_ARRAY ) );
	if (fa == NULL)
	{
		printf("Failed to create float array\n");
		return;
	}
	std::string arrayName = srcName + std::string("-array");
	fa->setId( arrayName.c_str() );
	fa->setCount( triHull->numVertices () * 3);
	
	domListOfFloats &posSrcArray = fa->getValue();
	for (int i = 0; i < triHull->numVertices (); i++)
	{
		btVector3 p = triHull->getVertexPointer()[i];
		posSrcArray.append3(p.getX(), p.getY(), p.getZ());
	}

	//create the accessor
	domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>( positionSrc->createAndPlace( COLLADA_ELEMENT_TECHNIQUE_COMMON ) );
	domAccessor *acc = daeSafeCast<domAccessor>( srcTeqC->createAndPlace( COLLADA_ELEMENT_ACCESSOR ) );
	acc->setCount( triHull->numVertices () );
	acc->setStride( 3 );
	daeURI uri;
	uri.setElement( fa );
	uri.resolveURI();
	acc->setSource( uri );

	domParam *param = daeSafeCast<domParam>( acc->createAndPlace( COLLADA_ELEMENT_PARAM ) );
	param->setName( "X" );
	param->setType( "float" );
	param = daeSafeCast<domParam>( acc->createAndPlace( COLLADA_ELEMENT_PARAM ) );
	param->setName( "Y" );
	param->setType( "float" );
	param = daeSafeCast<domParam>( acc->createAndPlace( COLLADA_ELEMENT_PARAM ) );
	param->setName( "Z" );
	param->setType( "float" );

	domVertices *verts = daeSafeCast<domVertices>( convexMesh->createAndPlace( COLLADA_ELEMENT_VERTICES ) );
	srcName = std::string(nodeName) + std::string("-vertices");
	verts->setId( srcName.c_str() );
	domInputLocal *inputLocal = daeSafeCast<domInputLocal>( verts->createAndPlace( COLLADA_ELEMENT_INPUT ) );
	inputLocal->setSemantic( COMMON_PROFILE_INPUT_POSITION );
	uri.setElement( positionSrc );
	uri.resolveURI();
	inputLocal->setSource( uri );

	domTriangles *tris = daeSafeCast<domTriangles>( convexMesh->createAndPlace( COLLADA_ELEMENT_TRIANGLES ) );
	tris->setCount( triHull->numTriangles() );
	domInputLocalOffset *ilo = daeSafeCast<domInputLocalOffset>( tris->createAndPlace( COLLADA_ELEMENT_INPUT ) );
	ilo->setSemantic( COMMON_PROFILE_INPUT_VERTEX );
	ilo->setOffset( 0 );
	uri.setElement( verts );
	uri.resolveURI();
	ilo->setSource( uri );

	domP *p = daeSafeCast<domP>( tris->createAndPlace( COLLADA_ELEMENT_P ) );

	domListOfUInts &indices = p->getValue();
	//each set of three is one number per input-offset. for this example it's vert, normal, uv.
	//three sets of three indices per triangle

	const unsigned int* indexBase = triHull->getIndexPointer ();
	for (int t = 0; t < triHull->numTriangles(); t++)
	{
		int* index = (int*)indexBase;
		indices.append3( index[0], index[1], index[2]);
		indexBase += 3;
	}

	delete triHull;
}

void
ColladaConverter::addConvexMesh (btCollisionShape* shape, const char* nodeName, domLibrary_geometries* geomLib)
{
	printf("convex Triangle Mesh Shape\n");
	printf("ERROR: Unsupported.\n");
}

void
ColladaConverter::addConcaveMesh(btCollisionShape* shape, const char* nodeName, domLibrary_geometries* geomLib)
{
	btTriangleMeshShape* meshShape = (btTriangleMeshShape*)shape;
	btStridingMeshInterface* meshInterface = meshShape->getMeshInterface ();

	domGeometry *geo = daeSafeCast<domGeometry>( geomLib->createAndPlace( COLLADA_ELEMENT_GEOMETRY ) );
	if ( geo == NULL )
	{
		printf("Failed to create the geometry element\n");
		return;
	}
	//set it's id
	geo->setId( nodeName );
	geo->setName ( nodeName);

	for (int i = 0; i < meshInterface->getNumSubParts (); i++)
	{

		domMesh *mesh = daeSafeCast<domMesh>( geo->createAndPlace( COLLADA_ELEMENT_MESH ) );
		if ( mesh == NULL )
		{
			printf("Failed to create the mesh element\n");
			return;
		}

		const unsigned char* vertexBase = NULL;
		int numVerts;
		PHY_ScalarType vertexType;
		int vertexStride;
		const unsigned char* indexBase = NULL;
		int indexStride;
		int numFaces;
		PHY_ScalarType indexType;

		meshInterface->getLockedReadOnlyVertexIndexBase (&vertexBase, numVerts, vertexType, vertexStride, &indexBase, indexStride, numFaces, indexType, i);

		btAssert (vertexBase);
		btAssert (indexBase);
		btAssert (vertexType == PHY_FLOAT);
		btAssert (indexType == PHY_INTEGER);

		//we will need 3 sources for this mesh. positions, normals, and UVs
		domSource *positionSrc = daeSafeCast<domSource>( mesh->createAndPlace( COLLADA_ELEMENT_SOURCE ) );

		//create the positions source.
		std::string srcName = std::string(nodeName) + std::string("-position");
		positionSrc->setId( srcName.c_str() );
		domFloat_array *fa = daeSafeCast<domFloat_array>( positionSrc->createAndPlace( COLLADA_ELEMENT_FLOAT_ARRAY ) );
		std::string arrayName = srcName + std::string("-array");
		fa->setId( arrayName.c_str() );
		fa->setCount( numVerts * 3);
		
		domListOfFloats &posSrcArray = fa->getValue();
		for (int v = 0; v < numVerts; v++)
		{
			float* p = (float*)vertexBase;
			posSrcArray.append3(p[0], p[1], p[2]);
			vertexBase += vertexStride;
		}

		//create the accessor
		domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>( positionSrc->createAndPlace( COLLADA_ELEMENT_TECHNIQUE_COMMON ) );
		domAccessor *acc = daeSafeCast<domAccessor>( srcTeqC->createAndPlace( COLLADA_ELEMENT_ACCESSOR ) );
		acc->setCount( numVerts );
		acc->setStride( 3 );
		daeURI uri;
		uri.setElement( fa );
		uri.resolveURI();
		acc->setSource( uri );

		domParam *param = daeSafeCast<domParam>( acc->createAndPlace( COLLADA_ELEMENT_PARAM ) );
		param->setName( "X" );
		param->setType( "float" );
		param = daeSafeCast<domParam>( acc->createAndPlace( COLLADA_ELEMENT_PARAM ) );
		param->setName( "Y" );
		param->setType( "float" );
		param = daeSafeCast<domParam>( acc->createAndPlace( COLLADA_ELEMENT_PARAM ) );
		param->setName( "Z" );
		param->setType( "float" );

		domVertices *verts = daeSafeCast<domVertices>( mesh->createAndPlace( COLLADA_ELEMENT_VERTICES ) );
		srcName = std::string(nodeName) + std::string("-vertex");
		verts->setId( srcName.c_str() );
		domInputLocal *inputLocal = daeSafeCast<domInputLocal>( verts->createAndPlace( COLLADA_ELEMENT_INPUT ) );
		inputLocal->setSemantic( COMMON_PROFILE_INPUT_POSITION );
		uri.setElement( positionSrc );
		uri.resolveURI();
		inputLocal->setSource( uri );


		domTriangles *tris = daeSafeCast<domTriangles>( mesh->createAndPlace( COLLADA_ELEMENT_TRIANGLES ) );
		tris->setCount( numFaces );
		domInputLocalOffset *ilo = daeSafeCast<domInputLocalOffset>( tris->createAndPlace( COLLADA_ELEMENT_INPUT ) );
		ilo->setSemantic( COMMON_PROFILE_INPUT_VERTEX );
		ilo->setOffset( 0 );
		uri.setElement( verts );
		uri.resolveURI();
		ilo->setSource( uri );

		domP *p = daeSafeCast<domP>( tris->createAndPlace( COLLADA_ELEMENT_P ) );

		domListOfUInts &indices = p->getValue();
		//each set of three is one number per input-offset. for this example it's vert, normal, uv.
		//three sets of three indices per triangle

		for (int t = 0; t < numFaces; t++)
		{
			int* index = (int*)indexBase;
			indices.append3( index[0], index[1], index[2]);
			indexBase += indexStride;
		}


		meshInterface->unLockReadOnlyVertexBase (i);
	}
}

void ColladaConverter::buildShape (btCollisionShape* shape, void* collada_shape, const char* shapeName, domLibrary_geometries* geomLib)
{
	domRigid_body::domTechnique_common::domShape* colladaShape = (domRigid_body::domTechnique_common::domShape*)collada_shape;
	switch (shape->getShapeType())
	{
	case BOX_SHAPE_PROXYTYPE:
	{
		btBoxShape* bs = (btBoxShape*)shape;
		btVector3 halfe = bs->getHalfExtentsWithMargin();
		domBox* box = (domBox*)colladaShape->createAndPlace (COLLADA_ELEMENT_BOX);
		domBox::domHalf_extents* he = (domBox::domHalf_extents*)box->createAndPlace (COLLADA_ELEMENT_HALF_EXTENTS);
		he->getValue().set3 (halfe[0], halfe[1], halfe[2]);
	}
	break;
	case SPHERE_SHAPE_PROXYTYPE:
	{
		btSphereShape* ss = (btSphereShape*)shape;
		domSphere* sphere = (domSphere*)colladaShape->createAndPlace (COLLADA_ELEMENT_SPHERE);
		domSphere::domRadius* radius = (domSphere::domRadius*)sphere->createAndPlace (COLLADA_ELEMENT_RADIUS);
		radius->setValue (ss->getRadius());
	}
	break;
	case CYLINDER_SHAPE_PROXYTYPE:
	{
		btCylinderShape* cs = (btCylinderShape*)shape;
		domCylinder* cylinder = (domCylinder*)colladaShape->createAndPlace (COLLADA_ELEMENT_CYLINDER);
		domCylinder::domRadius* radius = (domCylinder::domRadius*)cylinder->createAndPlace (COLLADA_ELEMENT_RADIUS);
		domCylinder::domHeight* height = (domCylinder::domHeight*)cylinder->createAndPlace (COLLADA_ELEMENT_HEIGHT);
		radius->setValue (cs->getRadius());
		height->setValue (cs->getHalfExtentsWithMargin()[1] * 1.0);
	}
	break;
	case STATIC_PLANE_PROXYTYPE:
	{
		btStaticPlaneShape* ps = (btStaticPlaneShape*)shape;
		btVector3 n = ps->getPlaneNormal ();
		btScalar d = ps->getPlaneConstant ();
		domPlane* plane = (domPlane*)colladaShape->createAndPlace (COLLADA_ELEMENT_PLANE);
		domPlane::domEquation* equation = (domPlane::domEquation*)plane->createAndPlace (COLLADA_ELEMENT_EQUATION);
		equation->getValue().set4 (n[0], n[1], n[2], d);
	}
	break;
	case CONE_SHAPE_PROXYTYPE:
	{
		printf("unhandled cone type\n");
	}
	break;
	case COMPOUND_SHAPE_PROXYTYPE:
	{
		btCompoundShape* cs = (btCompoundShape*)shape;
		for (int i = 0; i < cs->getNumChildShapes (); i++)
		{
			btTransform xform = cs->getChildTransform (i);
			{
				domTranslate* translation = (domTranslate*)colladaShape->createAndPlace (COLLADA_ELEMENT_TRANSLATE);
				{
					btVector3 np = xform.getOrigin();
					translation->getValue().append(np[0]);
					translation->getValue().append(np[1]);
					translation->getValue().append(np[2]);
				}
				domRotate* rotation = (domRotate*)colladaShape->createAndPlace (COLLADA_ELEMENT_ROTATE);
				{
					btQuaternion quat = xform.getRotation();
					btVector3 axis(quat.getX(),quat.getY(),quat.getZ());
					axis[3] = 0.f;
					//check for axis length
					btScalar len = axis.length2();
					if (len < SIMD_EPSILON*SIMD_EPSILON)
						axis = btVector3(1.f,0.f,0.f);
					else
						axis /= btSqrt(len);
					rotation->getValue().set(0,axis[0]);
					rotation->getValue().set(1,axis[1]);
					rotation->getValue().set(2,axis[2]);
					rotation->getValue().set(3,quat.getAngle()*SIMD_DEGS_PER_RAD);
				}
			}
			btCollisionShape* child_shape = cs->getChildShape (i);
			buildShape (child_shape, colladaShape, shapeName, geomLib);
		}
	}
	break;
	case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
	{
		addConvexMesh (shape, shapeName, geomLib);
		char shapeURL[512];
		snprintf(&shapeURL[0], 512, "#%s", shapeName);
		domInstance_geometry* gi = (domInstance_geometry*)colladaShape->createAndPlace (COLLADA_ELEMENT_INSTANCE_GEOMETRY);
		gi->setUrl (shapeURL);
	}
	break;
	case CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		addConvexHull (shape, shapeName, geomLib);
		char shapeURL[512];
		snprintf(&shapeURL[0], 512, "#%s", shapeName);
		domInstance_geometry* gi = (domInstance_geometry*)colladaShape->createAndPlace (COLLADA_ELEMENT_INSTANCE_GEOMETRY);
		gi->setUrl (shapeURL);
	}
	break;
	case TRIANGLE_MESH_SHAPE_PROXYTYPE:
	{
		addConcaveMesh (shape, shapeName, geomLib);
		char shapeURL[512];
		snprintf(&shapeURL[0], 512, "#%s", shapeName);
		domInstance_geometry* gi = (domInstance_geometry*)colladaShape->createAndPlace (COLLADA_ELEMENT_INSTANCE_GEOMETRY);
		gi->setUrl (shapeURL);
	}
	break;
	default:
		printf("Unhandled %d\n", shape->getShapeType ());
	break;
	}
}

void
ColladaConverter::addRigidBody (btRigidBody* rb, const char* nodeName, const char* shapeName, domPhysics_model* physicsModel, domLibrary_geometries* geomLib)
{
	btCollisionShape* shape = rb->getCollisionShape ();
	char modelName[512];
	char bodyName[512];
	char material_name[512];
	snprintf(&bodyName[0], 512, "%s-RigidBody", nodeName);
	snprintf(&modelName[0], 512, "%s-PhysicsModel", nodeName);
	physicsModel->setId (modelName);
	physicsModel->setName (modelName);
	domRigid_body* colladaRigidBody = daeSafeCast<domRigid_body>(physicsModel->createAndPlace (COLLADA_ELEMENT_RIGID_BODY));
	colladaRigidBody->setSid (bodyName);
	colladaRigidBody->setName (bodyName);
	domRigid_body::domTechnique_common* common = daeSafeCast<domRigid_body::domTechnique_common>(colladaRigidBody->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON));
	domRigid_body::domTechnique_common::domDynamic* dynamic = daeSafeCast<domRigid_body::domTechnique_common::domDynamic>(common->createAndPlace (COLLADA_ELEMENT_DYNAMIC));
	domTargetableFloat* mass = daeSafeCast<domTargetableFloat>(common->createAndPlace (COLLADA_ELEMENT_MASS));
	domTargetableFloat3* inertia = daeSafeCast<domTargetableFloat3>(common->createAndPlace (COLLADA_ELEMENT_INERTIA));
	if (rb->getInvMass() == 0.0)
	{
		domFloat3 inertia_value;
		inertia_value.append (0.0);
		inertia_value.append (0.0);
		inertia_value.append (0.0);
		mass->setValue (0.0);
		dynamic->setValue (false);
		inertia->setValue (inertia_value);
	} else {
		btVector3 II = rb->getInvInertiaDiagLocal ();
		domFloat3 inertia_value;
		inertia_value.append (II[0] == 0.0 ? 0.0 : 1.0 / II[0]);
		inertia_value.append (II[1] == 0.0 ? 0.0 : 1.0 / II[1]);
		inertia_value.append (II[2] == 0.0 ? 0.0 : 1.0 / II[2]);
		mass->setValue (1.0/rb->getInvMass());
		dynamic->setValue (true);
		inertia->setValue (inertia_value);
	}
	//domRigid_body::domTechnique_common::domMass_frame* massFrame = daeSafeCast<domRigid_body::domTechnique_common::domMass_frame>(common->createAndPlace (COLLADA_ELEMENT_MASS_FRAME));
	
	// physics material
	domInstance_physics_material* mi = (domInstance_physics_material*)common->createAndPlace (COLLADA_ELEMENT_INSTANCE_PHYSICS_MATERIAL);
	snprintf(&material_name[0], 512, "#%s-PhysicsMaterial", nodeName);
	mi->setUrl (material_name);
	// collision shape

	domRigid_body::domTechnique_common::domShape* colladaShape = (domRigid_body::domTechnique_common::domShape*)common->createAndPlace (COLLADA_ELEMENT_SHAPE);	
	buildShape (shape, colladaShape, shapeName, geomLib);
}

void ColladaConverter::addNode (btRigidBody* rb, const char* nodeName, const char* shapeName, domVisual_scene* vscene)
{
	domNode* node = (domNode*)vscene->createAndPlace (COLLADA_ELEMENT_NODE);
	node->setId (nodeName);
	node->setName (nodeName);

	domTranslate* translation = (domTranslate*)node->createAndPlace (COLLADA_ELEMENT_TRANSLATE);
	{
		btVector3 np = rb->getWorldTransform().getOrigin();
		translation->getValue().append(np[0]);
		translation->getValue().append(np[1]);
		translation->getValue().append(np[2]);
	}
	domRotate* rotation = (domRotate*)node->createAndPlace (COLLADA_ELEMENT_ROTATE);
	{
		btQuaternion quat = rb->getWorldTransform().getRotation();
		btVector3 axis(quat.getX(),quat.getY(),quat.getZ());
		axis[3] = 0.f;
		//check for axis length
		btScalar len = axis.length2();
		if (len < SIMD_EPSILON*SIMD_EPSILON)
			axis = btVector3(1.f,0.f,0.f);
		else
			axis /= btSqrt(len);
		rotation->getValue().set(0,axis[0]);
		rotation->getValue().set(1,axis[1]);
		rotation->getValue().set(2,axis[2]);
		rotation->getValue().set(3,quat.getAngle()*SIMD_DEGS_PER_RAD);
	}

}

void ColladaConverter::addConstraint (btTypedConstraint* constraint, class domPhysics_model* physicsModel)
{
	btGeneric6DofConstraint* g6c = (btGeneric6DofConstraint*)constraint;
	const btRigidBody& rb1 = g6c->getRigidBodyA ();
	const btRigidBody& rb2 = g6c->getRigidBodyB ();
	bool single = rb2.getName() == NULL;

	domRigid_constraint* domRigidConstraint = (domRigid_constraint*)physicsModel->createAndPlace (COLLADA_ELEMENT_RIGID_CONSTRAINT);
	domRigidConstraint->setName (daeStringRef(constraint->getName()));
	domRigidConstraint->setSid (daeStringRef(constraint->getName()));
	if (single)
	{
		char name[512];
		const char* rb1Name = rb1.getName ();
		snprintf(&name[0], 512, "%s-RigidBody", rb1Name);
		btTransform rb1Frame = g6c->getFrameOffsetA ();
		printf("Joint with single body\n");
		domRigid_constraint::domAttachment* attachment = (domRigid_constraint::domAttachment*)domRigidConstraint->createAndPlace (COLLADA_ELEMENT_ATTACHMENT);
		attachment->setRigid_body (name);
		{
			domTranslate* translation = (domTranslate*)attachment->createAndPlace (COLLADA_ELEMENT_TRANSLATE);
			{
				btVector3 np = rb1Frame.getOrigin();
				translation->getValue().append(np[0]);
				translation->getValue().append(np[1]);
				translation->getValue().append(np[2]);
			}
			domRotate* rotation = (domRotate*)attachment->createAndPlace (COLLADA_ELEMENT_ROTATE);
			{
				btQuaternion quat = rb1Frame.getRotation();
				btVector3 axis(quat.getX(),quat.getY(),quat.getZ());
				axis[3] = 0.f;
				//check for axis length
				btScalar len = axis.length2();
				if (len < SIMD_EPSILON*SIMD_EPSILON)
					axis = btVector3(1.f,0.f,0.f);
				else
					axis /= btSqrt(len);
				rotation->getValue().set(0,axis[0]);
				rotation->getValue().set(1,axis[1]);
				rotation->getValue().set(2,axis[2]);
				rotation->getValue().set(3,quat.getAngle()*SIMD_DEGS_PER_RAD);
			}

		}
	} else {
		const char* rb1Name = rb1.getName ();
		const char* rb2Name = rb2.getName ();
		btTransform rb1Frame = g6c->getFrameOffsetA ();
		btTransform rb2Frame = g6c->getFrameOffsetB ();
		char name[512];
		snprintf(&name[0],512, "%s-RigidBody", rb1Name);
		printf("Joint attached to two bodies\n");
		domRigid_constraint::domRef_attachment* refAttachment = (domRigid_constraint::domRef_attachment*)domRigidConstraint->createAndPlace (COLLADA_ELEMENT_REF_ATTACHMENT);
		domRigid_constraint::domAttachment* attachment = (domRigid_constraint::domAttachment*)domRigidConstraint->createAndPlace (COLLADA_ELEMENT_ATTACHMENT);
		refAttachment->setRigid_body (name);
		snprintf(&name[0],512, "%s-RigidBody", rb2Name);
		attachment->setRigid_body (name);
		{
			domTranslate* translation = (domTranslate*)refAttachment->createAndPlace (COLLADA_ELEMENT_TRANSLATE);
			{
				btVector3 np = rb1Frame.getOrigin();
				translation->getValue().append(np[0]);
				translation->getValue().append(np[1]);
				translation->getValue().append(np[2]);
			}
			domRotate* rotation = (domRotate*)refAttachment->createAndPlace (COLLADA_ELEMENT_ROTATE);
			{
				btQuaternion quat = rb1Frame.getRotation();
				btVector3 axis(quat.getX(),quat.getY(),quat.getZ());
				axis[3] = 0.f;
				//check for axis length
				btScalar len = axis.length2();
				if (len < SIMD_EPSILON*SIMD_EPSILON)
					axis = btVector3(1.f,0.f,0.f);
				else
					axis /= btSqrt(len);
				rotation->getValue().set(0,axis[0]);
				rotation->getValue().set(1,axis[1]);
				rotation->getValue().set(2,axis[2]);
				rotation->getValue().set(3,quat.getAngle()*SIMD_DEGS_PER_RAD);
			}

		}
		{
			domTranslate* translation = (domTranslate*)attachment->createAndPlace (COLLADA_ELEMENT_TRANSLATE);
			{
				btVector3 np = rb2Frame.getOrigin();
				translation->getValue().append(np[0]);
				translation->getValue().append(np[1]);
				translation->getValue().append(np[2]);
			}
			domRotate* rotation = (domRotate*)attachment->createAndPlace (COLLADA_ELEMENT_ROTATE);
			{
				btQuaternion quat = rb2Frame.getRotation();
				btVector3 axis(quat.getX(),quat.getY(),quat.getZ());
				axis[3] = 0.f;
				//check for axis length
				btScalar len = axis.length2();
				if (len < SIMD_EPSILON*SIMD_EPSILON)
					axis = btVector3(1.f,0.f,0.f);
				else
					axis /= btSqrt(len);
				rotation->getValue().set(0,axis[0]);
				rotation->getValue().set(1,axis[1]);
				rotation->getValue().set(2,axis[2]);
				rotation->getValue().set(3,quat.getAngle()*SIMD_DEGS_PER_RAD);
			}
		}
	}
	domRigid_constraint::domTechnique_common* techniqueCommon = (domRigid_constraint::domTechnique_common*)domRigidConstraint->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON);
	domRigid_constraint::domTechnique_common::domEnabled* enabled = (domRigid_constraint::domTechnique_common::domEnabled*)techniqueCommon->createAndPlace (COLLADA_ELEMENT_ENABLED);
	enabled->setValue (true);
	domRigid_constraint::domTechnique_common::domInterpenetrate* interpenetrate = (domRigid_constraint::domTechnique_common::domInterpenetrate*)techniqueCommon->createAndPlace (COLLADA_ELEMENT_INTERPENETRATE);
	interpenetrate->setValue (false);
	domRigid_constraint::domTechnique_common::domLimits* limits = (domRigid_constraint::domTechnique_common::domLimits*)techniqueCommon->createAndPlace (COLLADA_ELEMENT_LIMITS);
	domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist* swingConeAndTwist = (domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist*)limits->createAndPlace (COLLADA_ELEMENT_SWING_CONE_AND_TWIST);
	domRigid_constraint::domTechnique_common::domLimits::domLinear* linear = (domRigid_constraint::domTechnique_common::domLimits::domLinear*)limits->createAndPlace (COLLADA_ELEMENT_LINEAR);

	{
		domTargetableFloat3* min = (domTargetableFloat3*)swingConeAndTwist->createAndPlace (COLLADA_ELEMENT_MIN);
		domTargetableFloat3* max = (domTargetableFloat3*)swingConeAndTwist->createAndPlace (COLLADA_ELEMENT_MAX);
		btRotationalLimitMotor* limit = g6c->getRotationalLimitMotor (0);
		min->getValue().set(0, limit->m_loLimit);
		max->getValue().set(0, limit->m_hiLimit);
		limit = g6c->getRotationalLimitMotor (1);
		min->getValue().set(1, limit->m_loLimit);
		max->getValue().set(1, limit->m_hiLimit);
		limit = g6c->getRotationalLimitMotor (2);
		min->getValue().set(2, limit->m_loLimit);
		max->getValue().set(2, limit->m_hiLimit);
	}
	{
		domTargetableFloat3* min = (domTargetableFloat3*)linear->createAndPlace (COLLADA_ELEMENT_MIN);
		domTargetableFloat3* max = (domTargetableFloat3*)linear->createAndPlace (COLLADA_ELEMENT_MAX);
		btTranslationalLimitMotor* limit = g6c->getTranslationalLimitMotor ();
		min->getValue().set (0, limit->m_lowerLimit[0]);
		min->getValue().set (1, limit->m_lowerLimit[1]);
		min->getValue().set (2, limit->m_lowerLimit[2]);
		max->getValue().set (0, limit->m_upperLimit[0]);
		max->getValue().set (1, limit->m_upperLimit[1]);
		max->getValue().set (2, limit->m_upperLimit[2]);
	}
}

void ColladaConverter::addConstraintInstance (btTypedConstraint* constraint, domInstance_physics_model* mi)
{
	domInstance_rigid_constraint* rci = (domInstance_rigid_constraint*)mi->createAndPlace (COLLADA_ELEMENT_INSTANCE_RIGID_CONSTRAINT);
	rci->setConstraint (daeStringRef(constraint->getName()));
}


void ColladaConverter::addRigidBodyInstance (btRigidBody* rb, const char* nodeName, domInstance_physics_model* mi)
{
	char targetName[512];
	char bodyName[512];
	char modelName[512];
	snprintf(&modelName[0], 512, "#%s-PhysicsModel", nodeName);
	snprintf(&targetName[0], 512, "#%s", nodeName);
	snprintf(&bodyName[0], 512, "%s-RigidBody", nodeName);

	domInstance_rigid_body* rbi = (domInstance_rigid_body*)mi->createAndPlace (COLLADA_ELEMENT_INSTANCE_RIGID_BODY);
	domInstance_rigid_body::domTechnique_common* common = (domInstance_rigid_body::domTechnique_common*)rbi->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON);
	mi->setUrl (modelName);
	rbi->setBody (bodyName);
	rbi->setTarget (targetName);

	domInstance_rigid_body::domTechnique_common::domAngular_velocity* av = (domInstance_rigid_body::domTechnique_common::domAngular_velocity*)common->createAndPlace (COLLADA_ELEMENT_ANGULAR_VELOCITY);
	{
		btVector3 btAv = rb->getAngularVelocity ();
		av->getValue().set3 (btAv[0], btAv[1], btAv[2]);
	}
	domInstance_rigid_body::domTechnique_common::domVelocity* lv = (domInstance_rigid_body::domTechnique_common::domVelocity*)common->createAndPlace (COLLADA_ELEMENT_VELOCITY);
	{
		btVector3 btLv = rb->getLinearVelocity ();
		lv->getValue().set3 (btLv[0], btLv[1], btLv[2]);
	}
}

void ColladaConverter::addMaterial (btRigidBody* rb, const char* nodeName, domLibrary_physics_materials* materialsLib)
{
	btScalar friction = rb->getFriction ();
	btScalar restitution = rb->getRestitution ();
	domPhysics_material* material = (domPhysics_material*)materialsLib->createAndPlace (COLLADA_ELEMENT_PHYSICS_MATERIAL);
	char material_name[512];
	snprintf(&material_name[0], 512, "%s-PhysicsMaterial", nodeName);
	material->setName (material_name);
	material->setId (material_name);
	domPhysics_material::domTechnique_common* material_common = (domPhysics_material::domTechnique_common*)material->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON);
	domTargetableFloat* mf = (domTargetableFloat*)material_common->createAndPlace (COLLADA_ELEMENT_DYNAMIC_FRICTION);
	mf->setValue (friction);
	mf = (domTargetableFloat*)material_common->createAndPlace (COLLADA_ELEMENT_STATIC_FRICTION);
	mf->setValue (friction);
	mf = (domTargetableFloat*)material_common->createAndPlace (COLLADA_ELEMENT_RESTITUTION);
	mf->setValue (restitution);

}

bool ColladaConverter::save(const char* filename)
{
	m_collada = new DAE;
	daeInt error;
	const char* documentName = "bullet snapshot";

	//set the default IOPlugin and Database
	m_collada->setIOPlugin( NULL );
	m_collada->setDatabase( NULL );

	//create a new document. Calling daeDatabase::insertDocument will create the 
	//daeDocument for you. This function will also create a domCOLLADA root 
	//element for you.
	daeDocument *doc = NULL;
	error = m_collada->getDatabase()->insertDocument(documentName, &doc );
	if ( error != DAE_OK || doc == NULL )
	{
		printf("Failed to create new document\n");
		return false;
	}

	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>( doc->getDomRoot() );

	//create the required asset tag
	domAsset *asset = daeSafeCast<domAsset>( domRoot->createAndPlace( COLLADA_ELEMENT_ASSET ) );
	domAsset::domCreated *created = daeSafeCast<domAsset::domCreated>( asset->createAndPlace( COLLADA_ELEMENT_CREATED ) );
	created->setValue("2006-10-23T13:30:00Z");

	domAsset::domModified *modified = daeSafeCast<domAsset::domModified>( asset->createAndPlace( COLLADA_ELEMENT_MODIFIED ) );
	modified->setValue("2006-10-23T13:30:00Z");

	//create the library_* elements to store the COLLADA elements we will use
	domLibrary_geometries	 *geomLib    = daeSafeCast<domLibrary_geometries>( domRoot->createAndPlace( COLLADA_ELEMENT_LIBRARY_GEOMETRIES ) );
	domLibrary_visual_scenes* vscenesLib = daeSafeCast<domLibrary_visual_scenes> (domRoot->createAndPlace (COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES));
	domLibrary_physics_materials* materialsLib = daeSafeCast<domLibrary_physics_materials> (domRoot->createAndPlace (COLLADA_ELEMENT_LIBRARY_PHYSICS_MATERIALS ));
	domLibrary_physics_models* modelsLib = daeSafeCast<domLibrary_physics_models> (domRoot->createAndPlace (COLLADA_ELEMENT_LIBRARY_PHYSICS_MODELS ));
	domLibrary_physics_scenes* scenesLib = daeSafeCast<domLibrary_physics_scenes> (domRoot->createAndPlace (COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES ));


	domPhysics_model* physicsModel = daeSafeCast<domPhysics_model>(modelsLib->createAndPlace (COLLADA_ELEMENT_PHYSICS_MODEL));
	domPhysics_scene* scene = (domPhysics_scene*)scenesLib->createAndPlace (COLLADA_ELEMENT_PHYSICS_SCENE);
	scene->setId("Scene-Physics");
	scene->setName("Scene-Physics");
	domVisual_scene* vscene = (domVisual_scene*)vscenesLib->createAndPlace (COLLADA_ELEMENT_VISUAL_SCENE);
	vscene->setId("Scene-Visuals");
	vscene->setName("Scene-Visuals");

	domInstance_physics_model* mi = (domInstance_physics_model*)scene->createAndPlace (COLLADA_ELEMENT_INSTANCE_PHYSICS_MODEL);

	{
		domPhysics_scene::domTechnique_common* common = (domPhysics_scene::domTechnique_common*)scene->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON);
		domTargetableFloat3* g = (domTargetableFloat3*)common->createAndPlace (COLLADA_ELEMENT_GRAVITY);
		btVector3 btG = getGravity ();
		g->getValue().set3 (btG[0], btG[1], btG[2]);
	}

	int random_node_name_key = 0;
	/* Dump the scene */
	for (int i = 0; i < getNumRigidBodies (); i++)
	{
		const char* shapeName = NULL;
		const char* nodeName = NULL;
		char nodeNameGen[512];
		char shapeNameGen[512];


		btRigidBody* rb = getRigidBody (i);
		btCollisionShape* shape = rb->getCollisionShape ();

		nodeName = rb->getName ();
		shapeName = shape->getShapeName ();

		if (!nodeName)
		{
			snprintf(&nodeNameGen[0], 512, "BulletUnnamed-%d", random_node_name_key++);
			nodeName = &nodeNameGen[0];
		}
		if (!shapeName)
		{
			snprintf(&shapeNameGen[0], 512, "%s-Geometry", nodeName);
			shapeName = &shapeNameGen[0];
		}

		if (shape->getShapeType () == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
			addConcaveMesh (shape, shapeName, geomLib);
		} else if (!shape->isConvex () && !shape->isCompound()) {
			printf("Unknown shape type. %d Skipping rigidbody.\n", shape->getShapeType());
			continue;
		}


		printf("Adding %s to COLLADA DOM.\n", nodeName);

		addNode (rb, nodeName, shapeName, vscene);
		addMaterial (rb, nodeName, materialsLib);
		addRigidBody (rb, nodeName, shapeName, physicsModel, geomLib);
		addRigidBodyInstance (rb, nodeName, mi);
	}

	for (int i = 0; i < getNumConstraints (); i++)
	{
		btTypedConstraint* constraint = getConstraint (i);
		addConstraint (constraint, physicsModel);
		addConstraintInstance (constraint, mi);
	}

	{
		char	saveName[550];
		static int saveCount=1;
		sprintf(saveName,"%s%i",getLastFileName(),saveCount++);
		char* name = &saveName[0];
		if (name[0] == '/')
		{
			name = &saveName[1];
		} 

		m_collada->saveAs (name);
	}
#if 0
	if (m_collada)
		{
			for (int i=0;i<m_numObjects;i++)
			{
				btAssert(m_colladadomNodes[i]);
				if (!m_colladadomNodes[i]->getTranslate_array().getCount())
				{
					domTranslate* transl = (domTranslate*) m_colladadomNodes[i]->createAndPlace("translate");
					transl->getValue().append(0.);
					transl->getValue().append(0.);
					transl->getValue().append(0.);
				}

				while (m_colladadomNodes[i]->getTranslate_array().getCount() > 1)
				{
					m_colladadomNodes[i]->removeFromParent(m_colladadomNodes[i]->getTranslate_array().get(1));
					//m_colladadomNodes[i]->getTranslate_array().removeIndex(1);
				}

				{

					btVector3 np = m_rigidBodies[i]->getWorldTransform().getOrigin();
					domFloat3 newPos = m_colladadomNodes[i]->getTranslate_array().get(0)->getValue();
					newPos.set(0,np[0]);
					newPos.set(1,np[1]);
					newPos.set(2,np[2]);
					m_colladadomNodes[i]->getTranslate_array().get(0)->setValue(newPos);

				}
				

				if (!m_colladadomNodes[i]->getRotate_array().getCount())
				{
					domRotate* rot = (domRotate*)m_colladadomNodes[i]->createAndPlace("rotate");
					rot->getValue().append(1.0);
					rot->getValue().append(0.0);
					rot->getValue().append(0.0);
					rot->getValue().append(0.0);
				}

				while (m_colladadomNodes[i]->getRotate_array().getCount()>1)
				{
					m_colladadomNodes[i]->removeFromParent(m_colladadomNodes[i]->getRotate_array().get(1));
					//m_colladadomNodes[i]->getRotate_array().removeIndex(1);

				}

				{
					btQuaternion quat = m_rigidBodies[i]->getCenterOfMassTransform().getRotation();
					btVector3 axis(quat.getX(),quat.getY(),quat.getZ());
					axis[3] = 0.f;
					//check for axis length
					btScalar len = axis.length2();
					if (len < SIMD_EPSILON*SIMD_EPSILON)
						axis = btVector3(1.f,0.f,0.f);
					else
						axis /= btSqrt(len);
					m_colladadomNodes[i]->getRotate_array().get(0)->getValue().set(0,axis[0]);
					m_colladadomNodes[i]->getRotate_array().get(0)->getValue().set(1,axis[1]);
					m_colladadomNodes[i]->getRotate_array().get(0)->getValue().set(2,axis[2]);
					m_colladadomNodes[i]->getRotate_array().get(0)->getValue().set(3,quat.getAngle()*SIMD_DEGS_PER_RAD);
				}

				while (m_colladadomNodes[i]->getMatrix_array().getCount())
				{
					m_colladadomNodes[i]->removeFromParent(m_colladadomNodes[i]->getMatrix_array().get(0));
					//m_colladadomNodes[i]->getMatrix_array().removeIndex(0);
				}
			}
			char	saveName[550];
			static int saveCount=1;
			sprintf(saveName,"%s%i",getLastFileName(),saveCount++);
			char* name = &saveName[0];
			if (name[0] == '/')
			{
				name = &saveName[1];
			} 
			
			if (m_dom->getAsset()->getContributor_array().getCount())
			{
				if (!m_dom->getAsset()->getContributor_array().get(0)->getAuthor())
				{
					m_dom->getAsset()->getContributor_array().get(0)->createAndPlace("author");
				}

				m_dom->getAsset()->getContributor_array().get(0)->getAuthor()->setValue
					("http://bullet.sf.net Erwin Coumans");

				if (!m_dom->getAsset()->getContributor_array().get(0)->getAuthoring_tool())
				{
					m_dom->getAsset()->getContributor_array().get(0)->createAndPlace("authoring_tool");
				}

				m_dom->getAsset()->getContributor_array().get(0)->getAuthoring_tool()->setValue
#ifdef WIN32
					("Bullet ColladaPhysicsViewer-Win32-0.8");
#else
#ifdef __APPLE__
					("Bullet ColladaPhysicsViewer-MacOSX-0.8");
#else
					("Bullet ColladaPhysicsViewer-UnknownPlatform-0.8");
#endif
#endif
				if (!m_dom->getAsset()->getContributor_array().get(0)->getComments())
				{
					m_dom->getAsset()->getContributor_array().get(0)->createAndPlace("comments");
				}
				 m_dom->getAsset()->getContributor_array().get(0)->getComments()->setValue
					 ("Comments to Physics Forum at http://www.continuousphysics.com/Bullet/phpBB2/index.php");
			}

			m_collada->saveAs(name);
			return true;
			
		}
#endif
		return false;
}

//some code that de-mangles the windows filename passed in as argument
char cleaned_filename[513];
char* getLastFileName()
{
	return cleaned_filename;
}


char* fixFileName(const char* lpCmdLine)
{


	// We might get a windows-style path on the command line, this can mess up the DOM which expects
	// all paths to be URI's.  This block of code does some conversion to try and make the input
	// compliant without breaking the ability to accept a properly formatted URI.  Right now this only
	// displays the first filename
	const char *in = lpCmdLine;
	char* out = cleaned_filename;
	*out = '\0';
	// If the first character is a ", skip it (filenames with spaces in them are quoted)
	if(*in == '\"')
	{
		in++;
	}
	if(*(in+1) == ':')
	{
		// Second character is a :, assume we have a path with a drive letter and add a slash at the beginning
		*(out++) = '/';
	}
	int i;
	for(i =0; i<512; i++)
	{
		// If we hit a null or a quote, stop copying.  This will get just the first filename.
		if(*in == '\0' || *in == '\"')
			break;
		// Copy while swapping backslashes for forward ones
		if(*in == '\\')
		{
			*out = '/';
		}
		else
		{
			*out = *in;
		}
		in++;
		out++;
	}
	
	cleaned_filename[i+1] = '\0'; 
	return cleaned_filename;
}


void	ColladaConverter::ConvertRigidBodyRef( btRigidBodyInput& rbInput,btRigidBodyOutput& rbOutput)
{
	
	const domRigid_body::domTechnique_commonRef techniqueRef = rbInput.m_rigidBodyRef2->getTechnique_common();
	if (techniqueRef)
	{

		if (techniqueRef->getMass())
		{
			rbOutput.m_mass = techniqueRef->getMass()->getValue();
		}
		if (techniqueRef->getDynamic())
		{
			rbOutput.m_isDynamics = techniqueRef->getDynamic()->getValue();
		}
		//a hack to interpret <extra> PhysX profile:
		//when <kinematic> is true, make <dynamic> false...
		//using the DOM is a pain...
		const domExtra_Array& extraArray = rbInput.m_rigidBodyRef2->getExtra_array();
		unsigned int s=0;

		for (s = 0;s< extraArray.getCount();s++)
		{
			const domExtraRef extraRef = extraArray[s];
			const domTechnique_Array techniqueArray = extraRef->getTechnique_array();
			unsigned int t=0;
			for (t=0;t<techniqueArray.getCount();t++)
			{
				const domTechniqueRef techRef = techniqueArray[t];
				const daeElementRefArray elemRefArray = techRef->getContents();
				unsigned int u = 0;
				for (u=0;u<elemRefArray.getCount();u++)
				{
					daeElementRef elemRef = elemRefArray[u];
					daeString elemName = elemRef->getElementName();
					if (elemName && !strcmp(elemName,"kinematic"))
					{
						//daeMemoryRef memRef = elemRef->getValuePointer();
						
						//daeBool hasVal = elemRef->hasValue();

						//COLLADA_TYPE::TypeEnum mytype = elemRef->getElementType();
						//how can I make this cast safe?
						const domAny* myAny = (const domAny*)elemRef.cast();
						daeString myVal = myAny->getValue();
						if (myVal)
						{
							if (!strcmp(myVal,"true"))
							{
								printf("revert bug in PhysX .dae export -> <kinematic>true</kinematic> means <dynamic>false</dynamic>\n");
								rbOutput.m_isDynamics = false;
							}
						}
					}
				}
			}
		}

//			<extra>
//				<technique profile="PhysX">
//						<wakeUpCounter>0.399999976</wakeUpCounter>
//							<kinematic>false</kinematic>



		//shapes
		for (s=0;s<techniqueRef->getShape_array().getCount();s++)
		{
			domRigid_body::domTechnique_common::domShapeRef shapeRef = techniqueRef->getShape_array()[s];

			if (shapeRef->getPlane())
			{
				domPlaneRef planeRef = shapeRef->getPlane();
				if (planeRef->getEquation())
				{
					const domFloat4 planeEq = planeRef->getEquation()->getValue();
					btVector3 planeNormal(planeEq.get(0),planeEq.get(1),planeEq.get(2));
					btScalar planeConstant = planeEq.get(3)*m_unitMeterScaling;
					rbOutput.m_colShape = new btStaticPlaneShape(planeNormal,planeConstant);
				}

			}

			if (shapeRef->getBox())
			{
				domBoxRef boxRef = shapeRef->getBox();
				domBox::domHalf_extentsRef	domHalfExtentsRef = boxRef->getHalf_extents();
				domFloat3& halfExtents = domHalfExtentsRef->getValue();
				float x = halfExtents.get(0)*m_unitMeterScaling;
				float y = halfExtents.get(1)*m_unitMeterScaling;
				float z = halfExtents.get(2)*m_unitMeterScaling;
				rbOutput.m_colShape = new btBoxShape(btVector3(x,y,z));
			}
			if (shapeRef->getSphere())
			{
				domSphereRef sphereRef = shapeRef->getSphere();
				domSphere::domRadiusRef radiusRef = sphereRef->getRadius();
				domFloat radius = radiusRef->getValue()*m_unitMeterScaling;
				rbOutput.m_colShape = new btSphereShape(radius);
			}

			if (shapeRef->getCylinder())
			{
				domCylinderRef cylinderRef = shapeRef->getCylinder();
				domFloat height = cylinderRef->getHeight()->getValue()*m_unitMeterScaling;
				domFloat2 radius2 = cylinderRef->getRadius()->getValue();
				domFloat radius0 = radius2.get(0)*m_unitMeterScaling;

				//Cylinder around the local Y axis
				rbOutput.m_colShape = new btCylinderShape(btVector3(radius0,height,radius0));

			}

			if (shapeRef->getInstance_geometry())
			{
				const domInstance_geometryRef geomInstRef = shapeRef->getInstance_geometry();
				daeElement* geomElem = geomInstRef->getUrl().getElement();
				//elemRef->getTypeName();
				domGeometry* geom = (domGeometry*) geomElem;
				if (geom && geom->getMesh())
				{
					const domMeshRef meshRef = geom->getMesh();

					//it can be either triangle mesh, or we just pick the vertices/positions

					if (meshRef->getTriangles_array().getCount())
					{

						btTriangleMesh* trimesh = new btTriangleMesh();
						
						for (unsigned int tg = 0;tg<meshRef->getTriangles_array().getCount();tg++)
						{


							domTrianglesRef triRef = meshRef->getTriangles_array()[tg];
							const domPRef pRef = triRef->getP();
							btIndexedMesh meshPart;
							meshPart.m_triangleIndexStride=0;


							
							int vertexoffset = -1;
							domInputLocalOffsetRef indexOffsetRef;
							

							for (unsigned int w=0;w<triRef->getInput_array().getCount();w++)
							{
								int offset = triRef->getInput_array()[w]->getOffset();
								daeString str = triRef->getInput_array()[w]->getSemantic();
								if (!strcmp(str,"VERTEX"))
								{
									indexOffsetRef = triRef->getInput_array()[w];
									vertexoffset = offset;
								}
								if (offset > meshPart.m_triangleIndexStride)
								{
									meshPart.m_triangleIndexStride = offset;
								}
							}
							meshPart.m_triangleIndexStride++;
							domListOfUInts indexArray =triRef->getP()->getValue(); 

							//int*		m_triangleIndexBase;



							meshPart.m_numTriangles = triRef->getCount();

							const domVerticesRef vertsRef = meshRef->getVertices();
							int numInputs = vertsRef->getInput_array().getCount();
							for (int i=0;i<numInputs;i++)
							{
								domInputLocalRef localRef = vertsRef->getInput_array()[i];
								daeString str = localRef->getSemantic();
								if ( !strcmp(str,"POSITION"))
								{
									const domURIFragmentType& frag = localRef->getSource();

									daeElementConstRef constElem = frag.getElement();

									const domSourceRef node = *(const domSourceRef*)&constElem;
									const domFloat_arrayRef flArray = node->getFloat_array();
									if (flArray)
									{
										const domListOfFloats& listFloats = flArray->getValue();

										int k=vertexoffset;
										int t=0;
										int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'
										for (;t<meshPart.m_numTriangles;t++)
										{
											btVector3 verts[3];
											int index0;
											for (int i=0;i<3;i++)
											{
												index0 = indexArray.get(k)*vertexStride;
												domFloat fl0 = listFloats.get(index0);
												domFloat fl1 = listFloats.get(index0+1);
												domFloat fl2 = listFloats.get(index0+2);
												k+=meshPart.m_triangleIndexStride;
												verts[i].setValue(fl0*m_unitMeterScaling,fl1*m_unitMeterScaling,fl2*m_unitMeterScaling);
											}
											trimesh->addTriangle(verts[0],verts[1],verts[2]);
										}
									}
								}
							}





							if (rbOutput.m_isDynamics)
							{
								printf("moving concave <mesh> not supported, transformed into convex\n");
								rbOutput.m_colShape = new btConvexTriangleMeshShape(trimesh);
								rbOutput.m_colShape->setShapeName (strdup(geom->getId()));
							} else
							{
								printf("static concave triangle <mesh> added\n");
								bool useQuantizedAabbCompression = true;
								rbOutput.m_colShape = new btBvhTriangleMeshShape(trimesh,useQuantizedAabbCompression);
								rbOutput.m_colShape->setShapeName (strdup(geom->getId()));
								//rbOutput.m_colShape = new btBvhTriangleMeshShape(trimesh);
								//rbOutput.m_colShape = new btConvexTriangleMeshShape(trimesh);
								
								//btTriangleMeshShape
							}

						} 
					} else
						{

							btConvexHullShape* convexHull = new btConvexHullShape();
							int numAddedVerts = 0;

							const domVerticesRef vertsRef = meshRef->getVertices();
							int numInputs = vertsRef->getInput_array().getCount();
							for (int i=0;i<numInputs;i++)
							{
								domInputLocalRef localRef = vertsRef->getInput_array()[i];
								daeString str = localRef->getSemantic();
								if ( !strcmp(str,"POSITION"))
								{
									const domURIFragmentType& frag = localRef->getSource();

									daeElementConstRef constElem = frag.getElement();

									const domSourceRef node = *(const domSourceRef*)&constElem;
									const domFloat_arrayRef flArray = node->getFloat_array();
									if (flArray)
									{
										const domListOfFloats& listFloats = flArray->getValue();
										int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'
										unsigned int vertIndex = 0;
										for (vertIndex = 0;vertIndex < listFloats.getCount();vertIndex+=vertexStride)
										{
											//btVector3 verts[3];
											domFloat fl0 = listFloats.get(vertIndex);
											domFloat fl1 = listFloats.get(vertIndex+1);
											domFloat fl2 = listFloats.get(vertIndex+2);
											convexHull->addPoint(btPoint3(fl0,fl1,fl2) * m_unitMeterScaling);
										}
									}
								}
							}
							//convexHull->addPoint();
							if (numAddedVerts > 0)
							{
								rbOutput.m_colShape = convexHull;
								rbOutput.m_colShape->setShapeName (strdup(geom->getId()));
							} else
							{
								delete convexHull;
								printf("no vertices found for convex hull\n");
							}

					}
						

				}

				if (geom && geom->getConvex_mesh())
				{

					{
						const domConvex_meshRef convexRef = geom->getConvex_mesh();
						daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
						if ( otherElemRef != NULL )
						{
							domGeometryRef linkedGeom = *(domGeometryRef*)&otherElemRef;
							printf( "otherLinked\n");
						} else
						{
							printf("convexMesh polyCount = %i\n",convexRef->getPolygons_array().getCount());
							printf("convexMesh triCount = %i\n",convexRef->getTriangles_array().getCount());

						}
					}



					btConvexHullShape* convexHullShape = new btConvexHullShape(0,0);

					//it is quite a trick to get to the vertices, using Collada.
					//we are not there yet...

					const domConvex_meshRef convexRef = geom->getConvex_mesh();
					//daeString urlref = convexRef->getConvex_hull_of().getURI();
					daeString urlref2 = convexRef->getConvex_hull_of().getOriginalURI();
					if (urlref2)
					{
						daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
						//	if ( otherElemRef != NULL )
						//		domGeometryRef linkedGeom = *(domGeometryRef*)&otherElemRef;

						// Load all the geometry libraries
						for ( unsigned int i = 0; i < m_dom->getLibrary_geometries_array().getCount(); i++)
						{
							domLibrary_geometriesRef libgeom = m_dom->getLibrary_geometries_array()[i];
							//int index = libgeom->findLastIndexOf(urlref2);
							//can't find it

							for ( unsigned int  i = 0; i < libgeom->getGeometry_array().getCount(); i++)
							{
								//ReadGeometry(  ); 
								domGeometryRef lib = libgeom->getGeometry_array()[i];
								if (!strcmp(lib->getId(),urlref2))
								{
									//found convex_hull geometry
									domMesh			*meshElement		= lib->getMesh();//linkedGeom->getMesh();
									if (meshElement)
									{
										const domVerticesRef vertsRef = meshElement->getVertices();
										int numInputs = vertsRef->getInput_array().getCount();
										for (int i=0;i<numInputs;i++)
										{
											domInputLocalRef localRef = vertsRef->getInput_array()[i];
											daeString str = localRef->getSemantic();
											if ( !strcmp(str,"POSITION"))
											{
												const domURIFragmentType& frag = localRef->getSource();

												daeElementConstRef constElem = frag.getElement();

												const domSourceRef node = *(const domSourceRef*)&constElem;
												const domFloat_arrayRef flArray = node->getFloat_array();
												if (flArray)
												{
													int numElem = flArray->getCount();
													const domListOfFloats& listFloats = flArray->getValue();

													for (int k=0;k+2<numElem;k+=3)
													{
														domFloat fl0 = listFloats.get(k);
														domFloat fl1 = listFloats.get(k+1);
														domFloat fl2 = listFloats.get(k+2);
														//printf("float %f %f %f\n",fl0,fl1,fl2);

														convexHullShape->addPoint(btPoint3(fl0,fl1,fl2) * m_unitMeterScaling);
													}

												}

											}


										}

									}
								}
							
									
								
							}
						}
					} else {
						//no getConvex_hull_of but direct vertices
						const domVerticesRef vertsRef = convexRef->getVertices();
						int numInputs = vertsRef->getInput_array().getCount();
						for (int i=0;i<numInputs;i++)
						{
							domInputLocalRef localRef = vertsRef->getInput_array()[i];
							daeString str = localRef->getSemantic();
							if ( !strcmp(str,"POSITION"))
							{
								const domURIFragmentType& frag = localRef->getSource();

								daeElementConstRef constElem = frag.getElement();

								const domSourceRef node = *(const domSourceRef*)&constElem;
								const domFloat_arrayRef flArray = node->getFloat_array();
								if (flArray)
								{
									int numElem = flArray->getCount();
									const domListOfFloats& listFloats = flArray->getValue();

									for (int k=0;k+2<numElem;k+=3)
									{
										domFloat fl0 = listFloats.get(k);
										domFloat fl1 = listFloats.get(k+1);
										domFloat fl2 = listFloats.get(k+2);
										//printf("float %f %f %f\n",fl0,fl1,fl2);

										convexHullShape->addPoint(btPoint3(fl0,fl1,fl2)*m_unitMeterScaling);
									}

								}
							}

						}


					}

					if (convexHullShape->getNumVertices())
					{
						rbOutput.m_colShape = convexHullShape;
						rbOutput.m_colShape->setShapeName (strdup(geom->getId()));
						printf("created convexHullShape with %i points\n",convexHullShape->getNumVertices());
					} else
					{
						delete convexHullShape;
						printf("failed to create convexHullShape\n");
					}


					//domGeometryRef linkedGeom = *(domGeometryRef*)&otherElemRef;

					printf("convexmesh\n");

				}
			}

			//if more then 1 shape, or a non-identity local shapetransform
			//use a compound

			bool hasShapeLocalTransform = ((shapeRef->getRotate_array().getCount() > 0) ||
				(shapeRef->getTranslate_array().getCount() > 0));
			
			if (rbOutput.m_colShape)
			{
				if ((techniqueRef->getShape_array().getCount()>1) ||
					(hasShapeLocalTransform))
				{
					
					if (!rbOutput.m_compoundShape)
					{
						rbOutput.m_compoundShape = new btCompoundShape();
					}

					btTransform localTransform;
					localTransform.setIdentity();
					if (hasShapeLocalTransform)
					{
					localTransform = GetbtTransformFromCOLLADA_DOM(
						emptyMatrixArray,
						shapeRef->getRotate_array(),
						shapeRef->getTranslate_array(),
						m_unitMeterScaling
						);
					}

					rbOutput.m_compoundShape->addChildShape(localTransform,rbOutput.m_colShape);
					rbOutput.m_colShape = 0;
				}
			}


		}//for each shape
	}


}
