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

#include "ColladaConverter.h"

#include "dae.h"
#include "dom/domCOLLADA.h"

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

#include "CcdPhysicsController.h"


char* getLastFileName();
char* fixFileName(const char* lpCmdLine);
//todo: sort out this domInstance_rigid_bodyRef forward definition, put it in the headerfile and make it virtual (make code more re-usable)

struct	RigidBodyInput
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


struct	RigidBodyOutput
{
	float	m_mass;
	bool	m_isDynamics;
	CollisionShape*	m_colShape;
	CompoundShape*	m_compoundShape;
};



domMatrix_Array emptyMatrixArray;
domNodeRef	m_colladadomNodes[COLLADA_CONVERTER_MAX_NUM_OBJECTS];


SimdTransform	GetSimdTransformFromCOLLADA_DOM(domMatrix_Array& matrixArray,
														domRotate_Array& rotateArray,
														domTranslate_Array& translateArray
														)

{
	SimdTransform	startTransform;
	startTransform.setIdentity();
	
	int i;
	//either load the matrix (worldspace) or incrementally build the transform from 'translate'/'rotate'
	for (i=0;i<matrixArray.getCount();i++)
	{
		domMatrixRef matrixRef = matrixArray[i];
		domFloat4x4 fl16 = matrixRef->getValue();
		SimdVector3 origin(fl16.get(3),fl16.get(7),fl16.get(11));
		startTransform.setOrigin(origin);
		SimdMatrix3x3 basis(fl16.get(0),fl16.get(1),fl16.get(2),
							fl16.get(4),fl16.get(5),fl16.get(6),
							fl16.get(8),fl16.get(9),fl16.get(10));
		startTransform.setBasis(basis);
	}

	for (i=0;i<rotateArray.getCount();i++)
	{
		domRotateRef rotateRef = rotateArray[i];
		domFloat4 fl4 = rotateRef->getValue();
		float angleRad = SIMD_RADS_PER_DEG*fl4.get(3);
		SimdQuaternion rotQuat(SimdVector3(fl4.get(0),fl4.get(1),fl4.get(2)),angleRad);
		startTransform.getBasis() = startTransform.getBasis() * SimdMatrix3x3(rotQuat);
	}

	for (i=0;i<translateArray.getCount();i++)
	{
		domTranslateRef translateRef = translateArray[i];
		domFloat3 fl3 = translateRef->getValue();
		startTransform.getOrigin() += SimdVector3(fl3.get(0),fl3.get(1),fl3.get(2));
	}
	return startTransform;
}





ColladaConverter::ColladaConverter()
:m_collada(0),
m_dom(0),
m_filename(0),
m_numObjects(0)
{
	//Collada-m_dom
	m_collada = new DAE;

	//clear 
	{

		for (int i=0;i<COLLADA_CONVERTER_MAX_NUM_OBJECTS;i++)
		{
			m_colladadomNodes[i] = 0;
		}
	}
}
	

bool	ColladaConverter::load(const char* orgfilename)
{

	const char* filename = fixFileName(orgfilename);
	
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
	return true;
}





bool ColladaConverter::convert()
{

//succesfully loaded file, now convert data

			if ( m_dom->getAsset()->getUp_axis() )
			{
				domAsset::domUp_axis * up = m_dom->getAsset()->getUp_axis();
				switch( up->getValue() )
				{
				case UPAXISTYPE_X_UP:
					printf("	X is Up Data and Hiearchies must be converted!\n" ); 
					printf("  Conversion to X axis Up isn't currently supported!\n" ); 
					printf("  COLLADA_RT defaulting to Y Up \n" ); 
					SetGravity(SimdVector3(-10,0,0));
					SetCameraInfo(SimdVector3(1,0,0),1);
					break; 
				case UPAXISTYPE_Y_UP:
					printf("	Y Axis is Up for this file \n" ); 
					printf("  COLLADA_RT set to Y Up \n" ); 
					SetGravity(SimdVector3(0,-10,0));
					SetCameraInfo(SimdVector3(0,1,0),0);

					break;
				case UPAXISTYPE_Z_UP:
					printf("	Z Axis is Up for this file \n" ); 
					printf("  All Geometry and Hiearchies must be converted!\n" ); 
					SetGravity(SimdVector3(0,0,-10));
					break; 
				default:

					break; 
				}
			}


			//we don't handle visual objects, physics objects are rendered as such
			for (int s=0;s<m_dom->getLibrary_visual_scenes_array().getCount();s++)
			{
				domLibrary_visual_scenesRef scenesRef = m_dom->getLibrary_visual_scenes_array()[s];
				for (int i=0;i<scenesRef->getVisual_scene_array().getCount();i++)
				{
					domVisual_sceneRef sceneRef = scenesRef->getVisual_scene_array()[i];
					for (int n=0;n<sceneRef->getNode_array().getCount();n++)
					{
						domNodeRef nodeRef = sceneRef->getNode_array()[n];
						nodeRef->getRotate_array();
						nodeRef->getTranslate_array();
						nodeRef->getScale_array();

					}
				}
			}




			// Load all the geometry libraries
			for ( int i = 0; i < m_dom->getLibrary_geometries_array().getCount(); i++)
			{
				domLibrary_geometriesRef libgeom = m_dom->getLibrary_geometries_array()[i];

				printf(" CrtScene::Reading Geometry Library \n" );
				for ( int  i = 0; i < libgeom->getGeometry_array().getCount(); i++)
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

			for ( int i = 0; i < m_dom->getLibrary_physics_scenes_array().getCount(); i++)
			{
				domLibrary_physics_scenesRef physicsScenesRef = m_dom->getLibrary_physics_scenes_array()[i];
				for (int s=0;s<physicsScenesRef->getPhysics_scene_array().getCount();s++)
				{
					domPhysics_sceneRef physicsSceneRef = physicsScenesRef->getPhysics_scene_array()[s];

					if (physicsSceneRef->getTechnique_common())
					{
						if (physicsSceneRef->getTechnique_common()->getGravity())
						{
							const domFloat3 grav = physicsSceneRef->getTechnique_common()->getGravity()->getValue();
							printf("gravity set to %f,%f,%f\n",grav.get(0),grav.get(1),grav.get(2));

							SetGravity(SimdVector3(grav.get(0),grav.get(1),grav.get(2)));
						}

					} 

					for (int m=0;m<physicsSceneRef->getInstance_physics_model_array().getCount();m++)
					{
						domInstance_physics_modelRef instance_physicsModelRef = physicsSceneRef->getInstance_physics_model_array()[m];

						daeElementRef ref = instance_physicsModelRef->getUrl().getElement();

						domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 


						int p,r;
						for ( p=0;p<model->getInstance_physics_model_array().getCount();p++)
						{
							domInstance_physics_modelRef	instancePhysicsModelRef = model->getInstance_physics_model_array()[p];

							daeElementRef ref = instancePhysicsModelRef->getUrl().getElement();
	
							domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 

							//todo: group some shared functionality in following 2 'blocks'.
							for (r=0;r<instancePhysicsModelRef->getInstance_rigid_body_array().getCount();r++)
							{
								domInstance_rigid_bodyRef instRigidbodyRef = instancePhysicsModelRef->getInstance_rigid_body_array()[r];

								float mass = 1.f;
								bool isDynamics = true;
								CollisionShape* colShape = 0;
								CompoundShape* compoundShape = 0;

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
								}

								printf("mass = %f, isDynamics %i\n",mass,isDynamics);

								if (bodyName && model)
								{
									//try to find the rigid body
									int numBody = model->getRigid_body_array().getCount();

									for (int r=0;r<model->getRigid_body_array().getCount();r++)
									{
										domRigid_bodyRef rigidBodyRef = model->getRigid_body_array()[r];
										if (rigidBodyRef->getSid() && !strcmp(rigidBodyRef->getSid(),bodyName))
										{


											RigidBodyOutput output;
											output.m_colShape = colShape;
											output.m_compoundShape = compoundShape;

											RigidBodyInput rbInput;
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
									RigidBodyInput input;
									input.m_instanceRigidBodyRef = instRigidbodyRef;
									input.m_rigidBodyRef2 = 0;
									input.m_bodyName = (char*)bodyName;
									PreparePhysicsObject(input, isDynamics,mass,colShape);
								}
							}
						}


						for (r=0;r<instance_physicsModelRef->getInstance_rigid_body_array().getCount();r++)
						{

							domInstance_rigid_bodyRef instRigidbodyRef = instance_physicsModelRef->getInstance_rigid_body_array()[r];

							

							float mass = 1.f;
							bool isDynamics = true;
							CollisionShape* colShape = 0;
							CompoundShape* compoundShape = 0;

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
							}

							printf("mass = %f, isDynamics %i\n",mass,isDynamics);

							if (bodyName && model)
							{
								//try to find the rigid body
								int numBody = model->getRigid_body_array().getCount();

								for (int r=0;r<model->getRigid_body_array().getCount();r++)
								{
									domRigid_bodyRef rigidBodyRef = model->getRigid_body_array()[r];
									if (rigidBodyRef->getSid() && !strcmp(rigidBodyRef->getSid(),bodyName))
									{


										RigidBodyOutput output;
										output.m_colShape = colShape;
										output.m_compoundShape = compoundShape;

										RigidBodyInput rbInput;
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
								RigidBodyInput input;
								input.m_instanceRigidBodyRef = instRigidbodyRef;
								input.m_rigidBodyRef2 = 0;
								input.m_bodyName = (char*)bodyName;
								PreparePhysicsObject(input, isDynamics,mass,colShape);
							}

						} //for  each  instance_rigid_body

						
					} //for each physics model

					
					//handle constraints
					for (int m=0;m<physicsSceneRef->getInstance_physics_model_array().getCount();m++)
					{
						domInstance_physics_modelRef instance_physicsModelRef = physicsSceneRef->getInstance_physics_model_array()[m];

						daeElementRef ref = instance_physicsModelRef->getUrl().getElement();

						domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 

						{
							ConstraintInput cInput;
							cInput.m_instance_physicsModelRef = instance_physicsModelRef;
							cInput.m_model = model;
							PrepareConstraints(cInput);
						}

						//also don't forget the model's 'instance_physics_models!
						for ( int p=0;p<model->getInstance_physics_model_array().getCount();p++)
						{
							domInstance_physics_modelRef	instancePhysicsModelRef = model->getInstance_physics_model_array()[p];

							daeElementRef ref = instancePhysicsModelRef->getUrl().getElement();
	
							domPhysics_modelRef model = *(domPhysics_modelRef*)&ref; 
							
							ConstraintInput cInput;
							cInput.m_instance_physicsModelRef = instancePhysicsModelRef;
							cInput.m_model = model;
							PrepareConstraints(cInput);
						}

											
					} //2nd time, for each physics model

				}
			}

			return true;
}


void	ColladaConverter::PrepareConstraints(ConstraintInput& input)
{
	domInstance_physics_modelRef instance_physicsModelRef = input.m_instance_physicsModelRef;
	domPhysics_modelRef model = input.m_model;

	for (int c=0;c<instance_physicsModelRef->getInstance_rigid_constraint_array().getCount();c++)
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

					daeString uri = attachRefBody->getRigid_body().getURI();
					daeString orgUri0 = attachRefBody->getRigid_body().getOriginalURI();
					daeString orgUri1 = attachBody1->getRigid_body().getOriginalURI();
					CcdPhysicsController* ctrl0=0,*ctrl1=0;
					
					for (int i=0;i<m_numObjects;i++)
					{
						char* bodyName = (char*)m_physObjects[i]->getNewClientInfo();
						if (!strcmp(bodyName,orgUri0))
						{
							ctrl0=m_physObjects[i];
						}
						if (!strcmp(bodyName,orgUri1))
						{
							ctrl1=m_physObjects[i];
						}
					}



					const domRigid_constraint::domAttachmentRef attachOtherBody = rigidConstraintRef->getAttachment();

					
					const domRigid_constraint::domTechnique_commonRef commonRef = rigidConstraintRef->getTechnique_common();
					
					domFloat3 flMin = commonRef->getLimits()->getLinear()->getMin()->getValue();
					SimdVector3 minLinearLimit(flMin.get(0),flMin.get(1),flMin.get(2));
					
					domFloat3 flMax = commonRef->getLimits()->getLinear()->getMax()->getValue();
					SimdVector3 maxLinearLimit(flMax.get(0),flMax.get(1),flMax.get(2));
														
					domFloat3 coneMinLimit = commonRef->getLimits()->getSwing_cone_and_twist()->getMin()->getValue();
					SimdVector3 angularMin(coneMinLimit.get(0),coneMinLimit.get(1),coneMinLimit.get(2));

					domFloat3 coneMaxLimit = commonRef->getLimits()->getSwing_cone_and_twist()->getMax()->getValue();
					SimdVector3 angularMax(coneMaxLimit.get(0),coneMaxLimit.get(1),coneMaxLimit.get(2));

					{
						int constraintId;

						SimdTransform attachFrameRef0;
						attachFrameRef0 = 
							GetSimdTransformFromCOLLADA_DOM
							(
							emptyMatrixArray,
							attachRefBody->getRotate_array(),
							attachRefBody->getTranslate_array());

						SimdTransform attachFrameOther;
						attachFrameOther =
							GetSimdTransformFromCOLLADA_DOM
							(
							emptyMatrixArray,
							attachBody1->getRotate_array(),
							attachBody1->getTranslate_array()
							);


						//convert INF / -INF into lower > upper

						//currently there is a hack in the DOM to detect INF / -INF
						//see daeMetaAttribute.cpp
						//INF -> 999999.9
						//-INF -> -999999.9
						float linearCheckTreshold = 999999.0;
						float angularCheckTreshold = 180.0;//check this



						
						//free means upper < lower, 
						//locked means upper == lower
						//limited means upper > lower
						//limitIndex: first 3 are linear, next 3 are angular

						SimdVector3 linearLowerLimits = minLinearLimit;
						SimdVector3 linearUpperLimits = maxLinearLimit;
						SimdVector3 angularLowerLimits = angularMin;
						SimdVector3 angularUpperLimits = angularMax;
						{
							for (int i=0;i<3;i++)
							{
								if  ((linearLowerLimits[i] < -linearCheckTreshold) ||
									(linearUpperLimits[i] > linearCheckTreshold))
								{
									//disable limits
									linearLowerLimits[i] = 1;
									linearUpperLimits[i] = 0;
								}

								if  ((angularLowerLimits[i] < -angularCheckTreshold) ||
									(angularUpperLimits[i] > angularCheckTreshold))
								{
									//disable limits
									angularLowerLimits[i] = 1;
									angularUpperLimits[i] = 0;
								}
							}
						}


						if (ctrl0 && ctrl1)
						{
						constraintId = createUniversalD6Constraint(
						ctrl0,
						ctrl1,
						attachFrameRef0,
						attachFrameOther,
						linearLowerLimits,
						linearUpperLimits,
						angularLowerLimits,
						angularUpperLimits
							);
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

void	ColladaConverter::PreparePhysicsObject(struct RigidBodyInput& input, bool isDynamics, float mass,CollisionShape* colShape)
{
	SimdTransform startTransform;
	startTransform.setIdentity();
	SimdVector3 startScale(1.f,1.f,1.f);

	//The 'target' points to a graphics element/node, which contains the start (world) transform
	daeElementRef elem = input.m_instanceRigidBodyRef->getTarget().getElement();
	if (elem)
	{
		domNodeRef node = *(domNodeRef*)&elem;
		m_colladadomNodes[m_numObjects] = node;

		//find transform of the node that this rigidbody maps to


		startTransform = GetSimdTransformFromCOLLADA_DOM(
							node->getMatrix_array(),
							node->getRotate_array(),
							node->getTranslate_array()
							);

		int i;
		for (i=0;i<node->getScale_array().getCount();i++)
		{
			domScaleRef scaleRef = node->getScale_array()[i];
			domFloat3 fl3 = scaleRef->getValue();
			startScale = SimdVector3(fl3.get(0),fl3.get(1),fl3.get(2));
		}

	}



	CcdPhysicsController* ctrl = CreatePhysicsObject(isDynamics,mass,startTransform,colShape);
	if (ctrl)
	{
		//for bodyName lookup in constraints
		ctrl->setNewClientInfo((void*)input.m_bodyName);
		m_physObjects[m_numObjects] = ctrl;
		m_numObjects++;
	}

}

bool ColladaConverter::saveAs(const char* filename)
{
	if (m_collada)
		{
			for (int i=0;i<m_numObjects;i++)
			{
				assert(m_colladadomNodes[i]);
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

					float np[3];
					domFloat3 newPos = m_colladadomNodes[i]->getTranslate_array().get(0)->getValue();
					m_physObjects[i]->GetMotionState()->getWorldPosition(
						np[0],
						np[1],
						np[2]);
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
					float quatIma0,quatIma1,quatIma2,quatReal;
					
					SimdQuaternion quat = m_physObjects[i]->GetRigidBody()->getCenterOfMassTransform().getRotation();
					SimdVector3 axis(quat.getX(),quat.getY(),quat.getZ());
					axis[3] = 0.f;
					//check for axis length
					SimdScalar len = axis.length2();
					if (len < SIMD_EPSILON*SIMD_EPSILON)
						axis = SimdVector3(1.f,0.f,0.f);
					else
						axis /= SimdSqrt(len);
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
		return false;
}

//some code that de-mangles the windows filename passed in as argument
char cleaned_filename[512];
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
	*out = NULL;
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
		if(*in == NULL || *in == '\"')
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
	
	return cleaned_filename;
}


void	ColladaConverter::ConvertRigidBodyRef( RigidBodyInput& rbInput,RigidBodyOutput& rbOutput)
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

		//shapes
		for (int s=0;s<techniqueRef->getShape_array().getCount();s++)
		{
			domRigid_body::domTechnique_common::domShapeRef shapeRef = techniqueRef->getShape_array()[s];

			if (shapeRef->getPlane())
			{
				domPlaneRef planeRef = shapeRef->getPlane();
				if (planeRef->getEquation())
				{
					const domFloat4 planeEq = planeRef->getEquation()->getValue();
					SimdVector3 planeNormal(planeEq.get(0),planeEq.get(1),planeEq.get(2));
					SimdScalar planeConstant = planeEq.get(3);
					rbOutput.m_colShape = new StaticPlaneShape(planeNormal,planeConstant);
				}

			}

			if (shapeRef->getBox())
			{
				domBoxRef boxRef = shapeRef->getBox();
				domBox::domHalf_extentsRef	domHalfExtentsRef = boxRef->getHalf_extents();
				domFloat3& halfExtents = domHalfExtentsRef->getValue();
				float x = halfExtents.get(0);
				float y = halfExtents.get(1);
				float z = halfExtents.get(2);
				rbOutput.m_colShape = new BoxShape(SimdVector3(x,y,z));
			}
			if (shapeRef->getSphere())
			{
				domSphereRef sphereRef = shapeRef->getSphere();
				domSphere::domRadiusRef radiusRef = sphereRef->getRadius();
				domFloat radius = radiusRef->getValue();
				rbOutput.m_colShape = new SphereShape(radius);
			}

			if (shapeRef->getCylinder())
			{
				domCylinderRef cylinderRef = shapeRef->getCylinder();
				domFloat height = cylinderRef->getHeight()->getValue();
				domFloat2 radius2 = cylinderRef->getRadius()->getValue();
				domFloat radius0 = radius2.get(0);

				//Cylinder around the local Y axis
				rbOutput.m_colShape = new CylinderShape(SimdVector3(radius0,height,radius0));

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
					TriangleIndexVertexArray* tindexArray = new TriangleIndexVertexArray();

					TriangleMesh* trimesh = new TriangleMesh();

					
					for (int tg = 0;tg<meshRef->getTriangles_array().getCount();tg++)
					{


						domTrianglesRef triRef = meshRef->getTriangles_array()[tg];
						const domPRef pRef = triRef->getP();
						daeMemoryRef memRef = pRef->getValue().getRawData();
						IndexedMesh meshPart;
						meshPart.m_triangleIndexStride=0;


						
						int vertexoffset = -1;
						domInputLocalOffsetRef indexOffsetRef;
						

						for (int w=0;w<triRef->getInput_array().getCount();w++)
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
						int count = indexArray.getCount();

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
									int numElem = flArray->getCount();
									const domListOfFloats& listFloats = flArray->getValue();

									int numVerts = listFloats.getCount()/3;
									int k=vertexoffset;
									int t=0;
									int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'
									for (;t<meshPart.m_numTriangles;t++)
									{
										SimdVector3 verts[3];
										int index0,index1,index2;
										for (int i=0;i<3;i++)
										{
											index0 = indexArray.get(k)*vertexStride;
											domFloat fl0 = listFloats.get(index0);
											domFloat fl1 = listFloats.get(index0+1);
											domFloat fl2 = listFloats.get(index0+2);
											k+=meshPart.m_triangleIndexStride;
											verts[i].setValue(fl0,fl1,fl2);
										}
										trimesh->AddTriangle(verts[0],verts[1],verts[2]);
									}
								}
							}
						}





							//int			m_triangleIndexStride;//calculate max offset
							//int			m_numVertices;
							//float*		m_vertexBase;//getRawData on floatArray
							//int			m_vertexStride;//use the accessor for this

						//};
						//tindexArray->AddIndexedMesh(meshPart);
						if (rbOutput.m_isDynamics)
						{
							printf("moving concave <mesh> not supported, transformed into convex\n");
							rbOutput.m_colShape = new ConvexTriangleMeshShape(trimesh);
						} else
						{
							printf("static concave triangle <mesh> added\n");
							rbOutput.m_colShape = new TriangleMeshShape(trimesh);
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



					ConvexHullShape* convexHullShape = new ConvexHullShape(0,0);

					//it is quite a trick to get to the vertices, using Collada.
					//we are not there yet...

					const domConvex_meshRef convexRef = geom->getConvex_mesh();
					//daeString urlref = convexRef->getConvex_hull_of().getURI();
					daeString urlref2 = convexRef->getConvex_hull_of().getOriginalURI();
					if (urlref2)
					{
						daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
						//	if ( otherElemRef != NULL )
						//	{
						//		domGeometryRef linkedGeom = *(domGeometryRef*)&otherElemRef;

						// Load all the geometry libraries
						for ( int i = 0; i < m_dom->getLibrary_geometries_array().getCount(); i++)
						{
							domLibrary_geometriesRef libgeom = m_dom->getLibrary_geometries_array()[i];
							//int index = libgeom->findLastIndexOf(urlref2);
							//can't find it

							for ( int  i = 0; i < libgeom->getGeometry_array().getCount(); i++)
							{
								//ReadGeometry(  ); 
								domGeometryRef lib = libgeom->getGeometry_array()[i];
								if (!strcmp(lib->getName(),urlref2))
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

														convexHullShape->AddPoint(SimdPoint3(fl0,fl1,fl2));
													}

												}

											}


										}

									}
								}
							
									
								
							}
						}


					} else
					{
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

										convexHullShape->AddPoint(SimdPoint3(fl0,fl1,fl2));
									}

								}

							}


						}


					}

					if (convexHullShape->GetNumVertices())
					{
						rbOutput.m_colShape = convexHullShape;
						printf("created convexHullShape with %i points\n",convexHullShape->GetNumVertices());
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
						rbOutput.m_compoundShape = new CompoundShape();
					}

					SimdTransform localTransform;
					localTransform.setIdentity();
					if (hasShapeLocalTransform)
					{
					localTransform = GetSimdTransformFromCOLLADA_DOM(
						emptyMatrixArray,
						shapeRef->getRotate_array(),
						shapeRef->getTranslate_array()
						);
					}

					rbOutput.m_compoundShape->AddChildShape(localTransform,rbOutput.m_colShape);
					rbOutput.m_colShape = 0;
				}
			}


		}//for each shape
	}


}