/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#define TEST_SERIALIZATION 1
//#undef DESERIALIZE_SOFT_BODIES

#ifdef BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
#define CREATE_NEW_BULLETFILE 1
#endif //BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "SerializeDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#ifdef TEST_SERIALIZATION
#include "LinearMath/btSerializer.h"
#include "btBulletFile.h"
#include "btBulletWorldImporter.h"
#endif //TEST_SERIALIZATION

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include <stdio.h> //printf debugging



#ifdef DESERIALIZE_SOFT_BODIES
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#endif


void SerializeDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}



void SerializeDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}




void	SerializeDemo::setupEmptyDynamicsWorld()
{
	///collision configuration contains default setup for memory, collision setup
	//m_collisionConfiguration = new btDefaultCollisionConfiguration();
#ifdef DESERIALIZE_SOFT_BODIES
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
#else
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
#endif //DESERIALIZE_SOFT_BODIES

	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
	

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

#ifdef DESERIALIZE_SOFT_BODIES
	btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;
	//world->setDrawFlags(world->getDrawFlags()^fDrawFlags::Clusters);
#else
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
#endif //DESERIALIZE_SOFT_BODIES

	//btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher*)m_dynamicsWorld->getDispatcher());

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

}


#ifdef DESERIALIZE_SOFT_BODIES
#include "BulletSoftBody/btSoftBodyData.h"
class MySoftBulletWorldImporter : public btBulletWorldImporter
{

	btSoftRigidDynamicsWorld* m_softRigidWorld;

	btHashMap<btHashPtr,btSoftBody::Material*>	m_materialMap;

	btHashMap<btHashPtr,btSoftBody*>	m_clusterBodyMap;
	btHashMap<btHashPtr,btSoftBody*>	m_softBodyMap;
	


public:

	MySoftBulletWorldImporter(btSoftRigidDynamicsWorld* world)
		:btBulletWorldImporter(world),
		m_softRigidWorld(world)
	{

	}

	virtual ~MySoftBulletWorldImporter()
	{
		
	}

	virtual bool convertAllObjects(  bParse::btBulletFile* bulletFile2)
	{
		bool result = btBulletWorldImporter::convertAllObjects(bulletFile2);
		int i;
		//now the soft bodies
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				int i;
				int numNodes = softBodyData->m_numNodes;
				
		
				btSoftBody*		psb=new btSoftBody(&m_softRigidWorld->getWorldInfo());
				m_softBodyMap.insert(softBodyData,psb);

				//materials
				for (i=0;i<softBodyData->m_numMaterials;i++)
				{
					SoftBodyMaterialData* matData = softBodyData->m_materials[i];
					btSoftBody::Material** matPtr = m_materialMap.find(matData);
					btSoftBody::Material* mat = 0;
					if (matPtr&& *matPtr)
					{
						mat = *matPtr;
					} else
					{
						mat = psb->appendMaterial();
						mat->m_flags = matData->m_flags;
						mat->m_kAST = matData->m_angularStiffness;
						mat->m_kLST = matData->m_linearStiffness;
						mat->m_kVST = matData->m_volumeStiffness;
						m_materialMap.insert(matData,mat);
					}
				}




				for (i=0;i<numNodes;i++)
				{
					SoftBodyNodeData& nodeData = softBodyData->m_nodes[i];
					btVector3 position;
					position.deSerializeFloat(nodeData.m_position);
					btScalar mass = nodeData.m_inverseMass? 1./nodeData.m_inverseMass : 0.f;
					psb->appendNode(position,mass);
					btSoftBody::Node* node = &psb->m_nodes[psb->m_nodes.size()-1];
					node->m_area = nodeData.m_area;
					node->m_battach = nodeData.m_attach;
					node->m_f.deSerializeFloat(nodeData.m_accumulatedForce);
					node->m_im = nodeData.m_inverseMass;

					btSoftBody::Material** matPtr = m_materialMap.find(nodeData.m_material);
					if (matPtr && *matPtr)
					{
						node->m_material = *matPtr;
					} else
					{
						printf("no mat?\n");
					}
					
					node->m_n.deSerializeFloat(nodeData.m_normal);
					node->m_q = node->m_x;
					node->m_v.deSerializeFloat(nodeData.m_velocity);
					
				}

				for (i=0;i<softBodyData->m_numLinks;i++)
				{
					SoftBodyLinkData& linkData = softBodyData->m_links[i];
					btSoftBody::Material** matPtr = m_materialMap.find(linkData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1],*matPtr);
					} else
					{
						psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1]);
					}
					btSoftBody::Link* link = &psb->m_links[psb->m_links.size()-1];
					link->m_bbending = linkData.m_bbending;
					link->m_rl = linkData.m_restLength;
				}

				for (i=0;i<softBodyData->m_numFaces;i++)
				{
					SoftBodyFaceData& faceData = softBodyData->m_faces[i];
					btSoftBody::Material** matPtr = m_materialMap.find(faceData.m_material);
					if (matPtr && *matPtr)
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2],*matPtr);
					} else
					{
						psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2]);
					}
					btSoftBody::Face* face = &psb->m_faces[psb->m_faces.size()-1];
					face->m_normal.deSerializeFloat(faceData.m_normal);
					face->m_ra = faceData.m_restArea;
				}

			

				//anchors
				for (i=0;i<softBodyData->m_numAnchors;i++)
				{
					btCollisionObject** colAptr = m_bodyMap.find(softBodyData->m_anchors[i].m_rigidBody);
					if (colAptr && *colAptr)
					{
						btRigidBody* body = btRigidBody::upcast(*colAptr);
						if (body)
						{
							bool disableCollision = false;
							btVector3 localPivot;
							localPivot.deSerializeFloat(softBodyData->m_anchors[i].m_localFrame);
							psb->appendAnchor(softBodyData->m_anchors[i].m_nodeIndex,body,localPivot, disableCollision);
						}
					}
				}

				if (softBodyData->m_pose)
				{
					psb->m_pose.m_aqq.deSerializeFloat(  softBodyData->m_pose->m_aqq);
					psb->m_pose.m_bframe = (softBodyData->m_pose->m_bframe!=0);
					psb->m_pose.m_bvolume = (softBodyData->m_pose->m_bvolume!=0);
					psb->m_pose.m_com.deSerializeFloat(softBodyData->m_pose->m_com);
					
					psb->m_pose.m_pos.resize(softBodyData->m_pose->m_numPositions);
					for (i=0;i<softBodyData->m_pose->m_numPositions;i++)
					{
						psb->m_pose.m_pos[i].deSerializeFloat(softBodyData->m_pose->m_positions[i]);
					}
					psb->m_pose.m_rot.deSerializeFloat(softBodyData->m_pose->m_rot);
					psb->m_pose.m_scl.deSerializeFloat(softBodyData->m_pose->m_scale);
					psb->m_pose.m_wgh.resize(softBodyData->m_pose->m_numWeigts);
					for (i=0;i<softBodyData->m_pose->m_numWeigts;i++)
					{
						psb->m_pose.m_wgh[i] = softBodyData->m_pose->m_weights[i];
					}
					psb->m_pose.m_volume = softBodyData->m_pose->m_restVolume;
				}

#if 1
				psb->m_cfg.piterations=softBodyData->m_config.m_positionIterations;
				psb->m_cfg.diterations=softBodyData->m_config.m_driftIterations;
				psb->m_cfg.citerations=softBodyData->m_config.m_clusterIterations;
				psb->m_cfg.viterations=softBodyData->m_config.m_velocityIterations;

				//psb->setTotalMass(0.1);
				psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_)softBodyData->m_config.m_aeroModel;
				psb->m_cfg.kLF = softBodyData->m_config.m_lift;
				psb->m_cfg.kDG = softBodyData->m_config.m_drag;
				psb->m_cfg.kMT = softBodyData->m_config.m_poseMatch;
				psb->m_cfg.collisions = softBodyData->m_config.m_collisionFlags;
				psb->m_cfg.kDF = softBodyData->m_config.m_dynamicFriction;
				psb->m_cfg.kDP = softBodyData->m_config.m_damping;
				psb->m_cfg.kPR = softBodyData->m_config.m_pressure;
				psb->m_cfg.kVC = softBodyData->m_config.m_volume;
				psb->m_cfg.kAHR = softBodyData->m_config.m_anchorHardness;
				psb->m_cfg.kKHR = softBodyData->m_config.m_kineticContactHardness;
				psb->m_cfg.kSHR = softBodyData->m_config.m_softContactHardness;
				psb->m_cfg.kSRHR_CL = softBodyData->m_config.m_softRigidClusterHardness;
				psb->m_cfg.kSKHR_CL = softBodyData->m_config.m_softKineticClusterHardness;
				psb->m_cfg.kSSHR_CL = softBodyData->m_config.m_softSoftClusterHardness;
#endif

//				pm->m_kLST				=	1;

#if 1
				//clusters
				if (softBodyData->m_numClusters)
				{
					m_clusterBodyMap.insert(softBodyData->m_clusters,psb);
					int j;
					psb->m_clusters.resize(softBodyData->m_numClusters);
					for (i=0;i<softBodyData->m_numClusters;i++)
					{
						psb->m_clusters[i] = new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();
						psb->m_clusters[i]->m_adamping = softBodyData->m_clusters[i].m_adamping;
						psb->m_clusters[i]->m_av.deSerializeFloat(softBodyData->m_clusters[i].m_av);
						psb->m_clusters[i]->m_clusterIndex = softBodyData->m_clusters[i].m_clusterIndex;
						psb->m_clusters[i]->m_collide = (softBodyData->m_clusters[i].m_collide!=0);
						psb->m_clusters[i]->m_com.deSerializeFloat(softBodyData->m_clusters[i].m_com);
						psb->m_clusters[i]->m_containsAnchor = (softBodyData->m_clusters[i].m_containsAnchor!=0);
						psb->m_clusters[i]->m_dimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[0]);
						psb->m_clusters[i]->m_dimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[1]);

						psb->m_clusters[i]->m_framerefs.resize(softBodyData->m_clusters[i].m_numFrameRefs);
						for (j=0;j<softBodyData->m_clusters[i].m_numFrameRefs;j++)
						{
							psb->m_clusters[i]->m_framerefs[j].deSerializeFloat(softBodyData->m_clusters[i].m_framerefs[j]);
						}
						psb->m_clusters[i]->m_nodes.resize(softBodyData->m_clusters[i].m_numNodes);
						for (j=0;j<softBodyData->m_clusters[i].m_numNodes;j++)
						{
							int nodeIndex = softBodyData->m_clusters[i].m_nodeIndices[j];
							psb->m_clusters[i]->m_nodes[j] = &psb->m_nodes[nodeIndex];
						}

						psb->m_clusters[i]->m_masses.resize(softBodyData->m_clusters[i].m_numMasses);
						for (j=0;j<softBodyData->m_clusters[i].m_numMasses;j++)
						{
							psb->m_clusters[i]->m_masses[j] = softBodyData->m_clusters[i].m_masses[j];
						}
						psb->m_clusters[i]->m_framexform.deSerializeFloat(softBodyData->m_clusters[i].m_framexform);
						psb->m_clusters[i]->m_idmass = softBodyData->m_clusters[i].m_idmass;
						psb->m_clusters[i]->m_imass = softBodyData->m_clusters[i].m_imass;
						psb->m_clusters[i]->m_invwi.deSerializeFloat(softBodyData->m_clusters[i].m_invwi);
						psb->m_clusters[i]->m_ldamping = softBodyData->m_clusters[i].m_ldamping;
						psb->m_clusters[i]->m_locii.deSerializeFloat(softBodyData->m_clusters[i].m_locii);
						psb->m_clusters[i]->m_lv.deSerializeFloat(softBodyData->m_clusters[i].m_lv);
						psb->m_clusters[i]->m_matching = softBodyData->m_clusters[i].m_matching;
						psb->m_clusters[i]->m_maxSelfCollisionImpulse = 0;//softBodyData->m_clusters[i].m_maxSelfCollisionImpulse;
						psb->m_clusters[i]->m_ndamping = softBodyData->m_clusters[i].m_ndamping;
						psb->m_clusters[i]->m_ndimpulses = softBodyData->m_clusters[i].m_ndimpulses;
						psb->m_clusters[i]->m_nvimpulses = softBodyData->m_clusters[i].m_nvimpulses;
						psb->m_clusters[i]->m_selfCollisionImpulseFactor = softBodyData->m_clusters[i].m_selfCollisionImpulseFactor;
						psb->m_clusters[i]->m_vimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[0]);
						psb->m_clusters[i]->m_vimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[1]);
						
					}
					//psb->initializeClusters();
					//psb->updateClusters();

				}
#else

				psb->m_cfg.piterations	=	2;
				psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS;
				//psb->setTotalMass(50,true);
				//psb->generateClusters(64);
				//psb->m_cfg.kDF=1;
				psb->generateClusters(8);


#endif //



				psb->updateConstants();
				m_softRigidWorld->getWorldInfo().m_dispatcher = m_softRigidWorld->getDispatcher();
				
				m_softRigidWorld->addSoftBody(psb);


			}
		}


		//now the soft body joints
		for (i=0;i<bulletFile2->m_softBodies.size();i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btAssert(0); //not yet
				//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
			} else
			{
				btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile2->m_softBodies[i];
				btSoftBody** sbp = m_softBodyMap.find(softBodyData);
				if (sbp && *sbp)
				{
					btSoftBody* sb = *sbp;
					for (int i=0;i<softBodyData->m_numJoints;i++)
					{
						btSoftBodyJointData* sbjoint = &softBodyData->m_joints[i];


						btSoftBody::Body bdyB;

						btSoftBody* sbB = 0;
						btTransform transA;
						transA.setIdentity();
						transA = sb->m_clusters[0]->m_framexform;

						btCollisionObject** colBptr = m_bodyMap.find(sbjoint->m_bodyB);
						if (colBptr && *colBptr)
						{
							btRigidBody* rbB = btRigidBody::upcast(*colBptr);
							if (rbB)
							{
								bdyB = rbB;
							} else
							{
								bdyB = *colBptr;
							}
						}


						btSoftBody** bodyBptr = m_clusterBodyMap.find(sbjoint->m_bodyB);
						if (bodyBptr && *bodyBptr )
						{
							sbB = *bodyBptr;
							bdyB = sbB->m_clusters[0];
						}


						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Linear)
						{
							btSoftBody::LJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.position = transA*relA;
							sb->appendLinearJoint(specs,sb->m_clusters[0],bdyB);
						}

						if (sbjoint->m_jointType==btSoftBody::Joint::eType::Angular)
						{
							btSoftBody::AJoint::Specs specs;
							specs.cfm = sbjoint->m_cfm;
							specs.erp = sbjoint->m_erp;
							specs.split = sbjoint->m_split;
							btVector3 relA;
							relA.deSerializeFloat(sbjoint->m_refs[0]);
							specs.axis = transA.getBasis()*relA;
							sb->appendAngularJoint(specs,sb->m_clusters[0],bdyB);
						}
					}
				}

			}
		}

		return result;

	}
};
#endif //DESERIALIZE_SOFT_BODIES


SerializeDemo::~SerializeDemo()
{
	m_fileLoader->deleteAllData();
	delete m_fileLoader;	
	exitPhysics();
}

void	SerializeDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	setupEmptyDynamicsWorld();
	
#ifdef DESERIALIZE_SOFT_BODIES
	m_fileLoader = new MySoftBulletWorldImporter((btSoftRigidDynamicsWorld*)m_dynamicsWorld);
#else
	m_fileLoader = new btBulletWorldImporter(m_dynamicsWorld);
#endif //DESERIALIZE_SOFT_BODIES
//	fileLoader->setVerboseMode(true);

	if (!m_fileLoader->loadFile("testFile.bullet"))
//	if (!m_fileLoader->loadFile("../SoftDemo/testFile.bullet"))
	{
		///create a few basic rigid bodies and save them to testFile.bullet
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
		btCollisionObject* groundObject = 0;

		
		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-50,0));

		//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
		{
			btScalar mass(0.);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				groundShape->calculateLocalInertia(mass,localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);

			//add the body to the dynamics world
			m_dynamicsWorld->addRigidBody(body);
			groundObject = body;
		}


		{
			//create a few dynamic rigidbodies
			// Re-using the same collision is better for memory usage and performance

			int numSpheres = 2;
			btVector3 positions[2] = {btVector3(0.1f,0.2f,0.3f),btVector3(0.4f,0.5f,0.6f)};
			btScalar	radii[2] = {0.3f,0.4f};

			btMultiSphereShape* colShape = new btMultiSphereShape(positions,radii,numSpheres);

			//btCollisionShape* colShape = new btCapsuleShapeZ(SCALING*1,SCALING*1);
			//btCollisionShape* colShape = new btCylinderShapeZ(btVector3(SCALING*1,SCALING*1,SCALING*1));
			//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
			//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
			m_collisionShapes.push_back(colShape);

			/// Create Dynamic Objects
			btTransform startTransform;
			startTransform.setIdentity();

			btScalar	mass(1.f);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				colShape->calculateLocalInertia(mass,localInertia);

			float start_x = START_POS_X - ARRAY_SIZE_X/2;
			float start_y = START_POS_Y;
			float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

			for (int k=0;k<ARRAY_SIZE_Y;k++)
			{
				for (int i=0;i<ARRAY_SIZE_X;i++)
				{
					for(int j = 0;j<ARRAY_SIZE_Z;j++)
					{
						startTransform.setOrigin(SCALING*btVector3(
											btScalar(2.0*i + start_x),
											btScalar(20+2.0*k + start_y),
											btScalar(2.0*j + start_z)));

				
						//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
						btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
						btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
						btRigidBody* body = new btRigidBody(rbInfo);
						
						body->setActivationState(ISLAND_SLEEPING);

						m_dynamicsWorld->addRigidBody(body);
						body->setActivationState(ISLAND_SLEEPING);
					}
				}
			}
		}

		int maxSerializeBufferSize = 1024*1024*5;

		btDefaultSerializer*	serializer = new btDefaultSerializer(maxSerializeBufferSize);

		static const char* groundName = "GroundName";
		serializer->registerNameForPointer(groundObject, groundName);

		for (int i=0;i<m_collisionShapes.size();i++)
		{
			char* name = new char[20];
			
			sprintf(name,"name%d",i);
			serializer->registerNameForPointer(m_collisionShapes[i],name);
		}

		btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*(btRigidBody*)getDynamicsWorld()->getCollisionObjectArray()[2],btVector3(0,1,0));
		m_dynamicsWorld->addConstraint(p2p);
		
		const char* name = "constraintje";
		serializer->registerNameForPointer(p2p,name);

		m_dynamicsWorld->serialize(serializer);
		
		FILE* f2 = fopen("testFile.bullet","wb");
		fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1,f2);
		fclose(f2);
	}

	//clientResetScene();

}
	

void	SerializeDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




