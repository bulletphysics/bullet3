


#include "PhysicsServer.h"

#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"

#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"

#include "SharedMemoryCommon.h"
//const char* blaatnaam = "basename";
struct UrdfLinkNameMapUtil
{
	btMultiBody* m_mb;
	btDefaultSerializer* m_memSerializer;

	UrdfLinkNameMapUtil():m_mb(0),m_memSerializer(0)
	{
	}
};

class PhysicsServer : public SharedMemoryCommon
{
	SharedMemoryInterface* m_sharedMemory;
    SharedMemoryExampleData* m_testBlock1;

	btAlignedObjectArray<btJointFeedback*> m_jointFeedbacks;
	btAlignedObjectArray<btBulletWorldImporter*> m_worldImporters;
	btAlignedObjectArray<UrdfLinkNameMapUtil*> m_urdfLinkNameMapper;
	btHashMap<btHashInt, btMultiBodyJointMotor*>	m_multiBodyJointMotorMap;
	btAlignedObjectArray<std::string*> m_strings;
	bool m_wantsShutdown;

	void	createJointMotors(btMultiBody* body);
	bool supportsJointMotor(btMultiBody* body, int linkIndex);
public:
    
	PhysicsServer(GUIHelperInterface* helper);
    
	virtual ~PhysicsServer();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
    void releaseSharedMemory();
    
    bool loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                  bool useMultiBody, bool useFixedBase);
    
	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={0,0,0};//-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
    
    virtual bool wantsTermination();
};

PhysicsServer::PhysicsServer(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_testBlock1(0),
m_wantsShutdown(false)
{
	b3Printf("Started PhysicsServer\n");
	bool useServer = true;
	#ifdef _WIN32
	m_sharedMemory = new Win32SharedMemoryServer();
	#else
	m_sharedMemory = new PosixSharedMemory();
	#endif
}

void PhysicsServer::releaseSharedMemory()
{
	b3Printf("releaseSharedMemory1\n");
    if (m_testBlock1)
    {
		b3Printf("m_testBlock1\n");
        m_testBlock1->m_magicId = 0;
        b3Printf("magic id = %d\n",m_testBlock1->m_magicId);
        btAssert(m_sharedMemory);
        m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
    }
    if (m_sharedMemory)
    {
		b3Printf("m_sharedMemory\n");
        delete m_sharedMemory;
        m_sharedMemory = 0;
        m_testBlock1 = 0;
    }
}

PhysicsServer::~PhysicsServer()
{
    releaseSharedMemory();
}

void	PhysicsServer::initPhysics()
{
	///for this testing we use Z-axis up
	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

    createEmptyDynamicsWorld();
	//todo: create a special debug drawer that will cache the lines, so we can send the debug info over the wire
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	btVector3 grav(0,0,0);
	grav[upAxis] = 0;//-9.8;
	this->m_dynamicsWorld->setGravity(grav);
    
    m_testBlock1 = (SharedMemoryExampleData*) m_sharedMemory->allocateSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
    
//    btAssert(m_testBlock1);
    if (m_testBlock1)
    {
      //  btAssert(m_testBlock1->m_magicId != SHARED_MEMORY_MAGIC_NUMBER);
        if (m_testBlock1->m_magicId == SHARED_MEMORY_MAGIC_NUMBER)
        {
            b3Printf("Warning: shared memory is already initialized, did you already spawn a server?\n");
        }
        
        m_testBlock1->m_numClientCommands = 0;
        m_testBlock1->m_numServerCommands = 0;
        m_testBlock1->m_numProcessedClientCommands=0;
        m_testBlock1->m_numProcessedServerCommands=0;
        
        m_testBlock1->m_magicId = SHARED_MEMORY_MAGIC_NUMBER;
        b3Printf("Shared memory succesfully allocated\n");
    } else
    {
        b3Error("Couldn't allocated shared memory, is it implemented on your operating system?\n");
    }
}


bool PhysicsServer::wantsTermination()
{
    return m_wantsShutdown;
}


bool PhysicsServer::supportsJointMotor(btMultiBody* mb, int mbLinkIndex)
{
	bool canHaveMotor = (mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::eRevolute
			||mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::ePrismatic);
	return canHaveMotor;

}

//for testing, create joint motors for revolute and prismatic joints
void	PhysicsServer::createJointMotors(btMultiBody* mb)
{
	int numLinks = mb->getNumLinks();
	for (int i=0;i<numLinks;i++)
	{
		int mbLinkIndex = i;
							
		if (supportsJointMotor(mb,mbLinkIndex))
		{
			float maxMotorImpulse = 0.f;
			int dof = 0;
			btScalar desiredVelocity = 0.f;
			btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb,mbLinkIndex,dof,desiredVelocity,maxMotorImpulse);
			//motor->setMaxAppliedImpulse(0);
			m_multiBodyJointMotorMap.insert(mbLinkIndex,motor);
			m_dynamicsWorld->addMultiBodyConstraint(motor);
		}

	}
}
bool PhysicsServer::loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase)
{
 
    BulletURDFImporter u2b(m_guiHelper);
    bool loadOk =  u2b.loadURDF(fileName);
    if (loadOk)
    {
        b3Printf("loaded %s OK!", fileName);
        
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(pos);
        tr.setRotation(orn);
        int rootLinkIndex = u2b.getRootLinkIndex();
        //                      printf("urdf root link index = %d\n",rootLinkIndex);
        MyMultiBodyCreator creation(m_guiHelper);
        
        ConvertURDF2Bullet(u2b,creation, tr,m_dynamicsWorld,useMultiBody,u2b.getPathPrefix());
        btMultiBody* mb = creation.getBulletMultiBody();
		if (useMultiBody)
		{
			if (mb)
			{
				createJointMotors(mb);

				//serialize the btMultiBody and send the data to the client. This is one way to get the link/joint names across the (shared memory) wire
			    UrdfLinkNameMapUtil* util = new UrdfLinkNameMapUtil;
			    m_urdfLinkNameMapper.push_back(util);
			    util->m_mb = mb;
			    util->m_memSerializer = new btDefaultSerializer(SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE,(unsigned char*)m_testBlock1->m_bulletStreamDataServerToClient);
			    //disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
				util->m_memSerializer->m_skipPointers.insert(mb->getBaseCollider(),0);

			    for (int i=0;i<mb->getNumLinks();i++)
                {
					//disable serialization of the collision objects
                   util->m_memSerializer->m_skipPointers.insert(mb->getLink(i).m_collider,0);
				   int urdfLinkIndex = creation.m_mb2urdfLink[i];
				   std::string* linkName = new std::string(u2b.getLinkName(urdfLinkIndex).c_str());
				   m_strings.push_back(linkName);
				   util->m_memSerializer->registerNameForPointer(linkName->c_str(),linkName->c_str());
				   mb->getLink(i).m_linkName = linkName->c_str();

				   std::string* jointName = new std::string(u2b.getJointName(urdfLinkIndex).c_str());
				   m_strings.push_back(jointName);
				   util->m_memSerializer->registerNameForPointer(jointName->c_str(),jointName->c_str());
				   mb->getLink(i).m_jointName = jointName->c_str();
                }
				
				std::string* baseName = new std::string(u2b.getLinkName(u2b.getRootLinkIndex()));
				m_strings.push_back(baseName);

				
				util->m_memSerializer->registerNameForPointer(baseName->c_str(),baseName->c_str());
				mb->setBaseName(baseName->c_str());
				
				
                util->m_memSerializer->insertHeader();
                
                int len = mb->calculateSerializeBufferSize();
                btChunk* chunk = util->m_memSerializer->allocate(len,1);
                const char* structType = mb->serialize(chunk->m_oldPtr, util->m_memSerializer);
                util->m_memSerializer->finalizeChunk(chunk,structType,BT_MULTIBODY_CODE,mb);
                
				return true;
			} else
			{
				b3Warning("No multibody loaded from URDF. Could add btRigidBody+btTypedConstraint solution later.");
				return false;
			}
		} else
		{
			for (int i=0;i<m_dynamicsWorld->getNumConstraints();i++)
			{
				btTypedConstraint* c = m_dynamicsWorld->getConstraint(i);
				btJointFeedback* fb = new btJointFeedback();
				m_jointFeedbacks.push_back(fb);
				c->setJointFeedback(fb);

			}
			return true;
		}
        
    }
    
    return false;
}

void	PhysicsServer::stepSimulation(float deltaTime)
{

    bool wantsShutdown = false;
    
    if (m_testBlock1)
    {
        ///we ignore overflow of integer for now
        if (m_testBlock1->m_numClientCommands> m_testBlock1->m_numProcessedClientCommands)
        {
            
            //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
            btAssert(m_testBlock1->m_numClientCommands==m_testBlock1->m_numProcessedClientCommands+1);
            
            const SharedMemoryCommand& clientCmd =m_testBlock1->m_clientCommands[0];
             m_testBlock1->m_numProcessedClientCommands++;
            
            //consume the command
            switch (clientCmd.m_type)
            {
				case CMD_SEND_BULLET_DATA_STREAM:
                {
					b3Printf("Processed CMD_SEND_BULLET_DATA_STREAM length %d",clientCmd.m_dataStreamArguments.m_streamChunkLength);
					
					btBulletWorldImporter* worldImporter = new btBulletWorldImporter(m_dynamicsWorld);
					this->m_worldImporters.push_back(worldImporter);
					bool completedOk = worldImporter->loadFileFromMemory(m_testBlock1->m_bulletStreamDataClientToServer,clientCmd.m_dataStreamArguments.m_streamChunkLength);
					
					m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

					SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
 
                    if (completedOk)
                    {
                        serverCmd.m_type =CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED;
                    } else
                    {
                        serverCmd.m_type =CMD_BULLET_DATA_STREAM_RECEIVED_FAILED;
                    
                    }
                    m_testBlock1->m_numServerCommands++;
					break;
				}
                case CMD_LOAD_URDF:
                {
                    const UrdfArgs& urdfArgs = clientCmd.m_urdfArguments;
                    b3Printf("Processed CMD_LOAD_URDF:%s", urdfArgs.m_urdfFileName);

                    //load the actual URDF and send a report: completed or failed
                    bool completedOk = loadUrdf(urdfArgs.m_urdfFileName,
                                                btVector3(urdfArgs.m_initialPosition[0],
                                                          urdfArgs.m_initialPosition[1],
                                                          urdfArgs.m_initialPosition[2]),
                                                btQuaternion(urdfArgs.m_initialOrientation[0],
                                                             urdfArgs.m_initialOrientation[1],
                                                             urdfArgs.m_initialOrientation[2],
                                                             urdfArgs.m_initialOrientation[3]),
                                                urdfArgs.m_useMultiBody, urdfArgs.m_useFixedBase);
                    SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
 
                    if (completedOk)
                    {
						if (this->m_urdfLinkNameMapper.size())
						{
							serverCmd.m_dataStreamArguments.m_streamChunkLength = m_urdfLinkNameMapper.at(m_urdfLinkNameMapper.size()-1)->m_memSerializer->getCurrentBufferSize();
						}
                        serverCmd.m_type =CMD_URDF_LOADING_COMPLETED;
                    } else
                    {
                        serverCmd.m_type =CMD_URDF_LOADING_FAILED;
                    
                    }
                    m_testBlock1->m_numServerCommands++;
                    

                    
                    break;
                }
                case CMD_SEND_DESIRED_STATE:
                    {
                        	//for (int i=0;i<MAX_DEGREE_OF_FREEDOM;i++)
                            //{
                            //    m_testBlock1->m_desiredStateForceTorque[i] = 100;
                            //}
                            
                            b3Printf("Processed CMD_SEND_DESIRED_STATE");
                            if (m_dynamicsWorld->getNumMultibodies()>0)
                            {
                                btMultiBody* mb = m_dynamicsWorld->getMultiBody(0);
                                btAssert(mb);
                                
                                switch (clientCmd.m_sendDesiredStateCommandArgument.m_controlMode)
                                {
                                case CONTROL_MODE_TORQUE:
                                    {
										b3Printf("Using CONTROL_MODE_TORQUE");
                                        mb->clearForcesAndTorques();
                                        
                                        int torqueIndex = 0;
                                        btVector3 f(m_testBlock1->m_desiredStateForceTorque[0],
                                                    m_testBlock1->m_desiredStateForceTorque[1],
                                                    m_testBlock1->m_desiredStateForceTorque[2]);
                                        btVector3 t(m_testBlock1->m_desiredStateForceTorque[3],
                                                    m_testBlock1->m_desiredStateForceTorque[4],
                                                    m_testBlock1->m_desiredStateForceTorque[5]);
                                        torqueIndex+=6;
                                        mb->addBaseForce(f);
                                        mb->addBaseTorque(t);
                                        for (int link=0;link<mb->getNumLinks();link++)
                                        {
                                            
                                            for (int dof=0;dof<mb->getLink(link).m_dofCount;dof++)
                                            {               
                                                double torque = m_testBlock1->m_desiredStateForceTorque[torqueIndex];
                                                mb->addJointTorqueMultiDof(link,dof,torque);
                                                torqueIndex++;
                                            }
                                        }
                                        break;
                                    }
								case CONTROL_MODE_VELOCITY:
									{
										int numMotors = 0;
										//find the joint motors and apply the desired velocity and maximum force/torque
										if (m_dynamicsWorld->getNumMultibodies()>0)
										{
											btMultiBody* mb = m_dynamicsWorld->getMultiBody(0);
											int dofIndex = 6;//skip the 3 linear + 3 angular degree of freedom entries of the base
											for (int link=0;link<mb->getNumLinks();link++)
											{
												if (supportsJointMotor(mb,link))
												{
													
													btMultiBodyJointMotor** motorPtr  = m_multiBodyJointMotorMap[link];
													if (motorPtr)
													{
														btMultiBodyJointMotor* motor = *motorPtr;
														btScalar desiredVelocity = m_testBlock1->m_desiredStateQdot[dofIndex];
														motor->setVelocityTarget(desiredVelocity);

														btScalar physicsDeltaTime=1./60.;//todo: set the physicsDeltaTime explicitly in the API, separate from the 'stepSimulation'

														btScalar maxImp = m_testBlock1->m_desiredStateForceTorque[dofIndex]*physicsDeltaTime;
														motor->setMaxAppliedImpulse(maxImp);
														numMotors++;

													}
												}
												dofIndex += mb->getLink(link).m_dofCount;
											}
										}
										b3Printf("Using CONTROL_MODE_TORQUE with %d motors", numMotors);
										break;
									}
                                default:
                                    {
                                        b3Printf("m_controlMode not implemented yet");
                                        break;
                                    }
                                    
                                }
                            }
                            
                            
                            
                            SharedMemoryCommand& serverCmd = m_testBlock1->m_serverCommands[0];
							serverCmd.m_type = CMD_DESIRED_STATE_RECEIVED_COMPLETED;
							m_testBlock1->m_numServerCommands++;
                        break;
                    }
				case CMD_REQUEST_ACTUAL_STATE:
					{
	                    b3Printf("Sending the actual state (Q,U)");
						if (m_dynamicsWorld->getNumMultibodies()>0)
						{
							btMultiBody* mb = m_dynamicsWorld->getMultiBody(0);
							SharedMemoryCommand& serverCmd = m_testBlock1->m_serverCommands[0];
							serverCmd.m_type = CMD_ACTUAL_STATE_UPDATE_COMPLETED;

							serverCmd.m_sendActualStateArgs.m_bodyUniqueId = 0;
							int totalDegreeOfFreedomQ = 0;
							int totalDegreeOfFreedomU = 0; 
							
							//always add the base, even for static (non-moving objects)
							//so that we can easily move the 'fixed' base when needed
							//do we don't use this conditional "if (!mb->hasFixedBase())"
							{
								btTransform tr;
								tr.setOrigin(mb->getBasePos());
								tr.setRotation(mb->getWorldToBaseRot().inverse());
								
								//base position in world space, carthesian
								m_testBlock1->m_actualStateQ[0] = tr.getOrigin()[0];
								m_testBlock1->m_actualStateQ[1] = tr.getOrigin()[1];
								m_testBlock1->m_actualStateQ[2] = tr.getOrigin()[2];

								//base orientation, quaternion x,y,z,w, in world space, carthesian
								m_testBlock1->m_actualStateQ[3] = tr.getRotation()[0]; 
								m_testBlock1->m_actualStateQ[4] = tr.getRotation()[1];
								m_testBlock1->m_actualStateQ[5] = tr.getRotation()[2];
								m_testBlock1->m_actualStateQ[6] = tr.getRotation()[3];
								totalDegreeOfFreedomQ +=7;//pos + quaternion

								//base linear velocity (in world space, carthesian)
								m_testBlock1->m_actualStateQdot[0] = mb->getBaseVel()[0];
								m_testBlock1->m_actualStateQdot[1] = mb->getBaseVel()[1];
								m_testBlock1->m_actualStateQdot[2] = mb->getBaseVel()[2];
	
								//base angular velocity (in world space, carthesian)
								m_testBlock1->m_actualStateQdot[3] = mb->getBaseOmega()[0];
								m_testBlock1->m_actualStateQdot[4] = mb->getBaseOmega()[1];
								m_testBlock1->m_actualStateQdot[5] = mb->getBaseOmega()[2];
								totalDegreeOfFreedomU += 6;//3 linear and 3 angular DOF
							}
							for (int l=0;l<mb->getNumLinks();l++)
							{
								for (int d=0;d<mb->getLink(l).m_posVarCount;d++)
								{
									m_testBlock1->m_actualStateQ[totalDegreeOfFreedomQ++] = mb->getJointPosMultiDof(l)[d];
								}
								for (int d=0;d<mb->getLink(l).m_dofCount;d++)
								{
									m_testBlock1->m_actualStateQdot[totalDegreeOfFreedomU++] = mb->getJointVelMultiDof(l)[d];
								}

							}

							serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomQ = totalDegreeOfFreedomQ;
							serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomU = totalDegreeOfFreedomU;
							
							
						} else
						{
							b3Warning("Request state but no multibody available");
							//rigid bodies?
						}
/*

											//now we send back the actual q, q' and force/torque and IMU sensor values
					for (int i=0;i<m_jointFeedbacks.size();i++)
					{
						printf("Applied force A:(%f,%f,%f), torque A:(%f,%f,%f)\nForce B:(%f,%f,%f), torque B:(%f,%f,%f)\n", 
							m_jointFeedbacks[i]->m_appliedForceBodyA.x(),
							m_jointFeedbacks[i]->m_appliedForceBodyA.y(),
							m_jointFeedbacks[i]->m_appliedForceBodyA.z(),
							m_jointFeedbacks[i]->m_appliedTorqueBodyA.x(),
							m_jointFeedbacks[i]->m_appliedTorqueBodyA.y(),
							m_jointFeedbacks[i]->m_appliedTorqueBodyA.z(),
							m_jointFeedbacks[i]->m_appliedForceBodyB.x(),
							m_jointFeedbacks[i]->m_appliedForceBodyB.y(),
							m_jointFeedbacks[i]->m_appliedForceBodyB.z(),
							m_jointFeedbacks[i]->m_appliedTorqueBodyB.x(),
							m_jointFeedbacks[i]->m_appliedTorqueBodyB.y(),
							m_jointFeedbacks[i]->m_appliedTorqueBodyB.z());
					}
					*/

						m_testBlock1->m_numServerCommands++;

						
						break;
					}
                case CMD_STEP_FORWARD_SIMULATION:
                {
                   
                    b3Printf("Step simulation request");
                    double timeStep = clientCmd.m_stepSimulationArguments.m_deltaTimeInSeconds;
                    m_dynamicsWorld->stepSimulation(timeStep);
                    
                    SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
                    
                    serverCmd.m_type =CMD_STEP_FORWARD_SIMULATION_COMPLETED;
                    m_testBlock1->m_numServerCommands++;

                    break;
                }
                case CMD_SHUTDOWN:
                {
                    wantsShutdown = true;
                    break;
                }
				case CMD_CREATE_BOX_COLLISION_SHAPE:
					{
						btVector3 halfExtents(30,30,1);
						btTransform startTrans;
						startTrans.setIdentity();
						startTrans.setOrigin(btVector3(0,0,-4));
						btCollisionShape* shape = createBoxShape(halfExtents);
						btScalar mass = 0.f;
						createRigidBody(mass,startTrans,shape);
						this->m_guiHelper->autogenerateGraphicsObjects(this->m_dynamicsWorld);
						SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
						serverCmd.m_type =CMD_STEP_FORWARD_SIMULATION_COMPLETED;
						m_testBlock1->m_numServerCommands++;
						break;
					}
                default:
                {
                    b3Error("Unsupported command encountered");
                    btAssert(0);
                }
            };
            
           
            
            //process the command right now
            
        }
    }
    if (wantsShutdown)
    {
		b3Printf("releaseSharedMemory!\n");
		
        m_wantsShutdown = true;
        releaseSharedMemory();
    }
}


class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsServer(options.m_guiHelper);
}


