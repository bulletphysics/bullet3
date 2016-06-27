#include "PhysicsClientC_API.h"
#include "PhysicsClientSharedMemory.h"
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3Vector3.h"
#include <string.h>
#include "SharedMemoryCommands.h"


b3SharedMemoryCommandHandle	b3LoadSdfCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
	command->m_type = CMD_LOAD_SDF;
	int len = strlen(sdfFileName);
	if (len<MAX_SDF_FILENAME_LENGTH)
	{
		strcpy(command->m_sdfArguments.m_sdfFileName,sdfFileName);
	} else
	{
		command->m_sdfArguments.m_sdfFileName[0] = 0;
	}
	command->m_updateFlags = SDF_ARGS_FILE_NAME;
	
	return (b3SharedMemoryCommandHandle) command;
}



b3SharedMemoryCommandHandle	b3LoadUrdfCommandInit(b3PhysicsClientHandle physClient, const char* urdfFileName)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    
    
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
	command->m_type = CMD_LOAD_URDF;
	int len = strlen(urdfFileName);
	if (len<MAX_URDF_FILENAME_LENGTH)
	{
		strcpy(command->m_urdfArguments.m_urdfFileName,urdfFileName);
	} else
	{
		command->m_urdfArguments.m_urdfFileName[0] = 0;
	}
	command->m_updateFlags = URDF_ARGS_FILE_NAME;
	
	return (b3SharedMemoryCommandHandle) command;
}


int	b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_LOAD_URDF);
    command->m_updateFlags |=URDF_ARGS_USE_FIXED_BASE;
    command->m_urdfArguments.m_useFixedBase = useFixedBase;
    
    return 0;
}



int	b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_LOAD_URDF);
	command->m_urdfArguments.m_initialPosition[0] = startPosX;
	command->m_urdfArguments.m_initialPosition[1] = startPosY;
	command->m_urdfArguments.m_initialPosition[2] = startPosZ;
	command->m_updateFlags|=URDF_ARGS_INITIAL_POSITION;
	return 0;
}

int	b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_LOAD_URDF);
	command->m_urdfArguments.m_initialOrientation[0] = startOrnX;
	command->m_urdfArguments.m_initialOrientation[1] = startOrnY;
	command->m_urdfArguments.m_initialOrientation[2] = startOrnZ;
	command->m_urdfArguments.m_initialOrientation[3] = startOrnW;
	command->m_updateFlags|=URDF_ARGS_INITIAL_ORIENTATION;
	return 0;
}

b3SharedMemoryCommandHandle     b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
	command->m_type = CMD_SEND_PHYSICS_SIMULATION_PARAMETERS;
	command->m_updateFlags = 0;

    return (b3SharedMemoryCommandHandle) command;
}
int     b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx,double gravy, double gravz)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_gravityAcceleration[0] = gravx;
	command->m_physSimParamArgs.m_gravityAcceleration[1] = gravy;
	command->m_physSimParamArgs.m_gravityAcceleration[2] = gravz; 
	command->m_updateFlags |= SIM_PARAM_UPDATE_GRAVITY;
	return 0;
}

int	b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_updateFlags |= SIM_PARAM_UPDATE_DELTA_TIME;	
	command->m_physSimParamArgs.m_deltaTime = timeStep;
	return 0;
}

b3SharedMemoryCommandHandle	b3InitStepSimulationCommand(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
	command->m_type = CMD_STEP_FORWARD_SIMULATION;
	command->m_updateFlags = 0;
	 return (b3SharedMemoryCommandHandle) command;
}

b3SharedMemoryCommandHandle     b3InitResetSimulationCommand(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_RESET_SIMULATION;
    command->m_updateFlags = 0;

     return (b3SharedMemoryCommandHandle) command;

}


b3SharedMemoryCommandHandle  b3JointControlCommandInit(b3PhysicsClientHandle physClient, int controlMode)
{
    return b3JointControlCommandInit2(physClient,0,controlMode);
}

b3SharedMemoryCommandHandle b3JointControlCommandInit2( b3PhysicsClientHandle physClient, int bodyUniqueId, int controlMode)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
	command->m_type = CMD_SEND_DESIRED_STATE;
    command->m_sendDesiredStateCommandArgument.m_controlMode = controlMode;
	command->m_sendDesiredStateCommandArgument.m_bodyUniqueId = bodyUniqueId;
	command->m_updateFlags = 0;
    for (int i=0;i<MAX_DEGREE_OF_FREEDOM;i++)
    {
        command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[i] = 0;
    }
    return (b3SharedMemoryCommandHandle) command;
}

int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateQ[qIndex] = value;
	command->m_updateFlags |= SIM_DESIRED_STATE_HAS_Q;
    command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[qIndex] |= SIM_DESIRED_STATE_HAS_Q;

    return 0;
}

int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_Kp[dofIndex] = value;
	command->m_updateFlags |= SIM_DESIRED_STATE_HAS_KP;
    command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_KP;

    return 0;
}

int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_Kd[dofIndex] = value;
	command->m_updateFlags |= SIM_DESIRED_STATE_HAS_KD;
    command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_KD;

    return 0;
}

int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateQdot[dofIndex] = value;
	command->m_updateFlags |= SIM_DESIRED_STATE_HAS_QDOT;
    command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_QDOT;

    return 0;
}


int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex,  double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
	command->m_updateFlags |= SIM_DESIRED_STATE_HAS_MAX_FORCE;
    command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_MAX_FORCE;

    return 0;
}

int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int  dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
    command->m_updateFlags |= SIM_DESIRED_STATE_HAS_MAX_FORCE;
    command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_MAX_FORCE;

    return 0;
}


b3SharedMemoryCommandHandle b3RequestActualStateCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_REQUEST_ACTUAL_STATE;
	command->m_requestActualStateInformationCommandArgument.m_bodyUniqueId = bodyUniqueId;
    return (b3SharedMemoryCommandHandle) command;
}

void b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, b3JointSensorState *state)
{
  const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
  b3Assert(status);
  int bodyIndex = status->m_sendActualStateArgs.m_bodyUniqueId;
  b3Assert(bodyIndex>=0);
  if (bodyIndex>=0)
  {
	  b3JointInfo info;
	  b3GetJointInfo(physClient, bodyIndex,jointIndex, &info);
	  state->m_jointPosition = status->m_sendActualStateArgs.m_actualStateQ[info.m_qIndex];
	  state->m_jointVelocity = status->m_sendActualStateArgs.m_actualStateQdot[info.m_uIndex];
	  for (int ii(0); ii < 6; ++ii) {
		state->m_jointForceTorque[ii] = status->m_sendActualStateArgs.m_jointReactionForces[6 * jointIndex + ii];
	  }
      state->m_jointMotorTorque = status->m_sendActualStateArgs.m_jointMotorForce[jointIndex];
  }
}

void b3GetLinkState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int linkIndex, b3LinkState *state)
{
  const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
  b3Assert(status);
  int bodyIndex = status->m_sendActualStateArgs.m_bodyUniqueId;
  b3Assert(bodyIndex>=0);
  if (bodyIndex>=0)
  {
    for (int i = 0; i < 3; ++i) 
    {
      state->m_worldPosition[i] = status->m_sendActualStateArgs.m_linkState[7 * linkIndex + i];
      state->m_localInertialPosition[i] = status->m_sendActualStateArgs.m_linkLocalInertialFrames[7 * linkIndex + i];
    }
    for (int i = 0; i < 4; ++i) 
    {
      state->m_worldOrientation[i] = status->m_sendActualStateArgs.m_linkState[7 * linkIndex + 3 + i];
      state->m_localInertialOrientation[i] = status->m_sendActualStateArgs.m_linkLocalInertialFrames[7 * linkIndex + 3 + i];
    }
  }
}

b3SharedMemoryCommandHandle b3CreateBoxShapeCommandInit(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_CREATE_BOX_COLLISION_SHAPE;
    command->m_updateFlags =0;
    return (b3SharedMemoryCommandHandle) command;
}

int	b3CreateBoxCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
    command->m_updateFlags |=BOX_SHAPE_HAS_INITIAL_POSITION;
    
    command->m_createBoxShapeArguments.m_initialPosition[0] = startPosX;
    command->m_createBoxShapeArguments.m_initialPosition[1] = startPosY;
    command->m_createBoxShapeArguments.m_initialPosition[2] = startPosZ;
    return 0;
}


int	b3CreateBoxCommandSetHalfExtents(b3SharedMemoryCommandHandle commandHandle, double halfExtentsX,double halfExtentsY,double halfExtentsZ)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
    command->m_updateFlags |=BOX_SHAPE_HAS_HALF_EXTENTS;
    
    command->m_createBoxShapeArguments.m_halfExtentsX = halfExtentsX;
    command->m_createBoxShapeArguments.m_halfExtentsY = halfExtentsY;
    command->m_createBoxShapeArguments.m_halfExtentsZ = halfExtentsZ;

    return 0;
}


int	b3CreateBoxCommandSetMass(b3SharedMemoryCommandHandle commandHandle, double mass)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
	command->m_updateFlags |=BOX_SHAPE_HAS_MASS;
	command->m_createBoxShapeArguments.m_mass = mass;
	return 0;
}


int	b3CreateBoxCommandSetCollisionShapeType(b3SharedMemoryCommandHandle commandHandle, int collisionShapeType)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
	command->m_updateFlags |=BOX_SHAPE_HAS_COLLISION_SHAPE_TYPE;
	command->m_createBoxShapeArguments.m_collisionShapeType = collisionShapeType;

	return 0;
}

int	b3CreateBoxCommandSetColorRGBA(b3SharedMemoryCommandHandle commandHandle, double red,double green,double blue, double alpha)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
	command->m_updateFlags |=BOX_SHAPE_HAS_COLOR;
	command->m_createBoxShapeArguments.m_colorRGBA[0] = red;
	command->m_createBoxShapeArguments.m_colorRGBA[1] = green;
	command->m_createBoxShapeArguments.m_colorRGBA[2] = blue;
	command->m_createBoxShapeArguments.m_colorRGBA[3] = alpha;
	return 0;
}

int	b3CreateBoxCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
    command->m_updateFlags |=BOX_SHAPE_HAS_INITIAL_ORIENTATION;
    
    command->m_createBoxShapeArguments.m_initialOrientation[0] = startOrnX;
    command->m_createBoxShapeArguments.m_initialOrientation[1] = startOrnY;
    command->m_createBoxShapeArguments.m_initialOrientation[2] = startOrnZ;
    command->m_createBoxShapeArguments.m_initialOrientation[3] = startOrnW;
    return 0;
}

b3SharedMemoryCommandHandle b3CreatePoseCommandInit(b3PhysicsClientHandle physClient, int bodyIndex)
{
    
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_INIT_POSE;
    command->m_updateFlags =0;
	command->m_initPoseArgs.m_bodyUniqueId = bodyIndex;
	//a bit slow, initialing the full range to zero...
	for (int i=0;i<MAX_DEGREE_OF_FREEDOM;i++)
    {
        command->m_initPoseArgs.m_hasInitialStateQ[i] = 0;
    }
    return (b3SharedMemoryCommandHandle) command;
}

int	b3CreatePoseCommandSetBasePosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_INIT_POSE);
    command->m_updateFlags |=INIT_POSE_HAS_INITIAL_POSITION;
	command->m_initPoseArgs.m_initialStateQ[0] = startPosX;
	command->m_initPoseArgs.m_initialStateQ[1] = startPosY;
	command->m_initPoseArgs.m_initialStateQ[2] = startPosZ;
	
	command->m_initPoseArgs.m_hasInitialStateQ[0] = 1;
	command->m_initPoseArgs.m_hasInitialStateQ[1] = 1;
	command->m_initPoseArgs.m_hasInitialStateQ[2] = 1;
	
	return 0;
}

int	b3CreatePoseCommandSetBaseOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_INIT_POSE);
    command->m_updateFlags |=INIT_POSE_HAS_INITIAL_ORIENTATION;
	command->m_initPoseArgs.m_initialStateQ[3] = startOrnX;
	command->m_initPoseArgs.m_initialStateQ[4] = startOrnY;
	command->m_initPoseArgs.m_initialStateQ[5] = startOrnZ;
	command->m_initPoseArgs.m_initialStateQ[6] = startOrnW;
	
	command->m_initPoseArgs.m_hasInitialStateQ[3] = 1;
	command->m_initPoseArgs.m_hasInitialStateQ[4] = 1;
	command->m_initPoseArgs.m_hasInitialStateQ[5] = 1;
	command->m_initPoseArgs.m_hasInitialStateQ[6] = 1;
	
	return 0;
}

int	b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_INIT_POSE);
    command->m_updateFlags |=INIT_POSE_HAS_JOINT_STATE;
	for (int i=0;i<numJointPositions;i++)
	{
		command->m_initPoseArgs.m_initialStateQ[i+7] = jointPositions[i];
		command->m_initPoseArgs.m_hasInitialStateQ[i+7] = 1;
	}
	return 0;
}

int	b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int jointIndex, double jointPosition)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_INIT_POSE);
	command->m_updateFlags |=INIT_POSE_HAS_JOINT_STATE;
	b3JointInfo info;
	b3GetJointInfo(physClient, command->m_initPoseArgs.m_bodyUniqueId,jointIndex, &info);
	btAssert((info.m_flags & JOINT_HAS_MOTORIZED_POWER) && info.m_qIndex >=0);
	if ((info.m_flags & JOINT_HAS_MOTORIZED_POWER) && info.m_qIndex >=0)
	{  
		command->m_initPoseArgs.m_initialStateQ[info.m_qIndex] = jointPosition;
		command->m_initPoseArgs.m_hasInitialStateQ[info.m_qIndex] = 1;
	}
	return 0;
}




b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_CREATE_SENSOR;
    command->m_updateFlags = 0;
    command->m_createSensorArguments.m_numJointSensorChanges = 0;
	command->m_createSensorArguments.m_bodyUniqueId = bodyUniqueId;
    return (b3SharedMemoryCommandHandle) command;
    
}

int b3CreateSensorEnable6DofJointForceTorqueSensor(b3SharedMemoryCommandHandle commandHandle, int jointIndex, int enable)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_SENSOR);
    int curIndex = command->m_createSensorArguments.m_numJointSensorChanges;
    command->m_createSensorArguments.m_sensorType[curIndex] = SENSOR_FORCE_TORQUE;
    
    command->m_createSensorArguments.m_jointIndex[curIndex] = jointIndex;
    command->m_createSensorArguments.m_enableJointForceSensor[curIndex] = enable;
    command->m_createSensorArguments.m_numJointSensorChanges++;
    return 0;
}

int b3CreateSensorEnableIMUForLink(b3SharedMemoryCommandHandle commandHandle, int linkIndex, int enable)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_SENSOR);
    int curIndex = command->m_createSensorArguments.m_numJointSensorChanges;
    command->m_createSensorArguments.m_sensorType[curIndex] = SENSOR_IMU;
    command->m_createSensorArguments.m_linkIndex[curIndex] = linkIndex;
    command->m_createSensorArguments.m_enableSensor[curIndex] = enable;
    command->m_createSensorArguments.m_numJointSensorChanges++;
	return 0;    
}


b3PhysicsClientHandle b3ConnectSharedMemory(int key)
{

	PhysicsClientSharedMemory* cl = new PhysicsClientSharedMemory();
    ///client should never create shared memory, only the server does
    cl->setSharedMemoryKey(key);
    cl->connect();
	return (b3PhysicsClientHandle ) cl;
}


void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	delete cl;
}

b3SharedMemoryStatusHandle b3ProcessServerStatus(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	if (cl && cl->isConnected())
	{
		const SharedMemoryStatus* stat = cl->processServerStatus();
		return (b3SharedMemoryStatusHandle) stat;
	}
	return 0;
}




int b3GetStatusType(b3SharedMemoryStatusHandle statusHandle)
{
    const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
    b3Assert(status);
    if (status)
    {
        return status->m_type;
    }
    return CMD_INVALID_STATUS;
}

int b3GetStatusBodyIndices(b3SharedMemoryStatusHandle statusHandle, int* bodyIndicesOut, int bodyIndicesCapacity)
{
    int numBodies = 0;
    const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
    b3Assert(status);
	
	if (status)
	{
			switch (status->m_type)
			{
				case CMD_SDF_LOADING_COMPLETED:
				{
				    int i,maxBodies;
				    numBodies = status->m_sdfLoadedArgs.m_numBodies;
				    maxBodies = btMin(bodyIndicesCapacity, numBodies);
				    for (i=0;i<maxBodies;i++)
				    {
                            bodyIndicesOut[i] = status->m_sdfLoadedArgs.m_bodyUniqueIds[i];
				    }
					break;
				}
			}
	}
	
	return numBodies;
}

int b3GetStatusBodyIndex(b3SharedMemoryStatusHandle statusHandle)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
    b3Assert(status);
	int bodyId = -1;

	if (status)
	{
			switch (status->m_type)
			{
				case CMD_URDF_LOADING_COMPLETED:
				{
					bodyId = status->m_dataStreamArguments.m_bodyUniqueId;
					break;
				}
				case CMD_RIGID_BODY_CREATION_COMPLETED:
				{
					bodyId = status->m_rigidBodyCreateArgs.m_bodyUniqueId;
					break;
				}
				default:
				{
					b3Assert(0);
				}
			};
	}
	return bodyId;
}

int b3GetStatusActualState(b3SharedMemoryStatusHandle statusHandle,
                           int* bodyUniqueId,
                           int* numDegreeOfFreedomQ,
                           int* numDegreeOfFreedomU,
                           const double* rootLocalInertialFrame[],
                           const double* actualStateQ[],
                           const double* actualStateQdot[],
                           const double* jointReactionForces[]) {
    const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
    const SendActualStateArgs &args = status->m_sendActualStateArgs;
    btAssert(status->m_type == CMD_ACTUAL_STATE_UPDATE_COMPLETED);
    if (status->m_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
        return false;
    
    if (bodyUniqueId) {
        *bodyUniqueId = args.m_bodyUniqueId;
    }
    if (numDegreeOfFreedomQ) {
        *numDegreeOfFreedomQ = args.m_numDegreeOfFreedomQ;
    }
    if (numDegreeOfFreedomU) {
        *numDegreeOfFreedomU = args.m_numDegreeOfFreedomU;
    }
    if (rootLocalInertialFrame) {
        *rootLocalInertialFrame = args.m_rootLocalInertialFrame;
    }
    if (actualStateQ) {
        *actualStateQ = args.m_actualStateQ;
    }
    if (actualStateQdot) {
        *actualStateQdot = args.m_actualStateQdot;
    }
    if (jointReactionForces) {
        *jointReactionForces = args.m_jointReactionForces;
    }
    return true;
}

int	b3CanSubmitCommand(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	if (cl)
	{
		return (int)cl->canSubmitCommand();
	}
	return false;
}

int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, const b3SharedMemoryCommandHandle commandHandle)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    return (int)cl->submitClientCommand(*command);
}

b3SharedMemoryStatusHandle b3SubmitClientCommandAndWaitStatus(b3PhysicsClientHandle physClient, const b3SharedMemoryCommandHandle commandHandle)
{
    int timeout = 1024*1024*1024;
    b3SharedMemoryStatusHandle statusHandle=0;
    
    b3SubmitClientCommand(physClient,commandHandle);
    
    while ((statusHandle==0) && (timeout-- > 0))
    {
        statusHandle =b3ProcessServerStatus(physClient);
    }
    return (b3SharedMemoryStatusHandle) statusHandle;
    
}


int	b3GetNumJoints(b3PhysicsClientHandle physClient, int bodyId)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getNumJoints(bodyId);
}


int	b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyIndex, int linkIndex, struct b3JointInfo* info)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getJointInfo(bodyIndex, linkIndex,*info);
}

b3SharedMemoryCommandHandle b3PickBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ)
{
    PhysicsClient *cl = (PhysicsClient *)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand *command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_PICK_BODY;
    command->m_pickBodyArguments.m_rayFromWorld[0] = rayFromWorldX;
    command->m_pickBodyArguments.m_rayFromWorld[1] = rayFromWorldY;
    command->m_pickBodyArguments.m_rayFromWorld[2] = rayFromWorldZ;
    command->m_pickBodyArguments.m_rayToWorld[0] = rayToWorldX;
    command->m_pickBodyArguments.m_rayToWorld[1] = rayToWorldY;
    command->m_pickBodyArguments.m_rayToWorld[2] = rayToWorldZ;
    return (b3SharedMemoryCommandHandle)command;
}

b3SharedMemoryCommandHandle b3MovePickedBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                             double rayFromWorldY, double rayFromWorldZ,
                                             double rayToWorldX, double rayToWorldY,
                                             double rayToWorldZ)
{
    PhysicsClient *cl = (PhysicsClient *)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand *command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_MOVE_PICKED_BODY;
    command->m_pickBodyArguments.m_rayFromWorld[0] = rayFromWorldX;
    command->m_pickBodyArguments.m_rayFromWorld[1] = rayFromWorldY;
    command->m_pickBodyArguments.m_rayFromWorld[2] = rayFromWorldZ;
    command->m_pickBodyArguments.m_rayToWorld[0] = rayToWorldX;
    command->m_pickBodyArguments.m_rayToWorld[1] = rayToWorldY;
    command->m_pickBodyArguments.m_rayToWorld[2] = rayToWorldZ;
    return (b3SharedMemoryCommandHandle)command;
}

b3SharedMemoryCommandHandle b3RemovePickingConstraint(b3PhysicsClientHandle physClient)
{
    PhysicsClient *cl = (PhysicsClient *)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand *command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_REMOVE_PICKING_CONSTRAINT_BODY;
    return (b3SharedMemoryCommandHandle)command;
}

b3SharedMemoryCommandHandle b3InitRequestDebugLinesCommand(b3PhysicsClientHandle physClient, int debugMode)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type =CMD_REQUEST_DEBUG_LINES;
    command->m_requestDebugLinesArguments.m_debugMode = debugMode;
    command->m_requestDebugLinesArguments.m_startingLineIndex = 0;
    return (b3SharedMemoryCommandHandle) command;
}
void    b3GetDebugLines(b3PhysicsClientHandle physClient, struct b3DebugLines* lines)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;

    b3Assert(lines);
    if (lines)
    {
        lines->m_numDebugLines = cl->getNumDebugLines();
        lines->m_linesFrom = cl->getDebugLinesFrom();
        lines->m_linesTo = cl->getDebugLinesTo();
        lines->m_linesColor = cl->getDebugLinesColor();
        
    }
    
}

///request an image from a simulated camera, using a software renderer.
b3SharedMemoryCommandHandle b3InitRequestCameraImage(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_REQUEST_CAMERA_IMAGE_DATA;
	command->m_requestPixelDataArguments.m_startPixelIndex = 0;
    command->m_updateFlags = 0;//REQUEST_PIXEL_ARGS_USE_HARDWARE_OPENGL;
    return (b3SharedMemoryCommandHandle) command;
}


void b3RequestCameraImageSetCameraMatrices(b3SharedMemoryCommandHandle commandHandle, float viewMatrix[16], float projectionMatrix[16])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
	for (int i=0;i<16;i++)
	{
		command->m_requestPixelDataArguments.m_projectionMatrix[i] = projectionMatrix[i];
		command->m_requestPixelDataArguments.m_viewMatrix[i] = viewMatrix[i];
	}
	command->m_updateFlags |= REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES;
}

void b3RequestCameraImageSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3])
{
    b3Vector3 eye = b3MakeVector3(cameraPosition[0], cameraPosition[1], cameraPosition[2]);
    b3Vector3 center = b3MakeVector3(cameraTargetPosition[0], cameraTargetPosition[1], cameraTargetPosition[2]);
    b3Vector3 up = b3MakeVector3(cameraUp[0], cameraUp[1], cameraUp[2]);
    b3Vector3 f = (center - eye).normalized();
    b3Vector3 u = up.normalized();
    b3Vector3 s = (f.cross(u)).normalized();
    u = s.cross(f);
    
    float viewMatrix[16];
    
    viewMatrix[0*4+0] = s.x;
    viewMatrix[1*4+0] = s.y;
    viewMatrix[2*4+0] = s.z;
    
    viewMatrix[0*4+1] = u.x;
    viewMatrix[1*4+1] = u.y;
    viewMatrix[2*4+1] = u.z;
    
    viewMatrix[0*4+2] =-f.x;
    viewMatrix[1*4+2] =-f.y;
    viewMatrix[2*4+2] =-f.z;
    
    viewMatrix[0*4+3] = 0.f;
    viewMatrix[1*4+3] = 0.f;
    viewMatrix[2*4+3] = 0.f;
    
    viewMatrix[3*4+0] = -s.dot(eye);
    viewMatrix[3*4+1] = -u.dot(eye);
    viewMatrix[3*4+2] = f.dot(eye);
    viewMatrix[3*4+3] = 1.f;
    
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    for (int i=0;i<16;i++)
    {
        command->m_requestPixelDataArguments.m_viewMatrix[i] = viewMatrix[i];
    }
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES;

}

void b3RequestCameraImageSetProjectionMatrix(b3SharedMemoryCommandHandle commandHandle, float left, float  right, float bottom, float top, float nearVal, float farVal)
{
    float frustum[16];
    
    frustum[0*4+0] = (float(2) * nearVal) / (right - left);
    frustum[0*4+1] = float(0);
    frustum[0*4+2] = float(0);
    frustum[0*4+3] = float(0);
    
    frustum[1*4+0] = float(0);
    frustum[1*4+1] = (float(2) * nearVal) / (top - bottom);
    frustum[1*4+2] = float(0);
    frustum[1*4+3] = float(0);
    
    frustum[2*4+0] = (right + left) / (right - left);
    frustum[2*4+1] = (top + bottom) / (top - bottom);
    frustum[2*4+2] = -(farVal + nearVal) / (farVal - nearVal);
    frustum[2*4+3] = float(-1);
    
    frustum[3*4+0] = float(0);
    frustum[3*4+1] = float(0);
    frustum[3*4+2] = -(float(2) * farVal * nearVal) / (farVal - nearVal);
    frustum[3*4+3] = float(0);
    
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    for (int i=0;i<16;i++)
    {
        command->m_requestPixelDataArguments.m_projectionMatrix[i] = frustum[i];
    }
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES;
}

void b3RequestCameraImageSetPixelResolution(b3SharedMemoryCommandHandle commandHandle, int width, int height )
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
	command->m_requestPixelDataArguments.m_pixelWidth = width;
	command->m_requestPixelDataArguments.m_pixelHeight = height;
	command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_PIXEL_WIDTH_HEIGHT;	
}

void b3GetCameraImageData(b3PhysicsClientHandle physClient, struct b3CameraImageData* imageData)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	if (cl)
	{
		cl->getCachedCameraImage(imageData);
	}
}

b3SharedMemoryCommandHandle b3ApplyExternalForceCommandInit(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_APPLY_EXTERNAL_FORCE;
    command->m_updateFlags = 0;
    command->m_externalForceArguments.m_numForcesAndTorques = 0;
    return (b3SharedMemoryCommandHandle) command;
}

void b3ApplyExternalForce(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[3], const double position[3], int flag)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_APPLY_EXTERNAL_FORCE);
    int index = command->m_externalForceArguments.m_numForcesAndTorques;
    command->m_externalForceArguments.m_bodyUniqueIds[index] = bodyUniqueId;
    command->m_externalForceArguments.m_linkIds[index] = linkId;
    command->m_externalForceArguments.m_forceFlags[index] = EF_FORCE+flag;
    for (int i = 0; i < 3; ++i) {
        command->m_externalForceArguments.m_forcesAndTorques[index+i] = force[i];
        command->m_externalForceArguments.m_positions[index+i] = position[i];
    }
    
    command->m_externalForceArguments.m_numForcesAndTorques++;
}

void b3ApplyExternalTorque(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[3], int flag)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_APPLY_EXTERNAL_FORCE);
    int index = command->m_externalForceArguments.m_numForcesAndTorques;
    command->m_externalForceArguments.m_bodyUniqueIds[index] = bodyUniqueId;
    command->m_externalForceArguments.m_linkIds[index] = linkId;
    command->m_externalForceArguments.m_forceFlags[index] = EF_TORQUE+flag;

    for (int i = 0; i < 3; ++i) {
        command->m_externalForceArguments.m_forcesAndTorques[index+i] = torque[i];
    }
    command->m_externalForceArguments.m_numForcesAndTorques++;
}

