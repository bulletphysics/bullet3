#include "PhysicsClientC_API.h"
#include "PhysicsClientSharedMemory.h"
#include "Bullet3Common/b3Scalar.h"
#include <string.h>
#include "SharedMemoryCommands.h"



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


b3SharedMemoryCommandHandle b3JointControlCommandInit( b3PhysicsClientHandle physClient, int controlMode)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
	command->m_type = CMD_SEND_DESIRED_STATE;
    command->m_sendDesiredStateCommandArgument.m_controlMode = controlMode;
	command->m_sendDesiredStateCommandArgument.m_bodyUniqueId = 0;
	command->m_updateFlags = 0;
    return (b3SharedMemoryCommandHandle) command;
}

int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateQ[qIndex] = value;
    return 0;
}

int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_Kp[dofIndex] = value;
    return 0;
}

int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_Kd[dofIndex] = value;
    return 0;
}

int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateQdot[dofIndex] = value;
    return 0;
}


int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex,  double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
    return 0;
}

int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int  dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
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
	}
	return 0;
}




b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_CREATE_SENSOR;
    command->m_updateFlags = 0;
    command->m_createSensorArguments.m_numJointSensorChanges = 0;
	command->m_createSensorArguments.m_bodyUniqueId = 0;
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
	const SharedMemoryStatus* stat = cl->processServerStatus();
    return (b3SharedMemoryStatusHandle) stat;
    
}




int b3GetStatusType(b3SharedMemoryStatusHandle statusHandle)
{
    const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
    b3Assert(status);
    if (status)
    {
        return status->m_type;
    }
    return 0;
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


void	b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyIndex, int linkIndex, struct b3JointInfo* info)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	cl->getJointInfo(bodyIndex, linkIndex,*info);
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
