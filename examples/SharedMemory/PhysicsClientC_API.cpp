#include "PhysicsClientC_API.h"
#include "PhysicsClient.h"
#include "Bullet3Common/b3Scalar.h"
#include <string.h>

int	b3LoadUrdfCommandInit(struct SharedMemoryCommand* command, const char* urdfFileName)
{
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
	
	return 0;
}




int	b3LoadUrdfCommandSetStartPosition(struct SharedMemoryCommand* command, double startPosX,double startPosY,double startPosZ)
{
	b3Assert(command);
	b3Assert(command->m_type == CMD_LOAD_URDF);
	command->m_urdfArguments.m_initialPosition[0] = startPosX;
	command->m_urdfArguments.m_initialPosition[1] = startPosY;
	command->m_urdfArguments.m_initialPosition[2] = startPosZ;
	command->m_updateFlags|=URDF_ARGS_INITIAL_POSITION;
	return 0;
}
int	b3LoadUrdfCommandSetStartOrientation(struct SharedMemoryCommand* command, double startOrnX,double startOrnY,double startOrnZ, double startOrnW)
{
	b3Assert(command);
	b3Assert(command->m_type == CMD_LOAD_URDF);
	command->m_urdfArguments.m_initialOrientation[0] = startOrnX;
	command->m_urdfArguments.m_initialOrientation[1] = startOrnY;
	command->m_urdfArguments.m_initialOrientation[2] = startOrnZ;
	command->m_urdfArguments.m_initialOrientation[3] = startOrnW;
	command->m_updateFlags|=URDF_ARGS_INITIAL_ORIENTATION;
	return 0;
}

int     b3InitPhysicsParamCommand(struct SharedMemoryCommand* command)
{
	b3Assert(command);
	command->m_type = CMD_SEND_PHYSICS_SIMULATION_PARAMETERS;
	command->m_updateFlags = 0;

	return 0;
}
int     b3PhysicsParamSetGravity(struct SharedMemoryCommand* command, double gravx,double gravy, double gravz)
{
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_gravityAcceleration[0] = gravx;
	command->m_physSimParamArgs.m_gravityAcceleration[1] = gravy;
	command->m_physSimParamArgs.m_gravityAcceleration[2] = gravz; 
	command->m_updateFlags |= SIM_PARAM_UPDATE_GRAVITY;
	return 0;
}

int	b3PhysicsParamSetTimeStep(struct SharedMemoryCommand* command, double timeStep)
{
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_deltaTime = timeStep;
	return 0;
}

int	b3InitStepSimulationCommand(struct SharedMemoryCommand* command)
{
	b3Assert(command);
	command->m_type = CMD_STEP_FORWARD_SIMULATION;
	command->m_updateFlags = 0;

	return 0;

}


int b3JointControlCommandInit(struct SharedMemoryCommand* command, int controlMode)
{
    b3Assert(command);
	command->m_type = CMD_SEND_DESIRED_STATE;
    command->m_sendDesiredStateCommandArgument.m_controlMode = controlMode;
	command->m_updateFlags = 0;
    return 0;
}

int b3JointControlSetDesiredVelocity(struct SharedMemoryCommand* command, int dofIndex, double value)
{
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateQdot[dofIndex] = 1;
    return 0;
}


int b3JointControlSetMaximumForce(struct SharedMemoryCommand* command, int dofIndex,  double value)
{
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
    return 0;
}

int b3JointControlSetDesiredForceTorque(struct SharedMemoryCommand* command, int  dofIndex, double value)
{
    b3Assert(command);
    command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
    return 0;
}


int b3RequestActualStateCommandInit(struct SharedMemoryCommand* command)
{
    b3Assert(command);
    command->m_type =CMD_REQUEST_ACTUAL_STATE;
    return 0;
}

int b3CreateBoxShapeCommandInit(struct SharedMemoryCommand* command)
{
    b3Assert(command);
    command->m_type = CMD_CREATE_BOX_COLLISION_SHAPE;
    command->m_updateFlags =0;
    return 0;
}

int	b3CreateBoxCommandSetStartPosition(struct SharedMemoryCommand* command, double startPosX,double startPosY,double startPosZ)
{
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
    command->m_updateFlags |=BOX_SHAPE_HAS_INITIAL_POSITION;
    
    command->m_createBoxShapeArguments.m_initialPosition[0] = startPosX;
    command->m_createBoxShapeArguments.m_initialPosition[1] = startPosY;
    command->m_createBoxShapeArguments.m_initialPosition[2] = startPosZ;
    return 0;
}

int	b3CreateBoxCommandSetStartOrientation(struct SharedMemoryCommand* command, double startOrnX,double startOrnY,double startOrnZ, double startOrnW)
{
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
    command->m_updateFlags |=BOX_SHAPE_HAS_INITIAL_ORIENTATION;
    
    command->m_createBoxShapeArguments.m_initialOrientation[0] = startOrnX;
    command->m_createBoxShapeArguments.m_initialOrientation[1] = startOrnY;
    command->m_createBoxShapeArguments.m_initialOrientation[2] = startOrnZ;
    command->m_createBoxShapeArguments.m_initialOrientation[3] = startOrnW;
    return 0;
}

int	b3CreateBoxCommandSetHalfExtents(struct SharedMemoryCommand* command, double halfExtentsX,double halfExtentsY,double halfExtentsZ)
{
    
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_BOX_COLLISION_SHAPE);
    command->m_updateFlags |=BOX_SHAPE_HAS_HALF_EXTENTS;
    
    command->m_createBoxShapeArguments.m_halfExtentsX = halfExtentsX;
    command->m_createBoxShapeArguments.m_halfExtentsY = halfExtentsY;
    command->m_createBoxShapeArguments.m_halfExtentsZ = halfExtentsZ;

    return 0;
}



int b3CreateSensorCommandInit(struct SharedMemoryCommand* command)
{
    b3Assert(command);
    command->m_type = CMD_CREATE_SENSOR;
    command->m_updateFlags = 0;
    command->m_createSensorArguments.m_numJointSensorChanges = 0;
    return 0;
}

int b3CreateSensorEnable6DofJointForceTorqueSensor(struct SharedMemoryCommand* command, int jointIndex, int enable)
{
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_SENSOR);
    int curIndex = command->m_createSensorArguments.m_numJointSensorChanges;
    
    command->m_createSensorArguments.m_jointIndex[curIndex] = jointIndex;
    command->m_createSensorArguments.m_enableJointForceSensor[curIndex] = enable;
    command->m_createSensorArguments.m_numJointSensorChanges++;
    return 0;
}


b3PhysicsClientHandle b3ConnectSharedMemory()
{
	PhysicsClientSharedMemory* cl = new PhysicsClientSharedMemory();
    ///client should never create shared memory, only the server does
	cl->connect();
	return (b3PhysicsClientHandle ) cl;
}

void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	delete cl;
}

int	b3ProcessServerStatus(b3PhysicsClientHandle physClient, struct SharedMemoryStatus* status)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	return (int)cl->processServerStatus(*status);
}

int	b3CanSubmitCommand(b3PhysicsClientHandle physClient)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	return (int)cl->canSubmitCommand();
}

int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, struct SharedMemoryCommand* command)
{
		PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
		return (int)cl->submitClientCommand(*command);
}



int	b3GetNumJoints(b3PhysicsClientHandle physClient)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	return cl->getNumJoints();
}


void	b3GetJointInfo(b3PhysicsClientHandle physClient, int linkIndex, struct b3JointInfo* info)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	cl->getJointInfo(linkIndex,*info);
}


