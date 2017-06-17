#include "PhysicsClientC_API.h"
#include "PhysicsClientSharedMemory.h"
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "Bullet3Common/b3Transform.h"

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

b3SharedMemoryCommandHandle	b3SaveWorldCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
	command->m_type = CMD_SAVE_WORLD;
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
    
	if (cl->canSubmitCommand())
	{
		struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
		b3Assert(command);
		command->m_type = CMD_LOAD_URDF;
		int len = strlen(urdfFileName);
		if (len < MAX_URDF_FILENAME_LENGTH)
		{
			strcpy(command->m_urdfArguments.m_urdfFileName, urdfFileName);
		}
		else
		{
			command->m_urdfArguments.m_urdfFileName[0] = 0;
		}
		command->m_updateFlags = URDF_ARGS_FILE_NAME;

		return (b3SharedMemoryCommandHandle)command;
	}
	return 0;
}

b3SharedMemoryCommandHandle	b3LoadBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());

	if (cl->canSubmitCommand())
	{
		struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
		b3Assert(command);
		command->m_type = CMD_LOAD_BULLET;
		int len = strlen(fileName);
		if (len < MAX_URDF_FILENAME_LENGTH)
		{
			strcpy(command->m_fileArguments.m_fileName, fileName);
		}
		else
		{
			command->m_fileArguments.m_fileName[0] = 0;
		}
		command->m_updateFlags = 0;

		return (b3SharedMemoryCommandHandle)command;
	}
	return 0;
}

b3SharedMemoryCommandHandle	b3SaveBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());

	if (cl->canSubmitCommand())
	{
		struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
		b3Assert(command);
		command->m_type = CMD_SAVE_BULLET;
		int len = strlen(fileName);
		if (len < MAX_URDF_FILENAME_LENGTH)
		{
			strcpy(command->m_fileArguments.m_fileName, fileName);
		}
		else
		{
			command->m_fileArguments.m_fileName[0] = 0;
		}
		command->m_updateFlags = 0;

		return (b3SharedMemoryCommandHandle)command;
	}
	return 0;
}
b3SharedMemoryCommandHandle	b3LoadMJCFCommandInit(b3PhysicsClientHandle physClient, const char* fileName)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());

	if (cl->canSubmitCommand())
	{
		struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
		b3Assert(command);
		command->m_type = CMD_LOAD_MJCF;
		int len = strlen(fileName);
		if (len < MAX_URDF_FILENAME_LENGTH)
		{
			strcpy(command->m_mjcfArguments.m_mjcfFileName, fileName);
		}
		else
		{
			command->m_mjcfArguments.m_mjcfFileName[0] = 0;
		}
		command->m_updateFlags = 0;

		return (b3SharedMemoryCommandHandle)command;
	}
	return 0;
}

void b3LoadMJCFCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command->m_type == CMD_LOAD_MJCF);
	if (command->m_type == CMD_LOAD_MJCF)
	{
		command->m_mjcfArguments.m_flags = flags;
		command->m_updateFlags |= URDF_ARGS_HAS_CUSTOM_URDF_FLAGS;
	}
}

b3SharedMemoryCommandHandle	b3LoadBunnyCommandInit(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_LOAD_BUNNY;
    command->m_updateFlags = 0;
    
    return (b3SharedMemoryCommandHandle) command;
}

int b3LoadBunnySetScale(b3SharedMemoryCommandHandle commandHandle, double scale)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command->m_type == CMD_LOAD_BUNNY);
    command->m_loadBunnyArguments.m_scale = scale;
    command->m_updateFlags |= LOAD_BUNNY_UPDATE_SCALE;
    return 0;
}

int b3LoadBunnySetMass(b3SharedMemoryCommandHandle commandHandle, double mass)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command->m_type == CMD_LOAD_BUNNY);
    command->m_loadBunnyArguments.m_mass = mass;
    command->m_updateFlags |= LOAD_BUNNY_UPDATE_MASS;
    return 0;
}

int b3LoadBunnySetCollisionMargin(b3SharedMemoryCommandHandle commandHandle, double collisionMargin)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command->m_type == CMD_LOAD_BUNNY);
    command->m_loadBunnyArguments.m_collisionMargin = collisionMargin;
    command->m_updateFlags |= LOAD_BUNNY_UPDATE_COLLISION_MARGIN;
    return 0;
}

int	b3LoadUrdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_LOAD_URDF);
    command->m_updateFlags |=URDF_ARGS_USE_MULTIBODY;
    command->m_urdfArguments.m_useMultiBody = useMultiBody;
    
    return 0;
}

int	b3LoadSdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_LOAD_SDF);
    command->m_updateFlags |=URDF_ARGS_USE_MULTIBODY;
    command->m_sdfArguments.m_useMultiBody = useMultiBody;
    
    return 0;
}

int	b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_LOAD_URDF);
	if (command && (command->m_type == CMD_LOAD_URDF))
	{
		command->m_updateFlags |= URDF_ARGS_USE_FIXED_BASE;
		command->m_urdfArguments.m_useFixedBase = useFixedBase;
		return 0;
	}
	return -1;
}

int	b3LoadUrdfCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_LOAD_URDF);
	if (command && (command->m_type == CMD_LOAD_URDF))
	{
		command->m_updateFlags |= 	URDF_ARGS_HAS_CUSTOM_URDF_FLAGS;
		command->m_urdfArguments.m_urdfFlags = flags;
	}
	return 0;
}

int	b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	if (command)
	{
		b3Assert(command->m_type == CMD_LOAD_URDF);
		if (command->m_type == CMD_LOAD_URDF)
		{
			command->m_urdfArguments.m_initialPosition[0] = startPosX;
			command->m_urdfArguments.m_initialPosition[1] = startPosY;
			command->m_urdfArguments.m_initialPosition[2] = startPosZ;
			command->m_updateFlags |= URDF_ARGS_INITIAL_POSITION;
		}
		return 0;
	}
	return -1;
}

int	b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	if (command)
	{
		b3Assert(command->m_type == CMD_LOAD_URDF);
		if (command->m_type == CMD_LOAD_URDF)
		{
			command->m_urdfArguments.m_initialOrientation[0] = startOrnX;
			command->m_urdfArguments.m_initialOrientation[1] = startOrnY;
			command->m_urdfArguments.m_initialOrientation[2] = startOrnZ;
			command->m_urdfArguments.m_initialOrientation[3] = startOrnW;
			command->m_updateFlags |= URDF_ARGS_INITIAL_ORIENTATION;
		}
		return 0;
	}
	return -1;
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

int     b3PhysicsParamSetRealTimeSimulation(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_allowRealTimeSimulation = (enableRealTimeSimulation!=0);
	command->m_updateFlags |= SIM_PARAM_UPDATE_REAL_TIME_SIMULATION;
	return 0;
}

int     b3PhysicsParamSetInternalSimFlags(b3SharedMemoryCommandHandle commandHandle, int flags)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_internalSimFlags = flags;
	command->m_updateFlags |= SIM_PARAM_UPDATE_INTERNAL_SIMULATION_FLAGS;
	return 0;
}

int b3PhysicsParamSetUseSplitImpulse(b3SharedMemoryCommandHandle commandHandle, int useSplitImpulse)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);

	command->m_physSimParamArgs.m_useSplitImpulse = useSplitImpulse;
	command->m_updateFlags |= SIM_PARAM_UPDATE_USE_SPLIT_IMPULSE;
	return 0;
}

int b3PhysicsParamSetSplitImpulsePenetrationThreshold(b3SharedMemoryCommandHandle commandHandle, double splitImpulsePenetrationThreshold)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);

	command->m_physSimParamArgs.m_splitImpulsePenetrationThreshold = splitImpulsePenetrationThreshold;
	command->m_updateFlags |= SIM_PARAM_UPDATE_SPLIT_IMPULSE_PENETRATION_THRESHOLD;
	return 0;
}

int b3PhysicsParamSetContactBreakingThreshold(b3SharedMemoryCommandHandle commandHandle, double contactBreakingThreshold)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);

	command->m_physSimParamArgs.m_contactBreakingThreshold = contactBreakingThreshold;
	command->m_updateFlags |= SIM_PARAM_UPDATE_CONTACT_BREAKING_THRESHOLD;
	return 0;
}
int b3PhysicsParamSetMaxNumCommandsPer1ms(b3SharedMemoryCommandHandle commandHandle, int maxNumCmdPer1ms)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);

	command->m_physSimParamArgs.m_maxNumCmdPer1ms = maxNumCmdPer1ms;
	command->m_updateFlags |= SIM_PARAM_MAX_CMD_PER_1MS;
	return 0;

}

int b3PhysicsParamSetEnableFileCaching(b3SharedMemoryCommandHandle commandHandle, int enableFileCaching)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);

	command->m_physSimParamArgs.m_enableFileCaching= enableFileCaching;
	command->m_updateFlags |= SIM_PARAM_ENABLE_FILE_CACHING ;
	return 0;

}

int b3PhysicsParamSetRestitutionVelocityThreshold(b3SharedMemoryCommandHandle commandHandle, double restitutionVelocityThreshold)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);

	command->m_physSimParamArgs.m_restitutionVelocityThreshold = restitutionVelocityThreshold;
	command->m_updateFlags |= SIM_PARAM_UPDATE_RESTITUTION_VELOCITY_THRESHOLD  ;
	return 0;

}


int b3PhysicsParamSetNumSolverIterations(b3SharedMemoryCommandHandle commandHandle, int numSolverIterations)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_numSolverIterations = numSolverIterations;
	command->m_updateFlags |= SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS;
	return 0;
}


int b3PhysicsParamSetCollisionFilterMode(b3SharedMemoryCommandHandle commandHandle, int filterMode)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_collisionFilterMode = filterMode;
	command->m_updateFlags |= SIM_PARAM_UPDATE_COLLISION_FILTER_MODE;
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

int	b3PhysicsParamSetNumSubSteps(b3SharedMemoryCommandHandle commandHandle, int numSubSteps)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
    command->m_updateFlags |= SIM_PARAM_UPDATE_NUM_SIMULATION_SUB_STEPS;
    command->m_physSimParamArgs.m_numSimulationSubSteps = numSubSteps;
    return 0;
}


int	b3PhysicsParamSetDefaultContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultContactERP)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
    command->m_updateFlags |= SIM_PARAM_UPDATE_DEFAULT_CONTACT_ERP;
    command->m_physSimParamArgs.m_defaultContactERP = defaultContactERP;
    return 0;
}
int	b3PhysicsParamSetDefaultNonContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultNonContactERP)
{
	    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
    command->m_updateFlags |= SIM_PARAM_UPDATE_DEFAULT_NON_CONTACT_ERP;
    command->m_physSimParamArgs.m_defaultNonContactERP = defaultNonContactERP;
    return 0;
}
int	b3PhysicsParamSetDefaultFrictionERP(b3SharedMemoryCommandHandle commandHandle, double frictionERP)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_updateFlags |= SIM_PARAM_UPDATE_DEFAULT_FRICTION_ERP;
	command->m_physSimParamArgs.m_frictionERP = frictionERP;
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
	if ((qIndex>=0) && (qIndex < MAX_DEGREE_OF_FREEDOM))
	{
		command->m_sendDesiredStateCommandArgument.m_desiredStateQ[qIndex] = value;
		command->m_updateFlags |= SIM_DESIRED_STATE_HAS_Q;
		command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[qIndex] |= SIM_DESIRED_STATE_HAS_Q;
	}
    return 0;
}

int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
	if ((dofIndex>=0) && (dofIndex < MAX_DEGREE_OF_FREEDOM))
	{
		command->m_sendDesiredStateCommandArgument.m_Kp[dofIndex] = value;
		command->m_updateFlags |= SIM_DESIRED_STATE_HAS_KP;
		command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_KP;
	}
    return 0;
}

int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
	if ((dofIndex>=0) && (dofIndex < MAX_DEGREE_OF_FREEDOM))
	{
		command->m_sendDesiredStateCommandArgument.m_Kd[dofIndex] = value;
		command->m_updateFlags |= SIM_DESIRED_STATE_HAS_KD;
	    command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_KD;
	}
    return 0;
}

int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
	if ((dofIndex>=0) && (dofIndex < MAX_DEGREE_OF_FREEDOM))
	{
		command->m_sendDesiredStateCommandArgument.m_desiredStateQdot[dofIndex] = value;
		command->m_updateFlags |= SIM_DESIRED_STATE_HAS_QDOT;
		command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_QDOT;
	}
    return 0;
}


int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex,  double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
	if ((dofIndex>=0) && (dofIndex < MAX_DEGREE_OF_FREEDOM))
	{
		command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
		command->m_updateFlags |= SIM_DESIRED_STATE_HAS_MAX_FORCE;
		command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_MAX_FORCE;
	}
    return 0;
}

int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int  dofIndex, double value)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
	if ((dofIndex>=0) && (dofIndex < MAX_DEGREE_OF_FREEDOM))
	{
		command->m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] = value;
		command->m_updateFlags |= SIM_DESIRED_STATE_HAS_MAX_FORCE;
		command->m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] |= SIM_DESIRED_STATE_HAS_MAX_FORCE;
	}
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
	command->m_updateFlags = 0;
	command->m_requestActualStateInformationCommandArgument.m_bodyUniqueId = bodyUniqueId;
    return (b3SharedMemoryCommandHandle) command;
}

int b3RequestActualStateCommandComputeLinkVelocity(b3SharedMemoryCommandHandle commandHandle, int computeLinkVelocity)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	btAssert(command->m_type == CMD_REQUEST_ACTUAL_STATE);
	if (computeLinkVelocity && command->m_type == CMD_REQUEST_ACTUAL_STATE)
	{
		command->m_updateFlags |= ACTUAL_STATE_COMPUTE_LINKVELOCITY;
	}
	return 0;
}


int b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, b3JointSensorState *state)
{
  const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
  b3Assert(status);
  int bodyIndex = status->m_sendActualStateArgs.m_bodyUniqueId;
  b3Assert(bodyIndex>=0);
  if (bodyIndex>=0)
  {
	  b3JointInfo info;
	  bool result = b3GetJointInfo(physClient, bodyIndex,jointIndex, &info)!=0;
	  if (result)
	  {
		  if ((info.m_qIndex>=0) && (info.m_uIndex>=0)  && (info.m_qIndex < MAX_DEGREE_OF_FREEDOM) && (info.m_uIndex < MAX_DEGREE_OF_FREEDOM))
		  {
			  state->m_jointPosition = status->m_sendActualStateArgs.m_actualStateQ[info.m_qIndex];
			  state->m_jointVelocity = status->m_sendActualStateArgs.m_actualStateQdot[info.m_uIndex];
          } else
          {
              state->m_jointPosition=0;
              state->m_jointVelocity=0;
          }
          for (int ii(0); ii < 6; ++ii)
          {
            state->m_jointForceTorque[ii] = status->m_sendActualStateArgs.m_jointReactionForces[6 * jointIndex + ii];
          }
          state->m_jointMotorTorque = status->m_sendActualStateArgs.m_jointMotorForce[jointIndex];
          return 1;
		  
	  }
  }
  return 0;
}

int b3GetLinkState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int linkIndex, b3LinkState *state)
{
  const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
  b3Assert(status);
  int bodyIndex = status->m_sendActualStateArgs.m_bodyUniqueId;
  b3Assert(bodyIndex>=0);
  b3Assert(linkIndex >= 0);
  int numJoints = b3GetNumJoints(physClient,bodyIndex);
  b3Assert(linkIndex < numJoints);

  if ((bodyIndex>=0) && (linkIndex >= 0) && linkIndex < numJoints)
  {
	  b3Transform wlf,com,inertial;
	
	  

    for (int i = 0; i < 3; ++i) 
    {
      state->m_worldPosition[i] = status->m_sendActualStateArgs.m_linkState[7 * linkIndex + i];
      state->m_localInertialPosition[i] = status->m_sendActualStateArgs.m_linkLocalInertialFrames[7 * linkIndex + i];
	  state->m_worldLinearVelocity[i] = status->m_sendActualStateArgs.m_linkWorldVelocities[6*linkIndex+i];
	  state->m_worldAngularVelocity[i] = status->m_sendActualStateArgs.m_linkWorldVelocities[6*linkIndex+i+3];
    }
    for (int i = 0; i < 4; ++i) 
    {
      state->m_worldOrientation[i] = status->m_sendActualStateArgs.m_linkState[7 * linkIndex + 3 + i];
      state->m_localInertialOrientation[i] = status->m_sendActualStateArgs.m_linkLocalInertialFrames[7 * linkIndex + 3 + i];
    }
	com.setOrigin(b3MakeVector3(state->m_worldPosition[0],state->m_worldPosition[1],state->m_worldPosition[2]));
	com.setRotation(b3Quaternion(state->m_worldOrientation[0],state->m_worldOrientation[1],state->m_worldOrientation[2],state->m_worldOrientation[3]));
	inertial.setOrigin(b3MakeVector3(state->m_localInertialPosition[0],state->m_localInertialPosition[1],state->m_localInertialPosition[2]));
	inertial.setRotation(b3Quaternion(state->m_localInertialOrientation[0],state->m_localInertialOrientation[1],state->m_localInertialOrientation[2],state->m_localInertialOrientation[3]));
	wlf = com*inertial.inverse();
	for (int i = 0; i < 3; ++i) 
    {
		state->m_worldLinkFramePosition[i] = wlf.getOrigin()[i];
	}
	b3Quaternion wlfOrn = wlf.getRotation();
	for (int i = 0; i < 4; ++i) 
    {
		state->m_worldLinkFrameOrientation[i] = wlfOrn[i];
	}
	return 1;
  }
  return 0;
}

b3SharedMemoryCommandHandle b3CreateCollisionShapeCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
	if (cl)
	{
		struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
		b3Assert(command);
		command->m_type = CMD_CREATE_COLLISION_SHAPE;
		command->m_updateFlags =0;
		command->m_createCollisionShapeArgs.m_numCollisionShapes = 0;
		return (b3SharedMemoryCommandHandle) command;
	}
	return 0;
}
void b3CreateCollisionShapeAddSphere(b3SharedMemoryCommandHandle commandHandle,double radius)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_COLLISION_SHAPE);
	if (command->m_type==CMD_CREATE_COLLISION_SHAPE)
	{
		int shapeIndex = command->m_createCollisionShapeArgs.m_numCollisionShapes;
		if (shapeIndex <MAX_COMPOUND_COLLISION_SHAPES)
		{
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_type = GEOM_SPHERE;
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_hasChildTransform = 0;
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_sphereRadius = radius;
			command->m_createCollisionShapeArgs.m_numCollisionShapes++;
		}
	}
}

void b3CreateCollisionShapeAddBox(b3SharedMemoryCommandHandle commandHandle,double halfExtents[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_COLLISION_SHAPE);
	if (command->m_type==CMD_CREATE_COLLISION_SHAPE)
	{
		int shapeIndex = command->m_createCollisionShapeArgs.m_numCollisionShapes;
		if (shapeIndex <MAX_COMPOUND_COLLISION_SHAPES)
		{
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_type = GEOM_BOX;
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_hasChildTransform = 0;
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_boxHalfExtents[0] = halfExtents[0];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_boxHalfExtents[1] = halfExtents[1];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_boxHalfExtents[2] = halfExtents[2];
			command->m_createCollisionShapeArgs.m_numCollisionShapes++;
		}
	}
}


void b3CreateCollisionShapeSetChildTransform(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, double childPosition[3], double childOrientation[4])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_COLLISION_SHAPE);
	if (command->m_type==CMD_CREATE_COLLISION_SHAPE)
	{
		if (shapeIndex<command->m_createCollisionShapeArgs.m_numCollisionShapes)
		{
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_hasChildTransform = 1;
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_childPosition[0] = childPosition[0];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_childPosition[1] = childPosition[1];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_childPosition[2] = childPosition[2];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_childOrientation[0] = childOrientation[0];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_childOrientation[1] = childOrientation[1];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_childOrientation[2] = childOrientation[2];
			command->m_createCollisionShapeArgs.m_shapes[shapeIndex].m_childOrientation[3] = childOrientation[3];
		}
	}
}


int b3GetStatusCollisionShapeUniqueId(b3SharedMemoryStatusHandle statusHandle)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
	b3Assert(status);
	b3Assert(status->m_type == CMD_CREATE_COLLISION_SHAPE_COMPLETED);
	if (status && status->m_type == CMD_CREATE_COLLISION_SHAPE_COMPLETED)
	{
		return status->m_createCollisionShapeResultArgs.m_collisionShapeUniqueId;
	}
	return -1;
}


b3SharedMemoryCommandHandle b3CreateVisualShapeCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
	if (cl)
	{
		struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
		b3Assert(command);
		command->m_type = CMD_CREATE_VISUAL_SHAPE;
		command->m_updateFlags =0;
		return (b3SharedMemoryCommandHandle) command;
	}
	return 0;
}

int b3GetStatusVisualShapeUniqueId(b3SharedMemoryStatusHandle statusHandle)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
	b3Assert(status);
	b3Assert(status->m_type == CMD_CREATE_VISUAL_SHAPE_COMPLETED);
	if (status && status->m_type == CMD_CREATE_VISUAL_SHAPE_COMPLETED)
	{
		return status->m_createVisualShapeResultArgs.m_visualShapeUniqueId;
	}
	return -1;
}

b3SharedMemoryCommandHandle b3CreateMultiBodyCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
	if (cl)
	{
		struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
		b3Assert(command);
		command->m_type = CMD_CREATE_MULTI_BODY;
		command->m_updateFlags =0;
		command->m_createMultiBodyArgs.m_bodyName[0] = 0;
		command->m_createMultiBodyArgs.m_baseLinkIndex = -1;
		command->m_createMultiBodyArgs.m_numLinks = 0;
		return (b3SharedMemoryCommandHandle) command;
	}
	return 0;
}


int b3CreateMultiBodyBase(b3SharedMemoryCommandHandle commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, double basePosition[3], double baseOrientation[4])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_MULTI_BODY);
	if (command->m_type==CMD_CREATE_MULTI_BODY)
	{
		int numLinks = command->m_createMultiBodyArgs.m_numLinks;

		if (numLinks<MAX_CREATE_MULTI_BODY_LINKS)
		{
			int baseLinkIndex = numLinks;
		    command->m_updateFlags |=MULTI_BODY_HAS_BASE;
			command->m_createMultiBodyArgs.m_baseLinkIndex = baseLinkIndex;
			command->m_createMultiBodyArgs.m_baseWorldPosition[0]=basePosition[0];
			command->m_createMultiBodyArgs.m_baseWorldPosition[1]=basePosition[1];
			command->m_createMultiBodyArgs.m_baseWorldPosition[2]=basePosition[2];

			command->m_createMultiBodyArgs.m_baseWorldOrientation[0]=baseOrientation[0];
			command->m_createMultiBodyArgs.m_baseWorldOrientation[1]=baseOrientation[1];
			command->m_createMultiBodyArgs.m_baseWorldOrientation[2]=baseOrientation[2];
			command->m_createMultiBodyArgs.m_baseWorldOrientation[3]=baseOrientation[3];

			command->m_createMultiBodyArgs.m_linkInertias[baseLinkIndex*3+0] = mass;
			command->m_createMultiBodyArgs.m_linkInertias[baseLinkIndex*3+1] = mass;
			command->m_createMultiBodyArgs.m_linkInertias[baseLinkIndex*3+2] = mass;


			command->m_createMultiBodyArgs.m_linkInertialFramePositions[baseLinkIndex*3+0] = 0;
			command->m_createMultiBodyArgs.m_linkInertialFramePositions[baseLinkIndex*3+1] = 0;
			command->m_createMultiBodyArgs.m_linkInertialFramePositions[baseLinkIndex*3+2] = 0;

			command->m_createMultiBodyArgs.m_linkInertialFrameOrientations[baseLinkIndex*4+0] = 0;
			command->m_createMultiBodyArgs.m_linkInertialFrameOrientations[baseLinkIndex*4+1] = 0;
			command->m_createMultiBodyArgs.m_linkInertialFrameOrientations[baseLinkIndex*4+2] = 0;
			command->m_createMultiBodyArgs.m_linkInertialFrameOrientations[baseLinkIndex*4+3] = 1;

			command->m_createMultiBodyArgs.m_linkCollisionShapeUniqueIds[baseLinkIndex]= collisionShapeUnique;
			command->m_createMultiBodyArgs.m_linkVisualShapeUniqueIds[baseLinkIndex] = visualShapeUniqueId;

			command->m_createMultiBodyArgs.m_linkMasses[command->m_createMultiBodyArgs.m_numLinks] = mass;
			command->m_createMultiBodyArgs.m_numLinks++;
		}
		return numLinks;
	}
	return -2;
}

//useMaximalCoordinates are disabled by default, enabling them is experimental and not fully supported yet
void b3CreateMultiBodyUseMaximalCoordinates(b3SharedMemoryCommandHandle commandHandle)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CREATE_MULTI_BODY);
	if (command->m_type==CMD_CREATE_MULTI_BODY)
	{
		command->m_updateFlags |= MULT_BODY_USE_MAXIMAL_COORDINATES;
	}
}

int b3GetStatusMultiBodyUniqueId(b3SharedMemoryStatusHandle statusHandle)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
	b3Assert(status);
	b3Assert(status->m_type == CMD_CREATE_MULTI_BODY_COMPLETED);
	if (status && status->m_type == CMD_CREATE_MULTI_BODY_COMPLETED)
	{
		return status->m_createMultiBodyResultArgs.m_bodyUniqueId;
	}
	return -1;
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
		command->m_initPoseArgs.m_hasInitialStateQdot[i] = 0;
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

int	b3CreatePoseCommandSetBaseLinearVelocity(b3SharedMemoryCommandHandle commandHandle, double linVel[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_INIT_POSE);
	command->m_updateFlags |= INIT_POSE_HAS_BASE_LINEAR_VELOCITY;
	command->m_initPoseArgs.m_hasInitialStateQdot[0] = 1;
	command->m_initPoseArgs.m_hasInitialStateQdot[1] = 1;
	command->m_initPoseArgs.m_hasInitialStateQdot[2] = 1;

	command->m_initPoseArgs.m_initialStateQdot[0] = linVel[0];
	command->m_initPoseArgs.m_initialStateQdot[1] = linVel[1];
	command->m_initPoseArgs.m_initialStateQdot[2] = linVel[2];

	return 0;
}

int	b3CreatePoseCommandSetBaseAngularVelocity(b3SharedMemoryCommandHandle commandHandle, double angVel[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_INIT_POSE);
	command->m_updateFlags |= INIT_POSE_HAS_BASE_ANGULAR_VELOCITY;
	command->m_initPoseArgs.m_hasInitialStateQdot[3] = 1;
	command->m_initPoseArgs.m_hasInitialStateQdot[4] = 1;
	command->m_initPoseArgs.m_hasInitialStateQdot[5] = 1;

	command->m_initPoseArgs.m_initialStateQdot[3] = angVel[0];
	command->m_initPoseArgs.m_initialStateQdot[4] = angVel[1];
	command->m_initPoseArgs.m_initialStateQdot[5] = angVel[2];

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
		if ((i+7)<MAX_DEGREE_OF_FREEDOM)
		{
			command->m_initPoseArgs.m_initialStateQ[i+7] = jointPositions[i];
			command->m_initPoseArgs.m_hasInitialStateQ[i+7] = 1;
		}
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

int	b3CreatePoseCommandSetJointVelocities(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int numJointVelocities, const double* jointVelocities)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_INIT_POSE);
  
	command->m_updateFlags |=INIT_POSE_HAS_JOINT_VELOCITY;
	for (int i=0;i<numJointVelocities;i++)
	{
		if ((i+6)<MAX_DEGREE_OF_FREEDOM)
		{
			command->m_initPoseArgs.m_initialStateQdot[i+6] = jointVelocities[i];
			command->m_initPoseArgs.m_hasInitialStateQdot[i+6] = 1;
		}
	}
	return 0;
}

int	b3CreatePoseCommandSetJointVelocity(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointVelocity)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_INIT_POSE);
	command->m_updateFlags |=INIT_POSE_HAS_JOINT_VELOCITY;
	b3JointInfo info;
	b3GetJointInfo(physClient, command->m_initPoseArgs.m_bodyUniqueId,jointIndex, &info);
	btAssert((info.m_flags & JOINT_HAS_MOTORIZED_POWER) && info.m_uIndex >=0);
	if ((info.m_flags & JOINT_HAS_MOTORIZED_POWER) && (info.m_uIndex >=0) && (info.m_uIndex<MAX_DEGREE_OF_FREEDOM))
	{
		command->m_initPoseArgs.m_initialStateQdot[info.m_uIndex] = jointVelocity;
		command->m_initPoseArgs.m_hasInitialStateQdot[info.m_uIndex] = 1;
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



void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	cl->disconnectSharedMemory();
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
    //b3Assert(status);
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
				case CMD_MJCF_LOADING_COMPLETED:
				case CMD_BULLET_LOADING_COMPLETED:
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
				case CMD_CREATE_MULTI_BODY_COMPLETED:
				{
					bodyId = status->m_dataStreamArguments.m_bodyUniqueId;
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

b3SharedMemoryCommandHandle b3RequestCollisionInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_REQUEST_COLLISION_INFO;
	command->m_updateFlags = 0;
	command->m_requestCollisionInfoArgs.m_bodyUniqueId = bodyUniqueId;
    return (b3SharedMemoryCommandHandle) command;
}

int b3GetStatusAABB(b3SharedMemoryStatusHandle statusHandle, int linkIndex, double aabbMin[3], double aabbMax[3])
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
    const b3SendCollisionInfoArgs &args = status->m_sendCollisionInfoArgs;
    btAssert(status->m_type == CMD_REQUEST_COLLISION_INFO_COMPLETED);
    if (status->m_type != CMD_REQUEST_COLLISION_INFO_COMPLETED)
        return 0;

	if (linkIndex==-1)
	{	
		aabbMin[0] = args.m_rootWorldAABBMin[0];
		aabbMin[1] = args.m_rootWorldAABBMin[1];
		aabbMin[2] = args.m_rootWorldAABBMin[2];

		aabbMax[0] = args.m_rootWorldAABBMax[0];
		aabbMax[1] = args.m_rootWorldAABBMax[1];
		aabbMax[2] = args.m_rootWorldAABBMax[2];
		return 1;
	}

	if (linkIndex >= 0 && linkIndex < args.m_numLinks)
	{
		aabbMin[0] = args.m_linkWorldAABBsMin[linkIndex*3+0];
		aabbMin[1] = args.m_linkWorldAABBsMin[linkIndex*3+1];
		aabbMin[2] = args.m_linkWorldAABBsMin[linkIndex*3+2];

		aabbMax[0] = args.m_linkWorldAABBsMax[linkIndex*3+0];
		aabbMax[1] = args.m_linkWorldAABBsMax[linkIndex*3+1];
		aabbMax[2] = args.m_linkWorldAABBsMax[linkIndex*3+2];
		return 1;
	}

	return 0;
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
	b3Assert(command);
	b3Assert(cl);
	if (command && cl)
	{
		return (int)cl->submitClientCommand(*command);
	}
	return -1;

}

#include "../Utils/b3Clock.h"


b3SharedMemoryStatusHandle b3SubmitClientCommandAndWaitStatus(b3PhysicsClientHandle physClient, const b3SharedMemoryCommandHandle commandHandle)
{
	B3_PROFILE("b3SubmitClientCommandAndWaitStatus");
	b3Clock clock;
	double startTime = clock.getTimeInSeconds();
	

	b3SharedMemoryStatusHandle statusHandle = 0;
	b3Assert(commandHandle);
	b3Assert(physClient);
	if (physClient && commandHandle)
	{
		PhysicsClient* cl = (PhysicsClient* ) physClient;

		double timeOutInSeconds = cl->getTimeOut();

		{
			B3_PROFILE("b3SubmitClientCommand");
			b3SubmitClientCommand(physClient, commandHandle);
		}
		{
			B3_PROFILE("b3ProcessServerStatus");
			while (cl->isConnected() && (statusHandle == 0) && (clock.getTimeInSeconds()-startTime < timeOutInSeconds))
			{
					clock.usleep(0);
					statusHandle = b3ProcessServerStatus(physClient);
			}
		}
		return (b3SharedMemoryStatusHandle)statusHandle;
	}

	return 0;
}


///return the total number of bodies in the simulation
int	b3GetNumBodies(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getNumBodies();
}

int b3GetNumUserConstraints(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    return cl->getNumUserConstraints();
}

int b3GetUserConstraintInfo(b3PhysicsClientHandle physClient, int constraintUniqueId, struct b3UserConstraint* infoPtr)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3UserConstraint constraintInfo1;
    b3Assert(physClient);
    b3Assert(infoPtr);
    b3Assert(constraintUniqueId>=0);
    
    if (infoPtr==0)
        return 0;

    if (cl->getUserConstraintInfo(constraintUniqueId, constraintInfo1))
    {
        *infoPtr = constraintInfo1;
        return 1;
    }
    return 0;
}

/// return the user constraint id, given the index in range [0 , b3GetNumUserConstraints() )
int b3GetUserConstraintId(b3PhysicsClientHandle physClient, int serialIndex)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getUserConstraintId(serialIndex);
}

/// return the body unique id, given the index in range [0 , b3GetNumBodies() )
int b3GetBodyUniqueId(b3PhysicsClientHandle physClient, int serialIndex)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getBodyUniqueId(serialIndex);
}

///given a body unique id, return the body information. See b3BodyInfo in SharedMemoryPublic.h
int b3GetBodyInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, struct b3BodyInfo* info)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getBodyInfo(bodyUniqueId,*info);
}



int	b3GetNumJoints(b3PhysicsClientHandle physClient, int bodyId)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getNumJoints(bodyId);
}

int	b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyIndex, int jointIndex, struct b3JointInfo* info)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	return cl->getJointInfo(bodyIndex, jointIndex, *info);
}

b3SharedMemoryCommandHandle b3GetDynamicsInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);
	command->m_type = CMD_GET_DYNAMICS_INFO;
	command->m_getDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_getDynamicsInfoArgs.m_linkIndex = linkIndex;
	return (b3SharedMemoryCommandHandle) command;
}

int b3GetDynamicsInfo(b3SharedMemoryStatusHandle statusHandle, struct b3DynamicsInfo* info)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
	const b3DynamicsInfo &dynamicsInfo = status->m_dynamicsInfo;
	btAssert(status->m_type == CMD_GET_DYNAMICS_INFO);
	if (status->m_type != CMD_GET_DYNAMICS_INFO_COMPLETED)
		return false;

	info->m_mass = dynamicsInfo.m_mass;
	info->m_lateralFrictionCoeff = dynamicsInfo.m_lateralFrictionCoeff;
	return true;
}

b3SharedMemoryCommandHandle b3InitChangeDynamicsInfo(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);
	command->m_type = CMD_CHANGE_DYNAMICS_INFO;
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = -1;
	command->m_changeDynamicsInfoArgs.m_linkIndex = -2;
	command->m_updateFlags = 0;
	
	return (b3SharedMemoryCommandHandle) command;
}

int b3ChangeDynamicsInfoSetMass(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double mass)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	b3Assert(mass > 0);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linkIndex = linkIndex;
	command->m_changeDynamicsInfoArgs.m_mass = mass;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_MASS;
	return 0;
}

int b3ChangeDynamicsInfoSetLateralFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double lateralFriction)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linkIndex = linkIndex;
	command->m_changeDynamicsInfoArgs.m_lateralFriction = lateralFriction;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_LATERAL_FRICTION;
	return 0;
}

int b3ChangeDynamicsInfoSetSpinningFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linkIndex = linkIndex;
	command->m_changeDynamicsInfoArgs.m_spinningFriction = friction;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_SPINNING_FRICTION;
	return 0;

}
int b3ChangeDynamicsInfoSetRollingFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linkIndex = linkIndex;
	command->m_changeDynamicsInfoArgs.m_rollingFriction = friction;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_ROLLING_FRICTION;
	return 0;

}


int b3ChangeDynamicsInfoSetRestitution(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double restitution)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linkIndex = linkIndex;
	command->m_changeDynamicsInfoArgs.m_restitution = restitution;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_RESTITUTION;
	return 0;
}
int b3ChangeDynamicsInfoSetLinearDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double linearDamping)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linearDamping = linearDamping;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_LINEAR_DAMPING;
	return 0;

}


int b3ChangeDynamicsInfoSetAngularDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double angularDamping)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linearDamping = angularDamping;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_ANGULAR_DAMPING;
	return 0;
}

int b3ChangeDynamicsInfoSetContactStiffnessAndDamping(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex,double contactStiffness, double contactDamping)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linkIndex = linkIndex;
	command->m_changeDynamicsInfoArgs.m_contactStiffness =contactStiffness;
	command->m_changeDynamicsInfoArgs.m_contactDamping = contactDamping;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_CONTACT_STIFFNESS_AND_DAMPING;
	return 0;
}

int b3ChangeDynamicsInfoSetFrictionAnchor(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex, int frictionAnchor)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command->m_type == CMD_CHANGE_DYNAMICS_INFO);
	command->m_changeDynamicsInfoArgs.m_bodyUniqueId = bodyUniqueId;
	command->m_changeDynamicsInfoArgs.m_linkIndex = linkIndex;
	command->m_changeDynamicsInfoArgs.m_frictionAnchor = frictionAnchor;
	command->m_updateFlags |= CHANGE_DYNAMICS_INFO_SET_FRICTION_ANCHOR;
	return 0;
}



b3SharedMemoryCommandHandle b3InitCreateUserConstraintCommand(b3PhysicsClientHandle physClient, int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, struct b3JointInfo* info)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_USER_CONSTRAINT;
	command->m_updateFlags = USER_CONSTRAINT_ADD_CONSTRAINT;

    command->m_userConstraintArguments.m_parentBodyIndex = parentBodyIndex;
    command->m_userConstraintArguments.m_parentJointIndex = parentJointIndex;
    command->m_userConstraintArguments.m_childBodyIndex = childBodyIndex;
    command->m_userConstraintArguments.m_childJointIndex = childJointIndex;
    for (int i = 0; i < 7; ++i) {
        command->m_userConstraintArguments.m_parentFrame[i] = info->m_parentFrame[i];
        command->m_userConstraintArguments.m_childFrame[i] = info->m_childFrame[i];
    }
    for (int i = 0; i < 3; ++i) {
        command->m_userConstraintArguments.m_jointAxis[i] = info->m_jointAxis[i];
    }
    command->m_userConstraintArguments.m_jointType = info->m_jointType;
    return (b3SharedMemoryCommandHandle)command;
}

b3SharedMemoryCommandHandle  b3InitChangeUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_USER_CONSTRAINT;
	command->m_updateFlags = USER_CONSTRAINT_CHANGE_CONSTRAINT;
	command->m_userConstraintArguments.m_userConstraintUniqueId = userConstraintUniqueId;
	return (b3SharedMemoryCommandHandle)command;
}

int b3InitChangeUserConstraintSetPivotInB(b3SharedMemoryCommandHandle commandHandle, double pivotInB[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_CONSTRAINT);

	b3Assert(command->m_updateFlags & USER_CONSTRAINT_CHANGE_CONSTRAINT);

	command->m_updateFlags |= USER_CONSTRAINT_CHANGE_PIVOT_IN_B;

	command->m_userConstraintArguments.m_childFrame[0] = pivotInB[0];
	command->m_userConstraintArguments.m_childFrame[1] = pivotInB[1];
	command->m_userConstraintArguments.m_childFrame[2] = pivotInB[2];
	return 0;
}
int b3InitChangeUserConstraintSetFrameInB(b3SharedMemoryCommandHandle commandHandle, double frameOrnInB[4])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_CONSTRAINT);
	b3Assert(command->m_updateFlags & USER_CONSTRAINT_CHANGE_CONSTRAINT);

	command->m_updateFlags |= USER_CONSTRAINT_CHANGE_FRAME_ORN_IN_B;

	command->m_userConstraintArguments.m_childFrame[3] = frameOrnInB[0];
	command->m_userConstraintArguments.m_childFrame[4] = frameOrnInB[1];
	command->m_userConstraintArguments.m_childFrame[5] = frameOrnInB[2];
	command->m_userConstraintArguments.m_childFrame[6] = frameOrnInB[3];

	return 0;
}

int b3InitChangeUserConstraintSetMaxForce(b3SharedMemoryCommandHandle commandHandle, double maxAppliedForce)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_CONSTRAINT);
	b3Assert(command->m_updateFlags & USER_CONSTRAINT_CHANGE_CONSTRAINT);
	
	command->m_updateFlags |=USER_CONSTRAINT_CHANGE_MAX_FORCE;
	command->m_userConstraintArguments.m_maxAppliedForce = maxAppliedForce;

	return 0;
}
int b3InitChangeUserConstraintSetGearRatio(b3SharedMemoryCommandHandle commandHandle, double gearRatio)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_CONSTRAINT);
	b3Assert(command->m_updateFlags & USER_CONSTRAINT_CHANGE_CONSTRAINT);
	
	command->m_updateFlags |=USER_CONSTRAINT_CHANGE_GEAR_RATIO;
	command->m_userConstraintArguments.m_gearRatio = gearRatio;

	return 0;
}

b3SharedMemoryCommandHandle  b3InitRemoveUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId)
{
	 PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_USER_CONSTRAINT;
	command->m_updateFlags = USER_CONSTRAINT_REMOVE_CONSTRAINT;
	command->m_userConstraintArguments.m_userConstraintUniqueId = userConstraintUniqueId;
	return (b3SharedMemoryCommandHandle)command;
}

b3SharedMemoryCommandHandle b3InitRemoveBodyCommand(b3PhysicsClientHandle physClient, int bodyUniqueId)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_REMOVE_BODY;
	command->m_updateFlags = BODY_DELETE_FLAG;
	command->m_removeObjectArgs.m_numBodies = 1;
	command->m_removeObjectArgs.m_bodyUniqueIds[0] = bodyUniqueId;
	command->m_removeObjectArgs.m_numUserConstraints = 0;
	
	return (b3SharedMemoryCommandHandle)command;
}

int b3GetStatusUserConstraintUniqueId(b3SharedMemoryStatusHandle statusHandle)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
	b3Assert(status);
	b3Assert(status->m_type == CMD_USER_CONSTRAINT_COMPLETED);
	if (status && status->m_type == CMD_USER_CONSTRAINT_COMPLETED)
	{
		return status->m_userConstraintResultArgs.m_userConstraintUniqueId;
	}

	return -1;

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

b3SharedMemoryCommandHandle b3CreateRaycastCommandInit(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ)
{
    PhysicsClient *cl = (PhysicsClient *)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand *command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_REQUEST_RAY_CAST_INTERSECTIONS;
	command->m_requestRaycastIntersections.m_numRays = 1;
	command->m_requestRaycastIntersections.m_rayFromPositions[0][0] = rayFromWorldX;
	command->m_requestRaycastIntersections.m_rayFromPositions[0][1] = rayFromWorldY;
	command->m_requestRaycastIntersections.m_rayFromPositions[0][2] = rayFromWorldZ;
	command->m_requestRaycastIntersections.m_rayToPositions[0][0] = rayToWorldX;
	command->m_requestRaycastIntersections.m_rayToPositions[0][1] = rayToWorldY;
	command->m_requestRaycastIntersections.m_rayToPositions[0][2] = rayToWorldZ;

    return (b3SharedMemoryCommandHandle)command;
}

b3SharedMemoryCommandHandle b3CreateRaycastBatchCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient *cl = (PhysicsClient *)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand *command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_REQUEST_RAY_CAST_INTERSECTIONS;
	command->m_updateFlags = 0;
	command->m_requestRaycastIntersections.m_numRays = 0;
	return (b3SharedMemoryCommandHandle)command;
}

void b3RaycastBatchAddRay(b3SharedMemoryCommandHandle commandHandle, const double rayFromWorld[3], const double rayToWorld[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_REQUEST_RAY_CAST_INTERSECTIONS);
	if (command->m_type == CMD_REQUEST_RAY_CAST_INTERSECTIONS)
	{
		int numRays = command->m_requestRaycastIntersections.m_numRays;
		if (numRays<MAX_RAY_INTERSECTION_BATCH_SIZE)
		{
			command->m_requestRaycastIntersections.m_rayFromPositions[numRays][0] = rayFromWorld[0];
			command->m_requestRaycastIntersections.m_rayFromPositions[numRays][1] = rayFromWorld[1];
			command->m_requestRaycastIntersections.m_rayFromPositions[numRays][2] = rayFromWorld[2];
			command->m_requestRaycastIntersections.m_rayToPositions[numRays][0] = rayToWorld[0];
			command->m_requestRaycastIntersections.m_rayToPositions[numRays][1] = rayToWorld[1];
			command->m_requestRaycastIntersections.m_rayToPositions[numRays][2] = rayToWorld[2];
			command->m_requestRaycastIntersections.m_numRays++;
		}
	}
}


void b3GetRaycastInformation(b3PhysicsClientHandle physClient, struct b3RaycastInformation* raycastInfo)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	if (cl)
	{
		cl->getCachedRaycastHits(raycastInfo);
	}
}


///If you re-connected to an existing server, or server changed otherwise, sync the body info
b3SharedMemoryCommandHandle b3InitSyncBodyInfoCommand(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type =CMD_SYNC_BODY_INFO;
	return (b3SharedMemoryCommandHandle) command;
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


/// Add/remove user-specific debug lines and debug text messages
b3SharedMemoryCommandHandle b3InitUserDebugDrawAddLine3D(b3PhysicsClientHandle physClient, double fromXYZ[3], double toXYZ[3], double colorRGB[3], double lineWidth, double lifeTime)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_USER_DEBUG_DRAW;
	command->m_updateFlags = USER_DEBUG_HAS_LINE; //USER_DEBUG_HAS_TEXT
	
	command->m_userDebugDrawArgs.m_debugLineFromXYZ[0] = fromXYZ[0];
	command->m_userDebugDrawArgs.m_debugLineFromXYZ[1] = fromXYZ[1];
	command->m_userDebugDrawArgs.m_debugLineFromXYZ[2] = fromXYZ[2];

	command->m_userDebugDrawArgs.m_debugLineToXYZ[0] = toXYZ[0];
	command->m_userDebugDrawArgs.m_debugLineToXYZ[1] = toXYZ[1];
	command->m_userDebugDrawArgs.m_debugLineToXYZ[2] = toXYZ[2];
	
	command->m_userDebugDrawArgs.m_debugLineColorRGB[0] = colorRGB[0];
	command->m_userDebugDrawArgs.m_debugLineColorRGB[1] = colorRGB[1];
	command->m_userDebugDrawArgs.m_debugLineColorRGB[2] = colorRGB[2];
		
	command->m_userDebugDrawArgs.m_lineWidth = lineWidth;
	command->m_userDebugDrawArgs.m_lifeTime = lifeTime;
	command->m_userDebugDrawArgs.m_parentObjectUniqueId = -1;
	command->m_userDebugDrawArgs.m_parentLinkIndex = -1;
	command->m_userDebugDrawArgs.m_optionFlags = 0;

	return (b3SharedMemoryCommandHandle) command;
}

b3SharedMemoryCommandHandle b3InitUserDebugDrawAddText3D(b3PhysicsClientHandle physClient, const char* txt, double positionXYZ[3], double colorRGB[3], double textSize, double lifeTime)
{

	PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_USER_DEBUG_DRAW;
	command->m_updateFlags = USER_DEBUG_HAS_TEXT;
	
	int len = strlen(txt);
    if (len<MAX_FILENAME_LENGTH)
    {
		strcpy(command->m_userDebugDrawArgs.m_text,txt);
    } else
    {
        command->m_userDebugDrawArgs.m_text[0] = 0;
    }
	command->m_userDebugDrawArgs.m_textPositionXYZ[0] = positionXYZ[0];
	command->m_userDebugDrawArgs.m_textPositionXYZ[1] = positionXYZ[1];
	command->m_userDebugDrawArgs.m_textPositionXYZ[2] = positionXYZ[2];

	command->m_userDebugDrawArgs.m_textColorRGB[0] = colorRGB[0];
	command->m_userDebugDrawArgs.m_textColorRGB[1] = colorRGB[1];
	command->m_userDebugDrawArgs.m_textColorRGB[2] = colorRGB[2];
	
	command->m_userDebugDrawArgs.m_textSize = textSize;

	command->m_userDebugDrawArgs.m_lifeTime = lifeTime;
	command->m_userDebugDrawArgs.m_parentObjectUniqueId = -1;
	command->m_userDebugDrawArgs.m_parentLinkIndex = -1;

	command->m_userDebugDrawArgs.m_optionFlags = 0;

	return (b3SharedMemoryCommandHandle) command;
}

void b3UserDebugTextSetOptionFlags(b3SharedMemoryCommandHandle commandHandle, int optionFlags)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_DEBUG_DRAW);
	b3Assert(command->m_updateFlags & USER_DEBUG_HAS_TEXT);
	command->m_userDebugDrawArgs.m_optionFlags = optionFlags;
	command->m_updateFlags |= USER_DEBUG_HAS_OPTION_FLAGS;
}

void b3UserDebugTextSetOrientation(b3SharedMemoryCommandHandle commandHandle, double orientation[4])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_DEBUG_DRAW);
	b3Assert(command->m_updateFlags & USER_DEBUG_HAS_TEXT);
	command->m_userDebugDrawArgs.m_textOrientation[0] = orientation[0];
	command->m_userDebugDrawArgs.m_textOrientation[1] = orientation[1];
	command->m_userDebugDrawArgs.m_textOrientation[2] = orientation[2];
	command->m_userDebugDrawArgs.m_textOrientation[3] = orientation[3];
	command->m_updateFlags |= USER_DEBUG_HAS_TEXT_ORIENTATION;

}


void b3UserDebugItemSetParentObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_DEBUG_DRAW);
	
	command->m_updateFlags |= USER_DEBUG_HAS_PARENT_OBJECT;
	command->m_userDebugDrawArgs.m_parentObjectUniqueId = objectUniqueId;
	command->m_userDebugDrawArgs.m_parentLinkIndex = linkIndex;
}


b3SharedMemoryCommandHandle b3InitUserDebugAddParameter(b3PhysicsClientHandle physClient, const char* txt, double rangeMin, double rangeMax, double startValue)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_USER_DEBUG_DRAW;
	command->m_updateFlags = USER_DEBUG_ADD_PARAMETER;
	int len = strlen(txt);
    if (len<MAX_FILENAME_LENGTH)
    {
		strcpy(command->m_userDebugDrawArgs.m_text,txt);
    } else
    {
        command->m_userDebugDrawArgs.m_text[0] = 0;
    }
	command->m_userDebugDrawArgs.m_rangeMin = rangeMin;
	command->m_userDebugDrawArgs.m_rangeMax = rangeMax;
	command->m_userDebugDrawArgs.m_startValue = startValue;
	command->m_userDebugDrawArgs.m_parentObjectUniqueId = -1;
	command->m_userDebugDrawArgs.m_optionFlags = 0;
	return (b3SharedMemoryCommandHandle)command;
}

b3SharedMemoryCommandHandle b3InitUserDebugReadParameter(b3PhysicsClientHandle physClient, int debugItemUniqueId)
{
		PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_USER_DEBUG_DRAW;
	command->m_updateFlags = USER_DEBUG_READ_PARAMETER;
	command->m_userDebugDrawArgs.m_itemUniqueId = debugItemUniqueId;
	return (b3SharedMemoryCommandHandle) command;
}

int b3GetStatusDebugParameterValue(b3SharedMemoryStatusHandle statusHandle, double* paramValue)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus*)statusHandle;
	btAssert(status->m_type == CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED);
	if (paramValue && (status->m_type == CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED))
	{
		*paramValue = status->m_userDebugDrawArgs.m_parameterValue;
		return 1;
	}
	return 0;
}


b3SharedMemoryCommandHandle b3InitUserDebugDrawRemove(b3PhysicsClientHandle physClient, int debugItemUniqueId)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_USER_DEBUG_DRAW;
	command->m_updateFlags = USER_DEBUG_REMOVE_ONE_ITEM;
	command->m_userDebugDrawArgs.m_itemUniqueId = debugItemUniqueId;
	return (b3SharedMemoryCommandHandle) command;

}

b3SharedMemoryCommandHandle b3InitUserDebugDrawRemoveAll(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_USER_DEBUG_DRAW;
	command->m_updateFlags = USER_DEBUG_REMOVE_ALL;
	return (b3SharedMemoryCommandHandle) command;
}

int b3GetDebugItemUniqueId(b3SharedMemoryStatusHandle statusHandle)
{
    const SharedMemoryStatus* status = (const SharedMemoryStatus*)statusHandle;
    btAssert(status->m_type == CMD_USER_DEBUG_DRAW_COMPLETED);
    if (status->m_type != CMD_USER_DEBUG_DRAW_COMPLETED)
        return -1;

	return status->m_userDebugDrawArgs.m_debugItemUniqueId;
}

b3SharedMemoryCommandHandle b3InitDebugDrawingCommand(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);
	command->m_type = CMD_USER_DEBUG_DRAW;
	command->m_updateFlags = 0;
	return  (b3SharedMemoryCommandHandle)command;
}



void b3SetDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex, double objectColorRGB[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_DEBUG_DRAW);
	command->m_updateFlags |= USER_DEBUG_SET_CUSTOM_OBJECT_COLOR;
	command->m_userDebugDrawArgs.m_objectUniqueId = objectUniqueId;
	command->m_userDebugDrawArgs.m_linkIndex = linkIndex;
	command->m_userDebugDrawArgs.m_objectDebugColorRGB[0] = objectColorRGB[0];
	command->m_userDebugDrawArgs.m_objectDebugColorRGB[1] = objectColorRGB[1];
	command->m_userDebugDrawArgs.m_objectDebugColorRGB[2] = objectColorRGB[2];
}

void b3RemoveDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_USER_DEBUG_DRAW);
	command->m_updateFlags |= USER_DEBUG_REMOVE_CUSTOM_OBJECT_COLOR;
	command->m_userDebugDrawArgs.m_objectUniqueId = objectUniqueId;
	command->m_userDebugDrawArgs.m_linkIndex = linkIndex;

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

void b3RequestCameraImageSelectRenderer(b3SharedMemoryCommandHandle commandHandle, int renderer)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    b3Assert(renderer>(1<<15));
	if (renderer>(1<<15))
	{
	    command->m_updateFlags |= renderer;
	}
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

void b3RequestCameraImageSetLightDirection(b3SharedMemoryCommandHandle commandHandle, const float lightDirection[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
	for (int i = 0; i<3; i++)
	{
		command->m_requestPixelDataArguments.m_lightDirection[i] = lightDirection[i];
	}
	command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_LIGHT_DIRECTION;
}

void b3RequestCameraImageSetLightColor(b3SharedMemoryCommandHandle commandHandle, const float lightColor[3])
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    for (int i = 0; i<3; i++)
    {
        command->m_requestPixelDataArguments.m_lightColor[i] = lightColor[i];
    }
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_LIGHT_COLOR;
}

void b3RequestCameraImageSetLightDistance(b3SharedMemoryCommandHandle commandHandle, float lightDistance)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    command->m_requestPixelDataArguments.m_lightDistance = lightDistance;
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_LIGHT_DISTANCE;
}

void b3RequestCameraImageSetLightAmbientCoeff(b3SharedMemoryCommandHandle commandHandle, float lightAmbientCoeff)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    command->m_requestPixelDataArguments.m_lightAmbientCoeff = lightAmbientCoeff;
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_AMBIENT_COEFF;
}

void b3RequestCameraImageSetLightDiffuseCoeff(b3SharedMemoryCommandHandle commandHandle, float lightDiffuseCoeff)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    command->m_requestPixelDataArguments.m_lightDiffuseCoeff = lightDiffuseCoeff;
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_DIFFUSE_COEFF;
}

void b3RequestCameraImageSetLightSpecularCoeff(b3SharedMemoryCommandHandle commandHandle, float lightSpecularCoeff)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    command->m_requestPixelDataArguments.m_lightSpecularCoeff = lightSpecularCoeff;
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_SPECULAR_COEFF;
}

void b3RequestCameraImageSetShadow(b3SharedMemoryCommandHandle commandHandle, int hasShadow)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
    command->m_requestPixelDataArguments.m_hasShadow = hasShadow;
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_SET_SHADOW;
}

void b3ComputePositionFromViewMatrix(const float viewMatrix[16], float cameraPosition[3], float cameraTargetPosition[3], float cameraUp[3])
{
	b3Matrix3x3 r(viewMatrix[0], viewMatrix[4], viewMatrix[8], viewMatrix[1], viewMatrix[5], viewMatrix[9], viewMatrix[2], viewMatrix[6], viewMatrix[10]);
	b3Vector3 p = b3MakeVector3(viewMatrix[12], viewMatrix[13], viewMatrix[14]);
	b3Transform t(r,p);
	b3Transform tinv = t.inverse();
	b3Matrix3x3 basis = tinv.getBasis();
	b3Vector3 origin = tinv.getOrigin();
	b3Vector3 s = b3MakeVector3(basis[0][0], basis[1][0], basis[2][0]);
	b3Vector3 u = b3MakeVector3(basis[0][1], basis[1][1], basis[2][1]);
	b3Vector3 f = b3MakeVector3(-basis[0][2], -basis[1][2], -basis[2][2]);
	b3Vector3 eye = origin;
	cameraPosition[0] = eye[0];
	cameraPosition[1] = eye[1];
	cameraPosition[2] = eye[2];
	b3Vector3 center = f + eye;
	cameraTargetPosition[0] = center[0];
	cameraTargetPosition[1] = center[1];
	cameraTargetPosition[2] = center[2];
	cameraUp[0] = u[0];
	cameraUp[1] = u[1];
	cameraUp[2] = u[2];
}

void b3ComputeViewMatrixFromPositions(const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3], float viewMatrix[16])
{
	b3Vector3 eye = b3MakeVector3(cameraPosition[0], cameraPosition[1], cameraPosition[2]);
	b3Vector3 center = b3MakeVector3(cameraTargetPosition[0], cameraTargetPosition[1], cameraTargetPosition[2]);
	b3Vector3 up = b3MakeVector3(cameraUp[0], cameraUp[1], cameraUp[2]);
	b3Vector3 f = (center - eye).normalized();
	b3Vector3 u = up.normalized();
	b3Vector3 s = (f.cross(u)).normalized();
	u = s.cross(f);

	viewMatrix[0 * 4 + 0] = s.x;
	viewMatrix[1 * 4 + 0] = s.y;
	viewMatrix[2 * 4 + 0] = s.z;

	viewMatrix[0 * 4 + 1] = u.x;
	viewMatrix[1 * 4 + 1] = u.y;
	viewMatrix[2 * 4 + 1] = u.z;

	viewMatrix[0 * 4 + 2] = -f.x;
	viewMatrix[1 * 4 + 2] = -f.y;
	viewMatrix[2 * 4 + 2] = -f.z;

	viewMatrix[0 * 4 + 3] = 0.f;
	viewMatrix[1 * 4 + 3] = 0.f;
	viewMatrix[2 * 4 + 3] = 0.f;

	viewMatrix[3 * 4 + 0] = -s.dot(eye);
	viewMatrix[3 * 4 + 1] = -u.dot(eye);
	viewMatrix[3 * 4 + 2] = f.dot(eye);
	viewMatrix[3 * 4 + 3] = 1.f;
}


void b3ComputeViewMatrixFromYawPitchRoll(const float cameraTargetPosition[3], float distance, float yaw, float pitch, float roll, int upAxis, float viewMatrix[16])
{
	b3Vector3 camUpVector;
	b3Vector3 camForward;
	b3Vector3 camPos;
	b3Vector3 camTargetPos = b3MakeVector3(cameraTargetPosition[0], cameraTargetPosition[1], cameraTargetPosition[2]);
	b3Vector3 eyePos = b3MakeVector3(0, 0, 0);
	
	b3Scalar yawRad = yaw * b3Scalar(0.01745329251994329547);// rads per deg
	b3Scalar pitchRad = pitch * b3Scalar(0.01745329251994329547);// rads per deg
	b3Scalar rollRad = 0.0;
	b3Quaternion eyeRot;
	
	int forwardAxis(-1);
	switch (upAxis)
	{
		case 1:
			forwardAxis = 2;
			camUpVector = b3MakeVector3(0,1,0);
			eyeRot.setEulerZYX(rollRad, yawRad, -pitchRad);
			break;
		case 2:
			forwardAxis = 1;
			camUpVector = b3MakeVector3(0,0,1);
			eyeRot.setEulerZYX(yawRad, rollRad, pitchRad);
			break;
		default:
			return;
	};
	
	eyePos[forwardAxis] = -distance;
	
	camForward = b3MakeVector3(eyePos[0],eyePos[1],eyePos[2]);
	if (camForward.length2() < B3_EPSILON)
	{
		camForward.setValue(1.f,0.f,0.f);
	} else
	{
		camForward.normalize();
	}
	
	eyePos = b3Matrix3x3(eyeRot)*eyePos;
	camUpVector = b3Matrix3x3(eyeRot)*camUpVector;
	
	camPos = eyePos;
	camPos += camTargetPos;

	float camPosf[4] = { camPos[0],camPos[1],camPos[2],0 };
	float camPosTargetf[4] = { camTargetPos[0],camTargetPos[1],camTargetPos[2],0 };
	float camUpf[4] = { camUpVector[0],camUpVector[1],camUpVector[2],0 };

	b3ComputeViewMatrixFromPositions(camPosf, camPosTargetf, camUpf,viewMatrix);

}


void b3ComputeProjectionMatrix(float left, float right, float bottom, float top, float nearVal, float farVal, float projectionMatrix[16])
{
	projectionMatrix[0 * 4 + 0] = (float(2) * nearVal) / (right - left);
	projectionMatrix[0 * 4 + 1] = float(0);
	projectionMatrix[0 * 4 + 2] = float(0);
	projectionMatrix[0 * 4 + 3] = float(0);

	projectionMatrix[1 * 4 + 0] = float(0);
	projectionMatrix[1 * 4 + 1] = (float(2) * nearVal) / (top - bottom);
	projectionMatrix[1 * 4 + 2] = float(0);
	projectionMatrix[1 * 4 + 3] = float(0);

	projectionMatrix[2 * 4 + 0] = (right + left) / (right - left);
	projectionMatrix[2 * 4 + 1] = (top + bottom) / (top - bottom);
	projectionMatrix[2 * 4 + 2] = -(farVal + nearVal) / (farVal - nearVal);
	projectionMatrix[2 * 4 + 3] = float(-1);

	projectionMatrix[3 * 4 + 0] = float(0);
	projectionMatrix[3 * 4 + 1] = float(0);
	projectionMatrix[3 * 4 + 2] = -(float(2) * farVal * nearVal) / (farVal - nearVal);
	projectionMatrix[3 * 4 + 3] = float(0);
}


void b3ComputeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, float projectionMatrix[16])
{
	float yScale = 1.0 / tan((3.141592538 / 180.0) * fov / 2);
	float xScale = yScale / aspect;

	projectionMatrix[0 * 4 + 0] = xScale;
	projectionMatrix[0 * 4 + 1] = float(0);
	projectionMatrix[0 * 4 + 2] = float(0);
	projectionMatrix[0 * 4 + 3] = float(0);

	projectionMatrix[1 * 4 + 0] = float(0);
	projectionMatrix[1 * 4 + 1] = yScale;
	projectionMatrix[1 * 4 + 2] = float(0);
	projectionMatrix[1 * 4 + 3] = float(0);

	projectionMatrix[2 * 4 + 0] = 0;
	projectionMatrix[2 * 4 + 1] = 0;
	projectionMatrix[2 * 4 + 2] = (nearVal + farVal) / (nearVal - farVal);
	projectionMatrix[2 * 4 + 3] = float(-1);

	projectionMatrix[3 * 4 + 0] = float(0);
	projectionMatrix[3 * 4 + 1] = float(0);
	projectionMatrix[3 * 4 + 2] = (float(2) * farVal * nearVal) / (nearVal - farVal);
	projectionMatrix[3 * 4 + 3] = float(0);
}


void b3RequestCameraImageSetViewMatrix2(b3SharedMemoryCommandHandle commandHandle, const float cameraTargetPosition[3], float distance, float yaw, float pitch, float roll, int upAxis)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);
 
	b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxis, command->m_requestPixelDataArguments.m_viewMatrix);
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES;
    
}




void b3RequestCameraImageSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3])
{
	float viewMatrix[16];
	b3ComputeViewMatrixFromPositions(cameraPosition, cameraTargetPosition, cameraUp, viewMatrix);
    
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);

	b3ComputeViewMatrixFromPositions(cameraPosition, cameraTargetPosition, cameraUp, command->m_requestPixelDataArguments.m_viewMatrix);
   
    command->m_updateFlags |= REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES;

}

void b3RequestCameraImageSetProjectionMatrix(b3SharedMemoryCommandHandle commandHandle, float left, float  right, float bottom, float top, float nearVal, float farVal)
{
    
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);

	b3ComputeProjectionMatrix(left, right, bottom, top, nearVal, farVal, command->m_requestPixelDataArguments.m_projectionMatrix);

    command->m_updateFlags |= REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES;
}

void b3RequestCameraImageSetFOVProjectionMatrix(b3SharedMemoryCommandHandle commandHandle, float fov, float aspect, float nearVal, float farVal)
{
  

  struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
  b3Assert(command);
  b3Assert(command->m_type == CMD_REQUEST_CAMERA_IMAGE_DATA);

  b3ComputeProjectionMatrixFOV(fov, aspect, nearVal, farVal, command->m_requestPixelDataArguments.m_projectionMatrix);

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

///request an contact point information
b3SharedMemoryCommandHandle b3InitRequestContactPointInformation(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type =CMD_REQUEST_CONTACT_POINT_INFORMATION;
	command->m_requestContactPointArguments.m_startingContactPointIndex = 0;
	command->m_requestContactPointArguments.m_objectAIndexFilter = -1;
	command->m_requestContactPointArguments.m_objectBIndexFilter = -1;
	command->m_requestContactPointArguments.m_linkIndexAIndexFilter = -2;
	command->m_requestContactPointArguments.m_linkIndexBIndexFilter = -2;
    command->m_updateFlags = 0;
    return (b3SharedMemoryCommandHandle) command;
}

void b3SetContactFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CONTACT_POINT_INFORMATION);
	command->m_requestContactPointArguments.m_objectAIndexFilter = bodyUniqueIdA;
}

void b3SetContactFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CONTACT_POINT_INFORMATION);
	command->m_updateFlags |= CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_A_FILTER;
	command->m_requestContactPointArguments.m_linkIndexAIndexFilter= linkIndexA;
}

void b3SetContactFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_REQUEST_CONTACT_POINT_INFORMATION);
	command->m_updateFlags |= CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_B_FILTER;
	command->m_requestContactPointArguments.m_linkIndexBIndexFilter = linkIndexB;
}

void b3SetClosestDistanceFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA)
{
	b3SetContactFilterLinkA(commandHandle, linkIndexA);
}

void b3SetClosestDistanceFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB)
{
	b3SetContactFilterLinkB(commandHandle, linkIndexB);
}




void b3SetContactFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_REQUEST_CONTACT_POINT_INFORMATION);
	command->m_requestContactPointArguments.m_objectBIndexFilter = bodyUniqueIdB;
}


///compute the closest points between two bodies
b3SharedMemoryCommandHandle b3InitClosestDistanceQuery(b3PhysicsClientHandle physClient)
{
	b3SharedMemoryCommandHandle commandHandle =b3InitRequestContactPointInformation(physClient);
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_REQUEST_CONTACT_POINT_INFORMATION);
	command->m_updateFlags = CMD_REQUEST_CONTACT_POINT_HAS_QUERY_MODE;
	command->m_requestContactPointArguments.m_mode = CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS;
	return commandHandle;
}

void b3SetClosestDistanceFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA)
{
	b3SetContactFilterBodyA(commandHandle,bodyUniqueIdA);
}

void b3SetClosestDistanceFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB)
{
	b3SetContactFilterBodyB(commandHandle,bodyUniqueIdB);
}

void b3SetClosestDistanceThreshold(b3SharedMemoryCommandHandle commandHandle, double distance)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_REQUEST_CONTACT_POINT_INFORMATION);
	command->m_updateFlags += CMD_REQUEST_CONTACT_POINT_HAS_CLOSEST_DISTANCE_THRESHOLD;
	command->m_requestContactPointArguments.m_closestDistanceThreshold = distance;
}


///get all the bodies that touch a given axis aligned bounding box specified in world space (min and max coordinates)
b3SharedMemoryCommandHandle b3InitAABBOverlapQuery(b3PhysicsClientHandle physClient, const double aabbMin[3], const double aabbMax[3])
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);
	command->m_type = CMD_REQUEST_AABB_OVERLAP;
	command->m_updateFlags = 0;
	command->m_requestOverlappingObjectsArgs.m_startingOverlappingObjectIndex = 0;
	command->m_requestOverlappingObjectsArgs.m_aabbQueryMin[0] = aabbMin[0];
	command->m_requestOverlappingObjectsArgs.m_aabbQueryMin[1] = aabbMin[1];
	command->m_requestOverlappingObjectsArgs.m_aabbQueryMin[2] = aabbMin[2];

	command->m_requestOverlappingObjectsArgs.m_aabbQueryMax[0] = aabbMax[0];
	command->m_requestOverlappingObjectsArgs.m_aabbQueryMax[1] = aabbMax[1];
	command->m_requestOverlappingObjectsArgs.m_aabbQueryMax[2] = aabbMax[2];
	return  (b3SharedMemoryCommandHandle)command;
}

void b3GetAABBOverlapResults(b3PhysicsClientHandle physClient, struct b3AABBOverlapData* data)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	if (cl)
	{
		cl->getCachedOverlappingObjects(data);
	}
}



void b3GetContactPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointData)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
	if (cl)
	{
	    cl->getCachedContactPointInformation(contactPointData);
	}
}

void b3GetClosestPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointInfo)
{
	b3GetContactPointInformation(physClient,contactPointInfo);
}



//request visual shape information
b3SharedMemoryCommandHandle b3InitRequestVisualShapeInformation(b3PhysicsClientHandle physClient, int bodyUniqueIdA)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_REQUEST_VISUAL_SHAPE_INFO;
	command->m_requestVisualShapeDataArguments.m_bodyUniqueId = bodyUniqueIdA;
	command->m_requestVisualShapeDataArguments.m_startingVisualShapeIndex = 0;
	command->m_updateFlags = 0;
    return (b3SharedMemoryCommandHandle) command;
}

void b3GetVisualShapeInformation(b3PhysicsClientHandle physClient, struct b3VisualShapeInformation* visualShapeInfo)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	if (cl)
	{
		cl->getCachedVisualShapeInformation(visualShapeInfo);
	}
}

b3SharedMemoryCommandHandle b3InitLoadTexture(b3PhysicsClientHandle physClient, const char* filename)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_LOAD_TEXTURE;
    int len = strlen(filename);
    if (len<MAX_FILENAME_LENGTH)
    {
        strcpy(command->m_loadTextureArguments.m_textureFileName,filename);
    } else
    {
        command->m_loadTextureArguments.m_textureFileName[0] = 0;
    }
    command->m_updateFlags = 0;
    return (b3SharedMemoryCommandHandle) command;
}

b3SharedMemoryCommandHandle b3InitUpdateVisualShape(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, int shapeIndex, int textureUniqueId)
{
    PhysicsClient* cl = (PhysicsClient* ) physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    command->m_type = CMD_UPDATE_VISUAL_SHAPE;
    command->m_updateVisualShapeDataArguments.m_bodyUniqueId = bodyUniqueId;
    command->m_updateVisualShapeDataArguments.m_jointIndex = jointIndex;
    command->m_updateVisualShapeDataArguments.m_shapeIndex = shapeIndex;
    command->m_updateVisualShapeDataArguments.m_textureUniqueId = textureUniqueId;
    command->m_updateFlags = 0;

	if (textureUniqueId>=0)
	{
		command->m_updateFlags |= CMD_UPDATE_VISUAL_SHAPE_TEXTURE;
	}
    return (b3SharedMemoryCommandHandle) command;
}

void b3UpdateVisualShapeRGBAColor(b3SharedMemoryCommandHandle commandHandle, double rgbaColor[4])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_UPDATE_VISUAL_SHAPE);

	if (command->m_type == CMD_UPDATE_VISUAL_SHAPE)
	{
		command->m_updateVisualShapeDataArguments.m_rgbaColor[0] = rgbaColor[0];
		command->m_updateVisualShapeDataArguments.m_rgbaColor[1] = rgbaColor[1];
		command->m_updateVisualShapeDataArguments.m_rgbaColor[2] = rgbaColor[2];
		command->m_updateVisualShapeDataArguments.m_rgbaColor[3] = rgbaColor[3];
		command->m_updateFlags |= CMD_UPDATE_VISUAL_SHAPE_RGBA_COLOR;
	}
}

void b3UpdateVisualShapeSpecularColor(b3SharedMemoryCommandHandle commandHandle, double specularColor[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type == CMD_UPDATE_VISUAL_SHAPE);

	if (command->m_type == CMD_UPDATE_VISUAL_SHAPE)
	{
		command->m_updateVisualShapeDataArguments.m_specularColor[0] = specularColor[0];
		command->m_updateVisualShapeDataArguments.m_specularColor[1] = specularColor[1];
		command->m_updateVisualShapeDataArguments.m_specularColor[2] = specularColor[2];
		command->m_updateFlags |= CMD_UPDATE_VISUAL_SHAPE_SPECULAR_COLOR;
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




///compute the forces to achieve an acceleration, given a state q and qdot using inverse dynamics
b3SharedMemoryCommandHandle	b3CalculateInverseDynamicsCommandInit(b3PhysicsClientHandle physClient, int bodyIndex,
	const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);

	command->m_type = CMD_CALCULATE_INVERSE_DYNAMICS;
	command->m_updateFlags = 0;
	command->m_calculateInverseDynamicsArguments.m_bodyUniqueId = bodyIndex;
	int numJoints = cl->getNumJoints(bodyIndex);
	for (int i = 0; i < numJoints;i++)
	{
		command->m_calculateInverseDynamicsArguments.m_jointPositionsQ[i] = jointPositionsQ[i];
		command->m_calculateInverseDynamicsArguments.m_jointVelocitiesQdot[i] = jointVelocitiesQdot[i];
		command->m_calculateInverseDynamicsArguments.m_jointAccelerations[i] = jointAccelerations[i];
	}

	return (b3SharedMemoryCommandHandle)command;
}

int b3GetStatusInverseDynamicsJointForces(b3SharedMemoryStatusHandle statusHandle,
	int* bodyUniqueId,
	int* dofCount,
	double* jointForces)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus*)statusHandle;
	btAssert(status->m_type == CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED);
	if (status->m_type != CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED)
		return false;


	if (dofCount)
	{
		*dofCount = status->m_inverseDynamicsResultArgs.m_dofCount;
	}
	if (bodyUniqueId)
	{
		*bodyUniqueId = status->m_inverseDynamicsResultArgs.m_bodyUniqueId;
	}
	if (jointForces)
	{
		for (int i = 0; i < status->m_inverseDynamicsResultArgs.m_dofCount; i++)
		{
			jointForces[i] = status->m_inverseDynamicsResultArgs.m_jointForces[i];
		}
	}

	
	return true;
}

b3SharedMemoryCommandHandle	b3CalculateJacobianCommandInit(b3PhysicsClientHandle physClient, int bodyIndex, int linkIndex, const double* localPosition, const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations)
{
    PhysicsClient* cl = (PhysicsClient*)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_CALCULATE_JACOBIAN;
    command->m_updateFlags = 0;
    command->m_calculateJacobianArguments.m_bodyUniqueId = bodyIndex;
    command->m_calculateJacobianArguments.m_linkIndex = linkIndex;
    command->m_calculateJacobianArguments.m_localPosition[0] = localPosition[0];
    command->m_calculateJacobianArguments.m_localPosition[1] = localPosition[1];
    command->m_calculateJacobianArguments.m_localPosition[2] = localPosition[2];
    int numJoints = cl->getNumJoints(bodyIndex);
    for (int i = 0; i < numJoints;i++)
    {
        command->m_calculateJacobianArguments.m_jointPositionsQ[i] = jointPositionsQ[i];
        command->m_calculateJacobianArguments.m_jointVelocitiesQdot[i] = jointVelocitiesQdot[i];
        command->m_calculateJacobianArguments.m_jointAccelerations[i] = jointAccelerations[i];
    }
    
    return (b3SharedMemoryCommandHandle)command;
}


int b3GetStatusJacobian(b3SharedMemoryStatusHandle statusHandle, double* linearJacobian, double* angularJacobian)
{
    const SharedMemoryStatus* status = (const SharedMemoryStatus*)statusHandle;
    btAssert(status->m_type == CMD_CALCULATED_JACOBIAN_COMPLETED);
    if (status->m_type != CMD_CALCULATED_JACOBIAN_COMPLETED)
        return false;
    
    if (linearJacobian)
    {
        for (int i = 0; i < status->m_jacobianResultArgs.m_dofCount*3; i++)
        {
            linearJacobian[i] = status->m_jacobianResultArgs.m_linearJacobian[i];
        }
    }
    if (angularJacobian)
    {
        for (int i = 0; i < status->m_jacobianResultArgs.m_dofCount*3; i++)
        {
            angularJacobian[i] = status->m_jacobianResultArgs.m_angularJacobian[i];
        }

    }
    
    return true;
}

///compute the joint positions to move the end effector to a desired target using inverse kinematics
b3SharedMemoryCommandHandle	b3CalculateInverseKinematicsCommandInit(b3PhysicsClientHandle physClient, int bodyIndex)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);

	command->m_type = CMD_CALCULATE_INVERSE_KINEMATICS;
	command->m_updateFlags = 0;
	command->m_calculateInverseKinematicsArguments.m_bodyUniqueId = bodyIndex;

	return (b3SharedMemoryCommandHandle)command;

}

void b3CalculateInverseKinematicsAddTargetPurePosition(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CALCULATE_INVERSE_KINEMATICS);
    command->m_updateFlags |= IK_HAS_TARGET_POSITION;
	command->m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex = endEffectorLinkIndex;
	
	command->m_calculateInverseKinematicsArguments.m_targetPosition[0] = targetPosition[0];
	command->m_calculateInverseKinematicsArguments.m_targetPosition[1] = targetPosition[1];
	command->m_calculateInverseKinematicsArguments.m_targetPosition[2] = targetPosition[2];
   

}
void b3CalculateInverseKinematicsAddTargetPositionWithOrientation(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[3], const double targetOrientation[4])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CALCULATE_INVERSE_KINEMATICS);
    command->m_updateFlags |= IK_HAS_TARGET_POSITION+IK_HAS_TARGET_ORIENTATION;
	command->m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex = endEffectorLinkIndex;

	command->m_calculateInverseKinematicsArguments.m_targetPosition[0] = targetPosition[0];
	command->m_calculateInverseKinematicsArguments.m_targetPosition[1] = targetPosition[1];
	command->m_calculateInverseKinematicsArguments.m_targetPosition[2] = targetPosition[2];
    
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[0] = targetOrientation[0];
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[1] = targetOrientation[1];
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[2] = targetOrientation[2];
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[3] = targetOrientation[3];

}

void b3CalculateInverseKinematicsPosWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[3], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CALCULATE_INVERSE_KINEMATICS);
    command->m_updateFlags |= IK_HAS_TARGET_POSITION+IK_HAS_NULL_SPACE_VELOCITY;
    command->m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex = endEffectorLinkIndex;
    
    command->m_calculateInverseKinematicsArguments.m_targetPosition[0] = targetPosition[0];
    command->m_calculateInverseKinematicsArguments.m_targetPosition[1] = targetPosition[1];
    command->m_calculateInverseKinematicsArguments.m_targetPosition[2] = targetPosition[2];
    
    for (int i = 0; i < numDof; ++i)
    {
        command->m_calculateInverseKinematicsArguments.m_lowerLimit[i] = lowerLimit[i];
        command->m_calculateInverseKinematicsArguments.m_upperLimit[i] = upperLimit[i];
        command->m_calculateInverseKinematicsArguments.m_jointRange[i] = jointRange[i];
        command->m_calculateInverseKinematicsArguments.m_restPose[i] = restPose[i];
    }
}

void b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[3], const double targetOrientation[4], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CALCULATE_INVERSE_KINEMATICS);
    command->m_updateFlags |= IK_HAS_TARGET_POSITION+IK_HAS_TARGET_ORIENTATION+IK_HAS_NULL_SPACE_VELOCITY;
    command->m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex = endEffectorLinkIndex;
    
    command->m_calculateInverseKinematicsArguments.m_targetPosition[0] = targetPosition[0];
    command->m_calculateInverseKinematicsArguments.m_targetPosition[1] = targetPosition[1];
    command->m_calculateInverseKinematicsArguments.m_targetPosition[2] = targetPosition[2];
    
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[0] = targetOrientation[0];
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[1] = targetOrientation[1];
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[2] = targetOrientation[2];
    command->m_calculateInverseKinematicsArguments.m_targetOrientation[3] = targetOrientation[3];
    
    for (int i = 0; i < numDof; ++i)
    {
        command->m_calculateInverseKinematicsArguments.m_lowerLimit[i] = lowerLimit[i];
        command->m_calculateInverseKinematicsArguments.m_upperLimit[i] = upperLimit[i];
        command->m_calculateInverseKinematicsArguments.m_jointRange[i] = jointRange[i];
        command->m_calculateInverseKinematicsArguments.m_restPose[i] = restPose[i];
    }

}

void b3CalculateInverseKinematicsSetJointDamping(b3SharedMemoryCommandHandle commandHandle, int numDof, const double* jointDampingCoeff)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CALCULATE_INVERSE_KINEMATICS);
    command->m_updateFlags |= IK_HAS_JOINT_DAMPING;
    
    for (int i = 0; i < numDof; ++i)
    {
        command->m_calculateInverseKinematicsArguments.m_jointDamping[i] = jointDampingCoeff[i];
    }
}

int b3GetStatusInverseKinematicsJointPositions(b3SharedMemoryStatusHandle statusHandle,
	int* bodyUniqueId,
	int* dofCount,
	double* jointPositions)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus*)statusHandle;
	btAssert(status->m_type == CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED);
	if (status->m_type != CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED)
		return false;


	if (dofCount)
	{
		*dofCount = status->m_inverseKinematicsResultArgs.m_dofCount;
	}
	if (bodyUniqueId)
	{
		*bodyUniqueId = status->m_inverseKinematicsResultArgs.m_bodyUniqueId;
	}
	if (jointPositions)
	{
		for (int i = 0; i < status->m_inverseKinematicsResultArgs.m_dofCount; i++)
		{
			jointPositions[i] = status->m_inverseKinematicsResultArgs.m_jointPositions[i];
		}
	}

	return true;
}

b3SharedMemoryCommandHandle	b3RequestVREventsCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);

	command->m_type = CMD_REQUEST_VR_EVENTS_DATA;
	command->m_updateFlags = VR_DEVICE_CONTROLLER;
	return (b3SharedMemoryCommandHandle)command;
}

void b3VREventsSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
	if (command->m_type == CMD_REQUEST_VR_EVENTS_DATA)
	{
		command->m_updateFlags = deviceTypeFilter;
	}
}

void b3GetVREventsData(b3PhysicsClientHandle physClient, struct b3VREventsData* vrEventsData)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	if (cl)
	{
		cl->getCachedVREvents(vrEventsData);
	}
}

b3SharedMemoryCommandHandle	b3SetVRCameraStateCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);

	command->m_type = CMD_SET_VR_CAMERA_STATE;
	command->m_updateFlags = 0;

	return (b3SharedMemoryCommandHandle)command;

}

int b3SetVRCameraRootPosition(b3SharedMemoryCommandHandle commandHandle, double rootPos[3])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_SET_VR_CAMERA_STATE);
    command->m_updateFlags |= VR_CAMERA_ROOT_POSITION;
	command->m_vrCameraStateArguments.m_rootPosition[0] = rootPos[0];
	command->m_vrCameraStateArguments.m_rootPosition[1] = rootPos[1];
	command->m_vrCameraStateArguments.m_rootPosition[2] = rootPos[2];
	return 0;

}

int b3SetVRCameraRootOrientation(b3SharedMemoryCommandHandle commandHandle, double rootOrn[4])
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_SET_VR_CAMERA_STATE);
    command->m_updateFlags |= VR_CAMERA_ROOT_ORIENTATION;
	command->m_vrCameraStateArguments.m_rootOrientation[0] = rootOrn[0];
	command->m_vrCameraStateArguments.m_rootOrientation[1] = rootOrn[1];
	command->m_vrCameraStateArguments.m_rootOrientation[2] = rootOrn[2];
	command->m_vrCameraStateArguments.m_rootOrientation[3] = rootOrn[3];

	return 0;
}

int b3SetVRCameraTrackingObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_SET_VR_CAMERA_STATE);
    command->m_updateFlags |= VR_CAMERA_ROOT_TRACKING_OBJECT;
	command->m_vrCameraStateArguments.m_trackingObjectUniqueId = objectUniqueId;
	return 0;
}

int b3SetVRCameraTrackingObjectFlag(b3SharedMemoryCommandHandle commandHandle, int flag)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_SET_VR_CAMERA_STATE);
    command->m_updateFlags |= VR_CAMERA_FLAG;
	command->m_vrCameraStateArguments.m_trackingObjectFlag = flag;
	return 0;
}


b3SharedMemoryCommandHandle	b3RequestKeyboardEventsCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);

	command->m_type = CMD_REQUEST_KEYBOARD_EVENTS_DATA;
	command->m_updateFlags = 0;

	return (b3SharedMemoryCommandHandle)command;
}

void b3GetKeyboardEventsData(b3PhysicsClientHandle physClient, struct b3KeyboardEventsData* keyboardEventsData)
{
	PhysicsClient* cl = (PhysicsClient* ) physClient;
	if (cl)
	{
		cl->getCachedKeyboardEvents(keyboardEventsData);
	}
}


b3SharedMemoryCommandHandle	b3ProfileTimingCommandInit(b3PhysicsClientHandle physClient, const char* name)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);

	int len = strlen(name);
	if (len>=0 && len < (MAX_FILENAME_LENGTH+1))
	{
		command->m_type = CMD_PROFILE_TIMING;
		strcpy(command->m_profile.m_name,name);
		command->m_profile.m_name[len]=0;
	} else
	{
		const char* invalid = "InvalidProfileTimingName";
		int len = strlen(invalid);
		strcpy(command->m_profile.m_name,invalid);
		command->m_profile.m_name[len] = 0;
	}
	command->m_profile.m_durationInMicroSeconds = 0;
	return (b3SharedMemoryCommandHandle)command;
}

void b3SetProfileTimingDuractionInMicroSeconds(b3SharedMemoryCommandHandle commandHandle, int duration)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_PROFILE_TIMING);
	if (command->m_type == CMD_PROFILE_TIMING)
	{
		command->m_profile.m_durationInMicroSeconds  = duration;
	}
}

b3SharedMemoryCommandHandle	b3StateLoggingCommandInit(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	b3Assert(cl->canSubmitCommand());
	struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
	b3Assert(command);

	command->m_type = CMD_STATE_LOGGING;
	command->m_updateFlags = 0;
	command->m_stateLoggingArguments.m_numBodyUniqueIds = 0;
	command->m_stateLoggingArguments.m_deviceFilterType = VR_DEVICE_CONTROLLER;

	return (b3SharedMemoryCommandHandle)command;

}

int b3StateLoggingStart(b3SharedMemoryCommandHandle commandHandle, int loggingType, const char* fileName)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
	    command->m_updateFlags |= STATE_LOGGING_START_LOG;
		int len = strlen(fileName);
		if (len < MAX_FILENAME_LENGTH)
		{
			strcpy(command->m_stateLoggingArguments.m_fileName, fileName);
		}
		else
		{
			command->m_stateLoggingArguments.m_fileName[0] = 0;
		}
		command->m_stateLoggingArguments.m_logType = loggingType;
	}	
	return 0;
}

int b3GetStatusLoggingUniqueId(b3SharedMemoryStatusHandle statusHandle)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
	b3Assert(status);
	b3Assert(status->m_type == CMD_STATE_LOGGING_START_COMPLETED);
	if (status && status->m_type == CMD_STATE_LOGGING_START_COMPLETED)
	{
		return status->m_stateLoggingResultArgs.m_loggingUniqueId;
	}
	return -1;
}

int b3StateLoggingAddLoggingObjectUniqueId(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
	    command->m_updateFlags |= STATE_LOGGING_FILTER_OBJECT_UNIQUE_ID;
		if (command->m_stateLoggingArguments.m_numBodyUniqueIds < MAX_SDF_BODIES)
		{
			command->m_stateLoggingArguments.m_bodyUniqueIds[command->m_stateLoggingArguments.m_numBodyUniqueIds++] = objectUniqueId;
		}
	}
	return 0;
}

int b3StateLoggingSetLinkIndexA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type = CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
		command->m_updateFlags |= STATE_LOGGING_FILTER_LINK_INDEX_A;
		command->m_stateLoggingArguments.m_linkIndexA = linkIndexA;
	}
	return 0;
}

int b3StateLoggingSetLinkIndexB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type = CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
		command->m_updateFlags |= STATE_LOGGING_FILTER_LINK_INDEX_B;
		command->m_stateLoggingArguments.m_linkIndexB = linkIndexB;
	}
	return 0;
}

int b3StateLoggingSetBodyAUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyAUniqueId)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type = CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
		command->m_updateFlags |= STATE_LOGGING_FILTER_BODY_UNIQUE_ID_A;
		command->m_stateLoggingArguments.m_bodyUniqueIdA = bodyAUniqueId;
	}
	return 0;
}

int b3StateLoggingSetBodyBUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyBUniqueId)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
	b3Assert(command);
	b3Assert(command->m_type = CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
		command->m_updateFlags |= STATE_LOGGING_FILTER_BODY_UNIQUE_ID_B;
		command->m_stateLoggingArguments.m_bodyUniqueIdB = bodyBUniqueId;
	}
	return 0;
}

int b3StateLoggingSetMaxLogDof(b3SharedMemoryCommandHandle commandHandle, int maxLogDof)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
	    command->m_updateFlags |= STATE_LOGGING_MAX_LOG_DOF;
		command->m_stateLoggingArguments.m_maxLogDof = maxLogDof;
	}
	return 0;
}

int b3StateLoggingSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
	    command->m_updateFlags |= STATE_LOGGING_FILTER_DEVICE_TYPE;
		command->m_stateLoggingArguments.m_deviceFilterType = deviceTypeFilter;
	}
	return 0;
}

int b3StateLoggingStop(b3SharedMemoryCommandHandle commandHandle, int loggingUid)
{
	struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_STATE_LOGGING);
	if (command->m_type == CMD_STATE_LOGGING)
	{
	    command->m_updateFlags |= STATE_LOGGING_STOP_LOG;
		command->m_stateLoggingArguments.m_loggingUniqueId = loggingUid;
	}
	return 0;
}


///configure the 3D OpenGL debug visualizer (enable/disable GUI widgets, shadows, position camera etc)
b3SharedMemoryCommandHandle b3InitConfigureOpenGLVisualizer(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient*)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_CONFIGURE_OPENGL_VISUALIZER;
    command->m_updateFlags = 0;
    
    return (b3SharedMemoryCommandHandle)command;
}

void b3ConfigureOpenGLVisualizerSetVisualizationFlags(b3SharedMemoryCommandHandle commandHandle, int flag, int enabled)
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CONFIGURE_OPENGL_VISUALIZER);
    if (command->m_type == CMD_CONFIGURE_OPENGL_VISUALIZER)
    {
        command->m_updateFlags |= COV_SET_FLAGS;
        command->m_configureOpenGLVisualizerArguments.m_setFlag = flag;
        command->m_configureOpenGLVisualizerArguments.m_setEnabled = enabled;
    }
}

void b3ConfigureOpenGLVisualizerSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, float cameraDistance, float cameraPitch, float cameraYaw, const float cameraTargetPosition[3])
{
    struct SharedMemoryCommand* command = (struct SharedMemoryCommand*) commandHandle;
    b3Assert(command);
    b3Assert(command->m_type == CMD_CONFIGURE_OPENGL_VISUALIZER);
    
    if (command->m_type == CMD_CONFIGURE_OPENGL_VISUALIZER)
    {
        command->m_updateFlags |= COV_SET_CAMERA_VIEW_MATRIX;
        command->m_configureOpenGLVisualizerArguments.m_cameraDistance = cameraDistance;
        command->m_configureOpenGLVisualizerArguments.m_cameraPitch = cameraPitch;
        command->m_configureOpenGLVisualizerArguments.m_cameraYaw = cameraYaw;
        command->m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[0] = cameraTargetPosition[0];
        command->m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[1] = cameraTargetPosition[1];
        command->m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[2] = cameraTargetPosition[2];
    }
}

b3SharedMemoryCommandHandle b3InitRequestOpenGLVisualizerCameraCommand(b3PhysicsClientHandle physClient)
{
    PhysicsClient* cl = (PhysicsClient*)physClient;
    b3Assert(cl);
    b3Assert(cl->canSubmitCommand());
    struct SharedMemoryCommand* command = cl->getAvailableSharedMemoryCommand();
    b3Assert(command);
    
    command->m_type = CMD_REQUEST_OPENGL_VISUALIZER_CAMERA;
    command->m_updateFlags = 0;
    
    return (b3SharedMemoryCommandHandle)command;

}

int b3GetStatusOpenGLVisualizerCamera(b3SharedMemoryStatusHandle statusHandle, b3OpenGLVisualizerCameraInfo* camera)
{
	const SharedMemoryStatus* status = (const SharedMemoryStatus* ) statusHandle;
    //b3Assert(status);
    if (status && status->m_type == CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED)
    {
		*camera = status->m_visualizerCameraResultArgs;
		return 1;
	}
	return 0;
}

void b3SetTimeOut(b3PhysicsClientHandle physClient, double timeOutInSeconds)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	if (cl)
	{
		cl->setTimeOut(timeOutInSeconds);
	}
		
}

double b3GetTimeOut(b3PhysicsClientHandle physClient)
{
	PhysicsClient* cl = (PhysicsClient*)physClient;
	b3Assert(cl);
	if (cl)
	{
		return cl->getTimeOut();
	}
	return -1;
}

void b3MultiplyTransforms(const double posA[3], const double ornA[4], const double posB[3], const double ornB[4], double outPos[3], double outOrn[4])
{
	b3Transform trA;
	b3Transform trB;
	trA.setOrigin(b3MakeVector3(posA[0],posA[1],posA[2]));
	trA.setRotation(b3Quaternion(ornA[0],ornA[1],ornA[2],ornA[3]));
	trB.setOrigin(b3MakeVector3(posB[0],posB[1],posB[2]));
	trB.setRotation(b3Quaternion(ornB[0],ornB[1],ornB[2],ornB[3]));
	b3Transform res = trA*trB;
	outPos[0] = res.getOrigin()[0];
	outPos[1] = res.getOrigin()[1];
	outPos[2] = res.getOrigin()[2];
	b3Quaternion orn = res.getRotation();
	outOrn[0] = orn[0];
	outOrn[1] = orn[1];
	outOrn[2] = orn[2];
	outOrn[3] = orn[3];
}

void b3InvertTransform(const double pos[3], const double orn[4], double outPos[3], double outOrn[4])
{
	b3Transform tr;
	tr.setOrigin(b3MakeVector3(pos[0],pos[1],pos[2]));
	tr.setRotation(b3Quaternion(orn[0],orn[1],orn[2],orn[3]));
	b3Transform trInv = tr.inverse();
	outPos[0] = trInv.getOrigin()[0];
	outPos[1] = trInv.getOrigin()[1];
	outPos[2] = trInv.getOrigin()[2];
	b3Quaternion invOrn = trInv.getRotation();
	outOrn[0] = invOrn[0];
	outOrn[1] = invOrn[1];
	outOrn[2] = invOrn[2];
	outOrn[3] = invOrn[3];
}
