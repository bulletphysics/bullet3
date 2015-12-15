//#include "SharedMemoryCommands.h"
#include "SharedMemory/PhysicsClientC_API.h"

#ifdef PHYSICS_LOOP_BACK
#include "SharedMemory/PhysicsLoopBackC_API.h"
#endif //PHYSICS_LOOP_BACK

#ifdef PHYSICS_SERVER_DIRECT
#include "SharedMemory/PhysicsDirectC_API.h"
#endif //PHYSICS_SERVER_DIRECT


#include "SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"
#include <string.h>


#include <stdio.h>

int main(int argc, char* argv[])
{
	int i, dofCount , posVarCount, ret ,numJoints ;
    int sensorJointIndexLeft=-1;
    int sensorJointIndexRight=-1;
	const char* urdfFileName = "r2d2.urdf";
	double gravx=0, gravy=0, gravz=-9.8;
	double timeStep = 1./60.;
	double startPosX, startPosY,startPosZ;
	int imuLinkIndex = -1;

	
	b3PhysicsClientHandle sm=0;
	int bodyIndex = -1;


	printf("hello world\n");
#ifdef PHYSICS_LOOP_BACK
	sm = b3ConnectPhysicsLoopback(SHARED_MEMORY_KEY);
#endif

#ifdef PHYSICS_SERVER_DIRECT
	sm = b3ConnectPhysicsDirect();
#else//PHYSICS_SERVER_DIRECT
	sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
#endif //PHYSICS_SERVER_DIRECT
	
	

	if (b3CanSubmitCommand(sm))
	{
        {
        b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		ret = b3PhysicsParamSetGravity(command,  gravx,gravy, gravz);
		ret = b3PhysicsParamSetTimeStep(command,  timeStep);
		b3SubmitClientCommandAndWaitStatus(sm, command);
        }

		
        {
            b3SharedMemoryStatusHandle statusHandle;
			b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, urdfFileName);
			
            //setting the initial position, orientation and other arguments are optional
            startPosX =2;
            startPosY =3;
            startPosZ = 1;
            ret = b3LoadUrdfCommandSetStartPosition(command, startPosX,startPosY,startPosZ);
            statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			bodyIndex = b3GetStatusBodyIndex(statusHandle);
        }
        
		if (bodyIndex>=0)
		{
			numJoints = b3GetNumJoints(sm,bodyIndex);
			for (i=0;i<numJoints;i++)
			{
				struct b3JointInfo jointInfo;
				b3GetJointInfo(sm,bodyIndex, i,&jointInfo);
            
				printf("jointInfo[%d].m_jointName=%s\n",i,jointInfo.m_jointName);
				//pick the IMU link index based on torso name
				if (strstr(jointInfo.m_linkName,"base_link"))
				{
					imuLinkIndex = i;
				}
            
				//pick the joint index based on joint name
				if (strstr(jointInfo.m_jointName,"base_to_left_leg"))
				{
					sensorJointIndexLeft = i;
				}
				if (strstr(jointInfo.m_jointName,"base_to_right_leg"))
				{
					sensorJointIndexRight = i;
				}
            
			}
        
			if ((sensorJointIndexLeft>=0) || (sensorJointIndexRight>=0))
			{
				b3SharedMemoryCommandHandle command = b3CreateSensorCommandInit(sm);
				b3SharedMemoryStatusHandle statusHandle;
				if (imuLinkIndex>=0)
				{
					 ret = b3CreateSensorEnableIMUForLink(command, imuLinkIndex, 1);
				}
            
				if (sensorJointIndexLeft>=0)
				{
				  ret = b3CreateSensorEnable6DofJointForceTorqueSensor(command, sensorJointIndexLeft, 1);
				}
				if(sensorJointIndexRight>=0)
				{
					ret = b3CreateSensorEnable6DofJointForceTorqueSensor(command, sensorJointIndexRight, 1);
				}
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
            
			}
		}
        
        {
            b3SharedMemoryStatusHandle statusHandle;
             b3SharedMemoryCommandHandle command = b3CreateBoxShapeCommandInit(sm);
            ret = b3CreateBoxCommandSetStartPosition(command, 0,0,-1);
            ret = b3CreateBoxCommandSetStartOrientation(command,0,0,0,1);
            ret = b3CreateBoxCommandSetHalfExtents(command, 10,10,1);
            statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

        }

        {
        		int statusType;
            b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(sm,bodyIndex);
            b3SharedMemoryStatusHandle statusHandle;
            statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
            statusType = b3GetStatusType(statusHandle);
            
            if (statusType == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
            {
                b3GetStatusActualState(statusHandle,
                                       0, &posVarCount, &dofCount,
                                       0, 0, 0, 0);

                b3Printf("posVarCount = %d\n",posVarCount);
                printf("dofCount = %d\n",dofCount);
            }
        }
        
        {
#if 0
            b3SharedMemoryStatusHandle statusHandle;
             b3SharedMemoryCommandHandle command = b3JointControlCommandInit( sm, CONTROL_MODE_VELOCITY);
            for ( dofIndex=0;dofIndex<dofCount;dofIndex++)
            {
                b3JointControlSetDesiredVelocity(command,dofIndex,1);
                b3JointControlSetMaximumForce(command,dofIndex,100);
            }
             statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
#endif
        }
        ///perform some simulation steps for testing
        for ( i=0;i<100;i++)
        {
            b3SubmitClientCommandAndWaitStatus(sm, b3InitStepSimulationCommand(sm));
        }
        
        {
            b3SharedMemoryStatusHandle state = b3SubmitClientCommandAndWaitStatus(sm, b3RequestActualStateCommandInit(sm,bodyIndex));
        
			if (sensorJointIndexLeft>=0)
			{

				struct  b3JointSensorState sensorState;
				b3GetJointState(sm,state,sensorJointIndexLeft,&sensorState);
				
				b3Printf("Sensor for joint [%d] = %f,%f,%f\n", sensorJointIndexLeft,
					sensorState.m_jointForceTorque[0],
					sensorState.m_jointForceTorque[1],
					sensorState.m_jointForceTorque[2]);

			}
        
			if (sensorJointIndexRight>=0)
			{
				struct  b3JointSensorState sensorState;
				b3GetJointState(sm,state,sensorJointIndexRight,&sensorState);
				
				b3Printf("Sensor for joint [%d] = %f,%f,%f\n", sensorJointIndexRight,
						 sensorState.m_jointForceTorque[0],
						 sensorState.m_jointForceTorque[1],
						 sensorState.m_jointForceTorque[2]);

			}
		}
        

        {
            b3SubmitClientCommandAndWaitStatus(sm, b3InitResetSimulationCommand(sm));
        }
        
	}


	b3DisconnectSharedMemory(sm);
}
