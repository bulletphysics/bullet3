#include "SharedMemoryBlock.h"
#include "PhysicsClientC_API.h"
#include "SharedMemoryCommands.h"
#include "Bullet3Common/b3Logging.h"
#include <string.h>

struct test
{
	int unused;
};

#include <stdio.h>
#define MAX_TIMEOUT 1024*1024*1024

int main(int argc, char* argv[])
{
	int i, dofCount , posVarCount, dofIndex, ret ,numJoints, allowSharedMemoryInitialization=0;
	int timeout = MAX_TIMEOUT;
    int sensorJointIndexLeft=-1;
    int sensorJointIndexRight=-1;
	const char* urdfFileName = "r2d2.urdf";
	double gravx=0, gravy=0, gravz=-9.8;
	double timeStep = 1./60.;
	double startPosX, startPosY,startPosZ;
	SharedMemoryCommand_t command;
	SharedMemoryStatus_t status;
	b3PhysicsClientHandle sm;
	
	b3Printf("timeout = %d\n",timeout);
    

	printf("hello world\n");

	sm = b3ConnectSharedMemory( allowSharedMemoryInitialization);
	if (b3CanSubmitCommand(sm))
	{
		ret = b3InitPhysicsParamCommand(&command);
		ret = b3PhysicsParamSetGravity(&command,  gravx,gravy, gravz);
		ret = b3PhysicsParamSetTimeStep(&command,  timeStep);
		ret = b3SubmitClientCommand(sm, &command);
		timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
        b3Printf("timeout = %d\n",timeout);
        

		ret = b3LoadUrdfCommandInit(&command, urdfFileName);
		//setting the initial position, orientation and other arguments are optional
		startPosX =2;
		startPosY =3;
		startPosZ = 1;
		ret = b3LoadUrdfCommandSetStartPosition(&command, startPosX,startPosY,startPosZ);
		
		ret = b3SubmitClientCommand(sm, &command);
		timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
		b3Printf("timeout = %d\n",timeout);
        
		numJoints = b3GetNumJoints(sm);
        for (i=0;i<numJoints;i++)
        {
            struct b3JointInfo jointInfo;
            b3GetJointInfo(sm,i,&jointInfo);
            printf("jointInfo[%d].m_jointName=%s\n",i,jointInfo.m_jointName);
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
            ret = b3CreateSensorCommandInit(&command);
            if (sensorJointIndexLeft>=0)
            {
            ret = b3CreateSensorEnable6DofJointForceTorqueSensor(&command, sensorJointIndexLeft, 1);
            }
            if(sensorJointIndexRight>=0)
            {
                ret = b3CreateSensorEnable6DofJointForceTorqueSensor(&command, sensorJointIndexRight, 1);
            }
            ret = b3SubmitClientCommand(sm, &command);
            timeout = MAX_TIMEOUT;
            while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
        }
        
        ret = b3CreateBoxShapeCommandInit(&command);
        ret = b3CreateBoxCommandSetStartPosition(&command, 0,0,-1);
        ret = b3CreateBoxCommandSetStartOrientation(&command,0,0,0,1);
        ret = b3CreateBoxCommandSetHalfExtents(&command, 10,10,1);
        ret = b3SubmitClientCommand(sm, &command);
        timeout = MAX_TIMEOUT;
        while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
        
        
        b3RequestActualStateCommandInit(&command);
        ret = b3SubmitClientCommand(sm, &command);
        timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}

        posVarCount =status.m_sendActualStateArgs.m_numDegreeOfFreedomQ;
        dofCount =status.m_sendActualStateArgs.m_numDegreeOfFreedomU;
        
        b3Printf("posVarCount = %d\n",posVarCount);
        printf("dofCount = %d\n",dofCount);
        
        b3JointControlCommandInit(&command, CONTROL_MODE_VELOCITY);
        for ( dofIndex=0;dofIndex<dofCount;dofIndex++)
        {
            b3JointControlSetDesiredVelocity(&command,dofIndex,1);
            b3JointControlSetMaximumForce(&command,dofIndex,100);
        }
		ret = b3SubmitClientCommand(sm, &command);
		timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
        
        ///perform some simulation steps for testing
        for ( i=0;i<100;i++)
        {
            ret = b3InitStepSimulationCommand(&command);
            ret = b3SubmitClientCommand(sm, &command);
            timeout = MAX_TIMEOUT;
            while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
        }
        
        b3RequestActualStateCommandInit(&command);
        ret = b3SubmitClientCommand(sm, &command);
        timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
        
        if (sensorJointIndexLeft>=0)
        {
            b3Printf("Sensor for joint [%d] = %f,%f,%f\n", sensorJointIndexLeft,
                status.m_sendActualStateArgs.m_jointReactionForces[6*sensorJointIndexLeft+0],
                status.m_sendActualStateArgs.m_jointReactionForces[6*sensorJointIndexLeft+1],
                status.m_sendActualStateArgs.m_jointReactionForces[6*sensorJointIndexLeft+2]);
        }
        
        if (sensorJointIndexRight>=0)
        {
            b3Printf("Sensor for joint [%d] = %f,%f,%f\n", sensorJointIndexRight,
                     status.m_sendActualStateArgs.m_jointReactionForces[6*sensorJointIndexRight+0],
                     status.m_sendActualStateArgs.m_jointReactionForces[6*sensorJointIndexRight+1],
                     status.m_sendActualStateArgs.m_jointReactionForces[6*sensorJointIndexRight+2]);
        }
        
        
	}
	b3DisconnectSharedMemory(sm);
}
