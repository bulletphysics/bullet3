#include "MinitaurSetup.h"
#include "b3RobotSimulatorClientAPI.h"

#include "Bullet3Common/b3HashMap.h"

struct MinitaurSetupInternalData
{
	int m_quadrupedUniqueId;

	MinitaurSetupInternalData()
		:m_quadrupedUniqueId(-1)
	{
	}

	b3HashMap<b3HashString, int> m_jointNameToId;

};


MinitaurSetup::MinitaurSetup()
{
	m_data = new MinitaurSetupInternalData();
}

MinitaurSetup::~MinitaurSetup()
{
	delete m_data;
}

void MinitaurSetup::setDesiredMotorAngle(class b3RobotSimulatorClientAPI* sim, const char* motorName, double desiredAngle, double maxTorque, double kp, double kd)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = maxTorque;
	controlArgs.m_kd = kd;
	controlArgs.m_kp = kp;
	controlArgs.m_targetPosition = desiredAngle;
	sim->setJointMotorControl(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId[motorName],controlArgs);
}


void MinitaurSetup::resetPose(class b3RobotSimulatorClientAPI* sim)
{
	//release all motors
	int numJoints = sim->getNumJoints(m_data->m_quadrupedUniqueId);
	for (int i=0;i<numJoints;i++)
	{
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		controlArgs.m_maxTorqueValue = 0;
		sim->setJointMotorControl(m_data->m_quadrupedUniqueId,i,controlArgs);
	}

	b3Scalar startAngle = B3_HALF_PI;
	b3Scalar upperLegLength = 11.5;
	b3Scalar lowerLegLength = 20;
	b3Scalar kneeAngle = B3_PI+b3Acos(upperLegLength/lowerLegLength);
		
	b3Scalar motorDirs[8] = {-1,-1,-1,-1,1,1,1,1};
	b3JointInfo jointInfo;
	jointInfo.m_jointType = ePoint2PointType;
		//left front leg
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_leftL_joint"],motorDirs[0] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftL_link"],motorDirs[0]*kneeAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_leftR_joint"],motorDirs[1] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftR_link"],motorDirs[1]*kneeAngle);
	
	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = 0.005;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = 0.01;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_front_leftL_joint", motorDirs[0] * startAngle);
	setDesiredMotorAngle(sim,"motor_front_leftR_joint", motorDirs[1] * startAngle);
	
	//left back leg
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_leftL_joint"],motorDirs[2] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftL_link"],motorDirs[2] * kneeAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_leftR_joint"],motorDirs[3] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftR_link"],motorDirs[3] * kneeAngle);
	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = 0.005;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = 0.01;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_back_leftL_joint", motorDirs[2] * startAngle);
	setDesiredMotorAngle(sim,"motor_back_leftR_joint", motorDirs[3] * startAngle);


	//right front leg
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_rightL_joint"],motorDirs[4] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightL_link"],motorDirs[4] * kneeAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_rightR_joint"],motorDirs[5]*startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightR_link"],motorDirs[5] * kneeAngle);

	
	
	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = 0.005;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = 0.01;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_front_rightL_joint",motorDirs[4] * startAngle);
	setDesiredMotorAngle(sim,"motor_front_rightR_joint",motorDirs[5] * startAngle);


	//right back leg
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_rightL_joint"],motorDirs[6] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightL_link"],motorDirs[6] * kneeAngle);
  	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_rightR_joint"],motorDirs[7] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightR_link"],motorDirs[7] * kneeAngle);

	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = 0.005;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = 0.01;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_back_rightL_joint", motorDirs[6] * startAngle);
	setDesiredMotorAngle(sim,"motor_back_rightR_joint", motorDirs[7] * startAngle);

}

int MinitaurSetup::setupMinitaur(class b3RobotSimulatorClientAPI* sim, const b3Vector3& startPos, const b3Quaternion& startOrn)
{
	
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition = startPos;
	args.m_startOrientation = startOrn;

	m_data->m_quadrupedUniqueId = sim->loadURDF("quadruped/minitaur.urdf",args);

	int numJoints = sim->getNumJoints(m_data->m_quadrupedUniqueId);
	for (int i=0;i<numJoints;i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(m_data->m_quadrupedUniqueId,i,&jointInfo);
		if (jointInfo.m_jointName)
		{
			m_data->m_jointNameToId.insert(jointInfo.m_jointName,i);
		}
	}

	resetPose(sim);

	return m_data->m_quadrupedUniqueId;
}