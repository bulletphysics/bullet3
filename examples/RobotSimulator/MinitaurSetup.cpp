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

	//right front leg
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_rightR_joint"],1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightR_link"],-2.2);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_rightL_joint"],-1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightL_link"],2.2);
	b3JointInfo jointInfo;
	jointInfo.m_jointType = ePoint2PointType;
	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = 0.01;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = -0.015;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_rightL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_front_rightR_joint",1.57);
	setDesiredMotorAngle(sim,"motor_front_rightL_joint",-1.57);

	//left front leg
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_leftR_joint"],1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftR_link"],-2.2);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_front_leftL_joint"],-1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftL_link"],2.2);
	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = -0.01;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = 0.015;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_front_leftL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_front_leftR_joint", 1.57);
	setDesiredMotorAngle(sim,"motor_front_leftL_joint", -1.57);

	//right back leg
  	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_rightR_joint"],1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightR_link"],-2.2);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_rightL_joint"],-1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightL_link"],2.2);
	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = 0.01;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = -0.015;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_rightL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_back_rightR_joint", 1.57);
	setDesiredMotorAngle(sim,"motor_back_rightL_joint", -1.57);

	//left back leg
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_leftR_joint"],1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftR_link"],-2.2);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["motor_back_leftL_joint"],-1.57);
	sim->resetJointState(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftL_link"],2.2);
	jointInfo.m_parentFrame[0] = 0;	jointInfo.m_parentFrame[1] = -0.01;	jointInfo.m_parentFrame[2] = 0.2;
	jointInfo.m_childFrame[0] = 0;	jointInfo.m_childFrame[1] = 0.015;	jointInfo.m_childFrame[2] = 0.2;
	sim->createConstraint(m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftR_link"],
		m_data->m_quadrupedUniqueId,*m_data->m_jointNameToId["knee_back_leftL_link"],&jointInfo);
	setDesiredMotorAngle(sim,"motor_back_leftR_joint", 1.57);
	setDesiredMotorAngle(sim,"motor_back_leftL_joint", -1.57);


}

int MinitaurSetup::setupMinitaur(class b3RobotSimulatorClientAPI* sim, const b3Vector3& startPos, const b3Quaternion& startOrn)
{
	
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition = startPos;
	args.m_startOrientation = startOrn;

	m_data->m_quadrupedUniqueId = sim->loadURDF("quadruped/quadruped.urdf",args);

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