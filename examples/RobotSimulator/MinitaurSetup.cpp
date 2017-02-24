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

void MinitaurSetup::resetPose()
{
#if 0
	 def resetPose(self):
    #right front leg
    self.disableAllMotors()
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_front_rightR_link'],self.quadruped,self.jointNameToId['knee_front_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])
    self.setMotorAngleByName('motor_front_rightR_joint', 1.57)
    self.setMotorAngleByName('motor_front_rightL_joint',-1.57)

    #left front leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_front_leftR_link'],self.quadruped,self.jointNameToId['knee_front_leftL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])
    self.setMotorAngleByName('motor_front_leftR_joint', 1.57)
    self.setMotorAngleByName('motor_front_leftL_joint',-1.57)

    #right back leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_back_rightR_link'],self.quadruped,self.jointNameToId['knee_back_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])
    self.setMotorAngleByName('motor_back_rightR_joint', 1.57)
    self.setMotorAngleByName('motor_back_rightL_joint',-1.57)

    #left back leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_back_leftR_link'],self.quadruped,self.jointNameToId['knee_back_leftL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])
    self.setMotorAngleByName('motor_back_leftR_joint', 1.57)
    self.setMotorAngleByName('motor_back_leftL_joint',-1.57)
#endif
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

	resetPose();

	return m_data->m_quadrupedUniqueId;
}