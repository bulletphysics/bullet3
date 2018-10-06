#include "MinitaurSetup.h"
#include "b3RobotSimulatorClientAPI_NoGUI.h"

#include "Bullet3Common/b3HashMap.h"

struct MinitaurSetupInternalData
{
	int m_quadrupedUniqueId;

	MinitaurSetupInternalData()
		: m_quadrupedUniqueId(-1)
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

void MinitaurSetup::setDesiredMotorAngle(class b3RobotSimulatorClientAPI_NoGUI* sim, const char* motorName, double desiredAngle, double maxTorque, double kp, double kd)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = maxTorque;
	controlArgs.m_kd = kd;
	controlArgs.m_kp = kp;
	controlArgs.m_targetPosition = desiredAngle;
	sim->setJointMotorControl(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorName], controlArgs);
}

//pick exactly 1 configuration of the following
#define MINITAUR_RAINBOWDASH_V1
//#define MINITAUR_RAINBOWDASH_V0
//#define MINITAUR_V0

#if defined(MINITAUR_RAINBOWDASH_V1)
#define MINITAUR_HAS_DEFORMABLE_BRACKETS
static const char* minitaurURDF = "quadruped/minitaur_rainbow_dash_v1.urdf";

static const char* kneeNames[] = {
	"knee_front_leftL_joint",   //1
	"knee_front_leftR_joint",   //3
	"knee_back_leftL_joint",    //5
	"knee_back_leftR_joint",    //7
	"knee_front_rightL_joint",  //9
	"knee_back_rightL_joint",   //10
	"knee_back_rightR_joint",   //13
	"knee_front_rightR_joint",  //15
};

static const char* motorNames[] = {
	"motor_front_leftL_joint",   //0
	"knee_front_leftL_joint",    //1
	"motor_front_leftR_joint",   //2
	"knee_front_leftR_joint",    //3
	"motor_back_leftL_joint",    //4
	"knee_back_leftL_joint",     //5
	"motor_back_leftR_joint",    //6
	"knee_back_leftR_joint",     //7
	"motor_front_rightL_joint",  //8
	"knee_front_rightL_joint",   //9
	"knee_back_rightL_joint",    //10
	"motor_back_rightL_joint",   //11
	"motor_back_rightR_joint",   //12
	"knee_back_rightR_joint",    //13
	"motor_front_rightR_joint",  //14
	"knee_front_rightR_joint",   //15
};

static const char* bracketNames[] = {
	"motor_front_rightR_bracket_joint",
	"motor_front_leftL_bracket_joint",
	"motor_back_rightR_bracket_joint",
	"motor_back_leftL_bracket_joint",
};

static btVector3 KNEE_CONSTRAINT_POINT_LONG = btVector3(0, 0.0045, 0.088);
static btVector3 KNEE_CONSTRAINT_POINT_SHORT = btVector3(0, 0.0045, 0.100);
#elif defined(MINITAUR_RAINBOWDASH_V0)
static const char* minitaurURDF = "quadruped/minitaur_rainbow_dash.urdf";

static const char* kneeNames[] = {
	"knee_front_leftL_joint",   //1
	"knee_front_leftR_joint",   //3
	"knee_back_leftL_joint",    //5
	"knee_back_leftR_joint",    //7
	"knee_front_rightL_joint",  //9
	"knee_back_rightL_joint",   //10
	"knee_back_rightR_joint",   //13
	"knee_front_rightR_joint",  //15
};

static const char* motorNames[] = {
	"motor_front_leftL_joint",   //0
	"knee_front_leftL_joint",    //1
	"motor_front_leftR_joint",   //2
	"knee_front_leftR_joint",    //3
	"motor_back_leftL_joint",    //4
	"knee_back_leftL_joint",     //5
	"motor_back_leftR_joint",    //6
	"knee_back_leftR_joint",     //7
	"motor_front_rightL_joint",  //8
	"knee_front_rightL_joint",   //9
	"knee_back_rightL_joint",    //10
	"motor_back_rightL_joint",   //11
	"motor_back_rightR_joint",   //12
	"knee_back_rightR_joint",    //13
	"motor_front_rightR_joint",  //14
	"knee_front_rightR_joint",   //15
};
static btVector3 KNEE_CONSTRAINT_POINT_LONG = btVector3(0, 0.0045, 0.088);
static btVector3 KNEE_CONSTRAINT_POINT_SHORT = btVector3(0, 0.0045, 0.100);
#elif defined(MINITAUR_V0)
static const char* minitaurURDF = "quadruped/minitaur.urdf";

static const char* kneeNames[] = {
	"knee_front_leftL_link",
	"knee_front_leftR_link",
	"knee_back_leftL_link",
	"knee_back_leftR_link",
	"knee_front_rightL_link",
	"knee_back_rightL_link",
	"knee_back_rightR_link",
	"knee_front_rightR_link",
};

static const char* motorNames[] = {
	"motor_front_leftL_joint",
	"knee_front_leftL_link",
	"motor_front_leftR_joint",
	"knee_front_leftR_link",
	"motor_back_leftL_joint",
	"knee_back_leftL_link",
	"motor_back_leftR_joint",
	"knee_back_leftR_link",
	"motor_front_rightL_joint",
	"knee_front_rightL_link",
	"knee_back_rightL_link",
	"motor_back_rightL_joint",
	"motor_back_rightR_joint",
	"knee_back_rightR_link",
	"motor_front_rightR_joint",
	"knee_front_rightR_link",
};
static btVector3 KNEE_CONSTRAINT_POINT_LONG = btVector3(0, 0.005, 0.2);
static btVector3 KNEE_CONSTRAINT_POINT_SHORT = btVector3(0, 0.01, 0.2);
#endif

void MinitaurSetup::resetPose(class b3RobotSimulatorClientAPI_NoGUI* sim)
{
	//release all motors
	int numJoints = sim->getNumJoints(m_data->m_quadrupedUniqueId);
	for (int i = 0; i < numJoints; i++)
	{
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		controlArgs.m_maxTorqueValue = 0;
		sim->setJointMotorControl(m_data->m_quadrupedUniqueId, i, controlArgs);
	}

	b3Scalar startAngle = B3_HALF_PI;
	b3Scalar upperLegLength = 11.5;
	b3Scalar lowerLegLength = 20;
	b3Scalar kneeAngle = B3_PI + b3Acos(upperLegLength / lowerLegLength);

	b3Scalar motorDirs[8] = {-1, -1, -1, -1, 1, 1, 1, 1};
	b3JointInfo jointInfo;
	jointInfo.m_jointType = ePoint2PointType;
	//left front leg
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[0]], motorDirs[0] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[0]], motorDirs[0] * kneeAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[2]], motorDirs[1] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[1]], motorDirs[1] * kneeAngle);

	jointInfo.m_parentFrame[0] = KNEE_CONSTRAINT_POINT_SHORT[0];
	jointInfo.m_parentFrame[1] = KNEE_CONSTRAINT_POINT_SHORT[1];
	jointInfo.m_parentFrame[2] = KNEE_CONSTRAINT_POINT_SHORT[2];
	jointInfo.m_childFrame[0] = KNEE_CONSTRAINT_POINT_LONG[0];
	jointInfo.m_childFrame[1] = KNEE_CONSTRAINT_POINT_LONG[1];
	jointInfo.m_childFrame[2] = KNEE_CONSTRAINT_POINT_LONG[2];

	//jointInfo.m_parentFrame[0] = KNEE_CONSTRAINT_POINT_LONG[0];	jointInfo.m_parentFrame[1] = KNEE_CONSTRAINT_POINT_LONG[1];	jointInfo.m_parentFrame[2] = KNEE_CONSTRAINT_POINT_LONG[2];
	//jointInfo.m_childFrame[0] = KNEE_CONSTRAINT_POINT_SHORT[0];	jointInfo.m_childFrame[1] = KNEE_CONSTRAINT_POINT_SHORT[1];	jointInfo.m_childFrame[2] = KNEE_CONSTRAINT_POINT_SHORT[2];
	sim->createConstraint(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[1]],
						  m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[0]], &jointInfo);
	setDesiredMotorAngle(sim, motorNames[0], motorDirs[0] * startAngle);
	setDesiredMotorAngle(sim, motorNames[2], motorDirs[1] * startAngle);

	//left back leg
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[4]], motorDirs[2] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[2]], motorDirs[2] * kneeAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[6]], motorDirs[3] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[3]], motorDirs[3] * kneeAngle);
	jointInfo.m_parentFrame[0] = KNEE_CONSTRAINT_POINT_SHORT[0];
	jointInfo.m_parentFrame[1] = KNEE_CONSTRAINT_POINT_SHORT[1];
	jointInfo.m_parentFrame[2] = KNEE_CONSTRAINT_POINT_SHORT[2];
	jointInfo.m_childFrame[0] = KNEE_CONSTRAINT_POINT_LONG[0];
	jointInfo.m_childFrame[1] = KNEE_CONSTRAINT_POINT_LONG[1];
	jointInfo.m_childFrame[2] = KNEE_CONSTRAINT_POINT_LONG[2];

	//jointInfo.m_parentFrame[0] = KNEE_CONSTRAINT_POINT_LONG[0];	jointInfo.m_parentFrame[1] = KNEE_CONSTRAINT_POINT_LONG[1];	jointInfo.m_parentFrame[2] = KNEE_CONSTRAINT_POINT_LONG[2];
	//jointInfo.m_childFrame[0] = KNEE_CONSTRAINT_POINT_SHORT[0];	jointInfo.m_childFrame[1] = KNEE_CONSTRAINT_POINT_SHORT[1];	jointInfo.m_childFrame[2] = KNEE_CONSTRAINT_POINT_SHORT[2];
	sim->createConstraint(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[3]],
						  m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[2]], &jointInfo);
	setDesiredMotorAngle(sim, motorNames[4], motorDirs[2] * startAngle);
	setDesiredMotorAngle(sim, motorNames[6], motorDirs[3] * startAngle);

	//right front leg
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[8]], motorDirs[4] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[4]], motorDirs[4] * kneeAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[14]], motorDirs[5] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[7]], motorDirs[5] * kneeAngle);

	jointInfo.m_parentFrame[0] = KNEE_CONSTRAINT_POINT_LONG[0];
	jointInfo.m_parentFrame[1] = KNEE_CONSTRAINT_POINT_LONG[1];
	jointInfo.m_parentFrame[2] = KNEE_CONSTRAINT_POINT_LONG[2];
	jointInfo.m_childFrame[0] = KNEE_CONSTRAINT_POINT_SHORT[0];
	jointInfo.m_childFrame[1] = KNEE_CONSTRAINT_POINT_SHORT[1];
	jointInfo.m_childFrame[2] = KNEE_CONSTRAINT_POINT_SHORT[2];
	sim->createConstraint(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[7]],
						  m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[4]], &jointInfo);
	setDesiredMotorAngle(sim, motorNames[8], motorDirs[4] * startAngle);
	setDesiredMotorAngle(sim, motorNames[14], motorDirs[5] * startAngle);

	//right back leg
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[11]], motorDirs[6] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[5]], motorDirs[6] * kneeAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[motorNames[12]], motorDirs[7] * startAngle);
	sim->resetJointState(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[6]], motorDirs[7] * kneeAngle);

	jointInfo.m_parentFrame[0] = KNEE_CONSTRAINT_POINT_LONG[0];
	jointInfo.m_parentFrame[1] = KNEE_CONSTRAINT_POINT_LONG[1];
	jointInfo.m_parentFrame[2] = KNEE_CONSTRAINT_POINT_LONG[2];
	jointInfo.m_childFrame[0] = KNEE_CONSTRAINT_POINT_SHORT[0];
	jointInfo.m_childFrame[1] = KNEE_CONSTRAINT_POINT_SHORT[1];
	jointInfo.m_childFrame[2] = KNEE_CONSTRAINT_POINT_SHORT[2];
	sim->createConstraint(m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[6]],
						  m_data->m_quadrupedUniqueId, *m_data->m_jointNameToId[kneeNames[5]], &jointInfo);
	setDesiredMotorAngle(sim, motorNames[11], motorDirs[6] * startAngle);
	setDesiredMotorAngle(sim, motorNames[12], motorDirs[7] * startAngle);

#ifdef MINITAUR_HAS_DEFORMABLE_BRACKETS
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
	controlArgs.m_maxTorqueValue = 6;
	controlArgs.m_kd = 1;
	controlArgs.m_kp = 0;
	controlArgs.m_targetPosition = 0;
	for (int i = 0; i < 4; i++)
	{
		const char* bracketName = bracketNames[i];
		int* bracketId = m_data->m_jointNameToId[bracketName];
		sim->setJointMotorControl(m_data->m_quadrupedUniqueId, *bracketId, controlArgs);
	}

#endif
}

int MinitaurSetup::setupMinitaur(class b3RobotSimulatorClientAPI_NoGUI* sim, const btVector3& startPos, const btQuaternion& startOrn)
{
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition = startPos;
	args.m_startOrientation = startOrn;

	m_data->m_quadrupedUniqueId = sim->loadURDF(minitaurURDF, args);

	int numJoints = sim->getNumJoints(m_data->m_quadrupedUniqueId);
	for (int i = 0; i < numJoints; i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(m_data->m_quadrupedUniqueId, i, &jointInfo);
		if (jointInfo.m_jointName[0])
		{
			m_data->m_jointNameToId.insert(jointInfo.m_jointName, i);
		}
	}

	resetPose(sim);

	return m_data->m_quadrupedUniqueId;
}
