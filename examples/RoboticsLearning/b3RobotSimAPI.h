#ifndef B3_ROBOT_SIM_API_H
#define B3_ROBOT_SIM_API_H

///todo: remove those includes from this header
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include <string>


struct b3RobotSimLoadURDFArgs
{
	std::string m_urdfFileName;
	b3Vector3 m_startPosition;
	b3Quaternion m_startOrientation;
	bool m_forceOverrideFixedBase;


	b3RobotSimLoadURDFArgs(const std::string& urdfFileName)
		:m_urdfFileName(urdfFileName),
		m_startPosition(b3MakeVector3(0,0,0)),
		m_startOrientation(b3Quaternion(0,0,0,1)),
		m_forceOverrideFixedBase(false)
	{
	}
};

struct b3JointMotorArgs
{
	int m_controlMode;
	
	double m_targetPosition;
	double m_kp;

	double m_targetVelocity;
	double m_kd;

	double m_maxTorqueValue;

	b3JointMotorArgs(int controlMode)
		:m_controlMode(controlMode),
		m_targetPosition(0),
		m_kp(0.1),
		m_targetVelocity(0),
		m_kd(0.1),
		m_maxTorqueValue(1000)
	{
	}
};


class b3RobotSimAPI
{
	struct b3RobotSimAPI_InternalData* m_data;
	void processMultiThreadedGraphicsRequests();
	b3SharedMemoryStatusHandle  submitClientCommandAndWaitStatusMultiThreaded(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);
	
public:
	b3RobotSimAPI();
	virtual ~b3RobotSimAPI();

	bool connect(struct GUIHelperInterface* guiHelper);
	void disconnect();

	int loadURDF(const struct b3RobotSimLoadURDFArgs& args);

	int getNumJoints(int bodyUniqueId) const;

	bool getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo* jointInfo);

	void setJointMotorControl(int bodyUniqueId, int jointIndex, const struct b3JointMotorArgs& args);

	void stepSimulation();

	void setGravity(const b3Vector3& gravityAcceleration);

	void renderScene();
};

#endif //B3_ROBOT_SIM_API_H
