#ifndef B3_ROBOT_SIMULATOR_CLIENT_API_H
#define B3_ROBOT_SIMULATOR_CLIENT_API_H

#include "../SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Common/b3AlignedObjectArray.h"

#include <string>

struct b3RobotSimulatorLoadUrdfFileArgs
{
	b3Vector3 m_startPosition;
	b3Quaternion m_startOrientation;
	bool m_forceOverrideFixedBase;
	bool m_useMultiBody;
	int m_flags;

	b3RobotSimulatorLoadUrdfFileArgs(const b3Vector3& startPos, const b3Quaternion& startOrn)
		: m_startPosition(startPos),
		  m_startOrientation(startOrn),
		  m_forceOverrideFixedBase(false),
		  m_useMultiBody(true),
		  m_flags(0)
	{
	}

	b3RobotSimulatorLoadUrdfFileArgs()
		: m_startPosition(b3MakeVector3(0, 0, 0)),
		  m_startOrientation(b3Quaternion(0, 0, 0, 1)),
		  m_forceOverrideFixedBase(false),
		  m_useMultiBody(true),
		  m_flags(0)
	{
	}
};

struct b3RobotSimulatorLoadSdfFileArgs
{
	bool m_forceOverrideFixedBase;
	bool m_useMultiBody;

	b3RobotSimulatorLoadSdfFileArgs()
		: m_forceOverrideFixedBase(false),
		  m_useMultiBody(true)
	{
	}
};

struct b3RobotSimulatorLoadFileResults
{
	b3AlignedObjectArray<int> m_uniqueObjectIds;
	b3RobotSimulatorLoadFileResults()
	{
	}
};

struct b3RobotSimulatorJointMotorArgs
{
	int m_controlMode;

	double m_targetPosition;
	double m_kp;

	double m_targetVelocity;
	double m_kd;

	double m_maxTorqueValue;

	b3RobotSimulatorJointMotorArgs(int controlMode)
		: m_controlMode(controlMode),
		  m_targetPosition(0),
		  m_kp(0.1),
		  m_targetVelocity(0),
		  m_kd(0.9),
		  m_maxTorqueValue(1000)
	{
	}
};

enum b3RobotSimulatorInverseKinematicsFlags
{
	B3_HAS_IK_TARGET_ORIENTATION = 1,
	B3_HAS_NULL_SPACE_VELOCITY = 2,
	B3_HAS_JOINT_DAMPING = 4,
};

struct b3RobotSimulatorInverseKinematicArgs
{
	int m_bodyUniqueId;
	//	double* m_currentJointPositions;
	//	int m_numPositions;
	double m_endEffectorTargetPosition[3];
	double m_endEffectorTargetOrientation[4];
	int m_endEffectorLinkIndex;
	int m_flags;
	int m_numDegreeOfFreedom;
	b3AlignedObjectArray<double> m_lowerLimits;
	b3AlignedObjectArray<double> m_upperLimits;
	b3AlignedObjectArray<double> m_jointRanges;
	b3AlignedObjectArray<double> m_restPoses;
	b3AlignedObjectArray<double> m_jointDamping;

	b3RobotSimulatorInverseKinematicArgs()
		: m_bodyUniqueId(-1),
		  m_endEffectorLinkIndex(-1),
		  m_flags(0)
	{
		m_endEffectorTargetPosition[0] = 0;
		m_endEffectorTargetPosition[1] = 0;
		m_endEffectorTargetPosition[2] = 0;

		m_endEffectorTargetOrientation[0] = 0;
		m_endEffectorTargetOrientation[1] = 0;
		m_endEffectorTargetOrientation[2] = 0;
		m_endEffectorTargetOrientation[3] = 1;
	}
};

struct b3RobotSimulatorInverseKinematicsResults
{
	int m_bodyUniqueId;
	b3AlignedObjectArray<double> m_calculatedJointPositions;
};

struct b3JointStates2
{
    int m_bodyUniqueId;
    int m_numDegreeOfFreedomQ;
    int m_numDegreeOfFreedomU;
    b3Transform m_rootLocalInertialFrame;
    b3AlignedObjectArray<double> m_actualStateQ;
    b3AlignedObjectArray<double> m_actualStateQdot;
    b3AlignedObjectArray<double> m_jointReactionForces;
};

///The b3RobotSimulatorClientAPI is pretty much the C++ version of pybullet
///as documented in the pybullet Quickstart Guide
///https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA
class b3RobotSimulatorClientAPI
{
	struct b3RobotSimulatorClientAPI_InternalData* m_data;

public:


	b3RobotSimulatorClientAPI();
	virtual ~b3RobotSimulatorClientAPI();

	bool connect(int mode, const std::string& hostName = "localhost", int portOrKey = -1);

	void disconnect();

	bool isConnected() const;

	void setTimeOut(double timeOutInSec);

	void syncBodies();

	void resetSimulation();

	b3Quaternion getQuaternionFromEuler(const b3Vector3& rollPitchYaw);
	b3Vector3 getEulerFromQuaternion(const b3Quaternion& quat);

	int loadURDF(const std::string& fileName, const struct b3RobotSimulatorLoadUrdfFileArgs& args = b3RobotSimulatorLoadUrdfFileArgs());
	bool loadSDF(const std::string& fileName, b3RobotSimulatorLoadFileResults& results, const struct b3RobotSimulatorLoadSdfFileArgs& args = b3RobotSimulatorLoadSdfFileArgs());
	bool loadMJCF(const std::string& fileName, b3RobotSimulatorLoadFileResults& results);
	bool loadBullet(const std::string& fileName, b3RobotSimulatorLoadFileResults& results);

	bool getBodyInfo(int bodyUniqueId, struct b3BodyInfo* bodyInfo);

	bool getBasePositionAndOrientation(int bodyUniqueId, b3Vector3& basePosition, b3Quaternion& baseOrientation) const;
	bool resetBasePositionAndOrientation(int bodyUniqueId, b3Vector3& basePosition, b3Quaternion& baseOrientation);

	bool getBaseVelocity(int bodyUniqueId, b3Vector3& baseLinearVelocity, b3Vector3& baseAngularVelocity) const;
	bool resetBaseVelocity(int bodyUniqueId, const b3Vector3& linearVelocity, const b3Vector3& angularVelocity) const;

	int getNumJoints(int bodyUniqueId) const;

	bool getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo* jointInfo);

	int createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, b3JointInfo* jointInfo);

	int changeConstraint(int constraintId, b3JointInfo* jointInfo);

	void removeConstraint(int constraintId);

	bool getJointState(int bodyUniqueId, int jointIndex, struct b3JointSensorState* state);

	bool getJointStates(int bodyUniqueId, b3JointStates2& state);

	bool resetJointState(int bodyUniqueId, int jointIndex, double targetValue);

	void setJointMotorControl(int bodyUniqueId, int jointIndex, const struct b3RobotSimulatorJointMotorArgs& args);

	void stepSimulation();

	bool canSubmitCommand() const;

	void setRealTimeSimulation(bool enableRealTimeSimulation);

	void setInternalSimFlags(int flags);

	void setGravity(const b3Vector3& gravityAcceleration);

	void setTimeStep(double timeStepInSeconds);
	void setNumSimulationSubSteps(int numSubSteps);
	void setNumSolverIterations(int numIterations);
	void setContactBreakingThreshold(double threshold);

	bool calculateInverseKinematics(const struct b3RobotSimulatorInverseKinematicArgs& args, struct b3RobotSimulatorInverseKinematicsResults& results);

	bool getBodyJacobian(int bodyUniqueId, int linkIndex, const double* localPosition, const double* jointPositions, const double* jointVelocities, const double* jointAccelerations, double* linearJacobian, double* angularJacobian);

	bool getLinkState(int bodyUniqueId, int linkIndex, b3LinkState* linkState);

	void configureDebugVisualizer(enum b3ConfigureDebugVisualizerEnum flag, int enable);
	void resetDebugVisualizerCamera(double cameraDistance, double cameraPitch, double cameraYaw, const b3Vector3& targetPos);

	int startStateLogging(b3StateLoggingType loggingType, const std::string& fileName, const b3AlignedObjectArray<int>& objectUniqueIds=b3AlignedObjectArray<int>(), int maxLogDof = -1);
	void stopStateLogging(int stateLoggerUniqueId);

	void getVREvents(b3VREventsData* vrEventsData, int deviceTypeFilter);
	void getKeyboardEvents(b3KeyboardEventsData* keyboardEventsData);

	void submitProfileTiming(const std::string&  profileName, int durationInMicroSeconds=1);				


	//////////////// INTERNAL

	void loadBunny(double scale, double mass, double collisionMargin);

	//setGuiHelper is only used when embedded in existing example browser
	void setGuiHelper(struct GUIHelperInterface* guiHelper);
	//renderScene is only used when embedded in existing example browser
	virtual void renderScene();
	//debugDraw is only used when embedded in existing example browser
	virtual void debugDraw(int debugDrawMode);
	virtual bool	mouseMoveCallback(float x,float y);
	virtual bool	mouseButtonCallback(int button, int state, float x, float y);

	////////////////INTERNAL
};

#endif  //B3_ROBOT_SIMULATOR_CLIENT_API_H
