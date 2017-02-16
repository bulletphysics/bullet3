#ifndef B3_ROBOT_SIM_API_H
#define B3_ROBOT_SIM_API_H

///todo: remove those includes from this header
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Common/b3AlignedObjectArray.h"

#include <string>

enum b3RigidSimFileType
{
	B3_URDF_FILE=1,
	B3_SDF_FILE,
	B3_AUTO_DETECT_FILE//todo based on extension
};

struct b3JointStates
{
    int m_bodyUniqueId;
    int m_numDegreeOfFreedomQ;
    int m_numDegreeOfFreedomU;
    b3Transform m_rootLocalInertialFrame;
    b3AlignedObjectArray<double> m_actualStateQ;
    b3AlignedObjectArray<double> m_actualStateQdot;
    b3AlignedObjectArray<double> m_jointReactionForces;
};

struct b3RobotSimLoadFileArgs
{
	std::string m_fileName;
	b3Vector3 m_startPosition;
	b3Quaternion m_startOrientation;
	bool m_forceOverrideFixedBase;
    bool m_useMultiBody;
	int m_fileType;


	b3RobotSimLoadFileArgs(const std::string& fileName)
		:m_fileName(fileName),
		m_startPosition(b3MakeVector3(0,0,0)),
		m_startOrientation(b3Quaternion(0,0,0,1)),
		m_forceOverrideFixedBase(false),
        m_useMultiBody(true),
		m_fileType(B3_URDF_FILE)
	{
	}
};


struct b3RobotSimLoadFileResults
{
	b3AlignedObjectArray<int> m_uniqueObjectIds;
	b3RobotSimLoadFileResults()
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
		m_kd(0.9),
		m_maxTorqueValue(1000)
	{
	}
};

enum b3InverseKinematicsFlags
{
	B3_HAS_IK_TARGET_ORIENTATION=1,
    B3_HAS_NULL_SPACE_VELOCITY=2,
    B3_HAS_JOINT_DAMPING=4,
};

struct b3RobotSimInverseKinematicArgs
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

	b3RobotSimInverseKinematicArgs()
		:m_bodyUniqueId(-1),
		m_endEffectorLinkIndex(-1),
		m_flags(0)
	{
		m_endEffectorTargetPosition[0]=0;
		m_endEffectorTargetPosition[1]=0;
		m_endEffectorTargetPosition[2]=0;

		m_endEffectorTargetOrientation[0]=0;
		m_endEffectorTargetOrientation[1]=0;
		m_endEffectorTargetOrientation[2]=0;
		m_endEffectorTargetOrientation[3]=1;
	}
};

struct b3RobotSimInverseKinematicsResults
{
	int m_bodyUniqueId;
	b3AlignedObjectArray<double> m_calculatedJointPositions;
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

	bool loadFile(const struct b3RobotSimLoadFileArgs& args, b3RobotSimLoadFileResults& results);

	int getNumJoints(int bodyUniqueId) const;

	bool getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo* jointInfo);
    
    void createJoint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, b3JointInfo* jointInfo);

    bool getJointStates(int bodyUniqueId, b3JointStates& state);
    
	void setJointMotorControl(int bodyUniqueId, int jointIndex, const struct b3JointMotorArgs& args);

	void stepSimulation();

	void setGravity(const b3Vector3& gravityAcceleration);
    
    void setNumSimulationSubSteps(int numSubSteps);
	void setNumSolverIterations(int numIterations);

	bool calculateInverseKinematics(const struct b3RobotSimInverseKinematicArgs& args, struct b3RobotSimInverseKinematicsResults& results);

	void renderScene();
	void debugDraw(int debugDrawMode);
    
    void getBodyJacobian(int bodyUniqueId, int linkIndex, const double* localPosition, const double* jointPositions, const double* jointVelocities, const double* jointAccelerations, double* linearJacobian, double* angularJacobian);
    
    void getLinkState(int bodyUniqueId, int linkIndex, double* worldPosition, double* worldOrientation);
    
    void loadBunny(double scale, double mass, double collisionMargin);
};

#endif //B3_ROBOT_SIM_API_H
