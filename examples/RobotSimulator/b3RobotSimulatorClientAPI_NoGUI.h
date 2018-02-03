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


struct b3RobotSimulatorJointMotorArrayArgs
{
	int m_controlMode;
	int m_numControlledDofs;

	int *m_jointIndices;

	double *m_targetPositions;
	double *m_kps;

	double *m_targetVelocities;
	double *m_kds;

	double *m_forces;

	b3RobotSimulatorJointMotorArrayArgs(int controlMode, int numControlledDofs)
		: m_controlMode(controlMode), m_numControlledDofs(numControlledDofs)
	{
	}
};

struct b3RobotSimulatorGetCameraImageArgs
{
	int m_width;
	int m_height;
	float *m_viewMatrix;
	float *m_projectionMatrix;
	float *m_lightDirection;
	float *m_lightColor;
	float m_lightDistance;
	int m_hasShadow;
	float m_lightAmbientCoeff;
	float m_lightDiffuseCoeff;
	float m_lightSpecularCoeff;
	int m_renderer;

	b3RobotSimulatorGetCameraImageArgs(int width, int height)
		: m_width(width),
		m_height(height),
		m_viewMatrix(NULL),
		m_projectionMatrix(NULL),
		m_lightDirection(NULL),
		m_lightColor(NULL),
		m_lightDistance(-1),
		m_hasShadow(-1),
		m_lightAmbientCoeff(-1),
		m_lightDiffuseCoeff(-1),
		m_lightSpecularCoeff(-1),
		m_renderer(-1)
	{
	}
};

struct b3RobotSimulatorSetPhysicsEngineParameters 
{
	double m_fixedTimeStep;
	int m_numSolverIterations;
	int m_useSplitImpulse;
	double m_splitImpulsePenetrationThreshold;
	int m_numSubSteps;
	int m_collisionFilterMode;
	double m_contactBreakingThreshold;
	int m_maxNumCmdPer1ms;
	int m_enableFileCaching;
	double m_restitutionVelocityThreshold;
	double m_erp;
	double m_contactERP;
	double m_frictionERP;

	b3RobotSimulatorSetPhysicsEngineParameters()
		: m_fixedTimeStep(-1),
		m_numSolverIterations(-1),
		m_useSplitImpulse(-1),
		m_splitImpulsePenetrationThreshold(-1),
		m_numSubSteps(-1),
		m_collisionFilterMode(-1),
		m_contactBreakingThreshold(-1),
		m_maxNumCmdPer1ms(-1),
		m_enableFileCaching(-1),
		m_restitutionVelocityThreshold(-1),
		m_erp(-1),
		m_contactERP(-1),
		m_frictionERP(-1)
	{}
};

struct b3RobotSimulatorChangeDynamicsArgs
{
	double m_mass;
	double m_lateralFriction;
	double m_spinningFriction;
	double m_rollingFriction;
	double m_restitution;
	double m_linearDamping;
	double m_angularDamping;
	double m_contactStiffness;
	double m_contactDamping;
	int m_frictionAnchor;

	b3RobotSimulatorChangeDynamicsArgs()
		: m_mass(-1),
		m_lateralFriction(-1),
		m_spinningFriction(-1),
		m_rollingFriction(-1),
		m_restitution(-1),
		m_linearDamping(-1),
		m_angularDamping(-1),
		m_contactStiffness(-1),
		m_contactDamping(-1),
		m_frictionAnchor(-1)
	{}
};

struct b3RobotSimulatorAddUserDebugLineArgs
{
	double m_colorRGB[3];
	double m_lineWidth;
	double m_lifeTime;
	int m_parentObjectUniqueId;
	int m_parentLinkIndex;

	b3RobotSimulatorAddUserDebugLineArgs()
		: 
	m_lineWidth(1),
		m_lifeTime(0),
		m_parentObjectUniqueId(-1),
		m_parentLinkIndex(-1)
	{
		m_colorRGB[0] = 1;
		m_colorRGB[1] = 1;
		m_colorRGB[2] = 1;
	}
};

enum b3AddUserDebugTextFlags {
	DEBUG_TEXT_HAS_ORIENTATION = 1
};

struct b3RobotSimulatorAddUserDebugTextArgs
{
	double m_colorRGB[3];
	double m_size;
	double m_lifeTime;
	double m_textOrientation[4];
	int m_parentObjectUniqueId;
	int m_parentLinkIndex;
	int m_flags;

	b3RobotSimulatorAddUserDebugTextArgs()
		: m_size(1),
		m_lifeTime(0),
		m_parentObjectUniqueId(-1),
		m_parentLinkIndex(-1),
		m_flags(0)
	{
		m_colorRGB[0] = 1;
		m_colorRGB[1] = 1;
		m_colorRGB[2] = 1;

		m_textOrientation[0] = 0;
		m_textOrientation[1] = 0;
		m_textOrientation[2] = 0;
		m_textOrientation[3] = 1;

	}
};

struct b3RobotSimulatorGetContactPointsArgs
{
	int m_bodyUniqueIdA;
	int m_bodyUniqueIdB;
	int m_linkIndexA;
	int m_linkIndexB;

	b3RobotSimulatorGetContactPointsArgs()
		: m_bodyUniqueIdA(-1),
		m_bodyUniqueIdB(-1),
		m_linkIndexA(-2),
		m_linkIndexB(-2)
	{}
};

struct b3RobotSimulatorCreateCollisionShapeArgs
{
	int m_shapeType;
	double m_radius;
	b3Vector3 m_halfExtents;
	double m_height;
	char* m_fileName;
	b3Vector3 m_meshScale;
	b3Vector3 m_planeNormal;
	int m_flags;
	b3RobotSimulatorCreateCollisionShapeArgs()
		: m_shapeType(-1),
		m_radius(0.5),
		m_height(1),
		m_fileName(NULL),
		m_flags(0)
	{
		m_halfExtents.m_floats[0] = 1;
		m_halfExtents.m_floats[1] = 1;
		m_halfExtents.m_floats[2] = 1;

		m_meshScale.m_floats[0] = 1;
		m_meshScale.m_floats[1] = 1;
		m_meshScale.m_floats[2] = 1;

		m_planeNormal.m_floats[0] = 0;
		m_planeNormal.m_floats[1] = 0;
		m_planeNormal.m_floats[2] = 1;
	}

};

struct b3RobotSimulatorCreateMultiBodyArgs
{
	double m_baseMass;
	int m_baseCollisionShapeIndex;
	int m_baseVisualShapeIndex;
	b3Vector3 m_basePosition;
	b3Quaternion m_baseOrientation;
	b3Vector3 m_baseInertialFramePosition;
	b3Quaternion m_baseInertialFrameOrientation;

	int m_numLinks;
	double *m_linkMasses;
	int *m_linkCollisionShapeIndices;
	int *m_linkVisualShapeIndices;
	b3Vector3 *m_linkPositions;
	b3Quaternion *m_linkOrientations;
	b3Vector3 *m_linkInertialFramePositions;
	b3Quaternion *m_linkInertialFrameOrientations;
	int *m_linkParentIndices;
	int *m_linkJointTypes;
	b3Vector3 *m_linkJointAxes;

	int m_useMaximalCoordinates;

	b3RobotSimulatorCreateMultiBodyArgs()
		: m_numLinks(0), m_baseMass(0), m_baseCollisionShapeIndex(-1), m_baseVisualShapeIndex(-1), m_useMaximalCoordinates(0), 
		m_linkMasses(NULL), 
		m_linkCollisionShapeIndices(NULL), 
		m_linkVisualShapeIndices(NULL), 
		m_linkPositions(NULL), 
		m_linkOrientations(NULL), 
		m_linkInertialFramePositions(NULL), 
		m_linkInertialFrameOrientations(NULL), 
		m_linkParentIndices(NULL), 
		m_linkJointTypes(NULL), 
		m_linkJointAxes(NULL)      
	{
		m_basePosition.setValue(0,0,0);
		m_baseOrientation.setValue(0,0,0,1);
		m_baseInertialFramePosition.setValue(0,0,0);
		m_baseInertialFrameOrientation.setValue(0,0,0,1);
	}
};



///The b3RobotSimulatorClientAPI is pretty much the C++ version of pybullet
///as documented in the pybullet Quickstart Guide
///https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA
class b3RobotSimulatorClientAPI_NoGUI
{
protected:

	struct b3RobotSimulatorClientAPI_InternalData* m_data;

public:
	
	b3RobotSimulatorClientAPI_NoGUI();
	virtual ~b3RobotSimulatorClientAPI_NoGUI();

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

	bool setJointMotorControlArray(int bodyUniqueId, int controlMode, int numControlledDofs,
		int *jointIndices, double *targetVelocities, double *targetPositions,
		double *forces, double *kps, double *kds);

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

	// JFC: added these 24 methods

	void getMouseEvents(b3MouseEventsData* mouseEventsData);

	bool getLinkState(int bodyUniqueId, int linkIndex, int computeLinkVelocity, int computeInverseKinematics, b3LinkState* linkState);

	bool getCameraImage(int width, int height, struct b3RobotSimulatorGetCameraImageArgs args, b3CameraImageData &imageData);

	bool calculateInverseDynamics(int bodyUniqueId, double *jointPositions, double *jointVelocities, double *jointAccelerations, double *jointForcesOutput);

	int getNumBodies() const;

	int getBodyUniqueId(int bodyId) const;

	bool removeBody(int bodyUniqueId);

	bool getDynamicsInfo(int bodyUniqueId, int linkIndex, b3DynamicsInfo *dynamicsInfo);

	bool changeDynamics(int bodyUniqueId, int linkIndex, struct b3RobotSimulatorChangeDynamicsArgs &args);

	int addUserDebugParameter(char *paramName, double rangeMin, double rangeMax, double startValue);

	double readUserDebugParameter(int itemUniqueId);

	bool removeUserDebugItem(int itemUniqueId);

	int addUserDebugText(char *text, double *textPosition, struct b3RobotSimulatorAddUserDebugTextArgs &args);

	int addUserDebugText(char *text, b3Vector3 &textPosition, struct b3RobotSimulatorAddUserDebugTextArgs &args);

	int addUserDebugLine(double *fromXYZ, double *toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs &args);

	int addUserDebugLine(b3Vector3 &fromXYZ, b3Vector3 &toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs &args);

	bool setJointMotorControlArray(int bodyUniqueId, struct b3RobotSimulatorJointMotorArrayArgs &args);

	bool setPhysicsEngineParameter(struct b3RobotSimulatorSetPhysicsEngineParameters &args);

	bool applyExternalForce(int objectUniqueId, int linkIndex, double *force, double *position, int flags);

	bool applyExternalForce(int objectUniqueId, int linkIndex, b3Vector3 &force, b3Vector3 &position, int flags);

	bool applyExternalTorque(int objectUniqueId, int linkIndex, double *torque, int flags);

	bool applyExternalTorque(int objectUniqueId, int linkIndex, b3Vector3 &torque, int flags);

	bool enableJointForceTorqueSensor(int bodyUniqueId, int linkIndex, bool enable);

	bool getDebugVisualizerCamera(struct b3OpenGLVisualizerCameraInfo *cameraInfo);

	bool getContactPoints(struct b3RobotSimulatorGetContactPointsArgs &args, struct b3ContactInformation *contactInfo);

	bool getClosestPoints(struct b3RobotSimulatorGetContactPointsArgs &args, double distance, struct b3ContactInformation *contactInfo);

	bool getOverlappingObjects(double *aabbMin, double *aabbMax, struct b3AABBOverlapData *overlapData);

	bool getOverlappingObjects(b3Vector3 &aabbMin, b3Vector3 &aabbMax, struct b3AABBOverlapData *overlapData);

	bool getAABB(int bodyUniqueId, int linkIndex, double *aabbMin, double *aabbMax);

	bool getAABB(int bodyUniqueId, int linkIndex, b3Vector3 &aabbMin, b3Vector3 &aabbMax);

	int createCollisionShape(int shapeType, struct b3RobotSimulatorCreateCollisionShapeArgs &args);

	int createMultiBody(struct b3RobotSimulatorCreateMultiBodyArgs &args);

	int getNumConstraints() const;

	int getConstraintUniqueId(int serialIndex);

	void loadSoftBody(const std::string& fileName, double scale, double mass, double collisionMargin);
	
	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper);
	virtual struct GUIHelperInterface* getGuiHelper();
	
};

#endif  //B3_ROBOT_SIMULATOR_CLIENT_API_H
