#ifndef B3_ROBOT_SIMULATOR_CLIENT_API_NO_DIRECT_H
#define B3_ROBOT_SIMULATOR_CLIENT_API_NO_DIRECT_H

///The b3RobotSimulatorClientAPI is pretty much the C++ version of pybullet
///as documented in the pybullet Quickstart Guide
///https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA

#include "SharedMemoryPublic.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btAlignedObjectArray.h"

#include <string>

struct b3RobotSimulatorLoadUrdfFileArgs
{
	btVector3 m_startPosition;
	btQuaternion m_startOrientation;
	bool m_forceOverrideFixedBase;
	bool m_useMultiBody;
	int m_flags;

	b3RobotSimulatorLoadUrdfFileArgs(const btVector3 &startPos, const btQuaternion &startOrn)
		: m_startPosition(startPos),
		  m_startOrientation(startOrn),
		  m_forceOverrideFixedBase(false),
		  m_useMultiBody(true),
		  m_flags(0)
	{
	}

	b3RobotSimulatorLoadUrdfFileArgs()
		: m_startPosition(btVector3(0, 0, 0)),
		  m_startOrientation(btQuaternion(0, 0, 0, 1)),
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

struct b3RobotSimulatorLoadSoftBodyArgs
{
	btVector3 m_startPosition;
	btQuaternion m_startOrientation;
	double m_scale;
	double m_mass;
	double m_collisionMargin;

	b3RobotSimulatorLoadSoftBodyArgs(const btVector3 &startPos, const btQuaternion &startOrn, const double &scale, const double &mass, const double &collisionMargin)
		: m_startPosition(startPos),
		  m_startOrientation(startOrn),
		  m_scale(scale),
		  m_mass(mass),
		  m_collisionMargin(collisionMargin)
	{
	}

	b3RobotSimulatorLoadSoftBodyArgs(const btVector3 &startPos, const btQuaternion &startOrn)
	{
		b3RobotSimulatorLoadSoftBodyArgs(startPos, startOrn, 1.0, 1.0, 0.02);
	}

	b3RobotSimulatorLoadSoftBodyArgs()
	{
		b3RobotSimulatorLoadSoftBodyArgs(btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1));
	}

	b3RobotSimulatorLoadSoftBodyArgs(double scale, double mass, double collisionMargin)
		: m_startPosition(btVector3(0, 0, 0)),
		  m_startOrientation(btQuaternion(0, 0, 0, 1)),
		  m_scale(scale),
		  m_mass(mass),
		  m_collisionMargin(collisionMargin)
	{
	}
};


struct b3RobotSimulatorLoadDeformableBodyArgs
{
	btVector3 m_startPosition;
	btQuaternion m_startOrientation;
	double m_scale;
	double m_mass;
	double m_collisionMargin;
	double m_springElasticStiffness;
	double m_springDampingStiffness;
	double m_springBendingStiffness;
	double m_NeoHookeanMu;
	double m_NeoHookeanLambda;
	double m_NeoHookeanDamping;
	bool m_useSelfCollision;
	bool m_useFaceContact;
	bool m_useBendingSprings;
	double m_frictionCoeff;

	b3RobotSimulatorLoadDeformableBodyArgs(const btVector3 &startPos, const btQuaternion &startOrn, const double &scale, const double &mass, const double &collisionMargin)
	: m_startPosition(startPos),
	m_startOrientation(startOrn),
	m_scale(scale),
	m_mass(mass),
	m_collisionMargin(collisionMargin),
	m_springElasticStiffness(-1),
	m_springDampingStiffness(-1),
	m_springBendingStiffness(-1),
	m_NeoHookeanMu(-1),
	m_NeoHookeanDamping(-1),
	m_useSelfCollision(false),
	m_useFaceContact(false),
	m_useBendingSprings(false),
	m_frictionCoeff(0)
	{
	}

	b3RobotSimulatorLoadDeformableBodyArgs(const btVector3 &startPos, const btQuaternion &startOrn)
	{
		b3RobotSimulatorLoadSoftBodyArgs(startPos, startOrn, 1.0, 1.0, 0.02);
	}

	b3RobotSimulatorLoadDeformableBodyArgs()
	{
		b3RobotSimulatorLoadSoftBodyArgs(btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1));
	}

	b3RobotSimulatorLoadDeformableBodyArgs(double scale, double mass, double collisionMargin)
	: m_startPosition(btVector3(0, 0, 0)),
	m_startOrientation(btQuaternion(0, 0, 0, 1)),
	m_scale(scale),
	m_mass(mass),
	m_collisionMargin(collisionMargin)
	{
	}
};


struct b3RobotSimulatorLoadFileResults
{
	btAlignedObjectArray<int> m_uniqueObjectIds;
	b3RobotSimulatorLoadFileResults()
	{
	}
};

struct b3RobotSimulatorChangeVisualShapeArgs
{
	int m_objectUniqueId;
	int m_linkIndex;
	int m_shapeIndex;
	int m_textureUniqueId;
	btVector4 m_rgbaColor;
	bool m_hasRgbaColor;
	btVector3 m_specularColor;
	bool m_hasSpecularColor;

	b3RobotSimulatorChangeVisualShapeArgs()
		: m_objectUniqueId(-1),
		  m_linkIndex(-1),
		  m_shapeIndex(-1),
		  m_textureUniqueId(-2),
		  m_rgbaColor(0, 0, 0, 1),
		  m_hasRgbaColor(false),
		  m_specularColor(1, 1, 1),
		  m_hasSpecularColor(false)
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
	B3_HAS_CURRENT_POSITIONS = 8,
};

struct b3RobotSimulatorInverseKinematicArgs
{
	int m_bodyUniqueId;
	double m_endEffectorTargetPosition[3];
	double m_endEffectorTargetOrientation[4];
	int m_endEffectorLinkIndex;
	int m_flags;
	int m_numDegreeOfFreedom;
	btAlignedObjectArray<double> m_lowerLimits;
	btAlignedObjectArray<double> m_upperLimits;
	btAlignedObjectArray<double> m_jointRanges;
	btAlignedObjectArray<double> m_restPoses;
	btAlignedObjectArray<double> m_jointDamping;
	btAlignedObjectArray<double> m_currentJointPositions;

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
	btAlignedObjectArray<double> m_calculatedJointPositions;
};

struct b3JointStates2
{
	int m_bodyUniqueId;
	int m_numDegreeOfFreedomQ;
	int m_numDegreeOfFreedomU;
	btTransform m_rootLocalInertialFrame;
	btAlignedObjectArray<double> m_actualStateQ;
	btAlignedObjectArray<double> m_actualStateQdot;
	btAlignedObjectArray<double> m_jointReactionForces;
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
		: m_controlMode(controlMode),
		  m_numControlledDofs(numControlledDofs),
		  m_jointIndices(NULL),
		  m_targetPositions(NULL),
		  m_kps(NULL),
		  m_targetVelocities(NULL),
		  m_kds(NULL),
		  m_forces(NULL)
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

struct b3RobotSimulatorSetPhysicsEngineParameters : b3PhysicsSimulationParameters
{
	b3RobotSimulatorSetPhysicsEngineParameters()
	{
		m_deltaTime = -1;
		m_gravityAcceleration[0] = 0;
		m_gravityAcceleration[1] = 0;
		m_gravityAcceleration[2] = 0;

		m_numSimulationSubSteps = -1;
		m_numSolverIterations = -1;
		m_useRealTimeSimulation = -1;
		m_useSplitImpulse = -1;
		m_splitImpulsePenetrationThreshold = -1;
		m_contactBreakingThreshold = -1;
		m_internalSimFlags = -1;
		m_defaultContactERP = -1;
		m_collisionFilterMode = -1;
		m_enableFileCaching = -1;
		m_restitutionVelocityThreshold = -1;
		m_defaultNonContactERP = -1;
		m_frictionERP = -1;
		m_defaultGlobalCFM = -1;
		m_frictionCFM = -1;
		m_enableConeFriction = -1;
		m_deterministicOverlappingPairs = -1;
		m_allowedCcdPenetration = -1;
		m_jointFeedbackMode = -1;
		m_solverResidualThreshold = -1;
		m_contactSlop = -1;

		m_collisionFilterMode = -1;
		m_contactBreakingThreshold = -1;

		m_enableFileCaching = -1;
		m_restitutionVelocityThreshold = -1;

		m_frictionERP = -1;
		m_solverResidualThreshold = -1;
		m_constraintSolverType = -1;
		m_minimumSolverIslandSize = -1;
	}
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
	int m_activationState;

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
		  m_frictionAnchor(-1),
		  m_activationState(-1)
	{
	}
};

struct b3RobotSimulatorAddUserDebugLineArgs
{
	double m_colorRGB[3];
	double m_lineWidth;
	double m_lifeTime;
	int m_parentObjectUniqueId;
	int m_parentLinkIndex;

	b3RobotSimulatorAddUserDebugLineArgs()
		: m_lineWidth(1),
		  m_lifeTime(0),
		  m_parentObjectUniqueId(-1),
		  m_parentLinkIndex(-1)
	{
		m_colorRGB[0] = 1;
		m_colorRGB[1] = 1;
		m_colorRGB[2] = 1;
	}
};

enum b3AddUserDebugTextFlags
{
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
	{
	}
};

struct b3RobotSimulatorCreateCollisionShapeArgs
{
	int m_shapeType;
	double m_radius;
	btVector3 m_halfExtents;
	double m_height;
	char *m_fileName;
	btVector3 m_meshScale;
	btVector3 m_planeNormal;
	int m_flags;

	double m_heightfieldTextureScaling;
	btAlignedObjectArray<float> m_heightfieldData;
	int m_numHeightfieldRows;
	int m_numHeightfieldColumns;
	int m_replaceHeightfieldIndex;

	b3RobotSimulatorCreateCollisionShapeArgs()
		: m_shapeType(-1),
		  m_radius(0.5),
		  m_height(1),
		  m_fileName(NULL),
		  m_flags(0),
		  m_heightfieldTextureScaling(1),
		  m_numHeightfieldRows(0),
		  m_numHeightfieldColumns(0),
		  m_replaceHeightfieldIndex(-1)
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


struct b3RobotSimulatorCreateVisualShapeArgs
{
	int m_shapeType;
	double m_radius;
	btVector3 m_halfExtents;
	double m_height;
	char* m_fileName;
	btVector3 m_meshScale;
	btVector3 m_planeNormal;
	int m_flags;
	b3RobotSimulatorCreateVisualShapeArgs()
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
	btVector3 m_basePosition;
	btQuaternion m_baseOrientation;
	btVector3 m_baseInertialFramePosition;
	btQuaternion m_baseInertialFrameOrientation;

	int m_numLinks;
	double *m_linkMasses;
	int *m_linkCollisionShapeIndices;
	int *m_linkVisualShapeIndices;
	btVector3 *m_linkPositions;
	btQuaternion *m_linkOrientations;
	btVector3 *m_linkInertialFramePositions;
	btQuaternion *m_linkInertialFrameOrientations;
	int *m_linkParentIndices;
	int *m_linkJointTypes;
	btVector3 *m_linkJointAxes;
	btAlignedObjectArray<btVector3> m_batchPositions;
	int m_useMaximalCoordinates;

	b3RobotSimulatorCreateMultiBodyArgs()
		: m_baseMass(0), m_baseCollisionShapeIndex(-1), m_baseVisualShapeIndex(-1), m_numLinks(0), m_linkMasses(NULL), m_linkCollisionShapeIndices(NULL), m_linkVisualShapeIndices(NULL), m_linkPositions(NULL), m_linkOrientations(NULL), m_linkInertialFramePositions(NULL), m_linkInertialFrameOrientations(NULL), m_linkParentIndices(NULL), m_linkJointTypes(NULL), m_linkJointAxes(NULL), m_useMaximalCoordinates(0)
	{
		m_basePosition.setValue(0, 0, 0);
		m_baseOrientation.setValue(0, 0, 0, 1);
		m_baseInertialFramePosition.setValue(0, 0, 0);
		m_baseInertialFrameOrientation.setValue(0, 0, 0, 1);
	}
};


struct b3RobotUserConstraint : public b3UserConstraint
{
	int m_userUpdateFlags;//see EnumUserConstraintFlags

	void setErp(double erp)
	{
		m_erp = erp;
		m_userUpdateFlags |= USER_CONSTRAINT_CHANGE_ERP;
	}
	
	void setMaxAppliedForce(double maxForce)
	{
		m_maxAppliedForce = maxForce;
		m_userUpdateFlags |= USER_CONSTRAINT_CHANGE_MAX_FORCE;
	}
	
	void setGearRatio(double gearRatio)
	{
		m_gearRatio = gearRatio;
		m_userUpdateFlags |= USER_CONSTRAINT_CHANGE_GEAR_RATIO;
	}

	void setGearAuxLink(int link)
	{
		m_gearAuxLink = link;
		m_userUpdateFlags |= USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK;
	}

	void setRelativePositionTarget(double target)
	{
		m_relativePositionTarget = target;
		m_userUpdateFlags |= USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET;
	}

	void setChildPivot(double pivot[3])
	{
		m_childFrame[0] = pivot[0];
		m_childFrame[1] = pivot[1];
		m_childFrame[2] = pivot[2];
		m_userUpdateFlags |= USER_CONSTRAINT_CHANGE_PIVOT_IN_B;
	}

	void setChildFrameOrientation(double orn[4])
	{
		m_childFrame[3] = orn[0];
		m_childFrame[4] = orn[1];
		m_childFrame[5] = orn[2];
		m_childFrame[6] = orn[3];
		m_userUpdateFlags |= USER_CONSTRAINT_CHANGE_FRAME_ORN_IN_B;
	}

	b3RobotUserConstraint()
		:m_userUpdateFlags(0)
	{
		m_parentBodyIndex = -1;
		m_parentJointIndex = -1;
		m_childBodyIndex = -1;
		m_childJointIndex = -1;
		//position
		m_parentFrame[0] = 0;
		m_parentFrame[1] = 0;
		m_parentFrame[2] = 0;
		//orientation quaternion [x,y,z,w]
		m_parentFrame[3] = 0;
		m_parentFrame[4] = 0;
		m_parentFrame[5] = 0;
		m_parentFrame[6] = 1;

		//position
		m_childFrame[0] = 0;
		m_childFrame[1] = 0;
		m_childFrame[2] = 0;
		//orientation quaternion [x,y,z,w]
		m_childFrame[3] = 0;
		m_childFrame[4] = 0;
		m_childFrame[5] = 0;
		m_childFrame[6] = 1;

		m_jointAxis[0] = 0;
		m_jointAxis[1] = 0;
		m_jointAxis[2] = 1;

		m_jointType = eFixedType;

		m_maxAppliedForce = 500;
		m_userConstraintUniqueId = -1;
		m_gearRatio = -1;
		m_gearAuxLink = -1;
		m_relativePositionTarget = 0;
		m_erp = 0;
	}
};

struct b3RobotJointInfo : public b3JointInfo
{
	b3RobotJointInfo()
	{
		m_linkName[0] = 0;
		m_jointName[0] = 0;
		m_jointType = eFixedType;
		m_qIndex = -1;
		m_uIndex = -1;
		m_jointIndex = -1;
		m_flags = 0;
		m_jointDamping = 0;
		m_jointFriction = 0;
		m_jointLowerLimit = 1;
		m_jointUpperLimit = -1;
		m_jointMaxForce = 500;
		m_jointMaxVelocity = 100;
		m_parentIndex = -1;

		//position
		m_parentFrame[0] = 0;
		m_parentFrame[1] = 0;
		m_parentFrame[2] = 0;
		//orientation quaternion [x,y,z,w]
		m_parentFrame[3] = 0;
		m_parentFrame[4] = 0;
		m_parentFrame[5] = 0;
		m_parentFrame[6] = 1;

		//position
		m_childFrame[0] = 0;
		m_childFrame[1] = 0;
		m_childFrame[2] = 0;
		//orientation quaternion [x,y,z,w]
		m_childFrame[3] = 0;
		m_childFrame[4] = 0;
		m_childFrame[5] = 0;
		m_childFrame[6] = 1;

		m_jointAxis[0] = 0;
		m_jointAxis[1] = 0;
		m_jointAxis[2] = 1;
	}
};

class b3RobotSimulatorClientAPI_NoDirect
{
protected:
	struct b3RobotSimulatorClientAPI_InternalData *m_data;

public:
	b3RobotSimulatorClientAPI_NoDirect();
	virtual ~b3RobotSimulatorClientAPI_NoDirect();

	//No 'connect', use setInternalData to bypass the connect method, pass an existing client
	virtual void setInternalData(struct b3RobotSimulatorClientAPI_InternalData *data);

	void disconnect();

	bool isConnected() const;

	void setTimeOut(double timeOutInSec);

	void syncBodies();

	void resetSimulation();
    
	void resetSimulation(int flag);

	btQuaternion getQuaternionFromEuler(const btVector3 &rollPitchYaw);
	btVector3 getEulerFromQuaternion(const btQuaternion &quat);

	int loadURDF(const std::string &fileName, const struct b3RobotSimulatorLoadUrdfFileArgs &args = b3RobotSimulatorLoadUrdfFileArgs());
	bool loadSDF(const std::string &fileName, b3RobotSimulatorLoadFileResults &results, const struct b3RobotSimulatorLoadSdfFileArgs &args = b3RobotSimulatorLoadSdfFileArgs());
	bool loadMJCF(const std::string &fileName, b3RobotSimulatorLoadFileResults &results);
	bool loadBullet(const std::string &fileName, b3RobotSimulatorLoadFileResults &results);
	bool saveBullet(const std::string &fileName);

	int loadTexture(const std::string &fileName);

	bool changeVisualShape(const struct b3RobotSimulatorChangeVisualShapeArgs &args);

	bool savePythonWorld(const std::string &fileName);

	bool getBodyInfo(int bodyUniqueId, struct b3BodyInfo *bodyInfo);

	bool getBasePositionAndOrientation(int bodyUniqueId, btVector3 &basePosition, btQuaternion &baseOrientation) const;
	bool resetBasePositionAndOrientation(int bodyUniqueId, const btVector3 &basePosition, const btQuaternion &baseOrientation);

	bool getBaseVelocity(int bodyUniqueId, btVector3 &baseLinearVelocity, btVector3 &baseAngularVelocity) const;
	bool resetBaseVelocity(int bodyUniqueId, const btVector3 &linearVelocity, const btVector3 &angularVelocity) const;

	int getNumJoints(int bodyUniqueId) const;

	bool getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo *jointInfo);

	int createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, b3JointInfo *jointInfo);

	int changeConstraint(int constraintId, b3RobotUserConstraint*jointInfo);

	void removeConstraint(int constraintId);

	bool getConstraintInfo(int constraintUniqueId, struct b3UserConstraint &constraintInfo);

	bool getJointState(int bodyUniqueId, int jointIndex, struct b3JointSensorState *state);

	bool getJointStates(int bodyUniqueId, b3JointStates2 &state);

	bool resetJointState(int bodyUniqueId, int jointIndex, double targetValue);

	void setJointMotorControl(int bodyUniqueId, int jointIndex, const struct b3RobotSimulatorJointMotorArgs &args);

	bool setJointMotorControlArray(int bodyUniqueId, int controlMode, int numControlledDofs,
								   int *jointIndices, double *targetVelocities, double *targetPositions,
								   double *forces, double *kps, double *kds);

	void stepSimulation();

	bool canSubmitCommand() const;

	void setRealTimeSimulation(bool enableRealTimeSimulation);

	void setInternalSimFlags(int flags);

	void setGravity(const btVector3 &gravityAcceleration);

	void setTimeStep(double timeStepInSeconds);
	void setNumSimulationSubSteps(int numSubSteps);
	void setNumSolverIterations(int numIterations);
	void setContactBreakingThreshold(double threshold);

	int computeDofCount(int bodyUniqueId) const;
	
	bool calculateInverseKinematics(const struct b3RobotSimulatorInverseKinematicArgs &args, struct b3RobotSimulatorInverseKinematicsResults &results);

	int calculateMassMatrix(int bodyUniqueId, const double* jointPositions, int numJointPositions, double* massMatrix, int flags);
	
	bool getBodyJacobian(int bodyUniqueId, int linkIndex, const double *localPosition, const double *jointPositions, const double *jointVelocities, const double *jointAccelerations, double *linearJacobian, double *angularJacobian);

	void configureDebugVisualizer(enum b3ConfigureDebugVisualizerEnum flag, int enable);
	void resetDebugVisualizerCamera(double cameraDistance, double cameraPitch, double cameraYaw, const btVector3 &targetPos);

	int startStateLogging(b3StateLoggingType loggingType, const std::string &fileName, const btAlignedObjectArray<int> &objectUniqueIds = btAlignedObjectArray<int>(), int maxLogDof = -1);
	void stopStateLogging(int stateLoggerUniqueId);

	void getVREvents(b3VREventsData *vrEventsData, int deviceTypeFilter);
	void getKeyboardEvents(b3KeyboardEventsData *keyboardEventsData);

	void submitProfileTiming(const std::string &profileName);

	// JFC: added these 24 methods

	void getMouseEvents(b3MouseEventsData *mouseEventsData);

	bool getLinkState(int bodyUniqueId, int linkIndex, int computeLinkVelocity, int computeForwardKinematics, b3LinkState *linkState);

	bool getCameraImage(int width, int height, struct b3RobotSimulatorGetCameraImageArgs args, b3CameraImageData &imageData);

	bool calculateInverseDynamics(int bodyUniqueId, double *jointPositions, double *jointVelocities, double *jointAccelerations, double *jointForcesOutput);

	int getNumBodies() const;

	int getBodyUniqueId(int bodyId) const;

	bool removeBody(int bodyUniqueId);

	bool getDynamicsInfo(int bodyUniqueId, int linkIndex, b3DynamicsInfo *dynamicsInfo);

	bool changeDynamics(int bodyUniqueId, int linkIndex, struct b3RobotSimulatorChangeDynamicsArgs &args);

	int addUserDebugParameter(const char *paramName, double rangeMin, double rangeMax, double startValue);

	double readUserDebugParameter(int itemUniqueId);

	bool removeUserDebugItem(int itemUniqueId);

	int addUserDebugText(const char *text, double *textPosition, struct b3RobotSimulatorAddUserDebugTextArgs &args);

	int addUserDebugText(const char *text, btVector3 &textPosition, struct b3RobotSimulatorAddUserDebugTextArgs &args);

	int addUserDebugLine(double *fromXYZ, double *toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs &args);

	int addUserDebugLine(btVector3 &fromXYZ, btVector3 &toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs &args);

	bool setJointMotorControlArray(int bodyUniqueId, struct b3RobotSimulatorJointMotorArrayArgs &args);

	bool setPhysicsEngineParameter(const struct b3RobotSimulatorSetPhysicsEngineParameters &args);

	bool getPhysicsEngineParameters(struct b3RobotSimulatorSetPhysicsEngineParameters &args);

	bool applyExternalForce(int objectUniqueId, int linkIndex, double *force, double *position, int flags);

	bool applyExternalForce(int objectUniqueId, int linkIndex, btVector3 &force, btVector3 &position, int flags);

	bool applyExternalTorque(int objectUniqueId, int linkIndex, double *torque, int flags);

	bool applyExternalTorque(int objectUniqueId, int linkIndex, btVector3 &torque, int flags);

	bool enableJointForceTorqueSensor(int bodyUniqueId, int jointIndex, bool enable);

	bool getDebugVisualizerCamera(struct b3OpenGLVisualizerCameraInfo *cameraInfo);

	bool getContactPoints(struct b3RobotSimulatorGetContactPointsArgs &args, struct b3ContactInformation *contactInfo);

	bool getClosestPoints(struct b3RobotSimulatorGetContactPointsArgs &args, double distance, struct b3ContactInformation *contactInfo);

	bool getOverlappingObjects(double *aabbMin, double *aabbMax, struct b3AABBOverlapData *overlapData);

	bool getOverlappingObjects(btVector3 &aabbMin, btVector3 &aabbMax, struct b3AABBOverlapData *overlapData);

	bool getAABB(int bodyUniqueId, int linkIndex, double *aabbMin, double *aabbMax);

	bool getAABB(int bodyUniqueId, int linkIndex, btVector3 &aabbMin, btVector3 &aabbMax);

	int createVisualShape(int shapeType, struct b3RobotSimulatorCreateVisualShapeArgs& args);

	int createCollisionShape(int shapeType, struct b3RobotSimulatorCreateCollisionShapeArgs &args);

	int createMultiBody(struct b3RobotSimulatorCreateMultiBodyArgs &args);

	int getNumConstraints() const;

	int getConstraintUniqueId(int serialIndex);

	void loadSoftBody(const std::string &fileName, const struct b3RobotSimulatorLoadSoftBodyArgs &args);
    
	void loadDeformableBody(const std::string &fileName, const struct b3RobotSimulatorLoadDeformableBodyArgs &args);

	virtual void setGuiHelper(struct GUIHelperInterface *guiHelper);
	virtual struct GUIHelperInterface *getGuiHelper();

	bool getCollisionShapeData(int bodyUniqueId, int linkIndex, b3CollisionShapeInformation &collisionShapeInfo);

	bool getVisualShapeData(int bodyUniqueId, struct b3VisualShapeInformation &visualShapeInfo);

	int saveStateToMemory();
	void restoreStateFromMemory(int stateId);
	void removeState(int stateUniqueId);

	int getAPIVersion() const
	{
		return SHARED_MEMORY_MAGIC_NUMBER;
	}
	void setAdditionalSearchPath(const std::string &path);
    
    void setCollisionFilterGroupMask(int bodyUniqueIdA, int linkIndexA, int collisionFilterGroup, int collisionFilterMask);
};

#endif  //B3_ROBOT_SIMULATOR_CLIENT_API_NO_DIRECT_H
