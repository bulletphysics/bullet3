
#include "GripperGraspExample.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../SharedMemory/PhysicsServerSharedMemory.h"
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../SharedMemory/SharedMemoryPublic.h"
#include <string>

#include "../RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"

static btScalar sGripperVerticalVelocity = 0.f;
static btScalar sGripperClosingTargetVelocity = -0.7f;

class GripperGraspExample : public CommonExampleInterface
{
	CommonGraphicsApp* m_app;
	GUIHelperInterface* m_guiHelper;
	b3RobotSimulatorClientAPI m_robotSim;
	int m_options;
	int m_gripperIndex;
	double m_time;
	b3Vector3 m_targetPos;
	b3Vector3 m_worldPos;
	b3Vector4 m_targetOri;
	b3Vector4 m_worldOri;

	b3AlignedObjectArray<int> m_movingInstances;
	enum
	{
		numCubesX = 20,
		numCubesY = 20
	};

public:
	GripperGraspExample(GUIHelperInterface* helper, int options)
		: m_app(helper->getAppInterface()),
		  m_guiHelper(helper),
		  m_options(options),
		  m_gripperIndex(-1)
	{
		m_app->setUpAxis(2);
	}
	virtual ~GripperGraspExample()
	{
	}

	virtual void physicsDebugDraw(int debugDrawMode)
	{
		m_robotSim.debugDraw(debugDrawMode);
	}
	virtual void initPhysics()
	{
		int mode = eCONNECT_EXISTING_EXAMPLE_BROWSER;
		m_robotSim.setGuiHelper(m_guiHelper);
		bool connected = m_robotSim.connect(mode);

		m_robotSim.configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, 0);
		m_robotSim.configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0);
		m_robotSim.configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0);

		b3Printf("robotSim connected = %d", connected);

		if ((m_options & eGRIPPER_GRASP) != 0)
		{
			{
				SliderParams slider("Vertical velocity", &sGripperVerticalVelocity);
				slider.m_minVal = -2;
				slider.m_maxVal = 2;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}

			{
				SliderParams slider("Closing velocity", &sGripperClosingTargetVelocity);
				slider.m_minVal = -1;
				slider.m_maxVal = 1;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}
			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(0, 0, .107);
				args.m_startOrientation.setEulerZYX(0, 0, 0);
				args.m_useMultiBody = true;
				m_robotSim.loadURDF("cube_small.urdf", args);
			}

			{
				b3RobotSimulatorLoadFileResults results;
				m_robotSim.loadSDF("gripper/wsg50_with_r2d2_gripper.sdf", results);
				if (results.m_uniqueObjectIds.size() == 1)
				{
					m_gripperIndex = results.m_uniqueObjectIds[0];
					int numJoints = m_robotSim.getNumJoints(m_gripperIndex);
					b3Printf("numJoints = %d", numJoints);

					for (int i = 0; i < numJoints; i++)
					{
						b3JointInfo jointInfo;
						m_robotSim.getJointInfo(m_gripperIndex, i, &jointInfo);
						b3Printf("joint[%d].m_jointName=%s", i, jointInfo.m_jointName);
					}

					/*
                    int fingerJointIndices[2]={1,3};
                    double fingerTargetPositions[2]={-0.04,0.04};
                    for (int i=0;i<2;i++)
                    {
                        b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
                        controlArgs.m_targetPosition = fingerTargetPositions[i];
                        controlArgs.m_kp = 5.0;
                        controlArgs.m_kd = 3.0;
                        controlArgs.m_maxTorqueValue = 1.0;
                        m_robotSim.setJointMotorControl(m_gripperIndex,fingerJointIndices[i],controlArgs);
                    }
                    */
					int fingerJointIndices[3] = {0, 1, 3};
					double fingerTargetVelocities[3] = {-0.2, .5, -.5};
					double maxTorqueValues[3] = {40.0, 50.0, 50.0};
					for (int i = 0; i < 3; i++)
					{
						b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
						controlArgs.m_targetVelocity = fingerTargetVelocities[i];
						controlArgs.m_maxTorqueValue = maxTorqueValues[i];
						controlArgs.m_kd = 1.;
						m_robotSim.setJointMotorControl(m_gripperIndex, fingerJointIndices[i], controlArgs);
					}
				}
			}

			if (1)
			{
				m_robotSim.loadURDF("plane.urdf");
			}
			m_robotSim.setGravity(btVector3(0, 0, -10));
			m_robotSim.setNumSimulationSubSteps(4);
		}

		if ((m_options & eTWO_POINT_GRASP) != 0)
		{
			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(0, 0, .107);
				m_robotSim.loadURDF("cube_small.urdf", args);
			}

			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(0.068, 0.02, 0.11);
				m_robotSim.loadURDF("cube_gripper_left.urdf", args);

				b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
				controlArgs.m_targetVelocity = -0.1;
				controlArgs.m_maxTorqueValue = 10.0;
				controlArgs.m_kd = 1.;
				m_robotSim.setJointMotorControl(1, 0, controlArgs);
			}

			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(-0.068, 0.02, 0.11);
				m_robotSim.loadURDF("cube_gripper_right.urdf", args);

				b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
				controlArgs.m_targetVelocity = 0.1;
				controlArgs.m_maxTorqueValue = 10.0;
				controlArgs.m_kd = 1.;
				m_robotSim.setJointMotorControl(2, 0, controlArgs);
			}

			if (1)
			{
				m_robotSim.loadURDF("plane.urdf");
			}
			m_robotSim.setGravity(btVector3(0, 0, -10));
			m_robotSim.setNumSimulationSubSteps(4);
		}

		if ((m_options & eONE_MOTOR_GRASP) != 0)
		{
			m_robotSim.setNumSolverIterations(150);
			{
				SliderParams slider("Vertical velocity", &sGripperVerticalVelocity);
				slider.m_minVal = -2;
				slider.m_maxVal = 2;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}

			{
				SliderParams slider("Closing velocity", &sGripperClosingTargetVelocity);
				slider.m_minVal = -1;
				slider.m_maxVal = 1;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}
			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(0, -0.2, .47);
				args.m_startOrientation.setEulerZYX(SIMD_HALF_PI, 0, 0);
				m_robotSim.loadURDF("dinnerware/pan_tefal.urdf", args);
			}
			{
				b3RobotSimulatorLoadFileResults args;
				b3RobotSimulatorLoadFileResults results;
				m_robotSim.loadSDF("gripper/wsg50_one_motor_gripper_new.sdf", results);

				if (results.m_uniqueObjectIds.size() == 1)
				{
					m_gripperIndex = results.m_uniqueObjectIds[0];
					int numJoints = m_robotSim.getNumJoints(m_gripperIndex);
					b3Printf("numJoints = %d", numJoints);

					for (int i = 0; i < numJoints; i++)
					{
						b3JointInfo jointInfo;
						m_robotSim.getJointInfo(m_gripperIndex, i, &jointInfo);
						b3Printf("joint[%d].m_jointName=%s", i, jointInfo.m_jointName);
					}

					for (int i = 0; i < 8; i++)
					{
						b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
						controlArgs.m_maxTorqueValue = 0.0;
						m_robotSim.setJointMotorControl(m_gripperIndex, i, controlArgs);
					}
				}
			}

			if (1)
			{
				m_robotSim.loadURDF("plane.urdf");
			}
			m_robotSim.setGravity(btVector3(0, 0, -10));

			b3JointInfo revoluteJoint1;
			revoluteJoint1.m_parentFrame[0] = -0.055;
			revoluteJoint1.m_parentFrame[1] = 0;
			revoluteJoint1.m_parentFrame[2] = 0.02;
			revoluteJoint1.m_parentFrame[3] = 0;
			revoluteJoint1.m_parentFrame[4] = 0;
			revoluteJoint1.m_parentFrame[5] = 0;
			revoluteJoint1.m_parentFrame[6] = 1.0;
			revoluteJoint1.m_childFrame[0] = 0;
			revoluteJoint1.m_childFrame[1] = 0;
			revoluteJoint1.m_childFrame[2] = 0;
			revoluteJoint1.m_childFrame[3] = 0;
			revoluteJoint1.m_childFrame[4] = 0;
			revoluteJoint1.m_childFrame[5] = 0;
			revoluteJoint1.m_childFrame[6] = 1.0;
			revoluteJoint1.m_jointAxis[0] = 1.0;
			revoluteJoint1.m_jointAxis[1] = 0.0;
			revoluteJoint1.m_jointAxis[2] = 0.0;
			revoluteJoint1.m_jointType = ePoint2PointType;
			b3JointInfo revoluteJoint2;
			revoluteJoint2.m_parentFrame[0] = 0.055;
			revoluteJoint2.m_parentFrame[1] = 0;
			revoluteJoint2.m_parentFrame[2] = 0.02;
			revoluteJoint2.m_parentFrame[3] = 0;
			revoluteJoint2.m_parentFrame[4] = 0;
			revoluteJoint2.m_parentFrame[5] = 0;
			revoluteJoint2.m_parentFrame[6] = 1.0;
			revoluteJoint2.m_childFrame[0] = 0;
			revoluteJoint2.m_childFrame[1] = 0;
			revoluteJoint2.m_childFrame[2] = 0;
			revoluteJoint2.m_childFrame[3] = 0;
			revoluteJoint2.m_childFrame[4] = 0;
			revoluteJoint2.m_childFrame[5] = 0;
			revoluteJoint2.m_childFrame[6] = 1.0;
			revoluteJoint2.m_jointAxis[0] = 1.0;
			revoluteJoint2.m_jointAxis[1] = 0.0;
			revoluteJoint2.m_jointAxis[2] = 0.0;
			revoluteJoint2.m_jointType = ePoint2PointType;
			m_robotSim.createConstraint(1, 2, 1, 4, &revoluteJoint1);
			m_robotSim.createConstraint(1, 3, 1, 6, &revoluteJoint2);
		}

		if ((m_options & eGRASP_SOFT_BODY) != 0)
		{
			{
				SliderParams slider("Vertical velocity", &sGripperVerticalVelocity);
				slider.m_minVal = -2;
				slider.m_maxVal = 2;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}

			{
				SliderParams slider("Closing velocity", &sGripperClosingTargetVelocity);
				slider.m_minVal = -1;
				slider.m_maxVal = 1;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}
			{
				b3RobotSimulatorLoadFileResults results;
				m_robotSim.loadSDF("gripper/wsg50_one_motor_gripper_new.sdf", results);

				if (results.m_uniqueObjectIds.size() == 1)
				{
					m_gripperIndex = results.m_uniqueObjectIds[0];
					int numJoints = m_robotSim.getNumJoints(m_gripperIndex);
					b3Printf("numJoints = %d", numJoints);

					for (int i = 0; i < numJoints; i++)
					{
						b3JointInfo jointInfo;
						m_robotSim.getJointInfo(m_gripperIndex, i, &jointInfo);
						b3Printf("joint[%d].m_jointName=%s", i, jointInfo.m_jointName);
					}

					for (int i = 0; i < 8; i++)
					{
					b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
					controlArgs.m_maxTorqueValue = 0.0;
					m_robotSim.setJointMotorControl(m_gripperIndex, i, controlArgs);
					}
				}
			}
			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(0, 0, -0.2);
				args.m_startOrientation.setEulerZYX(0, 0, 0);
				m_robotSim.loadURDF("plane.urdf", args);
				}
				m_robotSim.setGravity(btVector3(0, 0, -10));
				b3RobotSimulatorLoadSoftBodyArgs args(0.1, 1, 0.02);
				args.m_startPosition.setValue(0, 0, 5);
				args.m_startOrientation.setValue(1, 0, 0, 1);
				m_robotSim.loadSoftBody("bunny.obj", args);

				b3JointInfo revoluteJoint1;
				revoluteJoint1.m_parentFrame[0] = -0.055;
				revoluteJoint1.m_parentFrame[1] = 0;
				revoluteJoint1.m_parentFrame[2] = 0.02;
				revoluteJoint1.m_parentFrame[3] = 0;
				revoluteJoint1.m_parentFrame[4] = 0;
				revoluteJoint1.m_parentFrame[5] = 0;
				revoluteJoint1.m_parentFrame[6] = 1.0;
				revoluteJoint1.m_childFrame[0] = 0;
				revoluteJoint1.m_childFrame[1] = 0;
				revoluteJoint1.m_childFrame[2] = 0;
				revoluteJoint1.m_childFrame[3] = 0;
				revoluteJoint1.m_childFrame[4] = 0;
				revoluteJoint1.m_childFrame[5] = 0;
				revoluteJoint1.m_childFrame[6] = 1.0;
				revoluteJoint1.m_jointAxis[0] = 1.0;
				revoluteJoint1.m_jointAxis[1] = 0.0;
				revoluteJoint1.m_jointAxis[2] = 0.0;
				revoluteJoint1.m_jointType = ePoint2PointType;
				b3JointInfo revoluteJoint2;
				revoluteJoint2.m_parentFrame[0] = 0.055;
				revoluteJoint2.m_parentFrame[1] = 0;
				revoluteJoint2.m_parentFrame[2] = 0.02;
				revoluteJoint2.m_parentFrame[3] = 0;
				revoluteJoint2.m_parentFrame[4] = 0;
				revoluteJoint2.m_parentFrame[5] = 0;
				revoluteJoint2.m_parentFrame[6] = 1.0;
				revoluteJoint2.m_childFrame[0] = 0;
				revoluteJoint2.m_childFrame[1] = 0;
				revoluteJoint2.m_childFrame[2] = 0;
				revoluteJoint2.m_childFrame[3] = 0;
				revoluteJoint2.m_childFrame[4] = 0;
				revoluteJoint2.m_childFrame[5] = 0;
				revoluteJoint2.m_childFrame[6] = 1.0;
				revoluteJoint2.m_jointAxis[0] = 1.0;
				revoluteJoint2.m_jointAxis[1] = 0.0;
				revoluteJoint2.m_jointAxis[2] = 0.0;
				revoluteJoint2.m_jointType = ePoint2PointType;
				m_robotSim.createConstraint(0, 2, 0, 4, &revoluteJoint1);
				m_robotSim.createConstraint(0, 3, 0, 6, &revoluteJoint2);
			}

			if ((m_options & eGRASP_DEFORMABLE_CLOTH) != 0)
			{
				m_robotSim.resetSimulation(RESET_USE_DEFORMABLE_WORLD);
			{
				SliderParams slider("Vertical velocity", &sGripperVerticalVelocity);
				slider.m_minVal = -2;
				slider.m_maxVal = 2;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}

			{
				SliderParams slider("Closing velocity", &sGripperClosingTargetVelocity);
				slider.m_minVal = -1;
				slider.m_maxVal = 1;
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}

			{
				b3RobotSimulatorLoadFileResults results;
				m_robotSim.loadSDF("gripper/wsg50_one_motor_gripper_new.sdf", results);

				if (results.m_uniqueObjectIds.size() == 1)
				{
					m_gripperIndex = results.m_uniqueObjectIds[0];
					int numJoints = m_robotSim.getNumJoints(m_gripperIndex);
					b3Printf("numJoints = %d", numJoints);

					for (int i = 0; i < numJoints; i++)
					{
						b3JointInfo jointInfo;
						m_robotSim.getJointInfo(m_gripperIndex, i, &jointInfo);
						b3Printf("joint[%d].m_jointName=%s", i, jointInfo.m_jointName);
					}

					for (int i = 0; i < 8; i++)
					{
						b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
						controlArgs.m_maxTorqueValue = 0.0;
						m_robotSim.setJointMotorControl(m_gripperIndex, i, controlArgs);
					}
				}
			}
			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(0, 0, -0.2);
				args.m_startOrientation.setEulerZYX(0, 0, 0);
				m_robotSim.loadURDF("plane.urdf", args);
			}
			m_robotSim.setGravity(btVector3(0, 0, -10));

			m_robotSim.setGravity(btVector3(0, 0, -10));
			b3RobotSimulatorLoadDeformableBodyArgs args(2, .01, 0.006);
			args.m_springElasticStiffness = 1;
			args.m_springDampingStiffness = .01;
			args.m_springBendingStiffness = .1;
			args.m_frictionCoeff = 10;
			args.m_useSelfCollision = false;
			args.m_useFaceContact = true;
			args.m_useBendingSprings = true;
			args.m_startPosition.setValue(0, 0, 0);
			args.m_startOrientation.setValue(0, 0, 1, 1);
			//m_robotSim.loadDeformableBody("flat_napkin.obj", args);

			b3JointInfo revoluteJoint1;
			revoluteJoint1.m_parentFrame[0] = -0.055;
			revoluteJoint1.m_parentFrame[1] = 0;
			revoluteJoint1.m_parentFrame[2] = 0.02;
			revoluteJoint1.m_parentFrame[3] = 0;
			revoluteJoint1.m_parentFrame[4] = 0;
			revoluteJoint1.m_parentFrame[5] = 0;
			revoluteJoint1.m_parentFrame[6] = 1.0;
			revoluteJoint1.m_childFrame[0] = 0;
			revoluteJoint1.m_childFrame[1] = 0;
			revoluteJoint1.m_childFrame[2] = 0;
			revoluteJoint1.m_childFrame[3] = 0;
			revoluteJoint1.m_childFrame[4] = 0;
			revoluteJoint1.m_childFrame[5] = 0;
			revoluteJoint1.m_childFrame[6] = 1.0;
			revoluteJoint1.m_jointAxis[0] = 1.0;
			revoluteJoint1.m_jointAxis[1] = 0.0;
			revoluteJoint1.m_jointAxis[2] = 0.0;
			revoluteJoint1.m_jointType = ePoint2PointType;
			b3JointInfo revoluteJoint2;
			revoluteJoint2.m_parentFrame[0] = 0.055;
			revoluteJoint2.m_parentFrame[1] = 0;
			revoluteJoint2.m_parentFrame[2] = 0.02;
			revoluteJoint2.m_parentFrame[3] = 0;
			revoluteJoint2.m_parentFrame[4] = 0;
			revoluteJoint2.m_parentFrame[5] = 0;
			revoluteJoint2.m_parentFrame[6] = 1.0;
			revoluteJoint2.m_childFrame[0] = 0;
			revoluteJoint2.m_childFrame[1] = 0;
			revoluteJoint2.m_childFrame[2] = 0;
			revoluteJoint2.m_childFrame[3] = 0;
			revoluteJoint2.m_childFrame[4] = 0;
			revoluteJoint2.m_childFrame[5] = 0;
			revoluteJoint2.m_childFrame[6] = 1.0;
			revoluteJoint2.m_jointAxis[0] = 1.0;
			revoluteJoint2.m_jointAxis[1] = 0.0;
			revoluteJoint2.m_jointAxis[2] = 0.0;
			revoluteJoint2.m_jointType = ePoint2PointType;
			m_robotSim.createConstraint(0, 2, 0, 4, &revoluteJoint1);
			m_robotSim.createConstraint(0, 3, 0, 6, &revoluteJoint2);
			m_robotSim.setNumSimulationSubSteps(2);
        }

		if ((m_options & eSOFTBODY_MULTIBODY_COUPLING) != 0)
		{
			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(-0.5, 0, 0.1);
				args.m_startOrientation.setEulerZYX(0, 0, 0);
				args.m_forceOverrideFixedBase = true;
				args.m_useMultiBody = true;
				int kukaId = m_robotSim.loadURDF("kuka_iiwa/model.urdf", args);

				int numJoints = m_robotSim.getNumJoints(kukaId);
				b3Printf("numJoints = %d", numJoints);

				for (int i = 0; i < numJoints; i++)
				{
					b3JointInfo jointInfo;
					m_robotSim.getJointInfo(kukaId, i, &jointInfo);
					b3Printf("joint[%d].m_jointName=%s", i, jointInfo.m_jointName);
					b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
					controlArgs.m_maxTorqueValue = 500.0;
					m_robotSim.setJointMotorControl(kukaId, i, controlArgs);
				}
			}
			{
				b3RobotSimulatorLoadUrdfFileArgs args;
				args.m_startPosition.setValue(0, 0, 0);
				args.m_startOrientation.setEulerZYX(0, 0, 0);
				args.m_forceOverrideFixedBase = true;
				args.m_useMultiBody = false;
				m_robotSim.loadURDF("plane.urdf", args);
			}
			m_robotSim.setGravity(btVector3(0, 0, -10));
			b3RobotSimulatorLoadSoftBodyArgs args(0.3, 10, 0.1);
			m_robotSim.loadSoftBody("bunny.obj", args);
		}
	}
	virtual void exitPhysics()
	{
		m_robotSim.disconnect();
	}
	virtual void stepSimulation(float deltaTime)
	{
		if ((m_options & eGRIPPER_GRASP) != 0)
		{
			if ((m_gripperIndex >= 0))
			{
				int fingerJointIndices[3] = {0, 1, 3};
				double fingerTargetVelocities[3] = {sGripperVerticalVelocity, sGripperClosingTargetVelocity, -sGripperClosingTargetVelocity};
				double maxTorqueValues[3] = {40.0, 50.0, 50.0};
				for (int i = 0; i < 3; i++)
				{
					b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
					controlArgs.m_targetVelocity = fingerTargetVelocities[i];
					controlArgs.m_maxTorqueValue = maxTorqueValues[i];
					controlArgs.m_kd = 1.;
					m_robotSim.setJointMotorControl(m_gripperIndex, fingerJointIndices[i], controlArgs);
				}
			}
		}

		if ((m_options & eONE_MOTOR_GRASP) != 0)
		{
			int fingerJointIndices[2] = {0, 1};
			double fingerTargetVelocities[2] = {sGripperVerticalVelocity, sGripperClosingTargetVelocity};
			double maxTorqueValues[2] = {800.0, 800.0};
			for (int i = 0; i < 2; i++)
			{
				b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
				controlArgs.m_targetVelocity = fingerTargetVelocities[i];
				controlArgs.m_maxTorqueValue = maxTorqueValues[i];
				controlArgs.m_kd = 1.;
				m_robotSim.setJointMotorControl(m_gripperIndex, fingerJointIndices[i], controlArgs);
			}
		}

		if ((m_options & eGRASP_SOFT_BODY) != 0)
		{
			int fingerJointIndices[2] = {0, 1};
			double fingerTargetVelocities[2] = {sGripperVerticalVelocity, sGripperClosingTargetVelocity};
			double maxTorqueValues[2] = {50.0, 10.0};
			for (int i = 0; i < 2; i++)
			{
				b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
				controlArgs.m_targetVelocity = fingerTargetVelocities[i];
				controlArgs.m_maxTorqueValue = maxTorqueValues[i];
				controlArgs.m_kd = 1.;
				m_robotSim.setJointMotorControl(m_gripperIndex, fingerJointIndices[i], controlArgs);
			}
		}
        
		if ((m_options & eGRASP_DEFORMABLE_CLOTH) != 0)
		{
			int fingerJointIndices[2] = {0, 1};
			double fingerTargetVelocities[2] = {sGripperVerticalVelocity, sGripperClosingTargetVelocity};
			double maxTorqueValues[2] = {250.0, 50.0};
			for (int i = 0; i < 2; i++)
			{
				b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
				controlArgs.m_targetVelocity = fingerTargetVelocities[i];
				controlArgs.m_maxTorqueValue = maxTorqueValues[i];
				controlArgs.m_kd = 1.;
				m_robotSim.setJointMotorControl(m_gripperIndex, fingerJointIndices[i], controlArgs);
			}
		}

		if ((m_options & eSOFTBODY_MULTIBODY_COUPLING) != 0)
		{
			float dt = deltaTime;
			btClamp(dt, 0.0001f, 0.01f);

			m_time += dt;
			m_targetPos.setValue(0, 0, 0.5 + 0.2 * b3Cos(m_time));
			m_targetOri.setValue(0, 1.0, 0, 0);

			int numJoints = m_robotSim.getNumJoints(0);

			if (numJoints == 7)
			{
				double q_current[7] = {0, 0, 0, 0, 0, 0, 0};

				b3JointStates2 jointStates;

				if (m_robotSim.getJointStates(0, jointStates))
				{
					//skip the base positions (7 values)
					b3Assert(7 + numJoints == jointStates.m_numDegreeOfFreedomQ);
					for (int i = 0; i < numJoints; i++)
					{
						q_current[i] = jointStates.m_actualStateQ[i + 7];
					}
				}
				// compute body position and orientation
				b3LinkState linkState;
				bool computeVelocity = true;
				bool computeForwardKinematics = true;
				m_robotSim.getLinkState(0, 6, computeVelocity, computeForwardKinematics, &linkState);

				m_worldPos.setValue(linkState.m_worldLinkFramePosition[0], linkState.m_worldLinkFramePosition[1], linkState.m_worldLinkFramePosition[2]);
				m_worldOri.setValue(linkState.m_worldLinkFrameOrientation[0], linkState.m_worldLinkFrameOrientation[1], linkState.m_worldLinkFrameOrientation[2], linkState.m_worldLinkFrameOrientation[3]);

				b3Vector3DoubleData targetPosDataOut;
				m_targetPos.serializeDouble(targetPosDataOut);
				b3Vector3DoubleData worldPosDataOut;
				m_worldPos.serializeDouble(worldPosDataOut);
				b3Vector3DoubleData targetOriDataOut;
				m_targetOri.serializeDouble(targetOriDataOut);
				b3Vector3DoubleData worldOriDataOut;
				m_worldOri.serializeDouble(worldOriDataOut);

				b3RobotSimulatorInverseKinematicArgs ikargs;
				b3RobotSimulatorInverseKinematicsResults ikresults;

				ikargs.m_bodyUniqueId = 0;
				//			ikargs.m_currentJointPositions = q_current;
				//			ikargs.m_numPositions = 7;
				ikargs.m_endEffectorTargetPosition[0] = targetPosDataOut.m_floats[0];
				ikargs.m_endEffectorTargetPosition[1] = targetPosDataOut.m_floats[1];
				ikargs.m_endEffectorTargetPosition[2] = targetPosDataOut.m_floats[2];

				ikargs.m_flags |= B3_HAS_IK_TARGET_ORIENTATION /* + B3_HAS_NULL_SPACE_VELOCITY*/;

				ikargs.m_endEffectorTargetOrientation[0] = targetOriDataOut.m_floats[0];
				ikargs.m_endEffectorTargetOrientation[1] = targetOriDataOut.m_floats[1];
				ikargs.m_endEffectorTargetOrientation[2] = targetOriDataOut.m_floats[2];
				ikargs.m_endEffectorTargetOrientation[3] = targetOriDataOut.m_floats[3];
				ikargs.m_endEffectorLinkIndex = 6;

				// Settings based on default KUKA arm setting
				ikargs.m_lowerLimits.resize(numJoints);
				ikargs.m_upperLimits.resize(numJoints);
				ikargs.m_jointRanges.resize(numJoints);
				ikargs.m_restPoses.resize(numJoints);
				ikargs.m_lowerLimits[0] = -2.32;
				ikargs.m_lowerLimits[1] = -1.6;
				ikargs.m_lowerLimits[2] = -2.32;
				ikargs.m_lowerLimits[3] = -1.6;
				ikargs.m_lowerLimits[4] = -2.32;
				ikargs.m_lowerLimits[5] = -1.6;
				ikargs.m_lowerLimits[6] = -2.4;
				ikargs.m_upperLimits[0] = 2.32;
				ikargs.m_upperLimits[1] = 1.6;
				ikargs.m_upperLimits[2] = 2.32;
				ikargs.m_upperLimits[3] = 1.6;
				ikargs.m_upperLimits[4] = 2.32;
				ikargs.m_upperLimits[5] = 1.6;
				ikargs.m_upperLimits[6] = 2.4;
				ikargs.m_jointRanges[0] = 5.8;
				ikargs.m_jointRanges[1] = 4;
				ikargs.m_jointRanges[2] = 5.8;
				ikargs.m_jointRanges[3] = 4;
				ikargs.m_jointRanges[4] = 5.8;
				ikargs.m_jointRanges[5] = 4;
				ikargs.m_jointRanges[6] = 6;
				ikargs.m_restPoses[0] = 0;
				ikargs.m_restPoses[1] = 0;
				ikargs.m_restPoses[2] = 0;
				ikargs.m_restPoses[3] = SIMD_HALF_PI;
				ikargs.m_restPoses[4] = 0;
				ikargs.m_restPoses[5] = -SIMD_HALF_PI * 0.66;
				ikargs.m_restPoses[6] = 0;
				ikargs.m_numDegreeOfFreedom = numJoints;

				if (m_robotSim.calculateInverseKinematics(ikargs, ikresults))
				{
					//copy the IK result to the desired state of the motor/actuator
					for (int i = 0; i < numJoints; i++)
					{
						b3RobotSimulatorJointMotorArgs t(CONTROL_MODE_POSITION_VELOCITY_PD);
						t.m_targetPosition = ikresults.m_calculatedJointPositions[i];
						t.m_maxTorqueValue = 100.0;
						t.m_kp = 1.0;
						t.m_targetVelocity = 0;
						t.m_kd = 1.0;
						m_robotSim.setJointMotorControl(0, i, t);
					}
				}
			}
		}

		m_robotSim.stepSimulation();
	}
	virtual void renderScene()
	{
		m_robotSim.renderScene();

		//m_app->m_renderer->renderScene();
	}

	virtual bool mouseMoveCallback(float x, float y)
	{
		return m_robotSim.mouseMoveCallback(x, y);
	}
	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return m_robotSim.mouseButtonCallback(button, state, x, y);
	}
	virtual bool keyboardCallback(int key, int state)
	{
		return false;
	}

	virtual void resetCamera()
	{
		float dist = 1.5;
		float pitch = -10;
		float yaw = 18;
		float targetPos[3] = {-0.2, 0.8, 0.3};
		if (m_app->m_renderer && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0], targetPos[1], targetPos[2]);
		}
	}
};

class CommonExampleInterface* GripperGraspExampleCreateFunc(struct CommonExampleOptions& options)
{
	return new GripperGraspExample(options.m_guiHelper, options.m_option);
}
