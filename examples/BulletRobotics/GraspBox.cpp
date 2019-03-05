
#include "GraspBox.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
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

static btScalar x = 0.401f;
static btScalar y = 0.025f;
static btScalar z = 0.44f;

class GraspBox : public CommonExampleInterface
{
	GUIHelperInterface* m_guiHelper;
	b3RobotSimulatorClientAPI m_robotSim;
	int m_options;

	int m_armIndex;
	int boxId;

public:
	GraspBox(GUIHelperInterface* helper, int options)
		: m_guiHelper(helper),
		  m_options(options)
	{
	}

	virtual ~GraspBox()
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

		b3Printf("robotSim connected = %d", connected);

		m_robotSim.configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, 0);
		m_robotSim.configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0);
		m_robotSim.configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0);

		double cubePos[3] = {0.41, 0, 0.5};
		double cubeHalfLength = 0.025;

		{
			SliderParams slider("position x", &x);
			slider.m_minVal = 0;
			slider.m_maxVal = 1;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}

		{
			SliderParams slider("position y", &y);
			slider.m_minVal = 0;
			slider.m_maxVal = 1;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}
		{
			SliderParams slider("position z", &z);
			slider.m_minVal = 0;
			slider.m_maxVal = 1;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}

		{
			b3RobotSimulatorLoadFileResults results;
			m_robotSim.loadSDF("kuka_iiwa/kuka_with_gripper.sdf", results);
			if (results.m_uniqueObjectIds.size() == 1)
			{
				m_armIndex = results.m_uniqueObjectIds[0];
				int numJoints = m_robotSim.getNumJoints(m_armIndex);
				b3Printf("numJoints = %d", numJoints);

				for (int i = 0; i < numJoints; i++)
				{
					b3RobotJointInfo jointInfo;
					m_robotSim.getJointInfo(m_armIndex, i, &jointInfo);
					b3Printf("joint[%d].m_jointName=%s", i, jointInfo.m_jointName);
				}

				//adjust arm position
				m_robotSim.resetJointState(m_armIndex, 3, -SIMD_HALF_PI);
				m_robotSim.resetJointState(m_armIndex, 5, SIMD_HALF_PI);

				// adjust gripper position
				m_robotSim.resetJointState(m_armIndex, 8, -0.12);
				m_robotSim.resetJointState(m_armIndex, 11, 0.12);

				m_robotSim.resetJointState(m_armIndex, 10, -0.12);
				m_robotSim.resetJointState(m_armIndex, 13, 0.12);

				// {
				//   b3RobotJointInfo jointInfo;
				//   m_robotSim.getJointInfo(m_armIndex, 8, &jointInfo);
				//   jointInfo.m_jointType = eGearType;
				//   m_robotSim.createConstraint(m_armIndex, , cubeIds[i - 1], -1, &jointInfo);
				// }
				// for(int i = 0; i<14; i++){
				//   b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
				//   controlArgs.m_maxTorqueValue = 0.0;
				//   m_robotSim.setJointMotorControl(m_armIndex, i, controlArgs);
				// }
			}
		}

		{
			b3RobotSimulatorLoadUrdfFileArgs args;
			args.m_startPosition.setValue(x, y, z);
			boxId = m_robotSim.loadURDF("cube_small.urdf", args);
		}
		m_robotSim.loadURDF("plane.urdf");
		m_robotSim.setGravity(btVector3(0, 0, -10));
	}

	virtual void exitPhysics()
	{
		m_robotSim.disconnect();
	}

	virtual void stepSimulation(float deltaTime)
	{
		btScalar sGripperVerticalVelocity = 0.f;
		btScalar sGripperClosingTargetVelocity = -0.7f;

		int fingerJointIndices[4] = {8, 10, 11, 13};
		double fingerTargetVelocities[4] = {0.4, 0.4, -0.4, -0.4};

		double maxTorqueValues[4] = {20.0, 20.0, 20.0, 20.0};
		for (int i = 0; i < 4; i++)
		{
			b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
			controlArgs.m_targetVelocity = fingerTargetVelocities[i];
			controlArgs.m_maxTorqueValue = maxTorqueValues[i];
			controlArgs.m_kd = 1.;
			m_robotSim.setJointMotorControl(m_armIndex, fingerJointIndices[i], controlArgs);
		}

		m_robotSim.stepSimulation();
	}
	virtual void renderScene()
	{
		m_robotSim.renderScene();
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
		float dist = 0.8;
		float pitch = 0;
		float yaw = 0;
		float targetPos[3] = {.4, 0.6, 0.45};

		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

class CommonExampleInterface* GraspBoxCreateFunc(struct CommonExampleOptions& options)
{
	return new GraspBox(options.m_guiHelper, options.m_option);
}
