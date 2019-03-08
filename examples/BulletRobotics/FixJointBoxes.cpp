
#include "FixJointBoxes.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../SharedMemory/PhysicsServerSharedMemory.h"
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/SharedMemoryPublic.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <string>
#include <vector>
#include "../RobotSimulator/b3RobotSimulatorClientAPI.h"

class FixJointBoxes : public CommonExampleInterface
{
	GUIHelperInterface* m_guiHelper;
	b3RobotSimulatorClientAPI m_robotSim;
	int m_options;

public:
	FixJointBoxes(GUIHelperInterface* helper, int options)
		: m_guiHelper(helper),
		  m_options(options)
	{
	}

	virtual ~FixJointBoxes()
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

		b3RobotSimulatorLoadUrdfFileArgs args;

		size_t numCubes = 10;
		std::vector<int> cubeIds(numCubes, 0);
		for (int i = 0; i < numCubes; i++)
		{
			args.m_forceOverrideFixedBase = (i == 0);
			args.m_startPosition.setValue(0, i * 0.05, 1);
			cubeIds[i] = m_robotSim.loadURDF("cube_small.urdf", args);

			b3RobotJointInfo jointInfo;

			jointInfo.m_parentFrame[1] = -0.025;
			jointInfo.m_childFrame[1] = 0.025;
			jointInfo.m_jointType = eFixedType;
			// jointInfo.m_jointType = ePoint2PointType;
			// jointInfo.m_jointType =	ePrismaticType;

			if (i > 0)
			{
				m_robotSim.createConstraint(cubeIds[i], -1, cubeIds[i - 1], -1, &jointInfo);
			}

			m_robotSim.loadURDF("plane.urdf");
		}
	}

	virtual void exitPhysics()
	{
		m_robotSim.disconnect();
	}
	virtual void stepSimulation(float deltaTime)
	{
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
		 float dist = 1;
		 float pitch = -20;
		 float yaw = -30;
		 float targetPos[3] = {0, 0.2, 0.5};
		 m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

class CommonExampleInterface* FixJointBoxesCreateFunc(struct CommonExampleOptions& options)
{
	return new FixJointBoxes(options.m_guiHelper, options.m_option);
}
