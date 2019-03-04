
#include "BoxStack.h"

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
#include "../Utils/b3Clock.h"

class BoxStackExample : public CommonExampleInterface
{
	CommonGraphicsApp* m_app;
	GUIHelperInterface* m_guiHelper;
	b3RobotSimulatorClientAPI m_robotSim;
	int m_options;

public:
	BoxStackExample(GUIHelperInterface* helper, int options)
		: m_app(helper->getAppInterface()),
		  m_guiHelper(helper),
		  m_options(options)
	{
		m_app->setUpAxis(2);
	}

	virtual ~BoxStackExample()
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
		b3RobotSimulatorChangeDynamicsArgs dynamicsArgs;
		int massRatio = 4;
		int mass = 1;
		for (int i = 0; i < 8; i++)
		{
			args.m_startPosition.setValue(0, 0, i * .06);
			int boxIdx = m_robotSim.loadURDF("cube_small.urdf", args);
			dynamicsArgs.m_mass = mass;
			m_robotSim.changeDynamics(boxIdx, -1, dynamicsArgs);
			mass *= massRatio;
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
		float dist = 1.5;
		float pitch = -10;
		float yaw = 18;
		float targetPos[3] = {-0.2, 0.8, 0.3};

		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);

		if (m_app->m_renderer && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0], targetPos[1], targetPos[2]);
		}
	}
};

class CommonExampleInterface* BoxStackExampleCreateFunc(struct CommonExampleOptions& options)
{
	return new BoxStackExample(options.m_guiHelper, options.m_option);
}
