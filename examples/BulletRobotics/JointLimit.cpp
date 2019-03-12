
#include "JointLimit.h"

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

#include "../RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"

class JointLimit : public CommonExampleInterface
{
	GUIHelperInterface* m_guiHelper;
	b3RobotSimulatorClientAPI m_robotSim;
	int m_options;

public:
	JointLimit(GUIHelperInterface* helper, int options)
		: m_guiHelper(helper),
		  m_options(options)
	{
	}

	virtual ~JointLimit()
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

		b3RobotSimulatorSetPhysicsEngineParameters physicsArgs;
		physicsArgs.m_constraintSolverType = eConstraintSolverLCP_DANTZIG;

		physicsArgs.m_defaultGlobalCFM = 1e-6;

		m_robotSim.setNumSolverIterations(10);

		b3RobotSimulatorLoadUrdfFileArgs loadArgs;
		int humanoid = m_robotSim.loadURDF("test_joints_MB.urdf", loadArgs);

		b3RobotSimulatorChangeDynamicsArgs dynamicsArgs;
		dynamicsArgs.m_linearDamping = 0;
		dynamicsArgs.m_angularDamping = 0;
		m_robotSim.changeDynamics(humanoid, -1, dynamicsArgs);

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
		float dist = 3;
		float pitch = -10;
		float yaw = 18;
		float targetPos[3] = {0.6, 0.8, 0.3};

		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

class CommonExampleInterface* JointLimitCreateFunc(struct CommonExampleOptions& options)
{
	return new JointLimit(options.m_guiHelper, options.m_option);
}
