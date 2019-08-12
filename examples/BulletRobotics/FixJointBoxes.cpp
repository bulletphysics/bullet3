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

static btScalar numSolverIterations = 1000;
static btScalar solverId = 0;

class FixJointBoxes : public CommonExampleInterface
{
	GUIHelperInterface* m_guiHelper;
	b3RobotSimulatorClientAPI m_robotSim;
	int m_options;
	b3RobotSimulatorSetPhysicsEngineParameters physicsArgs;
	int solver;

	const size_t numCubes;
	std::vector<int> cubeIds;

public:
	FixJointBoxes(GUIHelperInterface* helper, int options)
		: m_guiHelper(helper),
		  m_options(options),
		  numCubes(30),
		  cubeIds(numCubes, 0),
		  solver(solverId)
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

		{
			b3RobotSimulatorLoadUrdfFileArgs args;
			b3RobotSimulatorChangeDynamicsArgs dynamicsArgs;

			for (int i = 0; i < numCubes; i++)
			{
				args.m_forceOverrideFixedBase = (i == 0);
				args.m_startPosition.setValue(0, i * 0.05, 1);
				cubeIds[i] = m_robotSim.loadURDF("cube_small.urdf", args);

				b3RobotJointInfo jointInfo;

				jointInfo.m_parentFrame[1] = -0.025;
				jointInfo.m_childFrame[1] = 0.025;

				if (i > 0)
				{
					m_robotSim.createConstraint(cubeIds[i], -1, cubeIds[i - 1], -1, &jointInfo);
					m_robotSim.setCollisionFilterGroupMask(cubeIds[i], -1, 0, 0);
				}

				m_robotSim.loadURDF("plane.urdf");
			}
		}

		{
			SliderParams slider("Direct solver", &solverId);
			slider.m_minVal = 0;
			slider.m_maxVal = 1;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}
		{
			SliderParams slider("numSolverIterations", &numSolverIterations);
			slider.m_minVal = 50;
			slider.m_maxVal = 1e4;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}

		physicsArgs.m_defaultGlobalCFM = 1e-6;
		m_robotSim.setPhysicsEngineParameter(physicsArgs);

		m_robotSim.setGravity(btVector3(0, 0, -10));
		m_robotSim.setNumSolverIterations((int)numSolverIterations);
	}

	virtual void exitPhysics()
	{
		m_robotSim.disconnect();
	}

	void resetCubePosition()
	{
		for (int i = 0; i < numCubes; i++)
		{
			btVector3 pos(0, i * (btScalar)0.05, 1);
			btQuaternion quar(0, 0, 0, 1);
			m_robotSim.resetBasePositionAndOrientation(cubeIds[i], pos, quar);
		}
	}
	virtual void stepSimulation(float deltaTime)
	{
		int newSolver = (int)(solverId + 0.5);
		if (newSolver != solver)
		{
			printf("Switching solver, new %d, old %d\n", newSolver, solver);
			solver = newSolver;
			resetCubePosition();
			if (solver)
			{
				physicsArgs.m_constraintSolverType = eConstraintSolverLCP_DANTZIG;
			}
			else
			{
				physicsArgs.m_constraintSolverType = eConstraintSolverLCP_SI;
			}

			m_robotSim.setPhysicsEngineParameter(physicsArgs);
		}
		m_robotSim.setNumSolverIterations((int)numSolverIterations);
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
