
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
#include <string>

#include "b3RobotSimAPI.h"
#include "../Utils/b3Clock.h"

static btScalar sGripperVerticalVelocity = -0.2f;
static btScalar sGripperClosingTargetVelocity = 0.5f;

class GripperGraspExample : public CommonExampleInterface
{
    CommonGraphicsApp* m_app;
	GUIHelperInterface* m_guiHelper;
	b3RobotSimAPI m_robotSim;
	int m_options;
	int m_r2d2Index;
    int m_gripperIndex;
    
    float m_x;
    float m_y;
    float m_z;
	b3AlignedObjectArray<int> m_movingInstances;
	enum
	{
		numCubesX = 20,
		numCubesY = 20
	};
public:
    
    GripperGraspExample(GUIHelperInterface* helper, int options)
    :m_app(helper->getAppInterface()),
	m_guiHelper(helper),
	m_options(options),
	m_r2d2Index(-1),
    m_gripperIndex(-1),
    m_x(0),
    m_y(0),
	m_z(0)
    {
		m_app->setUpAxis(2);
    }
    virtual ~GripperGraspExample()
    {
        m_app->m_renderer->enableBlend(false);
    }

    
    virtual void physicsDebugDraw(int debugDrawMode)
    {
        
    }
    virtual void    initPhysics()
    {
	bool connected = m_robotSim.connect(m_guiHelper);
	b3Printf("robotSim connected = %d",connected);
	
        {
            
            {
                SliderParams slider("Vertical velocity",&sGripperVerticalVelocity);
                slider.m_minVal=-2;
                slider.m_maxVal=2;
                m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
            }
            
            {
                SliderParams slider("Closing velocity",&sGripperClosingTargetVelocity
                                    );
                slider.m_minVal=-1;
                slider.m_maxVal=1;
                m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
            }
            
            {
                b3RobotSimLoadFileArgs args("");
                args.m_fileName = "gripper/wsg50_with_r2d2_gripper.sdf";
                args.m_fileType = B3_SDF_FILE;
                args.m_useMultiBody = true;
                b3RobotSimLoadFileResults results;
                
                if (m_robotSim.loadFile(args, results) && results.m_uniqueObjectIds.size()==1)
                {
                    
                    m_gripperIndex = results.m_uniqueObjectIds[0];
                    int numJoints = m_robotSim.getNumJoints(m_gripperIndex);
                    b3Printf("numJoints = %d",numJoints);
                    
                    for (int i=0;i<numJoints;i++)
                    {
                        b3JointInfo jointInfo;
                        m_robotSim.getJointInfo(m_gripperIndex,i,&jointInfo);
                        b3Printf("joint[%d].m_jointName=%s",i,jointInfo.m_jointName);
                    }
                    
                    /*
                    int fingerJointIndices[2]={1,3};
                    double fingerTargetPositions[2]={-0.04,0.04};
                    for (int i=0;i<2;i++)
                    {
                        b3JointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
                        controlArgs.m_targetPosition = fingerTargetPositions[i];
                        controlArgs.m_kp = 5.0;
                        controlArgs.m_kd = 3.0;
                        controlArgs.m_maxTorqueValue = 1.0;
                        m_robotSim.setJointMotorControl(m_gripperIndex,fingerJointIndices[i],controlArgs);
                    }
                    */
                    int fingerJointIndices[3]={0,1,3};
                    double fingerTargetVelocities[3]={-0.2,.5,-.5};
                    double maxTorqueValues[3]={40.0,50.0,50.0};
                    for (int i=0;i<3;i++)
                    {
                        b3JointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
                        controlArgs.m_targetVelocity = fingerTargetVelocities[i];
                        controlArgs.m_maxTorqueValue = maxTorqueValues[i];
                        controlArgs.m_kd = 1.;
                        m_robotSim.setJointMotorControl(m_gripperIndex,fingerJointIndices[i],controlArgs);
                    }
                }
            }
            {
                b3RobotSimLoadFileArgs args("");
                b3RobotSimLoadFileResults results;
                args.m_fileName = "cube_small.urdf";
                args.m_startPosition.setValue(0,0,.107);
                args.m_startOrientation.setEulerZYX(0,0,0);
                args.m_useMultiBody = false;
                m_robotSim.loadFile(args,results);
            }
            if (1)
            {
                b3RobotSimLoadFileArgs args("");
                args.m_fileName = "plane.urdf";
                args.m_startPosition.setValue(0,0,0);
                args.m_startOrientation.setEulerZYX(0,0,0);
                args.m_forceOverrideFixedBase = true;
                args.m_useMultiBody = true;
                b3RobotSimLoadFileResults results;
                m_robotSim.loadFile(args,results);
                
            }
            m_robotSim.setGravity(b3MakeVector3(0,0,-10));
            
        }

	
		
    }
    virtual void    exitPhysics()
    {
		m_robotSim.disconnect();
    }
    virtual void	stepSimulation(float deltaTime)
	{
        if ((m_gripperIndex>=0))
        {
            int fingerJointIndices[3]={0,1,3};
            double fingerTargetVelocities[3]={sGripperVerticalVelocity,sGripperClosingTargetVelocity
                ,-sGripperClosingTargetVelocity
            };
            double maxTorqueValues[3]={40.0,50.0,50.0};
            for (int i=0;i<3;i++)
            {
                b3JointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
                controlArgs.m_targetVelocity = fingerTargetVelocities[i];
                controlArgs.m_maxTorqueValue = maxTorqueValues[i];
                controlArgs.m_kd = 1.;
                m_robotSim.setJointMotorControl(m_gripperIndex,fingerJointIndices[i],controlArgs);
            }
        }
	m_robotSim.stepSimulation();
    }
    virtual void	renderScene()
    {
		m_robotSim.renderScene();

		//m_app->m_renderer->renderScene();
    }

	
    virtual void	physicsDebugDraw()
    {
      	
    }
    virtual bool	mouseMoveCallback(float x,float y)
    {
		return false;   
    }
    virtual bool	mouseButtonCallback(int button, int state, float x, float y)
    {
        return false;   
    }
    virtual bool	keyboardCallback(int key, int state)
    {
        return false;   
    }
    
	virtual void resetCamera()
	{
		float dist = 1.5;
        float pitch = 12;
        float yaw = -10;
		float targetPos[3]={-0.2,0.8,0.3};
		if (m_app->m_renderer  && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0],targetPos[1],targetPos[2]);
		}
	}

};


class	CommonExampleInterface*    GripperGraspExampleCreateFunc(struct CommonExampleOptions& options)
{
	return new GripperGraspExample(options.m_guiHelper, options.m_option);
}



