
#include "R2D2GraspExample.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../SharedMemory/PhysicsServerSharedMemory.h"
#include "../SharedMemory/PhysicsClientC_API.h"
#include <string>

#include "b3RobotSimAPI.h"
#include "../Utils/b3Clock.h"

///quick demo showing the right-handed coordinate system and positive rotations around each axis
class R2D2GraspExample : public CommonExampleInterface
{
    CommonGraphicsApp* m_app;
	GUIHelperInterface* m_guiHelper;
	b3RobotSimAPI m_robotSim;
	int m_options;
	int m_r2d2Index;
	
	b3AlignedObjectArray<int> m_movingInstances;
	enum
	{
		numCubesX = 20,
		numCubesY = 20
	};
public:
    
    R2D2GraspExample(GUIHelperInterface* helper, int options)
    :m_app(helper->getAppInterface()),
	m_guiHelper(helper),
	m_options(options),
	m_r2d2Index(-1)
    {
		m_app->setUpAxis(2);
    }
    virtual ~R2D2GraspExample()
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
		
		
		if ((m_options & eROBOTIC_LEARN_GRASP)!=0)
		{
			{
				b3RobotSimLoadFileArgs args("");
				args.m_fileName = "r2d2.urdf";
				args.m_startPosition.setValue(0,0,.5);
				b3RobotSimLoadFileResults results;
				if (m_robotSim.loadFile(args, results) && results.m_uniqueObjectIds.size()==1)
				{
					m_r2d2Index = results.m_uniqueObjectIds[0];
					int numJoints = m_robotSim.getNumJoints(m_r2d2Index);
					b3Printf("numJoints = %d",numJoints);

					for (int i=0;i<numJoints;i++)
					{
						b3JointInfo jointInfo;
						m_robotSim.getJointInfo(m_r2d2Index,i,&jointInfo);
						b3Printf("joint[%d].m_jointName=%s",i,jointInfo.m_jointName);
					}
					int wheelJointIndices[4]={2,3,6,7};
					int wheelTargetVelocities[4]={-10,-10,-10,-10};
					for (int i=0;i<4;i++)
					{
						b3JointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
						controlArgs.m_targetVelocity = wheelTargetVelocities[i];
						controlArgs.m_maxTorqueValue = 1e30;
						m_robotSim.setJointMotorControl(m_r2d2Index,wheelJointIndices[i],controlArgs);
					}
				}
			}
			{
				b3RobotSimLoadFileArgs args("");
				args.m_fileName = "kiva_shelf/model.sdf";
                args.m_forceOverrideFixedBase = true;
				args.m_fileType = B3_SDF_FILE;
                args.m_startOrientation = b3Quaternion(0,0,0,1);
                b3RobotSimLoadFileResults results;
                m_robotSim.loadFile(args,results);
			}
			{
				b3RobotSimLoadFileArgs args("");
				args.m_fileName = "plane.urdf";
				args.m_startPosition.setValue(0,0,0);
				args.m_forceOverrideFixedBase = true;
				b3RobotSimLoadFileResults results;
				m_robotSim.loadFile(args,results);
				m_robotSim.setGravity(b3MakeVector3(0,0,-10));
			}
	
		}

		if ((m_options & eROBOTIC_LEARN_COMPLIANT_CONTACT)!=0)
		{
			b3RobotSimLoadFileArgs args("");
			b3RobotSimLoadFileResults results;
			{
				args.m_fileName = "cube_soft.urdf";
				args.m_startPosition.setValue(0,0,2.5);
				args.m_startOrientation.setEulerZYX(0,0.2,0);
				m_robotSim.loadFile(args,results);
			}
			{
				args.m_fileName = "cube_no_friction.urdf";
				args.m_startPosition.setValue(0,2,2.5);
				args.m_startOrientation.setEulerZYX(0,0.2,0);
				m_robotSim.loadFile(args,results);
			}
			{
				b3RobotSimLoadFileArgs args("");
				args.m_fileName = "plane.urdf";
				args.m_startPosition.setValue(0,0,0);
				args.m_startOrientation.setEulerZYX(0,0.2,0);
				args.m_forceOverrideFixedBase = true;
				b3RobotSimLoadFileResults results;
				m_robotSim.loadFile(args,results);
				m_robotSim.setGravity(b3MakeVector3(0,0,-10));
			}
		}
	
        if ((m_options & eROBOTIC_LEARN_ROLLING_FRICTION)!=0)
        {
            b3RobotSimLoadFileArgs args("");
            b3RobotSimLoadFileResults results;
            {
                args.m_fileName = "sphere2_rolling_friction.urdf";
                args.m_startPosition.setValue(0,0,2.5);
                args.m_startOrientation.setEulerZYX(0,0,0);
                args.m_useMultiBody = true;
                m_robotSim.loadFile(args,results);
            }
            {
                args.m_fileName = "sphere2.urdf";
                args.m_startPosition.setValue(0,2,2.5);
                args.m_startOrientation.setEulerZYX(0,0,0);
                args.m_useMultiBody = true;
                m_robotSim.loadFile(args,results);
            }
            {
                b3RobotSimLoadFileArgs args("");
                args.m_fileName = "plane.urdf";
                args.m_startPosition.setValue(0,0,0);
                args.m_startOrientation.setEulerZYX(0,0.2,0);
                args.m_useMultiBody = true;
                args.m_forceOverrideFixedBase = true;
                b3RobotSimLoadFileResults results;
                m_robotSim.loadFile(args,results);
                m_robotSim.setGravity(b3MakeVector3(0,0,-10));
            }
        }

		
    }
    virtual void    exitPhysics()
    {
		m_robotSim.disconnect();
    }
    virtual void	stepSimulation(float deltaTime)
	{
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
		float dist = 3;
		float pitch = -75;
		float yaw = 30;
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


class	CommonExampleInterface*    R2D2GraspExampleCreateFunc(struct CommonExampleOptions& options)
{
	return new R2D2GraspExample(options.m_guiHelper, options.m_option);
}



