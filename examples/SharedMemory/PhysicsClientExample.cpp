
#include "PhysicsClientExample.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "SharedMemoryCommon.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "PhysicsClientC_API.h"
#include "PhysicsClient.h"
//#include "SharedMemoryCommands.h"

#include "PhysicsClientC_API.h"

struct MyMotorInfo2
{
	btScalar m_velTarget;
	btScalar m_maxForce;
	int		m_uIndex;
};


#define MAX_NUM_MOTORS 128

class PhysicsClientExample : public SharedMemoryCommon
{
protected:
    b3PhysicsClientHandle m_physicsClientHandle;
	
	bool m_wantsTermination;
    btAlignedObjectArray<int> m_userCommandRequests;
    int m_sharedMemoryKey;
    
	void	createButton(const char* name, int id, bool isTrigger );

	void createButtons();
	
public:
    
    //@todo, add accessor methods
	MyMotorInfo2 m_motorTargetVelocities[MAX_NUM_MOTORS];
	int m_numMotors;

    
	PhysicsClientExample(GUIHelperInterface* helper);
	virtual ~PhysicsClientExample();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
	virtual void resetCamera()
	{
				float dist = 5;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={0,0,0};//-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);

	}
    
    virtual bool wantsTermination()
    {
        return m_wantsTermination;
    }
    
    virtual bool isConnected()
    {
        return (m_physicsClientHandle!=0);
    }
    
    
    
    void enqueueCommand(int commandId);
    
    void prepareAndSubmitCommand(int commandId);
    
    
	virtual void    exitPhysics(){};
	virtual void	renderScene()
	{
        b3DebugLines debugLines;
        b3GetDebugLines(m_physicsClientHandle,&debugLines);
        int numLines = debugLines.m_numDebugLines;

		int lineWidth = 1;

		if (1)
		{
			btAlignedObjectArray<btVector3FloatData> points;
			points.resize(numLines*2);
			btAlignedObjectArray<unsigned int> indices;
			indices.resize(numLines*2);

			for (int i=0;i<numLines;i++)
			{
				points[i*2].m_floats[0] = debugLines.m_linesFrom[i*3+0];
				points[i*2].m_floats[1] = debugLines.m_linesFrom[i*3+1];
                points[i*2].m_floats[2] = debugLines.m_linesFrom[i*3+2];
                points[i*2+1].m_floats[0] = debugLines.m_linesTo[i*3+0];
				points[i*2+1].m_floats[1] = debugLines.m_linesTo[i*3+1];
				points[i*2+1].m_floats[2] = debugLines.m_linesTo[i*3+2];
				indices[i*2] = i*2;
				indices[i*2+1] = i*2+1;
			}

		
			
			float color[4] = {1,1,0,1};
			
			if (points.size() && indices.size())
			{
				m_guiHelper->getRenderInterface()->drawLines(&points[0].m_floats[0],color,points.size(),sizeof(btVector3FloatData),&indices[0],indices.size(),lineWidth);
			}
		} else
		{
			for (int i=0;i<numLines;i++)
			{
				m_guiHelper->getRenderInterface()->drawLine(debugLines.m_linesFrom,debugLines.m_linesTo,debugLines.m_linesColor,lineWidth);
			}
		}
	}

	void prepareControlCommand(b3SharedMemoryCommandHandle commandHandle)
	{
        for (int i=0;i<m_numMotors;i++)
        {
            btScalar targetVel = m_motorTargetVelocities[i].m_velTarget;
                
            int uIndex = m_motorTargetVelocities[i].m_uIndex;
            b3JointControlSetDesiredVelocity(commandHandle, uIndex,targetVel);
            b3JointControlSetMaximumForce(commandHandle,uIndex,1000);
        }
	}
	virtual void	physicsDebugDraw(int debugFlags){}
	virtual bool	mouseMoveCallback(float x,float y){return false;};
	virtual bool	mouseButtonCallback(int button, int state, float x, float y){return false;}
	virtual bool	keyboardCallback(int key, int state){return false;}

	
	virtual void setSharedMemoryKey(int key)
	{
        m_sharedMemoryKey = key;
	}
};





void MyCallback(int buttonId, bool buttonState, void* userPtr)
{
	PhysicsClientExample* cl = (PhysicsClientExample*) userPtr;
    if (buttonState)
    {
        cl->enqueueCommand(buttonId);
    }
}

void PhysicsClientExample::enqueueCommand(int commandId)
{
    m_userCommandRequests.push_back(commandId);
}

void PhysicsClientExample::prepareAndSubmitCommand(int commandId)
{
    
    switch (commandId)
    {
        case  CMD_LOAD_URDF:
        {
            
            b3SharedMemoryCommandHandle commandHandle = b3LoadUrdfCommandInit(m_physicsClientHandle, "r2d2.urdf");//kuka_lwr/kuka.urdf");
            
            //setting the initial position, orientation and other arguments are optional
            double startPosX = 0;
            double startPosY = 0;
            double startPosZ = 0;
            b3LoadUrdfCommandSetStartPosition(commandHandle, startPosX,startPosY,startPosZ);
//            ret = b3LoadUrdfCommandSetUseFixedBase(commandHandle, 1);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);

            break;
        }
        case CMD_CREATE_BOX_COLLISION_SHAPE:
        {
            b3SharedMemoryCommandHandle commandHandle = b3CreateBoxShapeCommandInit(m_physicsClientHandle);
            b3CreateBoxCommandSetStartPosition(commandHandle,0,0,-3);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
        case CMD_REQUEST_ACTUAL_STATE:
        {
            b3SharedMemoryCommandHandle commandHandle = b3RequestActualStateCommandInit(m_physicsClientHandle);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        };

        case CMD_STEP_FORWARD_SIMULATION:
        {
        
            b3SharedMemoryCommandHandle commandHandle = b3InitStepSimulationCommand(m_physicsClientHandle);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
            
        case CMD_REQUEST_DEBUG_LINES:
        {
            b3SharedMemoryCommandHandle commandHandle = b3InitRequestDebugLinesCommand(m_physicsClientHandle, btIDebugDraw::DBG_DrawWireframe);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
        case CMD_SEND_DESIRED_STATE:
        {
            b3SharedMemoryCommandHandle command = b3JointControlCommandInit( m_physicsClientHandle, CONTROL_MODE_VELOCITY);
            prepareControlCommand(command);
            b3SubmitClientCommand(m_physicsClientHandle, command);
            break;
        }
        case CMD_RESET_SIMULATION:
        {
            b3SharedMemoryCommandHandle commandHandle = b3InitResetSimulationCommand(m_physicsClientHandle);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
        case CMD_SEND_BULLET_DATA_STREAM:
        {
#if 0

            //this worked, but needs C-API and a streaming options, similar to debug lines
            command.m_type = buttonId;
            cl->enqueueCommand(command);
#endif

            break;
        }
        default:
        {
            b3Error("Unknown buttonId");
            btAssert(0);
        }
    };
}

PhysicsClientExample::PhysicsClientExample(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_wantsTermination(false),
m_sharedMemoryKey(SHARED_MEMORY_KEY),
m_numMotors(0)
{
	b3Printf("Started PhysicsClientExample\n");
}

PhysicsClientExample::~PhysicsClientExample()
{
    b3Printf("~PhysicsClientExample\n");
}

void	PhysicsClientExample::createButton(const char* name, int buttonId, bool isTrigger )
{
	ButtonParams button(name,buttonId,  isTrigger);
	button.m_callback = MyCallback;
	button.m_userPointer = this;
	m_guiHelper->getParameterInterface()->registerButtonParameter(button);
}

void	PhysicsClientExample::createButtons()
{
	bool isTrigger = false;
	
    if (m_guiHelper && m_guiHelper->getParameterInterface())
    {
        createButton("Load URDF",CMD_LOAD_URDF,  isTrigger);
        createButton("Step Sim",CMD_STEP_FORWARD_SIMULATION,  isTrigger);
        createButton("Send Bullet Stream",CMD_SEND_BULLET_DATA_STREAM,  isTrigger);
        createButton("Get State",CMD_REQUEST_ACTUAL_STATE,  isTrigger);
        createButton("Send Desired State",CMD_SEND_DESIRED_STATE,  isTrigger);
        createButton("Create Box Collider",CMD_CREATE_BOX_COLLISION_SHAPE,isTrigger);
        createButton("Reset Simulation",CMD_RESET_SIMULATION,isTrigger);
    }
}

void	PhysicsClientExample::initPhysics()
{
	if (m_guiHelper && m_guiHelper->getParameterInterface())
	{
		int upAxis = 2;
		m_guiHelper->setUpAxis(upAxis);

		createButtons();		
		
	} else
	{
        MyCallback(CMD_LOAD_URDF, true, this);
        MyCallback(CMD_STEP_FORWARD_SIMULATION,true,this);
        MyCallback(CMD_STEP_FORWARD_SIMULATION,true,this);
        MyCallback(CMD_RESET_SIMULATION,true,this);
	}

    m_physicsClientHandle  = b3ConnectSharedMemory(m_sharedMemoryKey);
    if (!b3CanSubmitCommand(m_physicsClientHandle))
    {
		b3Warning("Cannot connect to physics client");
	}

}


void	PhysicsClientExample::stepSimulation(float deltaTime)
{
    b3SharedMemoryStatusHandle status = b3ProcessServerStatus(m_physicsClientHandle);
    bool hasStatus = (status != 0);

	if (hasStatus)
    {

        int statusType = b3GetStatusType(status);
        
      	if (statusType == CMD_URDF_LOADING_COMPLETED)
		{
            int numJoints = b3GetNumJoints(m_physicsClientHandle);
            
			for (int i=0;i<numJoints;i++)
			{
				b3JointInfo info;
                b3GetJointInfo(m_physicsClientHandle,i,&info);
                b3Printf("Joint %s at q-index %d and u-index %d\n",info.m_jointName,info.m_qIndex,info.m_uIndex);
				
			}
		
            for (int i=0;i<numJoints;i++)
            {
                b3JointInfo info;
                b3GetJointInfo(m_physicsClientHandle,i,&info);
                b3Printf("Joint %s at q-index %d and u-index %d\n",info.m_jointName,info.m_qIndex,info.m_uIndex);
                
                if (info.m_flags & JOINT_HAS_MOTORIZED_POWER)
                {
                    if (m_numMotors<MAX_NUM_MOTORS)
                    {
                        char motorName[1024];
                        sprintf(motorName,"%s q'", info.m_jointName);
                        MyMotorInfo2* motorInfo = &m_motorTargetVelocities[m_numMotors];
                        motorInfo->m_velTarget = 0.f;
                        motorInfo->m_uIndex = info.m_uIndex;
                        
                        SliderParams slider(motorName,&motorInfo->m_velTarget);
                        slider.m_minVal=-4;
                        slider.m_maxVal=4;
                        if (m_guiHelper && m_guiHelper->getParameterInterface())
                        {
                        m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
                        }
                        m_numMotors++;
                    }
                }
            }
		}
    
    }
    if (b3CanSubmitCommand(m_physicsClientHandle))
    {
        if (m_userCommandRequests.size())
        {
            //b3Printf("Outstanding user command requests: %d\n", m_userCommandRequests.size());
            int commandId = m_userCommandRequests[0];

            //a manual 'pop_front', we don't use 'remove' because it will re-order the commands
            for (int i=1;i<m_userCommandRequests.size();i++)
            {
                m_userCommandRequests[i-1] = m_userCommandRequests[i];
            }

            m_userCommandRequests.pop_back();
            
            //for the CMD_RESET_SIMULATION we need to do something special: clear the GUI sliders
            if (commandId ==CMD_RESET_SIMULATION)
            {
                if (m_guiHelper->getParameterInterface())
                {
                    m_guiHelper->getParameterInterface()->removeAllParameters();
                }
                m_numMotors=0;
                createButtons();
            }
            
            prepareAndSubmitCommand(commandId);
            
        }  else
        {
            if (m_numMotors)
            {
                enqueueCommand(CMD_SEND_DESIRED_STATE);
                enqueueCommand(CMD_STEP_FORWARD_SIMULATION);
                enqueueCommand(CMD_REQUEST_DEBUG_LINES);
                enqueueCommand(CMD_REQUEST_ACTUAL_STATE);
            }
        }
    }


}

extern int gSharedMemoryKey;


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    PhysicsClientExample* example = new PhysicsClientExample(options.m_guiHelper);
	if (gSharedMemoryKey>=0)
	{
		example->setSharedMemoryKey(gSharedMemoryKey);
	}
	return example;
}
