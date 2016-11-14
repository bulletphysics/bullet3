
#include "ImportMJCFSetup.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
//#define TEST_MULTIBODY_SERIALIZATION 1

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"

#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../../Utils/b3ResourcePath.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "../ImportURDFDemo/MyMultiBodyCreator.h"

class ImportMJCFSetup : public CommonMultiBodyBase
{
    char m_fileName[1024];

    struct ImportMJCFInternalData* m_data;
	bool m_useMultiBody;
	btAlignedObjectArray<std::string* > m_nameMemory;
	btScalar m_grav;
	int m_upAxis;
public:
    ImportMJCFSetup(struct GUIHelperInterface* helper, int option, const char* fileName);
    virtual ~ImportMJCFSetup();

	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);

    void setFileName(const char* mjcfFileName);

	virtual void resetCamera()
	{
		float dist = 3.5;
		float pitch = -136;
		float yaw = 28;
		float targetPos[3]={0.47,0,-0.64};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};


static btAlignedObjectArray<std::string> gMCFJFileNameArray;


#define MAX_NUM_MOTORS 1024

struct ImportMJCFInternalData
{
    ImportMJCFInternalData()
    :m_numMotors(0),
    m_mb(0)
    {
		for (int i=0;i<MAX_NUM_MOTORS;i++)
		{
			m_jointMotors[i] = 0;
			m_generic6DofJointMotors[i] = 0;
		}
    }

    
    btScalar m_motorTargetVelocities[MAX_NUM_MOTORS];
    btMultiBodyJointMotor* m_jointMotors [MAX_NUM_MOTORS];
	btGeneric6DofSpring2Constraint* m_generic6DofJointMotors [MAX_NUM_MOTORS];
    int m_numMotors;
    btMultiBody* m_mb;
    btRigidBody* m_rb;

};


ImportMJCFSetup::ImportMJCFSetup(struct GUIHelperInterface* helper, int option, const char* fileName)
	:CommonMultiBodyBase(helper),
	m_grav(0),
	m_upAxis(2)
{
	m_data = new ImportMJCFInternalData;

	if (option==1)
	{
		m_useMultiBody = true;
	} else
	{
		m_useMultiBody = false;
	}

	static int count = 0;
	if (fileName)
	{
		setFileName(fileName);
	} else
	{
		gMCFJFileNameArray.clear();
		


		//load additional MJCF file names from file

		FILE* f = fopen("mjcf_files.txt","r");
		if (f)
		{
			int result;
			//warning: we don't avoid string buffer overflow in this basic example in fscanf
			char fileName[1024];
			do
			{
				result = fscanf(f,"%s",fileName);
                b3Printf("mjcf_files.txt entry %s",fileName);
				if (result==1)
				{
					gMCFJFileNameArray.push_back(fileName);
				}
			} while (result==1);

			fclose(f);
		}
		
		if (gMCFJFileNameArray.size()==0)
		{
			gMCFJFileNameArray.push_back("quadruped/quadruped.mjcf");

		}

		int numFileNames = gMCFJFileNameArray.size();

		if (count>=numFileNames)
		{
			count=0;
		}
		sprintf(m_fileName,"%s",gMCFJFileNameArray[count++].c_str());
	}
}

ImportMJCFSetup::~ImportMJCFSetup()
{
	for (int i=0;i<m_nameMemory.size();i++)
	{
		delete m_nameMemory[i];
	}
	m_nameMemory.clear();
    delete m_data;
}

static btVector4 colors[4] =
{
	btVector4(1,0,0,1),
	btVector4(0,1,0,1),
	btVector4(0,1,1,1),
	btVector4(1,1,0,1),
};


static btVector3 selectColor()
{

	static int curColor = 0;
	btVector4 color = colors[curColor];
	curColor++;
	curColor&=3;
	return color;
}

void ImportMJCFSetup::setFileName(const char* mjcfFileName)
{
    memcpy(m_fileName,mjcfFileName,strlen(mjcfFileName)+1);
}




void ImportMJCFSetup::initPhysics()
{

	
	m_guiHelper->setUpAxis(m_upAxis);
	
	this->createEmptyDynamicsWorld();
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
    btIDebugDraw::DBG_DrawConstraints
    +btIDebugDraw::DBG_DrawContactPoints
    +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);


	if (m_guiHelper->getParameterInterface())
	{
		SliderParams slider("Gravity", &m_grav);
		slider.m_minVal = -10;
		slider.m_maxVal = 10;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	

		
}



void ImportMJCFSetup::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		btVector3 gravity(0, 0, 0);
		gravity[m_upAxis] = m_grav;
		m_dynamicsWorld->setGravity(gravity);

        for (int i=0;i<m_data->m_numMotors;i++)
        {
			if (m_data->m_jointMotors[i])
			{
				m_data->m_jointMotors[i]->setVelocityTarget(m_data->m_motorTargetVelocities[i]);
			}
			if (m_data->m_generic6DofJointMotors[i])
			{
				GenericConstraintUserInfo* jointInfo = (GenericConstraintUserInfo*)m_data->m_generic6DofJointMotors[i]->getUserConstraintPtr();
				m_data->m_generic6DofJointMotors[i]->setTargetVelocity(jointInfo->m_jointAxisIndex,m_data->m_motorTargetVelocities[i]);
				//jointInfo->
			}
        }

		//the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
		m_dynamicsWorld->stepSimulation(deltaTime,10,1./240.);
	}
}

class CommonExampleInterface*    ImportMJCFCreateFunc(struct CommonExampleOptions& options)
{

	return new ImportMJCFSetup(options.m_guiHelper, options.m_option,options.m_fileName);
}
