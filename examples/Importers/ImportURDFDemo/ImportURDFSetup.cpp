
#include "ImportURDFSetup.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"


#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"

#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "MyURDFImporter.h"


#include "URDF2Bullet.h"


//#include "urdf_samples.h"





#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "MyMultiBodyCreator.h"


class ImportUrdfSetup : public CommonMultiBodyBase
{
    char m_fileName[1024];
    
    struct ImportUrdfInternalData* m_data;
	bool m_useMultiBody;
    
public:
    ImportUrdfSetup(struct GUIHelperInterface* helper, int option, const char* fileName);
    virtual ~ImportUrdfSetup();

	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
    
    void setFileName(const char* urdfFileName);

	virtual void resetCamera()
	{
		float dist = 3.5;
		float pitch = -136;
		float yaw = 28;
		float targetPos[3]={0.47,0,-0.64};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};


btAlignedObjectArray<std::string> gFileNameArray;


#define MAX_NUM_MOTORS 1024

struct ImportUrdfInternalData
{
    ImportUrdfInternalData()
    :m_numMotors(0)
    {
    }
    
    btScalar m_motorTargetVelocities[MAX_NUM_MOTORS];
    btMultiBodyJointMotor* m_jointMotors [MAX_NUM_MOTORS];
    int m_numMotors;
};


ImportUrdfSetup::ImportUrdfSetup(struct GUIHelperInterface* helper, int option, const char* fileName)
	:CommonMultiBodyBase(helper)
{
	m_data = new ImportUrdfInternalData;

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
		gFileNameArray.clear();
		gFileNameArray.push_back("r2d2.urdf");

		
    
		//load additional urdf file names from file
    
		FILE* f = fopen("urdf_files.txt","r");
		if (f)
		{
			int result;
			//warning: we don't avoid string buffer overflow in this basic example in fscanf
			char fileName[1024];
			do
			{
				result = fscanf(f,"%s",fileName);
                b3Printf("urdf_files.txt entry %s",fileName);
				if (result==1)
				{
					gFileNameArray.push_back(fileName);
				}
			} while (result==1);
        
			fclose(f);
		}
    
		int numFileNames = gFileNameArray.size();

		if (count>=numFileNames)
		{
			count=0;
		}
		sprintf(m_fileName,gFileNameArray[count++].c_str());
	}
}

ImportUrdfSetup::~ImportUrdfSetup()
{
    delete m_data;
}

static btVector4 colors[4] =
{
	btVector4(1,0,0,1),
	btVector4(0,1,0,1),
	btVector4(0,1,1,1),
	btVector4(1,1,0,1),
};


btVector3 selectColor()
{

	static int curColor = 0;
	btVector4 color = colors[curColor];
	curColor++;
	curColor&=3;
	return color;
}

void ImportUrdfSetup::setFileName(const char* urdfFileName)
{
    memcpy(m_fileName,urdfFileName,strlen(urdfFileName)+1);
}




void ImportUrdfSetup::initPhysics()
{

	int upAxis = 2;
	m_guiHelper->setUpAxis(2);

	this->createEmptyDynamicsWorld();
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
    btIDebugDraw::DBG_DrawConstraints
    +btIDebugDraw::DBG_DrawContactPoints
    +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);

	
	btVector3 gravity(0,0,0);
	gravity[upAxis]=-9.8;

	m_dynamicsWorld->setGravity(gravity);
    

	
    //now print the tree using the new interface
    MyURDFImporter u2b(m_guiHelper);
	bool loadOk =  u2b.loadURDF(m_fileName);

	if (loadOk)
	{
		u2b.printTree();
    
		btTransform identityTrans;
		identityTrans.setIdentity();
	
	
		{
        
      
			btMultiBody* mb = 0;
        
            
			//todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
			int rootLinkIndex = u2b.getRootLinkIndex();
			printf("urdf root link index = %d\n",rootLinkIndex);
			MyMultiBodyCreator creation(m_guiHelper);

			ConvertURDF2Bullet(u2b,creation, identityTrans,m_dynamicsWorld,m_useMultiBody,u2b.getPathPrefix());
			mb = creation.getBulletMultiBody();

			if (m_useMultiBody)
			{
           
            
                
				//create motors for each joint
                
				for (int i=0;i<mb->getNumLinks();i++)
				{
					int mbLinkIndex = i;
					if (mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::eRevolute)
					{
						if (m_data->m_numMotors<MAX_NUM_MOTORS)
						{
							int urdfLinkIndex = creation.m_mb2urdfLink[mbLinkIndex];
                            
							std::string jointName = u2b.getJointName(urdfLinkIndex);
							char motorName[1024];
							sprintf(motorName,"%s q'", jointName.c_str());
							btScalar* motorVel = &m_data->m_motorTargetVelocities[m_data->m_numMotors];
							*motorVel = 0.f;
							SliderParams slider(motorName,motorVel);
							slider.m_minVal=-4;
							slider.m_maxVal=4;
							m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
							float maxMotorImpulse = 0.1f;
							btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb,mbLinkIndex,0,0,maxMotorImpulse);
							m_data->m_jointMotors[m_data->m_numMotors]=motor;
							m_dynamicsWorld->addMultiBodyConstraint(motor);
							m_data->m_numMotors++;
						}
					}
                    
				}
			}
		}
	
		//the btMultiBody support is work-in-progress :-)

	
	

		bool createGround=true;
		if (createGround)
		{
			btVector3 groundHalfExtents(20,20,20);
			groundHalfExtents[upAxis]=1.f;
			btBoxShape* box = new btBoxShape(groundHalfExtents);
			box->initializePolyhedralFeatures();

			m_guiHelper->createCollisionShapeGraphicsObject(box);
			btTransform start; start.setIdentity();
			btVector3 groundOrigin(0,0,0);
			groundOrigin[upAxis]=-2;//.5;
			start.setOrigin(groundOrigin);
			btRigidBody* body =  createRigidBody(0,start,box);
			//m_dynamicsWorld->removeRigidBody(body);
		   // m_dynamicsWorld->addRigidBody(body,2,1);
			btVector3 color(0.5,0.5,0.5);
			m_guiHelper->createRigidBodyGraphicsObject(body,color);
		}

		///this extra stepSimulation call makes sure that all the btMultibody transforms are properly propagates.
		m_dynamicsWorld->stepSimulation(1. / 240., 0);// 1., 10, 1. / 240.);
	}
}

void ImportUrdfSetup::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
        for (int i=0;i<m_data->m_numMotors;i++)
        {
            m_data->m_jointMotors[i]->setVelocityTarget(m_data->m_motorTargetVelocities[i]);
        }
        
		//the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
		m_dynamicsWorld->stepSimulation(deltaTime,10,1./240.);
	}
}

class CommonExampleInterface*    ImportURDFCreateFunc(struct CommonExampleOptions& options)
{

	return new ImportUrdfSetup(options.m_guiHelper, options.m_option,options.m_fileName);
}
