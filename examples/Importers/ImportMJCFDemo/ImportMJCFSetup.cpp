
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
#include "BulletMJCFImporter.h"
#include "../ImportURDFDemo/URDF2Bullet.h"

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
		float pitch = -28;
		float yaw = -136;
		float targetPos[3]={0.47,0,-0.64};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
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

    
    btScalar m_motorTargetPositions[MAX_NUM_MOTORS];
    btMultiBodyJointMotor* m_jointMotors [MAX_NUM_MOTORS];
	btGeneric6DofSpring2Constraint* m_generic6DofJointMotors [MAX_NUM_MOTORS];
    int m_numMotors;
    btMultiBody* m_mb;
    btRigidBody* m_rb;

};


ImportMJCFSetup::ImportMJCFSetup(struct GUIHelperInterface* helper, int option, const char* fileName)
	:CommonMultiBodyBase(helper),
	m_grav(-10),
	m_upAxis(2)
{
	m_data = new ImportMJCFInternalData;

	m_useMultiBody = true;
	
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
			gMCFJFileNameArray.push_back("mjcf/humanoid.xml");

			gMCFJFileNameArray.push_back("MPL/MPL.xml");

			gMCFJFileNameArray.push_back("mjcf/inverted_pendulum.xml");
			gMCFJFileNameArray.push_back("mjcf/ant.xml");
			gMCFJFileNameArray.push_back("mjcf/hello_mjcf.xml");
	
			gMCFJFileNameArray.push_back("mjcf/cylinder.xml");
			gMCFJFileNameArray.push_back("mjcf/cylinder_fromtoX.xml");
			gMCFJFileNameArray.push_back("mjcf/cylinder_fromtoY.xml");
			gMCFJFileNameArray.push_back("mjcf/cylinder_fromtoZ.xml");

			gMCFJFileNameArray.push_back("mjcf/capsule.xml");
			gMCFJFileNameArray.push_back("mjcf/capsule_fromtoX.xml");
			gMCFJFileNameArray.push_back("mjcf/capsule_fromtoY.xml");
			gMCFJFileNameArray.push_back("mjcf/capsule_fromtoZ.xml");
	
			gMCFJFileNameArray.push_back("mjcf/hopper.xml");
			gMCFJFileNameArray.push_back("mjcf/swimmer.xml");
			gMCFJFileNameArray.push_back("mjcf/reacher.xml");
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



void ImportMJCFSetup::setFileName(const char* mjcfFileName)
{
    memcpy(m_fileName,mjcfFileName,strlen(mjcfFileName)+1);
}


struct MyMJCFLogger : public MJCFErrorLogger
{
	virtual void reportError(const char* error)
	{
		b3Error(error);
	}
	virtual void reportWarning(const char* warning)
	{
		b3Warning(warning);
	}
	virtual void printMessage(const char* msg)
	{
		b3Printf(msg);
	}
};


void ImportMJCFSetup::initPhysics()
{

	
	m_guiHelper->setUpAxis(m_upAxis);
	
	createEmptyDynamicsWorld();
	
	//MuJoCo uses a slightly different collision filter mode, use the FILTER_GROUPAMASKB_OR_GROUPBMASKA2
	//@todo also use the modified collision filter for raycast and other collision related queries
	m_filterCallback->m_filterMode = FILTER_GROUPAMASKB_OR_GROUPBMASKA2;
	
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
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

	int flags=0;
	BulletMJCFImporter importer(m_guiHelper, 0,flags);
	MyMJCFLogger logger;
	bool result = importer.loadMJCF(m_fileName,&logger);
	if (result)
	{
		btTransform rootTrans;
		rootTrans.setIdentity();

		for (int m =0; m<importer.getNumModels();m++)
		{
			importer.activateModel(m);

			// normally used with PhysicsServerCommandProcessor that allocates unique ids to multibodies,
			// emulate this behavior here:
			importer.setBodyUniqueId(m);

			btMultiBody* mb = 0;


			//todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
			//int rootLinkIndex = importer.getRootLinkIndex();
			//b3Printf("urdf root link index = %d\n",rootLinkIndex);
			MyMultiBodyCreator creation(m_guiHelper);

			rootTrans.setIdentity();
			importer.getRootTransformInWorld(rootTrans);

			ConvertURDF2Bullet(importer,creation, rootTrans,m_dynamicsWorld,m_useMultiBody,importer.getPathPrefix(),CUF_USE_MJCF);

			mb = creation.getBulletMultiBody();
			if (mb)
			{
				std::string* name =
				new std::string(importer.getLinkName(
					importer.getRootLinkIndex()));
					m_nameMemory.push_back(name);
#ifdef TEST_MULTIBODY_SERIALIZATION
				s->registerNameForPointer(name->c_str(),name->c_str());
#endif//TEST_MULTIBODY_SERIALIZATION
				mb->setBaseName(name->c_str());
				mb->getBaseCollider()->setCollisionFlags(mb->getBaseCollider()->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);

				//create motors for each btMultiBody joint
				int numLinks = mb->getNumLinks();
				for (int i=0;i<numLinks;i++)
				{
					int mbLinkIndex = i;
					int urdfLinkIndex = creation.m_mb2urdfLink[mbLinkIndex];

					std::string* jointName = new std::string(importer.getJointName(urdfLinkIndex));
					std::string* linkName = new std::string(importer.getLinkName(urdfLinkIndex).c_str());
#ifdef TEST_MULTIBODY_SERIALIZATION					
					s->registerNameForPointer(jointName->c_str(),jointName->c_str());
					s->registerNameForPointer(linkName->c_str(),linkName->c_str());
#endif//TEST_MULTIBODY_SERIALIZATION
					m_nameMemory.push_back(jointName);
					m_nameMemory.push_back(linkName);
					mb->getLinkCollider(i)->setCollisionFlags(mb->getBaseCollider()->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);

					mb->getLink(i).m_linkName = linkName->c_str();
					mb->getLink(i).m_jointName = jointName->c_str();
					m_data->m_mb = mb;
					if (mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::eRevolute
					    ||mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::ePrismatic
					)
					{
						if (m_data->m_numMotors<MAX_NUM_MOTORS)
						{
							
							char motorName[1024];
							sprintf(motorName,"%s q ", jointName->c_str());
							btScalar* motorPos = &m_data->m_motorTargetPositions[m_data->m_numMotors];
							*motorPos = 0.f;
							SliderParams slider(motorName,motorPos);
							slider.m_minVal=-4;
							slider.m_maxVal=4;
							slider.m_clampToIntegers = false;
							slider.m_clampToNotches = false;
							m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
							float maxMotorImpulse = 5.f;
							btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb,mbLinkIndex,0,0,maxMotorImpulse);
							motor->setErp(0.1);
							//motor->setMaxAppliedImpulse(0);
							m_data->m_jointMotors[m_data->m_numMotors]=motor;
							m_dynamicsWorld->addMultiBodyConstraint(motor);
							m_data->m_numMotors++;
						}
					}
				}

			} else
			{
				// not multibody
				if (1)
				{
					//create motors for each generic joint
					int num6Dof = creation.getNum6DofConstraints();
					for (int i=0;i<num6Dof;i++)
					{
						btGeneric6DofSpring2Constraint* c = creation.get6DofConstraint(i);
						if (c->getUserConstraintPtr())
						{
							GenericConstraintUserInfo* jointInfo = (GenericConstraintUserInfo*)c->getUserConstraintPtr();
							if ((jointInfo->m_urdfJointType ==URDFRevoluteJoint) || 
								(jointInfo->m_urdfJointType ==URDFPrismaticJoint) ||
								(jointInfo->m_urdfJointType ==URDFContinuousJoint))
							{
								int urdfLinkIndex = jointInfo->m_urdfIndex;
								std::string jointName = importer.getJointName(urdfLinkIndex);
								char motorName[1024];
								sprintf(motorName,"%s q'", jointName.c_str());
								btScalar* motorVel = &m_data->m_motorTargetPositions[m_data->m_numMotors];

								*motorVel = 0.f;
								SliderParams slider(motorName,motorVel);
								slider.m_minVal=-4;
								slider.m_maxVal=4;
								m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
								m_data->m_generic6DofJointMotors[m_data->m_numMotors]=c;
								bool motorOn = true;
								c->enableMotor(jointInfo->m_jointAxisIndex,motorOn);
								c->setMaxMotorForce(jointInfo->m_jointAxisIndex,10000);
								c->setTargetVelocity(jointInfo->m_jointAxisIndex,0);
								
								m_data->m_numMotors++;
							}
						}
					}
				}
			
			}
		}
		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	}
		
}



void ImportMJCFSetup::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		btVector3 gravity(0, 0, -10);
		gravity[m_upAxis] = m_grav;
		m_dynamicsWorld->setGravity(gravity);

        for (int i=0;i<m_data->m_numMotors;i++)
        {
			if (m_data->m_jointMotors[i])
			{
				btScalar pos = m_data->m_motorTargetPositions[i];

				int link = m_data->m_jointMotors[i]->getLinkA();
				btScalar lowerLimit = m_data->m_mb->getLink(link).m_jointLowerLimit;
				btScalar upperLimit = m_data->m_mb->getLink(link).m_jointUpperLimit;
				if (lowerLimit < upperLimit)
				{
					btClamp(pos, lowerLimit, upperLimit);
				}
				m_data->m_jointMotors[i]->setPositionTarget(pos);
			}
			if (m_data->m_generic6DofJointMotors[i])
			{
				GenericConstraintUserInfo* jointInfo = (GenericConstraintUserInfo*)m_data->m_generic6DofJointMotors[i]->getUserConstraintPtr();
				m_data->m_generic6DofJointMotors[i]->setTargetVelocity(jointInfo->m_jointAxisIndex,m_data->m_motorTargetPositions[i]);
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
