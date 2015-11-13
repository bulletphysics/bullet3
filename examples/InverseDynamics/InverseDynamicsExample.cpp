/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "InverseDynamicsExample.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../../Utils/b3ResourcePath.h"
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

//those are static global to make it easy for the GUI to tweak them
static btScalar radius(0.2);
static btScalar kp = 100;
static btScalar kd = 20;
static btScalar maxForce = 100;

class InverseDynamicsExample : public CommonMultiBodyBase
{
	btInverseDynamicsExampleOptions m_option;
	btMultiBody* m_multiBody;
public:
    InverseDynamicsExample(struct GUIHelperInterface* helper, btInverseDynamicsExampleOptions option);
    virtual ~InverseDynamicsExample();

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

InverseDynamicsExample::InverseDynamicsExample(struct GUIHelperInterface* helper, btInverseDynamicsExampleOptions option)
	:CommonMultiBodyBase(helper),
	m_option(option),
	m_multiBody(0)
{
}

InverseDynamicsExample::~InverseDynamicsExample()
{
	
}

//todo(erwincoumans) Quick hack, reference to InvertedPendulumPDControl implementation. Will create a separate header/source file for this.
btMultiBody* createInvertedPendulumMultiBody(btMultiBodyDynamicsWorld* world, GUIHelperInterface* guiHelper, const btTransform& baseWorldTrans);

void InverseDynamicsExample::initPhysics()
{
	//roboticists like Z up
	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

	createEmptyDynamicsWorld();
	btVector3 gravity(0,0,0);
	gravity[upAxis]=-9.8;
	m_dynamicsWorld->setGravity(gravity);

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	{
		SliderParams slider("Kp",&kp);
		slider.m_minVal=-200;
		slider.m_maxVal=200;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
	{
		SliderParams slider("Kd",&kd);
		slider.m_minVal=-50;
		slider.m_maxVal=50;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
	{
		SliderParams slider("max force",&maxForce);
		slider.m_minVal=0;
		slider.m_maxVal=100;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

	switch (m_option)
	{
	case BT_ID_LOAD_URDF:
		{
			BulletURDFImporter u2b(m_guiHelper);
			bool loadOk =  u2b.loadURDF("kuka_lwr/kuka.urdf");
			if (loadOk)
			{
					int rootLinkIndex = u2b.getRootLinkIndex();
					b3Printf("urdf root link index = %d\n",rootLinkIndex);
					MyMultiBodyCreator creation(m_guiHelper);
					btTransform identityTrans;
					identityTrans.setIdentity();
					ConvertURDF2Bullet(u2b,creation, identityTrans,m_dynamicsWorld,true,u2b.getPathPrefix());
					m_multiBody = creation.getBulletMultiBody();
					if (m_multiBody)
					{
						//kuka without joint control/constraints will gain energy explode soon due to timestep/integrator
						//temporarily set some extreme damping factors until we have some joint control or constraints
						m_multiBody->setAngularDamping(0.99);
						m_multiBody->setLinearDamping(0.99);
						b3Printf("Root link name = %s",u2b.getLinkName(u2b.getRootLinkIndex()));
					}
			}
			break;
		}
	case BT_ID_PROGRAMMATICALLY:
		{
			btTransform baseWorldTrans;
			baseWorldTrans.setIdentity();
			m_multiBody = createInvertedPendulumMultiBody(m_dynamicsWorld, m_guiHelper, baseWorldTrans);
			break;
		}
	default:
		{
			b3Error("Unknown option in InverseDynamicsExample::initPhysics");
			b3Assert(0);
		}
	};
}


void InverseDynamicsExample::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(deltaTime,10,1./240.);
	}
}

CommonExampleInterface*    InverseDynamicsExampleCreateFunc(CommonExampleOptions& options)
{
	return new InverseDynamicsExample(options.m_guiHelper, btInverseDynamicsExampleOptions(options.m_option));
}



