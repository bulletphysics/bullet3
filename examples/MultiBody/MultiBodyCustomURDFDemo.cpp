
#include "MultiBodyCustomURDFDemo.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "CustomMultiBodyCreationCallback.h"


struct MultiBodyCustomURDFDemo : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;

public:

    MultiBodyCustomURDFDemo(struct GUIHelperInterface* helper);
    virtual ~MultiBodyCustomURDFDemo();

    virtual void initPhysics();

    virtual void stepSimulation(float deltaTime);

};

MultiBodyCustomURDFDemo::MultiBodyCustomURDFDemo(struct GUIHelperInterface* helper)
:CommonMultiBodyBase(helper)
{
}

MultiBodyCustomURDFDemo::~MultiBodyCustomURDFDemo()
{

}

void MultiBodyCustomURDFDemo::initPhysics()
{
    int upAxis = 2;
    m_guiHelper->setUpAxis(upAxis);
	createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
        //btIDebugDraw::DBG_DrawConstraints
        +btIDebugDraw::DBG_DrawWireframe
        +btIDebugDraw::DBG_DrawContactPoints
        +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);


    
}

void MultiBodyCustomURDFDemo::stepSimulation(float deltaTime)
{
   
    m_dynamicsWorld->stepSimulation(deltaTime);
}


class ExampleInterface*    MultiBodyCustomURDFDofCreateFunc(struct PhysicsInterface* pint, struct GUIHelperInterface* helper, int option)
{
	return new MultiBodyCustomURDFDemo(helper);
}