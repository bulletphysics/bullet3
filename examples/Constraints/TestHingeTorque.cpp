#include "TestHingeTorque.h"


#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"



struct TestHingeTorque : public CommonRigidBodyBase
{
    bool m_once;
    
	TestHingeTorque(struct GUIHelperInterface* helper);
	virtual ~ TestHingeTorque();
	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	
	virtual void resetCamera()
	{
    
        float dist = 5;
        float pitch = 270;
        float yaw = 21;
        float targetPos[3]={-1.34,3.4,-0.44};
        m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
	

};

TestHingeTorque::TestHingeTorque(struct GUIHelperInterface* helper)
:CommonRigidBodyBase(helper),
m_once(true)
{
}
TestHingeTorque::~ TestHingeTorque()
{
}


void TestHingeTorque::stepSimulation(float deltaTime)
{
    if (m_once)
    {
        m_once=false;
        btHingeConstraint* hinge = (btHingeConstraint*)m_dynamicsWorld->getConstraint(0);
        
        btRigidBody& bodyA = hinge->getRigidBodyA();
        btTransform trA = bodyA.getWorldTransform();
        btVector3 hingeAxisInWorld = trA.getBasis()*hinge->getFrameOffsetA().getBasis().getColumn(2);
        hinge->getRigidBodyA().applyTorque(-hingeAxisInWorld*10);
        hinge->getRigidBodyB().applyTorque(hingeAxisInWorld*10);
        
    }
    
    m_dynamicsWorld->stepSimulation(1./60,0);
    btRigidBody* base = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[0]);
    
    b3Printf("base angvel = %f,%f,%f",base->getAngularVelocity()[0],
             base->getAngularVelocity()[1],
             
             base->getAngularVelocity()[2]);
    
    btRigidBody* child = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[1]);
    
    b3Printf("child angvel = %f,%f,%f",child->getAngularVelocity()[0],
             child->getAngularVelocity()[1],

             child->getAngularVelocity()[2]);
    
    //CommonRigidBodyBase::stepSimulation(deltaTime);
}



void TestHingeTorque::initPhysics()
{
	m_guiHelper->setUpAxis(2);

	createEmptyDynamicsWorld();
    m_dynamicsWorld->setGravity(btVector3(0,0,0));
    
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	int mode = 	btIDebugDraw::DBG_DrawWireframe
				+btIDebugDraw::DBG_DrawConstraints
				+btIDebugDraw::DBG_DrawConstraintLimits;
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(mode);


	{ // create a door using hinge constraint attached to the world
        
        int numLinks = 1;
        bool spherical = false;					//set it ot false -to use 1DoF hinges instead of 3DoF sphericals
        bool canSleep = false;
        bool selfCollide = false;
        btVector3 linkHalfExtents(0.05, 0.37, 0.1);
        btVector3 baseHalfExtents(0.05, 0.37, 0.1);

        btBoxShape* baseBox = new btBoxShape(baseHalfExtents);
        btVector3 basePosition = btVector3(-0.4f, 3.f, 0.f);
        btTransform baseWorldTrans;
        baseWorldTrans.setIdentity();
        baseWorldTrans.setOrigin(basePosition);
        
        //mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm
        //init the base
        btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
        float baseMass = 1.f;
        float linkMass = 1.f;
        
        btRigidBody* base = createRigidBody(baseMass,baseWorldTrans,baseBox);
        m_dynamicsWorld->removeRigidBody(base);
        base->setDamping(0,0);
        m_dynamicsWorld->addRigidBody(base,0,0);
        btBoxShape* linkBox = new btBoxShape(linkHalfExtents);
        btRigidBody* prevBody = base;
        
        for (int i=0;i<numLinks;i++)
        {
            btTransform linkTrans;
            linkTrans = baseWorldTrans;
            
            linkTrans.setOrigin(basePosition-btVector3(0,linkHalfExtents[1]*2.f*(i+1),0));
            
            btRigidBody* linkBody = createRigidBody(linkMass,linkTrans,linkBox);
            m_dynamicsWorld->removeRigidBody(linkBody);
            m_dynamicsWorld->addRigidBody(linkBody,0,0);
            linkBody->setDamping(0,0);
            //create a hinge constraint
            btVector3 pivotInA(0,-linkHalfExtents[1],0);
            btVector3 pivotInB(0,linkHalfExtents[1],0);
            btVector3 axisInA(1,0,0);
            btVector3 axisInB(1,0,0);
            bool useReferenceA = true;
            btHingeConstraint* hinge = new btHingeConstraint(*prevBody,*linkBody,
                                                             pivotInA,pivotInB,
                                                             axisInA,axisInB,useReferenceA);
            m_dynamicsWorld->addConstraint(hinge,true);
            prevBody = linkBody;
            
        }
       
	}
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

class CommonExampleInterface*    TestHingeTorqueCreateFunc(CommonExampleOptions& options)
{
	return new TestHingeTorque(options.m_guiHelper);
}
