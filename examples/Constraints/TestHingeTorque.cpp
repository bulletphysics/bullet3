#include "TestHingeTorque.h"


#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

short collisionFilterGroup = short(btBroadphaseProxy::CharacterFilter);
short collisionFilterMask = short(btBroadphaseProxy::AllFilter ^ (btBroadphaseProxy::CharacterFilter));
static btScalar radius(0.2);

struct TestHingeTorque : public CommonRigidBodyBase
{
    bool m_once;
    btAlignedObjectArray<btJointFeedback*> m_jointFeedback;

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
	for (int i=0;i<m_jointFeedback.size();i++)
	{
		delete m_jointFeedback[i];
	}

}


void TestHingeTorque::stepSimulation(float deltaTime)
{
    if (0)//m_once)
    {
        m_once=false;
        btHingeConstraint* hinge = (btHingeConstraint*)m_dynamicsWorld->getConstraint(0);
        
        btRigidBody& bodyA = hinge->getRigidBodyA();
        btTransform trA = bodyA.getWorldTransform();
        btVector3 hingeAxisInWorld = trA.getBasis()*hinge->getFrameOffsetA().getBasis().getColumn(2);
        hinge->getRigidBodyA().applyTorque(-hingeAxisInWorld*10);
        hinge->getRigidBodyB().applyTorque(hingeAxisInWorld*10);
        
    }
    
    m_dynamicsWorld->stepSimulation(1./240,0);
	
	static int count = 0;
	if ((count& 0x0f)==0)
	{
		btRigidBody* base = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[0]);
		
		b3Printf("base angvel = %f,%f,%f",base->getAngularVelocity()[0],
				 base->getAngularVelocity()[1],
				 
				 base->getAngularVelocity()[2]);
		
		btRigidBody* child = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[1]);
    
	
		b3Printf("child angvel = %f,%f,%f",child->getAngularVelocity()[0],
				 child->getAngularVelocity()[1],

				 child->getAngularVelocity()[2]);
		
		for (int i=0;i<m_jointFeedback.size();i++)
		{
			b3Printf("Applied force at the COM/Inertial frame B[%d]:(%f,%f,%f), torque B:(%f,%f,%f)\n", i,

		
				m_jointFeedback[i]->m_appliedForceBodyB.x(),
				m_jointFeedback[i]->m_appliedForceBodyB.y(),
				m_jointFeedback[i]->m_appliedForceBodyB.z(),
				m_jointFeedback[i]->m_appliedTorqueBodyB.x(),
				m_jointFeedback[i]->m_appliedTorqueBodyB.y(),
				m_jointFeedback[i]->m_appliedTorqueBodyB.z());
		}
	}
	count++;

    //CommonRigidBodyBase::stepSimulation(deltaTime);
}



void TestHingeTorque::initPhysics()
{
	int upAxis = 1;
	m_guiHelper->setUpAxis(upAxis);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = false;
	
    m_dynamicsWorld->setGravity(btVector3(0,0,-10));
    
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	int mode = 	btIDebugDraw::DBG_DrawWireframe
				+btIDebugDraw::DBG_DrawConstraints
				+btIDebugDraw::DBG_DrawConstraintLimits;
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(mode);


	{ // create a door using hinge constraint attached to the world
        
        int numLinks = 2;
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
        float baseMass = 0.f;
        float linkMass = 1.f;
        
        btRigidBody* base = createRigidBody(baseMass,baseWorldTrans,baseBox);
        m_dynamicsWorld->removeRigidBody(base);
        base->setDamping(0,0);
        m_dynamicsWorld->addRigidBody(base,collisionFilterGroup,collisionFilterMask);
        btBoxShape* linkBox1 = new btBoxShape(linkHalfExtents);
		btSphereShape* linkSphere = new btSphereShape(radius);
		
        btRigidBody* prevBody = base;
        
        for (int i=0;i<numLinks;i++)
        {
            btTransform linkTrans;
            linkTrans = baseWorldTrans;
            
            linkTrans.setOrigin(basePosition-btVector3(0,linkHalfExtents[1]*2.f*(i+1),0));
            
			btCollisionShape* colOb = 0;
			
			if (i==0)
			{
				colOb = linkBox1;
			} else 
			{
				colOb = linkSphere;
			}
            btRigidBody* linkBody = createRigidBody(linkMass,linkTrans,colOb);
            m_dynamicsWorld->removeRigidBody(linkBody);
            m_dynamicsWorld->addRigidBody(linkBody,collisionFilterGroup,collisionFilterMask);
            linkBody->setDamping(0,0);
			btTypedConstraint* con = 0;
			
			if (i==0)
			{
				//create a hinge constraint
				btVector3 pivotInA(0,-linkHalfExtents[1],0);
				btVector3 pivotInB(0,linkHalfExtents[1],0);
				btVector3 axisInA(1,0,0);
				btVector3 axisInB(1,0,0);
				bool useReferenceA = true;
				btHingeConstraint* hinge = new btHingeConstraint(*prevBody,*linkBody,
																 pivotInA,pivotInB,
																 axisInA,axisInB,useReferenceA);
				con = hinge;
			} else
			{
				
				btTransform pivotInA(btQuaternion::getIdentity(),btVector3(0, -radius, 0));						//par body's COM to cur body's COM offset
				btTransform pivotInB(btQuaternion::getIdentity(),btVector3(0, radius, 0));							//cur body's COM to cur body's PIV offset
				btGeneric6DofSpring2Constraint* fixed = new btGeneric6DofSpring2Constraint(*prevBody, *linkBody,
																						   pivotInA,pivotInB);
				fixed->setLinearLowerLimit(btVector3(0,0,0));
				fixed->setLinearUpperLimit(btVector3(0,0,0));
				fixed->setAngularLowerLimit(btVector3(0,0,0));
				fixed->setAngularUpperLimit(btVector3(0,0,0));
				
				con = fixed;

			}
			btAssert(con);
			if (con)
			{
				btJointFeedback* fb = new btJointFeedback();
				m_jointFeedback.push_back(fb);
				con->setJointFeedback(fb);

				m_dynamicsWorld->addConstraint(con,true);
			}
			prevBody = linkBody;
            
        }
       
	}
	
	if (1)
	{
		btVector3 groundHalfExtents(1,1,0.2);
		groundHalfExtents[upAxis]=1.f;
		btBoxShape* box = new btBoxShape(groundHalfExtents);
		box->initializePolyhedralFeatures();
		
		btTransform start; start.setIdentity();
		btVector3 groundOrigin(-0.4f, 3.f, 0.f);
		btVector3 basePosition = btVector3(-0.4f, 3.f, 0.f);
		btQuaternion groundOrn(btVector3(0,1,0),0.25*SIMD_PI);
		
		groundOrigin[upAxis] -=.5;
		groundOrigin[2]-=0.6;
		start.setOrigin(groundOrigin);
	//	start.setRotation(groundOrn);
		btRigidBody* body =  createRigidBody(0,start,box);
		body->setFriction(0);
		
	}
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

class CommonExampleInterface*    TestHingeTorqueCreateFunc(CommonExampleOptions& options)
{
	return new TestHingeTorque(options.m_guiHelper);
}
