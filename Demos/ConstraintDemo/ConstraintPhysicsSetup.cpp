#include "ConstraintPhysicsSetup.h"

ConstraintPhysicsSetup::ConstraintPhysicsSetup()
{
}
ConstraintPhysicsSetup::~ConstraintPhysicsSetup()
{
}

btScalar val;
btHingeAccumulatedAngleConstraint* spDoorHinge=0;
void ConstraintPhysicsSetup::stepSimulation(float deltaTime)
{
	val=spDoorHinge->getAccumulatedHingeAngle()*SIMD_DEGS_PER_RAD;// spDoorHinge->getHingeAngle()*SIMD_DEGS_PER_RAD;
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(deltaTime);
	}
}



void ConstraintPhysicsSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	createEmptyDynamicsWorld();

	gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
int mode = 	btIDebugDraw::DBG_DrawWireframe
				+btIDebugDraw::DBG_DrawConstraints
				+btIDebugDraw::DBG_DrawConstraintLimits;
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(mode);

val=1.f;
	SliderParams slider("hinge angle",&val);
slider.m_minVal=-720;
slider.m_maxVal=720;
	gfxBridge.getParameterInterface()->registerSliderFloatParameter(slider);


	{ // create a door using hinge constraint attached to the world
		btCollisionShape* pDoorShape = new btBoxShape(btVector3(2.0f, 5.0f, 0.2f));
		m_collisionShapes.push_back(pDoorShape);
		btTransform doorTrans;
		doorTrans.setIdentity();
		doorTrans.setOrigin(btVector3(-5.0f, -2.0f, 0.0f));
		btRigidBody* pDoorBody = createRigidBody( 1.0, doorTrans, pDoorShape);
		pDoorBody->setActivationState(DISABLE_DEACTIVATION);
		const btVector3 btPivotA(10.f +  2.1f, -2.0f, 0.0f ); // right next to the door slightly outside
		btVector3 btAxisA( 0.0f, 1.0f, 0.0f ); // pointing upwards, aka Y-axis

		spDoorHinge = new btHingeAccumulatedAngleConstraint( *pDoorBody, btPivotA, btAxisA );

//		spDoorHinge->setLimit( 0.0f, SIMD_PI_2 );
		// test problem values
//		spDoorHinge->setLimit( -SIMD_PI, SIMD_PI*0.8f);

//		spDoorHinge->setLimit( 1.f, -1.f);
//		spDoorHinge->setLimit( -SIMD_PI*0.8f, SIMD_PI);
//		spDoorHinge->setLimit( -SIMD_PI*0.8f, SIMD_PI, 0.9f, 0.3f, 0.0f);
//		spDoorHinge->setLimit( -SIMD_PI*0.8f, SIMD_PI, 0.9f, 0.01f, 0.0f); // "sticky limits"
	//	spDoorHinge->setLimit( -SIMD_PI * 0.25f, SIMD_PI * 0.25f );
//		spDoorHinge->setLimit( 0.0f, 0.0f );
		m_dynamicsWorld->addConstraint(spDoorHinge);
		spDoorHinge->setDbgDrawSize(btScalar(5.f));

		//doorTrans.setOrigin(btVector3(-5.0f, 2.0f, 0.0f));
		//btRigidBody* pDropBody = localCreateRigidBody( 10.0, doorTrans, shape);
	}

}