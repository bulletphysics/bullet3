#include <iostream>
#include <btBulletDynamicsCommon.h>


btDiscreteDynamicsWorld* world;
btRigidBody* fallingRigidBody;

void InitDefaultWorld()
{
	{
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );
		btBroadphaseInterface* broadphase = new btDbvtBroadphase();
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();


		world = new btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );
	}
	{
		btCollisionShape* shape = new btBoxShape( btVector3::One );
		btCollisionShape* groundShape = new btStaticPlaneShape( btVector3::Zero, btVector3::yAxis );

		btDefaultMotionState* groundMotionState = new btDefaultMotionState();

		btRigidBody::btRigidBodyConstructionInfo*
			groundRigidBodyCI = new btRigidBody::btRigidBodyConstructionInfo( 0, groundMotionState
				, groundShape, btVector3::Zero );
		btRigidBody* groundRigidBody = new btRigidBody( *groundRigidBodyCI );
		world->addRigidBody( groundRigidBody );
	}


	{
		btCollisionShape* fallShape = new btBoxShape( btVector3::One );

		btVector3 origin( 0, 50, 0 );
		btTransform* init = new btTransform( btQuaternion::Identity, origin );
		btDefaultMotionState* fallMotionState = new btDefaultMotionState( *init );

		btScalar mass = 1;
		btVector3 fallInertia;
		fallShape->calculateLocalInertia( mass, fallInertia ); // fills fallInertia

		btRigidBody::btRigidBodyConstructionInfo*
			fallingRigidBodyCI = new btRigidBody::btRigidBodyConstructionInfo( mass, fallMotionState
				, fallShape, fallInertia );

		fallingRigidBody = new btRigidBody( *fallingRigidBodyCI );

		world->addRigidBody( fallingRigidBody );
	}


}

int main( void )
{
	InitDefaultWorld();

	for( int i = 0; i < 300; i++ ) {

		world->stepSimulation( 1 / 60.f, 10 );

		btTransform trans;
		fallingRigidBody->getMotionState()->getWorldTransform( trans );

		if( i == 188 )
		{
			int a = 3;
		}
		std::cout << "Iteration " << i << std::endl;
		std::cout << "cube height: (" << trans.getOrigin().getX() << "," << trans.getOrigin().getY() << "," << trans.getOrigin().getZ() << ")" << std::endl;
		std::cout << "cube orient:\t(" << trans.getBasis()[0].getX() << "," << trans.getBasis()[0].getY() << "," << trans.getBasis()[0].getZ() << ")" << std::endl;
		std::cout << "\t\t(" << trans.getBasis()[1].getX() << "," << trans.getBasis()[1].getY() << "," << trans.getBasis()[1].getZ() << ")"<< std::endl;
		std::cout << "\t\t(" << trans.getBasis()[2].getX() << "," << trans.getBasis()[2].getY() << "," << trans.getBasis()[2].getZ() << ")" << std::endl;
		btVector3 v = fallingRigidBody->getAngularVelocity();
		std::cout << "cube Ang Vel: (" << v.x() << "," << v.y() << "," << v.z() << ")" << std::endl;
		v = fallingRigidBody->getLinearVelocity();
		std::cout << "cube Lin Vel: (" << v.x() << "," << v.y() << "," << v.z() << ")" << std::endl;
	}
	int pause;
	std::cin >> pause;


}

