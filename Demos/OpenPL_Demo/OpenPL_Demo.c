
#include "Bullet-C-Api.h"


int main()
{
	float timeStep = 1.f/60.f;

	/* initialize */ 
	plPhysicsSdkHandle	sdk = plNewBulletSdk();
	
	plDynamicsWorldHandle world = plCreateDynamicsWorld(sdk);


	float radius = 1.f;
	plCollisionShapeHandle collisionShape = plNewSphereShape(radius);

	void* user_data = 0;/* can point to a graphics object */

	float mass = 1.f;

	plRigidBodyHandle body = plCreateRigidBody(user_data, mass, collisionShape );

	plAddRigidBody(world, body);

	
	
	plStepSimulation(world,0.1f);

	/* cleanup */

	plRemoveRigidBody(world, body);


	plDeleteRigidBody(body);

	plDeleteShape( collisionShape);

	plDeleteDynamicsWorld( world);

	plDeletePhysicsSdk(sdk);

	return 0;
}