#include "BasicDemo.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"




#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

static const float scaling=0.35f;

BasicDemo::BasicDemo(SimpleOpenGL3App* app, CommonPhysicsSetup* physicsSetup)
:Bullet2RigidBodyDemo(app,physicsSetup)
{
}

BasicDemo::~BasicDemo()
{
}

void	BasicDemo::createGround(int cubeShapeId)
{
}



