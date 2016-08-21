%module bullet
%{
/* Includes the header in the wrapper code */
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMotionState.h"
#include "LinearMath/btDefaultMotionState.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
%}

/* Parse the header file to generate wrappers */


%feature ("flatnested");

%typemap(out) float [ANY] {
	int i;
	$result = PyList_New($1_dim0);
	for (i = 0; i < $1_dim0; i++) {
		PyObject *o = PyFloat_FromDouble((double) $1[i]);
		PyList_SetItem($result,i,o);
	}
}

%include "LinearMath/btScalar.h"
%include "LinearMath/btVector3.h"
%include "LinearMath/btTransform.h"
%include "LinearMath/btMotionState.h"
%include "LinearMath/btDefaultMotionState.h"

%include "BulletCollision/CollisionDispatch/btCollisionConfiguration.h"
%include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"

%include "BulletCollision/BroadphaseCollision/btDispatcher.h"
%include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

%include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
%include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
%include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

%include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
%include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
%include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

%include "BulletCollision/CollisionDispatch/btCollisionObject.h"
%include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
%include "BulletDynamics/Dynamics/btDynamicsWorld.h"
%include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
%include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

%include "BulletCollision/CollisionShapes/btCollisionShape.h"
%include "BulletCollision/CollisionShapes/btConvexShape.h"
%include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
%include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
%include "BulletCollision/CollisionShapes/btBoxShape.h"
%include "BulletDynamics/Dynamics/btRigidBody.h"
