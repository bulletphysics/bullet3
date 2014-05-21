#ifndef BASIC_DEMO_PHYSICS_SETUP_H
#define BASIC_DEMO_PHYSICS_SETUP_H

class btRigidBody;
class btCollisionShape;
class btBroadphaseInterface;
class btConstraintSolver;
class btCollisionDispatcher;
class btDefaultCollisionConfiguration;
class btDiscreteDynamicsWorld;
class btTransform;
class btVector3;
class btBoxShape;
#include "LinearMath/btVector3.h"

#include "LinearMath/btAlignedObjectArray.h"


struct BasicDemoPhysicsSetup
{
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;
	
	virtual void initPhysics();
	
	virtual void exitPhysics();
	
	virtual void stepSimulation(float deltaTime);
	
	virtual btRigidBody*	createRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, const btVector4& color=btVector4(1,0,0,1));
	
	virtual btBoxShape* createBoxShape(const btVector3& halfExtents);

/*

	//bodies
	virtual btRigidBody*  createRigidBody(bool isDynamic, 	btScalar mass, 	const btTransform& startTransform,	btCollisionShape* shape,const char* bodyName);
	virtual btCollisionObject*  createCollisionObject(	const btTransform& startTransform,	btCollisionShape* shape,const char* bodyName);

	///shapes

	virtual btCollisionShape* createPlaneShape(const btVector3& planeNormal,btScalar planeConstant);
	
	virtual btCollisionShape* createSphereShape(btScalar radius);
	virtual btCollisionShape* createCapsuleShapeX(btScalar radius, btScalar height);
	virtual btCollisionShape* createCapsuleShapeY(btScalar radius, btScalar height);
	virtual btCollisionShape* createCapsuleShapeZ(btScalar radius, btScalar height);
	
	virtual btCollisionShape* createCylinderShapeX(btScalar radius,btScalar height);
	virtual btCollisionShape* createCylinderShapeY(btScalar radius,btScalar height);
	virtual btCollisionShape* createCylinderShapeZ(btScalar radius,btScalar height);
	virtual btCollisionShape* createConeShapeX(btScalar radius,btScalar height);
	virtual btCollisionShape* createConeShapeY(btScalar radius,btScalar height);
	virtual btCollisionShape* createConeShapeZ(btScalar radius,btScalar height);
	virtual class btTriangleIndexVertexArray*	createTriangleMeshContainer();
	virtual	btBvhTriangleMeshShape* createBvhTriangleMeshShape(btStridingMeshInterface* trimesh, btOptimizedBvh* bvh);
	virtual btCollisionShape* createConvexTriangleMeshShape(btStridingMeshInterface* trimesh);
	virtual btGImpactMeshShape* createGimpactShape(btStridingMeshInterface* trimesh);
	virtual btStridingMeshInterfaceData* createStridingMeshInterfaceData(btStridingMeshInterfaceData* interfaceData);

	virtual class btConvexHullShape* createConvexHullShape();
	virtual class btCompoundShape* createCompoundShape();
	virtual class btScaledBvhTriangleMeshShape* createScaledTrangleMeshShape(btBvhTriangleMeshShape* meshShape,const btVector3& localScalingbtBvhTriangleMeshShape);

	virtual class btMultiSphereShape* createMultiSphereShape(const btVector3* positions,const btScalar* radi,int numSpheres);

	virtual btTriangleIndexVertexArray* createMeshInterface(btStridingMeshInterfaceData& meshData);

	///acceleration and connectivity structures
	virtual btOptimizedBvh*	createOptimizedBvh();
	virtual btTriangleInfoMap* createTriangleInfoMap();

	///constraints
	virtual btPoint2PointConstraint* createPoint2PointConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB);
	virtual btPoint2PointConstraint* createPoint2PointConstraint(btRigidBody& rbA,const btVector3& pivotInA);
	virtual btHingeConstraint* createHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA=false);
	virtual btHingeConstraint* createHingeConstraint(btRigidBody& rbA,const btTransform& rbAFrame, bool useReferenceFrameA=false);
	virtual btConeTwistConstraint* createConeTwistConstraint(btRigidBody& rbA,btRigidBody& rbB,const btTransform& rbAFrame, const btTransform& rbBFrame);
	virtual btConeTwistConstraint* createConeTwistConstraint(btRigidBody& rbA,const btTransform& rbAFrame);
	virtual btGeneric6DofConstraint* createGeneric6DofConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
    virtual btGeneric6DofConstraint* createGeneric6DofConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB);
	virtual btGeneric6DofSpringConstraint* createGeneric6DofSpringConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
	virtual btSliderConstraint* createSliderConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
    virtual btSliderConstraint* createSliderConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameA);
	virtual btGearConstraint* createGearConstraint(btRigidBody& rbA, btRigidBody& rbB, const btVector3& axisInA,const btVector3& axisInB, btScalar ratio);
*/

};

#endif //BASIC_DEMO_PHYSICS_SETUP_H
