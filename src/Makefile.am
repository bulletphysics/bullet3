noinst_LIBRARIES	= libbulletmath.a libbulletcollision.a libbulletdynamics.a

libbulletmath_a_SOURCES	= \
	LinearMath/btAabbUtil2.h  LinearMath/btList.h LinearMath/btQuickprof.h	 \
	LinearMath/btAlignedAllocator.cpp 	LinearMath/btMatrix3x3.h	 \
	LinearMath/btRandom.h 			LinearMath/btAlignedAllocator.h  \
	LinearMath/btMinMax.h	 		LinearMath/btScalar.h		 \
	LinearMath/btAlignedObjectArray.h	LinearMath/btMotionState.h	 \
	LinearMath/btSimdMinMax.h 		LinearMath/btDefaultMotionState.h\
	LinearMath/btPoint3.h	 		LinearMath/btStackAlloc.h	 \
	LinearMath/btGeometryUtil.cpp		LinearMath/btQuadWord.h		 \
	LinearMath/btTransform.h 		LinearMath/btGeometryUtil.h	 \
	LinearMath/btQuaternion.h		LinearMath/btTransformUtil.h	 \
	LinearMath/btIDebugDraw.h		LinearMath/btQuickprof.cpp  	 \
	LinearMath/btVector3.h

libbulletcollision_a_SOURCES := \
	BulletCollision/BroadphaseCollision/btAxisSweep3.cpp			 \
	BulletCollision/BroadphaseCollision/btAxisSweep3.h			 \
	BulletCollision/BroadphaseCollision/btBroadphaseInterface.h		 \
	BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp		 \
	BulletCollision/BroadphaseCollision/btBroadphaseProxy.h			 \
	BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp		 \
	BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h		 \
	BulletCollision/BroadphaseCollision/btDispatcher.cpp			 \
	BulletCollision/BroadphaseCollision/btDispatcher.h			 \
	BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp		 \
	BulletCollision/BroadphaseCollision/btOverlappingPairCache.h		 \
	BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp		 \
	BulletCollision/BroadphaseCollision/btSimpleBroadphase.h		 \
	BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp		 \
	BulletCollision/CollisionDispatch/SphereTriangleDetector.h		 \
	BulletCollision/CollisionDispatch/btCollisionCreateFunc.h		 \
	BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp		 \
	BulletCollision/CollisionDispatch/btCollisionDispatcher.h		 \
	BulletCollision/CollisionDispatch/btCollisionObject.cpp			 \
	BulletCollision/CollisionDispatch/btCollisionObject.h			 \
	BulletCollision/CollisionDispatch/btCollisionWorld.cpp			 \
	BulletCollision/CollisionDispatch/btCollisionWorld.h			 \
	BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp	 \
	BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h	 \
	BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp	 \
	BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h	 \
	BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp		 \
	BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h		 \
	BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp		 \
	BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h		 \
	BulletCollision/CollisionDispatch/btManifoldResult.cpp			 \
	BulletCollision/CollisionDispatch/btManifoldResult.h			 \
	BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp		 \
	BulletCollision/CollisionDispatch/btSimulationIslandManager.h		 \
	BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp	 \
	BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h	 \
	BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp	 \
	BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h	 \
	BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp \
	BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h	 \
	BulletCollision/CollisionDispatch/btUnionFind.cpp			 \
	BulletCollision/CollisionDispatch/btUnionFind.h				 \
	BulletCollision/CollisionShapes/btBoxShape.cpp				 \
	BulletCollision/CollisionShapes/btConeShape.cpp				 \
	BulletCollision/CollisionShapes/btEmptyShape.h				 \
	BulletCollision/CollisionShapes/btSphereShape.cpp			 \
	BulletCollision/CollisionShapes/btTriangleCallback.h			 \
	BulletCollision/CollisionShapes/btBoxShape.h				 \
	BulletCollision/CollisionShapes/btConeShape.h				 \
	BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp		 \
	BulletCollision/CollisionShapes/btSphereShape.h				 \
	BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp		 \
	BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp		 \
	BulletCollision/CollisionShapes/btConvexHullShape.cpp			 \
	BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h    		 \
	BulletCollision/CollisionShapes/btStaticPlaneShape.cpp			 \
	BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h		 \
	BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h   	 	 \
	BulletCollision/CollisionShapes/btConvexHullShape.h			 \
	BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp			 \
	BulletCollision/CollisionShapes/btStaticPlaneShape.h			 \
	BulletCollision/CollisionShapes/btTriangleMesh.cpp			 \
	BulletCollision/CollisionShapes/btCollisionMargin.h			 \
	BulletCollision/CollisionShapes/btConvexShape.cpp			 \
	BulletCollision/CollisionShapes/btMinkowskiSumShape.h			 \
	BulletCollision/CollisionShapes/btStridingMeshInterface.cpp		 \
	BulletCollision/CollisionShapes/btTriangleMesh.h			 \
	BulletCollision/CollisionShapes/btCollisionShape.cpp			 \
	BulletCollision/CollisionShapes/btConvexShape.h			 	 \
	BulletCollision/CollisionShapes/btMultiSphereShape.cpp			 \
	BulletCollision/CollisionShapes/btStridingMeshInterface.h		 \
	BulletCollision/CollisionShapes/btTriangleMeshShape.cpp			 \
	BulletCollision/CollisionShapes/btCollisionShape.h			 \
	BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp		 \
	BulletCollision/CollisionShapes/btMultiSphereShape.h			 \
	BulletCollision/CollisionShapes/btTetrahedronShape.cpp			 \
	BulletCollision/CollisionShapes/btTriangleMeshShape.h			 \
	BulletCollision/CollisionShapes/btCompoundShape.cpp			 \
	BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h		 \
	BulletCollision/CollisionShapes/btOptimizedBvh.cpp			 \
	BulletCollision/CollisionShapes/btTetrahedronShape.h			 \
	BulletCollision/CollisionShapes/btTriangleShape.h			 \
	BulletCollision/CollisionShapes/btCompoundShape.h			 \
	BulletCollision/CollisionShapes/btCylinderShape.cpp			 \
	BulletCollision/CollisionShapes/btOptimizedBvh.h			 \
	BulletCollision/CollisionShapes/btTriangleBuffer.cpp			 \
	BulletCollision/CollisionShapes/btConcaveShape.cpp			 \
	BulletCollision/CollisionShapes/btCylinderShape.h			 \
	BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp		 \
	BulletCollision/CollisionShapes/btTriangleBuffer.h			 \
	BulletCollision/CollisionShapes/btConcaveShape.h			 \
	BulletCollision/CollisionShapes/btEmptyShape.cpp			 \
	BulletCollision/CollisionShapes/btPolyhedralConvexShape.h		 \
	BulletCollision/CollisionShapes/btTriangleCallback.cpp			 \
	BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp	 \
	BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h			 \
	BulletCollision/NarrowPhaseCollision/btManifoldPoint.h			 \
	BulletCollision/NarrowPhaseCollision/btRaycastCallback.h		 \
	BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h	 \
	BulletCollision/NarrowPhaseCollision/btGjkEpa.cpp			 \
	BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp\
	BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h		 \
	BulletCollision/NarrowPhaseCollision/btConvexCast.cpp			 \
	BulletCollision/NarrowPhaseCollision/btGjkEpa.h				 \
	BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h \
	BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp		 \
	BulletCollision/NarrowPhaseCollision/btConvexCast.h			 \
	BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp	 \
	BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp		 \
	BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h		 \
	BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h	 \
	BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h	 \
	BulletCollision/NarrowPhaseCollision/btPersistentManifold.h		 \
	BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp		 \
	BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h\
	BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp		 \
	BulletCollision/NarrowPhaseCollision/btPointCollector.h			 \
	BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h		 \
	BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp		 \
	BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h		 \
	BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp

libbulletdynamics_a_SOURCES	= \
	BulletDynamics/ConstraintSolver/btConstraintSolver.h			 \
	BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp  		 \
	BulletDynamics/ConstraintSolver/btJacobianEntry.h			 \
	BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h	 \
	BulletDynamics/ConstraintSolver/btTypedConstraint.h			 \
	BulletDynamics/ConstraintSolver/btContactConstraint.cpp			 \
	BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h		 \
	BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp		 \
	BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp		 \
	BulletDynamics/ConstraintSolver/btContactConstraint.h			 \
	BulletDynamics/ConstraintSolver/btHingeConstraint.cpp			 \
	BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h		 \
	BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h		 \
	BulletDynamics/ConstraintSolver/btContactSolverInfo.h			 \
	BulletDynamics/ConstraintSolver/btHingeConstraint.h			 \
	BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp	 \
	BulletDynamics/ConstraintSolver/btTypedConstraint.cpp			 \
	BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp			 \
	BulletDynamics/Dynamics/btDynamicsWorld.h				 \
	BulletDynamics/Dynamics/btRigidBody.h 					 \
	BulletDynamics/Dynamics/btSimpleDynamicsWorld.h				 \
	BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h			 \
	BulletDynamics/Dynamics/btRigidBody.cpp					 \
	BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp			 \
	BulletDynamics/Vehicle/btRaycastVehicle.cpp				 \
	BulletDynamics/Vehicle/btRaycastVehicle.h				 \
	BulletDynamics/Vehicle/btVehicleRaycaster.h				 \
	BulletDynamics/Vehicle/btWheelInfo.cpp					 \
	BulletDynamics/Vehicle/btWheelInfo.h
