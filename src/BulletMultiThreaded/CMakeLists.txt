INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/vectormath/scalar/cpp
)

ADD_LIBRARY(BulletMultiThreaded
		PlatformDefinitions.h
		SpuFakeDma.cpp
		SpuFakeDma.h
		SpuSync.h
		SpuDoubleBuffer.h
		SpuLibspe2Support.cpp
		SpuLibspe2Support.h
		btThreadSupportInterface.cpp
		btThreadSupportInterface.h
		
		Win32ThreadSupport.cpp
		Win32ThreadSupport.h
		PosixThreadSupport.cpp
		PosixThreadSupport.h
		SequentialThreadSupport.cpp
		SequentialThreadSupport.h
		SpuSampleTaskProcess.h
		SpuSampleTaskProcess.cpp

		SpuCollisionObjectWrapper.cpp 
		SpuCollisionObjectWrapper.h 
		SpuCollisionTaskProcess.h
		SpuCollisionTaskProcess.cpp
		SpuGatheringCollisionDispatcher.h
		SpuGatheringCollisionDispatcher.cpp
		SpuContactManifoldCollisionAlgorithm.cpp
		SpuContactManifoldCollisionAlgorithm.h
		SpuNarrowPhaseCollisionTask/Box.h
		SpuNarrowPhaseCollisionTask/boxBoxDistance.cpp
		SpuNarrowPhaseCollisionTask/boxBoxDistance.h
		SpuNarrowPhaseCollisionTask/SpuContactResult.cpp
		SpuNarrowPhaseCollisionTask/SpuContactResult.h
		SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.cpp
		SpuNarrowPhaseCollisionTask/SpuEpaPenetrationDepthSolver.cpp
		SpuNarrowPhaseCollisionTask/SpuEpaPenetrationDepthSolver.h
		SpuNarrowPhaseCollisionTask/SpuGjkEpa2.cpp
		SpuNarrowPhaseCollisionTask/SpuGjkEpa2.h
		SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.h
		SpuNarrowPhaseCollisionTask/SpuConvexPenetrationDepthSolver.h
		SpuNarrowPhaseCollisionTask/SpuPreferredPenetrationDirections.h
		SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.cpp
		SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h
		SpuNarrowPhaseCollisionTask/SpuVoronoiSimplexSolver.cpp
		SpuNarrowPhaseCollisionTask/SpuVoronoiSimplexSolver.h
		SpuNarrowPhaseCollisionTask/SpuGjkPairDetector.cpp
		SpuNarrowPhaseCollisionTask/SpuGjkPairDetector.h
		SpuNarrowPhaseCollisionTask/SpuCollisionShapes.cpp
		SpuNarrowPhaseCollisionTask/SpuCollisionShapes.h

		SpuParallelSolver.cpp
		SpuParallelSolver.h
		SpuSolverTask/SpuParallellSolverTask.cpp
		SpuSolverTask/SpuParallellSolverTask.h

		SpuBatchRaycaster.cpp
		SpuBatchRaycaster.h
		SpuRaycastTaskProcess.cpp
		SpuRaycastTaskProcess.h
		SpuRaycastTask/SpuRaycastTask.cpp
		SpuRaycastTask/SpuRaycastTask.h
		SpuRaycastTask/SpuSubSimplexConvexCast.cpp
		SpuRaycastTask/SpuSubSimplexConvexCast.h
)

IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletMultiThreaded BulletCollision)
ENDIF (BUILD_SHARED_LIBS)
