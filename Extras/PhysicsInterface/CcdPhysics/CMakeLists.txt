INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Extras/PhysicsInterface/Common ${BULLET_PHYSICS_SOURCE_DIR}/Extras/PhysicsInterface/CcdPhysics }
)
	
ADD_LIBRARY(LibCcdPhysicsInterface
		CcdPhysicsController.cpp
		CcdPhysicsEnvironment.cpp
		ParallelIslandDispatcher.cpp
		ParallelPhysicsEnvironment.cpp
		SimulationIsland.cpp
)
