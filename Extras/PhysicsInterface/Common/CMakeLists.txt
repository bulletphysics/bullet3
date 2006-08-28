INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/LinearMath  ${BULLET_PHYSICS_SOURCE_DIR}/Extras/PhysicsInterface/Common }
)
	
ADD_LIBRARY(LibPhysicsCommonInterface
	PHY_IMotionState.cpp
	PHY_IPhysicsController.cpp
	PHY_IPhysicsEnvironment.cpp
	PHY_IVehicle.cpp
)