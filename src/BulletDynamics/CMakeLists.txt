INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)
	
SET(LibBulletDynamics_SRCS
	ConstraintSolver/btContactConstraint.cpp
	ConstraintSolver/btConeTwistConstraint.cpp
	ConstraintSolver/btGeneric6DofConstraint.cpp
	ConstraintSolver/btHingeConstraint.cpp
	ConstraintSolver/btPoint2PointConstraint.cpp
	ConstraintSolver/btSequentialImpulseConstraintSolver.cpp
	ConstraintSolver/btSliderConstraint.cpp
	ConstraintSolver/btSolve2LinearConstraint.cpp
	ConstraintSolver/btTypedConstraint.cpp
	Dynamics/Bullet-C-API.cpp
	Dynamics/btDiscreteDynamicsWorld.cpp
	Dynamics/btSimpleDynamicsWorld.cpp
	Dynamics/btRigidBody.cpp
	Vehicle/btRaycastVehicle.cpp
	Vehicle/btWheelInfo.cpp
)

SET(LibBulletDynamics_HDRS
	ConstraintSolver/btContactConstraint.h
	ConstraintSolver/btConeTwistConstraint.h
	ConstraintSolver/btGeneric6DofConstraint.h
	ConstraintSolver/btHingeConstraint.h
	ConstraintSolver/btPoint2PointConstraint.h
	ConstraintSolver/btSequentialImpulseConstraintSolver.h
	ConstraintSolver/btSliderConstraint.h
	ConstraintSolver/btSolve2LinearConstraint.h
	ConstraintSolver/btTypedConstraint.h
	Dynamics/btDiscreteDynamicsWorld.h
	Dynamics/btSimpleDynamicsWorld.h
	Dynamics/btRigidBody.h
	Vehicle/btRaycastVehicle.h
	Vehicle/btWheelInfo.h
)



#ADD_LIBRARY(LibBulletDynamics SHARED ${LibBulletDynamics_SRCS} ${LibBulletDynamics_HDRS})
#TARGET_LINK_LIBRARIES(LibBulletDynamics LibBulletCollision LibLinearMath)

ADD_LIBRARY(LibBulletDynamics ${LibBulletDynamics_SRCS} ${LibBulletDynamics_HDRS})

IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)
	#INSTALL(TARGETS LibBulletDynamics DESTINATION lib)
	#INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.h")
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)
