SET(Bullet3Dynamics_SRCS
	b3CpuRigidBodyPipeline.cpp
	ConstraintSolver/b3FixedConstraint.cpp
	ConstraintSolver/b3Generic6DofConstraint.cpp
	ConstraintSolver/b3PgsJacobiSolver.cpp
	ConstraintSolver/b3Point2PointConstraint.cpp
	ConstraintSolver/b3TypedConstraint.cpp
)

SET(Bullet3Dynamics_HDRS
	Bullet3DynamicsApi.h
	b3CpuRigidBodyPipeline.h
	ConstraintSolver/b3ContactSolverInfo.h
	ConstraintSolver/b3FixedConstraint.h
	ConstraintSolver/b3Generic6DofConstraint.h
	ConstraintSolver/b3JacobianEntry.h
	ConstraintSolver/b3PgsJacobiSolver.h
	ConstraintSolver/b3Point2PointConstraint.h
	ConstraintSolver/b3SolverBody.h
	ConstraintSolver/b3SolverConstraint.h
	ConstraintSolver/b3TypedConstraint.h
	shared/b3ContactConstraint4.h
	shared/b3ConvertConstraint4.h
	shared/b3Inertia.h
	shared/b3IntegrateTransforms.h
)

ADD_LIBRARY(Bullet3Dynamics ${Bullet3Dynamics_SRCS} ${Bullet3Dynamics_HDRS})
TARGET_INCLUDE_DIRECTORIES(Bullet3Dynamics PUBLIC
	"$<BUILD_INTERFACE:${BULLET_PHYSICS_SOURCE_DIR}/src>"
	"$<INSTALL_INTERFACE:${BULLET_INSTALL_INCLUDEDIR}>")
TARGET_LINK_LIBRARIES(Bullet3Dynamics PUBLIC Bullet3Collision BulletInterface)
SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES SOVERSION ${BULLET_VERSION})

IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	BULLET3_INSTALL(TARGETS Bullet3Dynamics EXPORT BulletLibsExport DESTINATION .)

	SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES FRAMEWORK ON)
	SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES PUBLIC_HEADER "${Bullet3Dynamics_HDRS}")
ELSE()
	BULLET3_INSTALL(TARGETS Bullet3Dynamics EXPORT BulletLibsExport
		RUNTIME DESTINATION ${BULLET_INSTALL_BINDIR}
		LIBRARY DESTINATION ${BULLET_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${BULLET_INSTALL_LIBDIR})
	BULLET3_INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DESTINATION ${BULLET_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h" PATTERN "CMakeFiles" EXCLUDE)
ENDIF()
