SET(Bullet3Dynamics_SRCS
	b3CpuRigidBodyPipeline.cpp
	ConstraintSolver/b3FixedConstraint.cpp
	ConstraintSolver/b3Generic6DofConstraint.cpp
	ConstraintSolver/b3PgsJacobiSolver.cpp
	ConstraintSolver/b3Point2PointConstraint.cpp
	ConstraintSolver/b3TypedConstraint.cpp
)

SET(Bullet3Dynamics_HDRS
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
	"$<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>")
TARGET_LINK_LIBRARIES(Bullet3Dynamics Bullet3Collision)
SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES SOVERSION ${BULLET_VERSION})

IF(INSTALL_LIBS)
	IF(NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS Bullet3Dynamics DESTINATION .)
		ELSE()
			INSTALL(TARGETS Bullet3Dynamics
				RUNTIME DESTINATION bin
				LIBRARY DESTINATION lib${LIB_SUFFIX}
				ARCHIVE DESTINATION lib${LIB_SUFFIX})
			INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
				DESTINATION ${INCLUDE_INSTALL_DIR} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
		ENDIF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)

		IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(Bullet3Dynamics PROPERTIES PUBLIC_HEADER "${Bullet3Dynamics_HDRS}")
		ENDIF()
	ENDIF()
ENDIF()
