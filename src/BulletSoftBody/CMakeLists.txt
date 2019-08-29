SET(BulletSoftBody_SRCS
	btSoftBody.cpp
	btSoftBodyConcaveCollisionAlgorithm.cpp
	btSoftBodyHelpers.cpp
	btSoftBodyRigidBodyCollisionConfiguration.cpp
	btSoftRigidCollisionAlgorithm.cpp
	btSoftRigidDynamicsWorld.cpp
	btSoftMultiBodyDynamicsWorld.cpp
	btSoftSoftCollisionAlgorithm.cpp
	btDefaultSoftBodySolver.cpp

	btDeformableBackwardEulerObjective.cpp
	btDeformableBodySolver.cpp
	btDeformableMultiBodyConstraintSolver.cpp
	btDeformableContactProjection.cpp
	btDeformableMultiBodyDynamicsWorld.cpp
	btDeformableContactConstraint.cpp
)

SET(BulletSoftBody_HDRS
	BulletSoftBodyApi.h

	btSoftBody.h
	btSoftBodyData.h
	btSoftBodyConcaveCollisionAlgorithm.h
	btSoftBodyHelpers.h
	btSoftBodyRigidBodyCollisionConfiguration.h
	btSoftRigidCollisionAlgorithm.h
	btSoftRigidDynamicsWorld.h
	btSoftMultiBodyDynamicsWorld.h
	btSoftSoftCollisionAlgorithm.h
	btSparseSDF.h

	btSoftBodySolvers.h
	btDefaultSoftBodySolver.h

	btCGProjection.h
	btConjugateGradient.h
	btDeformableGravityForce.h
	btDeformableMassSpringForce.h
	btDeformableCorotatedForce.h
	btDeformableNeoHookeanForce.h
	btDeformableLinearElasticityForce.h
	btDeformableLagrangianForce.h
	btPreconditioner.h

	btDeformableBackwardEulerObjective.h
	btDeformableBodySolver.h
	btDeformableMultiBodyConstraintSolver.h
	btDeformableContactProjection.h
	btDeformableMultiBodyDynamicsWorld.h
	btDeformableContactConstraint.h

	btSoftBodySolverVertexBuffer.h
)

ADD_LIBRARY(BulletSoftBody ${BulletSoftBody_SRCS} ${BulletSoftBody_HDRS})
TARGET_INCLUDE_DIRECTORIES(BulletSoftBody PUBLIC
	"$<BUILD_INTERFACE:${BULLET_PHYSICS_SOURCE_DIR}/src>"
	"$<INSTALL_INTERFACE:${BULLET_INSTALL_INCLUDEDIR}>")
TARGET_LINK_LIBRARIES(BulletSoftBody BulletDynamics)
TARGET_COMPILE_DEFINITIONS(BulletSoftBody PUBLIC
	$<$<STREQUAL:$<TARGET_PROPERTY:BulletSoftBody,TYPE>,SHARED_LIBRARY>:BULLET_SHARED>)
SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES SOVERSION ${BULLET_VERSION})

IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	BULLET3_INSTALL(TARGETS BulletSoftBody EXPORT BulletLibsExport DESTINATION .)

	SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES FRAMEWORK ON)
	SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES PUBLIC_HEADER "${BulletSoftBody_HDRS}")
ELSE()
	BULLET3_INSTALL(TARGETS BulletSoftBody EXPORT BulletLibsExport
		RUNTIME DESTINATION ${BULLET_INSTALL_BINDIR}
		LIBRARY DESTINATION ${BULLET_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${BULLET_INSTALL_LIBDIR})
	BULLET3_INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DESTINATION ${BULLET_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h" PATTERN "CMakeFiles" EXCLUDE)
ENDIF()
