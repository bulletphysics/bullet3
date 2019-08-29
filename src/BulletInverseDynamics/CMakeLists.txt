SET(BulletInverseDynamics_SRCS
	IDMath.cpp
	MultiBodyTree.cpp
	details/MultiBodyTreeInitCache.cpp
	details/MultiBodyTreeImpl.cpp
)

SET(BulletInverseDynamicsRoot_HDRS
	BulletInverseDynamicsApi.h
	IDConfig.hpp
	IDConfigEigen.hpp
	IDMath.hpp
	IDConfigBuiltin.hpp
	IDErrorMessages.hpp
	MultiBodyTree.hpp
)
SET(BulletInverseDynamicsDetails_HDRS
	details/IDEigenInterface.hpp
	details/IDMatVec.hpp
	details/IDLinearMathInterface.hpp
	details/MultiBodyTreeImpl.hpp
	details/MultiBodyTreeInitCache.hpp
)

SET(BulletInverseDynamics_HDRS
	${BulletInverseDynamicsRoot_HDRS}
	${BulletInverseDynamicsDetails_HDRS}
)

ADD_LIBRARY(BulletInverseDynamics ${BulletInverseDynamics_SRCS} ${BulletInverseDynamics_HDRS})
TARGET_INCLUDE_DIRECTORIES(BulletInverseDynamics PUBLIC
	"$<BUILD_INTERFACE:${BULLET_PHYSICS_SOURCE_DIR}/src>"
	"$<INSTALL_INTERFACE:${BULLET_INSTALL_INCLUDEDIR}>")
TARGET_LINK_LIBRARIES(BulletInverseDynamics Bullet3Common LinearMath)
TARGET_COMPILE_DEFINITIONS(BulletInverseDynamics PUBLIC
	$<$<STREQUAL:$<TARGET_PROPERTY:BulletInverseDynamics,TYPE>,SHARED_LIBRARY>:BULLET_SHARED>)
SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES SOVERSION ${BULLET_VERSION})

IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	BULLET3_INSTALL(TARGETS BulletInverseDynamics EXPORT BulletLibsExport DESTINATION .)

	SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES FRAMEWORK ON)
	SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES PUBLIC_HEADER "${BulletInverseDynamicsRoot_HDRS}")
	# Have to list out sub-directories manually:
	SET_PROPERTY(SOURCE ${BulletInverseDynamicsDetails_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/details)
ELSE()
	BULLET3_INSTALL(TARGETS BulletInverseDynamics EXPORT BulletLibsExport
		RUNTIME DESTINATION ${BULLET_INSTALL_BINDIR}
		LIBRARY DESTINATION ${BULLET_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${BULLET_INSTALL_LIBDIR})
	BULLET3_INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DESTINATION ${BULLET_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h" PATTERN "CMakeFiles" EXCLUDE)
	BULLET3_INSTALL(FILES ../btBulletCollisionCommon.h DESTINATION ${BULLET_INSTALL_INCLUDEDIR}/BulletInverseDynamics)
ENDIF()
