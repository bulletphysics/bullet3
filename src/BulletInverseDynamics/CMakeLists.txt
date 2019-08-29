SET(BulletInverseDynamics_SRCS
	IDMath.cpp
	MultiBodyTree.cpp
	details/MultiBodyTreeInitCache.cpp
	details/MultiBodyTreeImpl.cpp
)

SET(BulletInverseDynamicsRoot_HDRS
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
	"$<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>")
TARGET_LINK_LIBRARIES(BulletInverseDynamics Bullet3Common LinearMath)
TARGET_COMPILE_DEFINITIONS(BulletInverseDynamics PUBLIC
	$<$<STREQUAL:$<TARGET_PROPERTY:BulletInverseDynamics,TYPE>,SHARED_LIBRARY>:BULLET_SHARED>)
SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES SOVERSION ${BULLET_VERSION})

IF(INSTALL_LIBS)
	IF(NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS BulletInverseDynamics DESTINATION .)
		ELSE()
			INSTALL(TARGETS BulletInverseDynamics RUNTIME DESTINATION bin
											LIBRARY DESTINATION lib${LIB_SUFFIX}
											ARCHIVE DESTINATION lib${LIB_SUFFIX})
			INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
				DESTINATION ${INCLUDE_INSTALL_DIR} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
			INSTALL(FILES ../btBulletCollisionCommon.h DESTINATION ${INCLUDE_INSTALL_DIR}/BulletInverseDynamics)
		ENDIF()

		IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(BulletInverseDynamics PROPERTIES PUBLIC_HEADER "${BulletInverseDynamicsRoot_HDRS}")
			# Have to list out sub-directories manually:
			SET_PROPERTY(SOURCE ${BulletInverseDynamicsDetails_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/details)
		ENDIF ()
	ENDIF()
ENDIF()
