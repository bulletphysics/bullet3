SET(LinearMath_SRCS
	btAlignedAllocator.cpp
	btConvexHull.cpp
	btConvexHullComputer.cpp
	btGeometryUtil.cpp
	btPolarDecomposition.cpp
	btQuickprof.cpp
	btSerializer.cpp
	btSerializer64.cpp
	btThreads.cpp
	btVector3.cpp
	TaskScheduler/btTaskScheduler.cpp
	TaskScheduler/btThreadSupportPosix.cpp
	TaskScheduler/btThreadSupportWin32.cpp
)

SET(LinearMath_HDRS
	LinearMathApi.h
	btAabbUtil2.h
	btAlignedAllocator.h
	btAlignedObjectArray.h
	btConvexHull.h
	btConvexHullComputer.h
	btDefaultMotionState.h
	btGeometryUtil.h
	btGrahamScan2dConvexHull.h
	btHashMap.h
	btIDebugDraw.h
	btList.h
	btMatrix3x3.h
	btMinMax.h
	btMotionState.h
	btPolarDecomposition.h
	btPoolAllocator.h
	btQuadWord.h
	btQuaternion.h
	btQuickprof.h
	btRandom.h
	btScalar.h
	btSerializer.h
	btStackAlloc.h
	btThreads.h
	btTransform.h
	btTransformUtil.h
	btVector3.h
	TaskScheduler/btThreadSupportInterface.h
)

ADD_LIBRARY(LinearMath ${LinearMath_SRCS} ${LinearMath_HDRS})
TARGET_INCLUDE_DIRECTORIES(LinearMath PUBLIC
	"$<BUILD_INTERFACE:${BULLET_PHYSICS_SOURCE_DIR}/src>"
	"$<INSTALL_INTERFACE:${BULLET_INSTALL_INCLUDEDIR}>")
TARGET_COMPILE_DEFINITIONS(LinearMath PUBLIC
	$<$<STREQUAL:$<TARGET_PROPERTY:LinearMath,TYPE>,SHARED_LIBRARY>:BULLET_SHARED>)
SET_TARGET_PROPERTIES(LinearMath PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(LinearMath PROPERTIES SOVERSION ${BULLET_VERSION})

IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	BULLET3_INSTALL(TARGETS LinearMath EXPORT BulletLibsExport DESTINATION .)

	SET_TARGET_PROPERTIES(LinearMath PROPERTIES FRAMEWORK ON)
	SET_TARGET_PROPERTIES(LinearMath PROPERTIES PUBLIC_HEADER "${LinearMath_HDRS}")
ELSE()
	BULLET3_INSTALL(TARGETS LinearMath EXPORT BulletLibsExport
		RUNTIME DESTINATION ${BULLET_INSTALL_BINDIR}
		LIBRARY DESTINATION ${BULLET_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${BULLET_INSTALL_LIBDIR})
	BULLET3_INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DESTINATION ${BULLET_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h" PATTERN "CMakeFiles" EXCLUDE)
ENDIF()
