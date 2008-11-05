
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

SET(LibLinearMath_SRCS
		btConvexHull.cpp
		btQuickprof.cpp
		btGeometryUtil.cpp
		btAlignedAllocator.cpp
)

SET(LibLinearMath_HDRS
		btAlignedObjectArray.h
		btList.h
		btPoolAllocator.h
		btRandom.h
		btVector3.h
		btDefaultMotionState.h
		btMatrix3x3.h
		btQuadWord.h
		btHashMap.h
		btScalar.h
		btAabbUtil2.h
		btConvexHull.h
		btMinMax.h
		btQuaternion.h
		btStackAlloc.h
		btGeometryUtil.h
		btMotionState.h
		btTransform.h
		btAlignedAllocator.h
		btIDebugDraw.h
		btQuickprof.h
		btTransformUtil.h
)

# ADD_LIBRARY(LibLinearMath SHARED ${LibLinearMath_SRCS} ${LibLinearMath_HDRS})
ADD_LIBRARY(LibLinearMath ${LibLinearMath_SRCS} ${LibLinearMath_HDRS})

#FILES_MATCHING requires CMake 2.6

IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)
INSTALL(TARGETS LibLinearMath DESTINATION lib)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.h")
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)
