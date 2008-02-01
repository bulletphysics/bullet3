
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

ADD_LIBRARY(LibLinearMath
		btAlignedObjectArray.h
		btList.h
		btPoolAllocator.h
		btRandom.h
		btVector3.h
		btDefaultMotionState.h
		btMatrix3x3.h
		btQuadWord.h
		btScalar.h
		btAabbUtil2.h
		btMinMax.h
		btQuaternion.h
		btStackAlloc.h
		btConvexHull.h
		btGeometryUtil.h
		btMotionState.h
		btTransform.h
		btAlignedAllocator.h
		btIDebugDraw.h
		btPoint3.h
		btQuickprof.h
		btTypedUserInfo.h
		btTransformUtil.h
		btQuickprof.cpp
		btConvexHull.cpp
		btGeometryUtil.cpp
		btAlignedAllocator.cpp
)

