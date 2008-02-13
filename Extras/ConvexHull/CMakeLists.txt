INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexHull
)
	
ADD_LIBRARY(LibConvexHull
btConvexHull.h
btConvexHull.cpp
btShapeHull.cpp
btShapeHull.h
)
