cmake_minimum_required(VERSION 2.4)
   
IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

INCLUDE_DIRECTORIES(${BULLET_PHYSICS_SOURCE_DIR}/src )

#FILE(GLOB INC_FILES ../*.h)

SET (INC_FILES
	DNA_rigidbody.h
	
	${BULLET_PHYSICS_SOURCE_DIR}/src/LinearMath/btVector3.h
	${BULLET_PHYSICS_SOURCE_DIR}/src/LinearMath/btMatrix3x3.h
	${BULLET_PHYSICS_SOURCE_DIR}/src/LinearMath/btTransform.h
	${BULLET_PHYSICS_SOURCE_DIR}/src/BulletCollision/CollisionShapes/btCollisionShape.h
	${BULLET_PHYSICS_SOURCE_DIR}/src/BulletCollision/CollisionShapes/btConvexInternalShape.h
	${BULLET_PHYSICS_SOURCE_DIR}/src/BulletCollision/CollisionDispatch/btCollisionObject.h
)

# Build makesdna executable
SET(SRC makesdna.cpp)
ADD_EXECUTABLE(makesdna ${SRC} ${INC_FILES})

# Output BulletDNA.c
ADD_CUSTOM_COMMAND(
	OUTPUT ${BULLET_PHYSICS_SOURCE_DIR}/src/LinearMath/btSerializer.cpp
	COMMAND ${CMAKE_CFG_INTDIR}/makesdna ${BULLET_PHYSICS_SOURCE_DIR}/src/LinearMath/btSerializer.cpp ${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/CommonSerialize/
	DEPENDS makesdna
)

# Build bf_dna library
SET(SRC  ${BULLET_PHYSICS_SOURCE_DIR}/src/LinearMath/btSerializer.cpp)
ADD_LIBRARY(BulletDNA ${SRC} ${INC_FILES})

MESSAGE(STATUS "Configuring makesdna")
