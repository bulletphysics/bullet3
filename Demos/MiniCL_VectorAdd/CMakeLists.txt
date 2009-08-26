# AppMiniCLVectorAdd is a very basic test for MiniCL.


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

LINK_LIBRARIES(
 BulletMultiThreaded LinearMath 
)

ADD_EXECUTABLE(AppMiniCLVectorAdd
MiniCL_VectorAdd.cpp
MiniCL.cpp
)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppMiniCLVectorAdd pthread)
ENDIF(UNIX)

