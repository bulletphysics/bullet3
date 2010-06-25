# AppMiniCLVectorAdd is a very basic test for MiniCL.


ADD_DEFINITIONS(-DUSE_MINICL)

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
)

LINK_LIBRARIES(
 BulletMultiThreaded LinearMath 
)

ADD_EXECUTABLE(AppVectorAdd_Mini
../MiniCL_VectorAdd.cpp
../VectorAddKernels.cl
)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppVectorAdd_Mini pthread)
ENDIF(UNIX)

