# AppVectorAdd is a very basic test OpenCL/MiniCL.

IF (APPLE)
  FIND_LIBRARY(OPENCL_LIBRARY OpenCL DOC "OpenCL lib for OSX")
  FIND_PATH(OPENCL_INCLUDE_DIR OpenCL/cl.h DOC "Include for OpenCL on OSX")
ENDIF (APPLE)


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
)

LINK_LIBRARIES(
  LinearMath ${OPENCL_LIBRARY}
)

ADD_EXECUTABLE(AppVectorAdd_Apple
../MiniCL_VectorAdd.cpp
../VectorAddKernels.cl
)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppVectorAdd_Apple pthread)
ENDIF(UNIX)

