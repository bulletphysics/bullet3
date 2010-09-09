



INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${NVIDIA_OPENCL_INCLUDES}
)

LINK_LIBRARIES(
	 LinearMath 
	${NVIDIA_OPENCL_LIBRARIES}
)

ADD_EXECUTABLE(AppVectorAdd_NVidia
../MiniCL_VectorAdd.cpp
../VectorAddKernels.cl
)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppVectorAdd_NVidia pthread)
ENDIF(UNIX)

