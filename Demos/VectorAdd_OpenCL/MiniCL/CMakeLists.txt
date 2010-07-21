# AppMiniCLVectorAdd is a very basic test for MiniCL.


ADD_DEFINITIONS(-DUSE_MINICL)

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
)

LINK_LIBRARIES(
	MiniCL
	BulletMultiThreaded 
	LinearMath
)

ADD_EXECUTABLE(AppVectorAdd_Mini
../MiniCL_VectorAdd.cpp
../VectorAddKernels.cl
)

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppVectorAdd_Mini pthread)
ENDIF(UNIX)


IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ADD_CUSTOM_COMMAND(
		TARGET AppVectorAdd_Mini
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/VectorAdd_OpenCL/VectorAddKernels.cl ${CMAKE_CURRENT_BINARY_DIR}
	)
ENDIF()
				