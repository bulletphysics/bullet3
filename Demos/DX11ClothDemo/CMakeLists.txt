cmake_minimum_required(VERSION 2.4)


#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(AppDX11ClothDemo)
SET(BULLET_VERSION 2.76)

IF (NOT CMAKE_BUILD_TYPE)
# SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 


OPTION(USE_MSVC_RUNTIME_LIBRARY_DLL "Use MSVC Runtime Library DLL (/MD or /MDd)" ON)
OPTION(USE_MSVC_INCREMENTAL_LINKING "Use MSVC Incremental Linking" OFF)



IF(MSVC)
	IF (NOT USE_MSVC_INCREMENTAL_LINKING)
		#MESSAGE("MSVC_INCREMENTAL_DEFAULT"+${MSVC_INCREMENTAL_DEFAULT})
		SET( MSVC_INCREMENTAL_YES_FLAG "/INCREMENTAL:NO")
		 
		STRING(REPLACE "INCREMENTAL:YES" "INCREMENTAL:NO" replacementFlags ${CMAKE_EXE_LINKER_FLAGS_DEBUG}) 
		SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "/INCREMENTAL:NO ${replacementFlags}" )
		MESSAGE("CMAKE_EXE_LINKER_FLAGS_DEBUG=${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
		
		STRING(REPLACE "INCREMENTAL:YES" "INCREMENTAL:NO" replacementFlags3 ${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO})
		SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO ${replacementFlags3}) 
		SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "/INCREMENTAL:NO ${replacementFlags3}" )
	ENDIF (NOT USE_MSVC_INCREMENTAL_LINKING)

	IF (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)
		#We statically link to reduce dependancies
		FOREACH(flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
			IF(${flag_var} MATCHES "/MD")
				STRING(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
			ENDIF(${flag_var} MATCHES "/MD")
			IF(${flag_var} MATCHES "/MDd")
				STRING(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
			ENDIF(${flag_var} MATCHES "/MDd")
		ENDFOREACH(flag_var)
	ENDIF (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)

	OPTION(USE_MSVC_SSE "Use MSVC /arch:sse option"	ON)
	IF (USE_MSVC_SSE)
		ADD_DEFINITIONS(/arch:SSE)
	ENDIF()
	OPTION(USE_MSVC_FAST_FLOATINGPOINT "Use MSVC /fp:fast option"	ON)
	IF (USE_MSVC_FAST_FLOATINGPOINT)
		ADD_DEFINITIONS(/fp:fast)
  ENDIF()
ENDIF(MSVC)

   
IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)
   



SET(DXSDK_DIR $ENV{DXSDK_DIR})
SET(DX11_INCLUDE_PATH  "${DXSDK_DIR}Include" CACHE DOCSTRING "Microsoft directX SDK include path")
IF (CMAKE_CL_64)
	SET(DX11_LIB_PATH "${DXSDK_DIR}Lib/x64" CACHE DOCSTRING "Microsoft DirectX SDK library path")
ELSE()
	SET(DX11_LIB_PATH "${DXSDK_DIR}Lib/x86" CACHE DOCSTRING "Microsoft DirectX SDK library path")	
ENDIF()

SET(BulletClothExample_SRCS
	DXUT/Core/DXUT.cpp
	DXUT/Optional/DXUTcamera.cpp
	DXUT/Core/DXUTDevice11.cpp
	DXUT/Core/DXUTDevice9.cpp
	DXUT/Optional/DXUTgui.cpp
	DXUT/Core/DXUTmisc.cpp
	DXUT/Optional/DXUTres.cpp
	DXUT/Optional/DXUTsettingsdlg.cpp
	DXUT/Optional/SDKmesh.cpp
	DXUT/Optional/SDKmisc.cpp	

	cloth_renderer.cpp
)


SET(BulletClothExample_HDRS
	DXUT/Core/DXUT.h
	DXUT/Optional/DXUTcamera.h
	DXUT/Core/DXUTDevice11.h
	DXUT/Core/DXUTDevice9.h
	DXUT/Optional/DXUTgui.h
	DXUT/Core/DXUTmisc.h
	DXUT/Optional/DXUTres.h
	DXUT/Optional/DXUTsettingsdlg.h
	DXUT/Optional/SDKmesh.h
	DXUT/Optional/SDKmisc.h

	btDirectComputeSupport.h
	cap.h
	cloth.h
	cylinder.h
#	FreeImage.h
	resource.h
)


INCLUDE_DIRECTORIES(
	${DX11_INCLUDE_PATH} 
	${BULLET_PHYSICS_SOURCE_DIR}/src  
	"DXUT/Core"
	"DXUT/Optional"
	${VECTOR_MATH_INCLUDE}
)
LINK_DIRECTORIES(
${DX11_LIB_PATH}
)

ADD_DEFINITIONS(-DUNICODE)
ADD_DEFINITIONS(-D_UNICODE)

ADD_EXECUTABLE(AppDX11ClothDemo WIN32
	${BulletClothExample_SRCS} 
	${BulletClothExample_HDRS} 
)
TARGET_LINK_LIBRARIES(AppDX11ClothDemo
	d3dcompiler
	dxerr
	dxguid
	d3dx9
	d3d9
	winmm
	comctl32
	d3dx11
	BulletDynamics
	BulletCollision
	LinearMath
	BulletSoftBody
	BulletSoftBodySolvers_DX11
)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
		SET_TARGET_PROPERTIES(AppDX11ClothDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
		SET_TARGET_PROPERTIES(AppDX11ClothDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
		SET_TARGET_PROPERTIES(AppDX11ClothDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF()


IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ADD_CUSTOM_COMMAND(
		TARGET AppDX11ClothDemo
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/cloth_renderer_PS.hlsl ${CMAKE_CURRENT_BINARY_DIR}/cloth_renderer_PS.hlsl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/cloth_renderer_VS.hlsl ${CMAKE_CURRENT_BINARY_DIR}/cloth_renderer_VS.hlsl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/texture.bmp ${CMAKE_CURRENT_BINARY_DIR}/texture.bmp
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/amdFlag.bmp  ${CMAKE_CURRENT_BINARY_DIR}/amdFlag.bmp
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/atiFlag.bmp  ${CMAKE_CURRENT_BINARY_DIR}/atiFlag.bmp
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/texture.png ${CMAKE_CURRENT_BINARY_DIR}/texture.png
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/Media/UI/Font.dds ${CMAKE_CURRENT_BINARY_DIR}/Media/UI/Font.dds
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/DX11ClothDemo/Media/Tiny/tiny.sdkmesh ${CMAKE_CURRENT_BINARY_DIR}/Media/Tiny/tiny.sdkmesh
		
	)
ENDIF()
	