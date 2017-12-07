
INCLUDE_DIRECTORIES(
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
)

SET(BulletRobotics_SRCS
		../../examples/SharedMemory/IKTrajectoryHelper.cpp
		../../examples/SharedMemory/IKTrajectoryHelper.h
		../../examples/SharedMemory/TinyRendererVisualShapeConverter.cpp
		../../examples/SharedMemory/TinyRendererVisualShapeConverter.h
		../../examples/OpenGLWindow/SimpleCamera.cpp
		../../examples/OpenGLWindow/SimpleCamera.h
		../../examples/TinyRenderer/geometry.cpp
		../../examples/TinyRenderer/model.cpp
		../../examples/TinyRenderer/tgaimage.cpp
		../../examples/TinyRenderer/our_gl.cpp
		../../examples/TinyRenderer/TinyRenderer.cpp
		../../examples/SharedMemory/InProcessMemory.cpp
		../../examples/SharedMemory/PhysicsClient.cpp
		../../examples/SharedMemory/PhysicsClient.h
		../../examples/SharedMemory/PhysicsServer.cpp
		../../examples/SharedMemory/PhysicsServer.h
		../../examples/SharedMemory/PhysicsServerSharedMemory.cpp
		../../examples/SharedMemory/PhysicsServerSharedMemory.h
		../../examples/SharedMemory/PhysicsDirect.cpp
		../../examples/SharedMemory/PhysicsDirect.h
		../../examples/SharedMemory/PhysicsDirectC_API.cpp
		../../examples/SharedMemory/PhysicsDirectC_API.h
		../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp
		../../examples/SharedMemory/PhysicsServerCommandProcessor.h
		../../examples/SharedMemory/b3PluginManager.cpp
		../../examples/SharedMemory/b3PluginManager.h
				
		../../examples/SharedMemory/PhysicsClientSharedMemory.cpp
		../../examples/SharedMemory/PhysicsClientSharedMemory.h
		../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp
		../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h
		../../examples/SharedMemory/PhysicsClientC_API.cpp
	
		../../examples/SharedMemory/PhysicsClientC_API.h
		../../examples/SharedMemory/SharedMemoryPublic.h

		../../examples/SharedMemory/Win32SharedMemory.cpp
		../../examples/SharedMemory/Win32SharedMemory.h
		../../examples/SharedMemory/PosixSharedMemory.cpp
		../../examples/SharedMemory/PosixSharedMemory.h

		../../examples/Utils/b3ResourcePath.cpp
		../../examples/Utils/b3ResourcePath.h
		../../examples/Utils/RobotLoggingUtil.cpp
		../../examples/Utils/RobotLoggingUtil.h
		../../examples/Utils/b3Clock.cpp
		../../examples/Utils/b3ResourcePath.cpp
		../../examples/Utils/b3ERPCFMHelper.hpp
		../../examples/Utils/b3ReferenceFrameHelper.hpp
		../../examples/Utils/ChromeTraceUtil.cpp

		../../examples/ThirdPartyLibs/tinyxml/tinystr.cpp
		../../examples/ThirdPartyLibs/tinyxml/tinyxml.cpp
		../../examples/ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
		../../examples/ThirdPartyLibs/tinyxml/tinyxmlparser.cpp

		../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
		../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h

		../../examples/ThirdPartyLibs/stb_image/stb_image.cpp

		../../examples/ThirdPartyLibs/BussIK/Jacobian.cpp
		../../examples/ThirdPartyLibs/BussIK/LinearR2.cpp
		../../examples/ThirdPartyLibs/BussIK/LinearR3.cpp
		../../examples/ThirdPartyLibs/BussIK/LinearR4.cpp
		../../examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp
		../../examples/ThirdPartyLibs/BussIK/Misc.cpp
		../../examples/ThirdPartyLibs/BussIK/Node.cpp
		../../examples/ThirdPartyLibs/BussIK/Tree.cpp
		../../examples/ThirdPartyLibs/BussIK/VectorRn.cpp

		../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
		../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
		../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
		../../examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp
		../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
		../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
		../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
		../../examples/Importers/ImportURDFDemo/UrdfParser.cpp
		../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
		../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp 

		../../examples/MultiThreading/b3PosixThreadSupport.cpp
		../../examples/MultiThreading/b3Win32ThreadSupport.cpp
		../../examples/MultiThreading/b3ThreadSupportInterface.cpp
		

)

IF(BUILD_CLSOCKET)
 ADD_DEFINITIONS(-DBT_ENABLE_CLSOCKET)
ENDIF(BUILD_CLSOCKET)

IF(WIN32)
	LINK_LIBRARIES(
		${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)
	IF(BUILD_ENET)
		ADD_DEFINITIONS(-DWIN32 -DBT_ENABLE_ENET)
	ENDIF(BUILD_ENET)
	IF(BUILD_CLSOCKET)
		ADD_DEFINITIONS(-DWIN32)
	ENDIF(BUILD_CLSOCKET)

ELSE(WIN32)
	IF(BUILD_ENET)
		ADD_DEFINITIONS(-DHAS_SOCKLEN_T -DBT_ENABLE_ENET)
	ENDIF(BUILD_ENET)

	IF(BUILD_CLSOCKET)	
		ADD_DEFINITIONS(${OSDEF})
	ENDIF(BUILD_CLSOCKET)
ENDIF(WIN32)

IF(BUILD_ENET)
	set(BulletRobotics_SRCS ${BulletRobotics_SRCS} 
		../../examples/SharedMemory/PhysicsClientUDP.cpp
		../../examples/SharedMemory/PhysicsClientUDP_C_API.cpp
		../../examples/SharedMemory/PhysicsClientUDP.h		
		../../examples/SharedMemory/PhysicsClientUDP_C_API.h
		../../examples/ThirdPartyLibs/enet/win32.c
		../../examples/ThirdPartyLibs/enet/unix.c
		../../examples/ThirdPartyLibs/enet/callbacks.c
		../../examples/ThirdPartyLibs/enet/compress.c
		../../examples/ThirdPartyLibs/enet/host.c
		../../examples/ThirdPartyLibs/enet/list.c
		../../examples/ThirdPartyLibs/enet/packet.c
		../../examples/ThirdPartyLibs/enet/peer.c
		../../examples/ThirdPartyLibs/enet/protocol.c
	)

ENDIF(BUILD_ENET)

IF(BUILD_CLSOCKET)
	set(BulletRobotics_SRCS ${BulletRobotics_SRCS}
		../../examples/SharedMemory/PhysicsClientTCP.cpp
		../../examples/SharedMemory/PhysicsClientTCP.h
		../../examples/SharedMemory/PhysicsClientTCP_C_API.cpp
		../../examples/SharedMemory/PhysicsClientTCP_C_API.h
		../../examples/ThirdPartyLibs/clsocket/src/SimpleSocket.cpp
		../../examples/ThirdPartyLibs/clsocket/src/ActiveSocket.cpp
		../../examples/ThirdPartyLibs/clsocket/src/PassiveSocket.cpp
	)
ENDIF()

ADD_DEFINITIONS(-DPHYSICS_SERVER_DIRECT)


ADD_LIBRARY(BulletRobotics ${BulletRobotics_SRCS})

SET_TARGET_PROPERTIES(BulletRobotics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletRobotics PROPERTIES SOVERSION ${BULLET_VERSION})

IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletRobotics BulletInverseDynamicsUtils BulletWorldImporter BulletFileLoader BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamics LinearMath Bullet3Common)
ENDIF (BUILD_SHARED_LIBS)

IF (INSTALL_EXTRA_LIBS)
	INSTALL(FILES
		../../examples/SharedMemory/PhysicsClientC_API.h
		../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h
		../../examples/SharedMemory/PhysicsClientSharedMemory2_C_API.h
		../../examples/SharedMemory/PhysicsDirectC_API.h
		../../examples/SharedMemory/PhysicsClientUDP_C_API.h
		../../examples/SharedMemory/PhysicsClientTCP_C_API.h
		../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.h
		../../examples/SharedMemory/SharedMemoryPublic.h
		DESTINATION include/bullet
		)
	INSTALL(TARGETS
		BulletRobotics
		LIBRARY DESTINATION lib${LIB_SUFFIX}
		ARCHIVE DESTINATION lib${LIB_SUFFIX}
		)
		
		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(BulletRobotics  PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(BulletRobotics PROPERTIES PUBLIC_HEADER "PhysicsClientC_API.h" )
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		
ENDIF (INSTALL_EXTRA_LIBS)
