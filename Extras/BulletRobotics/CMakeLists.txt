
INCLUDE_DIRECTORIES(
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
)
ADD_DEFINITIONS(-DSTATIC_LINK_SPD_PLUGIN)



SET(BulletRobotics_INCLUDES
  ../../examples/CommonInterfaces/Common2dCanvasInterface.h
  ../../examples/CommonInterfaces/CommonCallbacks.h
  ../../examples/CommonInterfaces/CommonCameraInterface.h
  ../../examples/CommonInterfaces/CommonExampleInterface.h
  ../../examples/CommonInterfaces/CommonFileIOInterface.h
  ../../examples/CommonInterfaces/CommonGraphicsAppInterface.h
  ../../examples/CommonInterfaces/CommonGUIHelperInterface.h
  ../../examples/CommonInterfaces/CommonMultiBodyBase.h
  ../../examples/CommonInterfaces/CommonParameterInterface.h
  ../../examples/CommonInterfaces/CommonRenderInterface.h
  ../../examples/CommonInterfaces/CommonRigidBodyBase.h
  ../../examples/CommonInterfaces/CommonWindowInterface.h

  ../../examples/OpenGLWindow/SimpleCamera.h

  ../../examples/TinyRenderer/geometry.h
  ../../examples/TinyRenderer/model.h
  ../../examples/TinyRenderer/tgaimage.h
  ../../examples/TinyRenderer/our_gl.h
  ../../examples/TinyRenderer/TinyRenderer.h

  ../../examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.h
  ../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h
  ../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h
  ../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h
  ../../examples/SharedMemory/IKTrajectoryHelper.h
  ../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.h
  ../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.h
  ../../examples/SharedMemory/InProcessMemory.h
  ../../examples/SharedMemory/PhysicsServer.h
  ../../examples/SharedMemory/PhysicsClient.h
  ../../examples/SharedMemory/PhysicsServerSharedMemory.h
  ../../examples/SharedMemory/PhysicsDirect.h
  ../../examples/SharedMemory/PhysicsDirectC_API.h
  ../../examples/SharedMemory/PhysicsServerCommandProcessor.h
  ../../examples/SharedMemory/b3PluginManager.h
  ../../examples/SharedMemory/PhysicsClientSharedMemory.h
  ../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h
  ../../examples/SharedMemory/PhysicsClientC_API.h
  ../../examples/SharedMemory/SharedMemoryPublic.h
  ../../examples/SharedMemory/Win32SharedMemory.h
  ../../examples/SharedMemory/PosixSharedMemory.h

  ../../examples/Utils/b3ResourcePath.h
  ../../examples/Utils/RobotLoggingUtil.h
  ../../examples/Utils/b3Clock.h
  ../../examples/Utils/b3ResourcePath.h
  ../../examples/Utils/ChromeTraceUtil.h
  ../../examples/Utils/b3ERPCFMHelper.hpp
  ../../examples/Utils/b3ReferenceFrameHelper.hpp  

  ../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.h
  ../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h
  ../../examples/ThirdPartyLibs/stb_image/stb_image.h
  ../../examples/ThirdPartyLibs/BussIK/Jacobian.h
  ../../examples/ThirdPartyLibs/BussIK/LinearR2.h
  ../../examples/ThirdPartyLibs/BussIK/LinearR3.h
  ../../examples/ThirdPartyLibs/BussIK/LinearR4.h
  ../../examples/ThirdPartyLibs/BussIK/MatrixRmn.h
  ../../examples/ThirdPartyLibs/BussIK/Node.h
  ../../examples/ThirdPartyLibs/BussIK/Tree.h
  ../../examples/ThirdPartyLibs/BussIK/VectorRn.h

  ../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.h
  ../../examples/Importers/ImportObjDemo/LoadMeshFromObj.h
  ../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h
  ../../examples/Importers/ImportMJCFDemo/BulletMJCFImporter.h
  ../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.h
  ../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.h
  ../../examples/Importers/ImportURDFDemo/URDF2Bullet.h
  ../../examples/Importers/ImportURDFDemo/UrdfParser.h
  ../../examples/Importers/ImportURDFDemo/urdfStringSplit.h
  ../../examples/Importers/ImportURDFDemo/URDFImporterInterface.h
  ../../examples/Importers/ImportURDFDemo/URDFJointTypes.h
  ../../examples/Importers/ImportURDFDemo/SDFAudioTypes.h
  ../../examples/Importers/ImportURDFDemo/UrdfRenderingInterface.h
  ../../examples/Importers/ImportURDFDemo/MultiBodyCreationInterface.h
  ../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.h
  
  ../../examples/MultiThreading/b3PosixThreadSupport.h
  ../../examples/MultiThreading/b3Win32ThreadSupport.h
  ../../examples/MultiThreading/b3ThreadSupportInterface.h
)

SET(BulletRobotics_SRCS ${BulletRobotics_INCLUDES}
	../../examples/SharedMemory/plugins/stablePDPlugin/SpAlg.cpp
	../../examples/SharedMemory/plugins/stablePDPlugin/SpAlg.h
	../../examples/SharedMemory/plugins/stablePDPlugin/Shape.cpp
	../../examples/SharedMemory/plugins/stablePDPlugin/Shape.h
	../../examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.cpp
	../../examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.h
	../../examples/SharedMemory/plugins/stablePDPlugin/RBDModel.cpp
	../../examples/SharedMemory/plugins/stablePDPlugin/RBDModel.h
	../../examples/SharedMemory/plugins/stablePDPlugin/MathUtil.cpp
	../../examples/SharedMemory/plugins/stablePDPlugin/MathUtil.h
	../../examples/SharedMemory/plugins/stablePDPlugin/KinTree.cpp
	../../examples/SharedMemory/plugins/stablePDPlugin/KinTree.h
	../../examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.cpp
	../../examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.h
	
	../../examples/OpenGLWindow/SimpleCamera.cpp

	../../examples/TinyRenderer/geometry.cpp
	../../examples/TinyRenderer/model.cpp
	../../examples/TinyRenderer/tgaimage.cpp
	../../examples/TinyRenderer/our_gl.cpp
	../../examples/TinyRenderer/TinyRenderer.cpp

	../../examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp
	../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp
	../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp
	../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp
	../../examples/SharedMemory/IKTrajectoryHelper.cpp
	../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp
	../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp
	../../examples/SharedMemory/InProcessMemory.cpp
	../../examples/SharedMemory/PhysicsClient.cpp
	../../examples/SharedMemory/PhysicsServer.cpp
	../../examples/SharedMemory/PhysicsServerSharedMemory.cpp
	../../examples/SharedMemory/PhysicsDirect.cpp
	../../examples/SharedMemory/PhysicsDirectC_API.cpp
	../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp
	../../examples/SharedMemory/b3PluginManager.cpp
	../../examples/SharedMemory/PhysicsClientSharedMemory.cpp
	../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp
	../../examples/SharedMemory/PhysicsClientC_API.cpp
	../../examples/SharedMemory/Win32SharedMemory.cpp
	../../examples/SharedMemory/PosixSharedMemory.cpp

	../../examples/Utils/b3ResourcePath.cpp
	../../examples/Utils/RobotLoggingUtil.cpp
	../../examples/Utils/b3Clock.cpp
	../../examples/Utils/b3ResourcePath.cpp
	../../examples/Utils/ChromeTraceUtil.cpp

	../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp
	../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
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

IF(ENABLE_VHACD)
ADD_DEFINITIONS(-DBT_ENABLE_VHACD)
SET(BulletRobotics_SRCS ${BulletRobotics_SRCS}
  ../../Extras/VHACD/test/src/main_vhacd.cpp
  ../../Extras/VHACD/src/VHACD.cpp
  ../../Extras/VHACD/src/vhacdICHull.cpp
  ../../Extras/VHACD/src/vhacdManifoldMesh.cpp
  ../../Extras/VHACD/src/vhacdMesh.cpp
  ../../Extras/VHACD/src/vhacdVolume.cpp
)
INCLUDE_DIRECTORIES( 
  ../../Extras/VHACD/inc
  ../../Extras/VHACD/public
)
ENDIF(ENABLE_VHACD)

IF(BUILD_CLSOCKET)
 ADD_DEFINITIONS(-DBT_ENABLE_CLSOCKET)
ENDIF(BUILD_CLSOCKET)

IF(WIN32)
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
		
	IF(NOT APPLE)
		FIND_PACKAGE(Threads)
		LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} ${DL} )
	ENDIF(NOT APPLE)
ENDIF(WIN32)

IF(BUILD_ENET)
  set(BulletRobotics_ENET_INCLUDES 
    ../../examples/SharedMemory/PhysicsClientUDP.h		
    ../../examples/SharedMemory/PhysicsClientUDP_C_API.h
    ../../examples/ThirdPartyLibs/enet/include/enet/win32.h
		../../examples/ThirdPartyLibs/enet/include/enet/unix.h
		../../examples/ThirdPartyLibs/enet/include/enet/callbacks.h
		../../examples/ThirdPartyLibs/enet/include/enet/list.h
		../../examples/ThirdPartyLibs/enet/include/enet/protocol.h
  )
  set(BulletRobotics_ENET_SRCS
    ../../examples/SharedMemory/PhysicsClientUDP.cpp
    ../../examples/SharedMemory/PhysicsClientUDP_C_API.cpp
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

  set(BulletRobotics_INCLUDES
    ${BulletRobotics_INCLUDES}
    ${BulletRobotics_ENET_INCLUDES}
	)

  set(BulletRobotics_SRCS
    ${BulletRobotics_SRCS}
    ${BulletRobotics_ENET_INCLUDES}
    ${BulletRobotics_ENET_SRCS}
	)

ENDIF(BUILD_ENET)

IF(BUILD_CLSOCKET)
  set(BulletRobotics_CLSOCKET_SRCS
    ../../examples/SharedMemory/PhysicsClientTCP.cpp
    ../../examples/SharedMemory/PhysicsClientTCP_C_API.cpp
    ../../examples/ThirdPartyLibs/clsocket/src/SimpleSocket.cpp
    ../../examples/ThirdPartyLibs/clsocket/src/ActiveSocket.cpp
    ../../examples/ThirdPartyLibs/clsocket/src/PassiveSocket.cpp
  )

  set(BulletRobotics_SRCS
    ${BulletRobotics_SRCS}
    ${BulletRobotics_CLSOCKET_SRCS}
  )
ENDIF()

ADD_DEFINITIONS(-DPHYSICS_SERVER_DIRECT)


ADD_LIBRARY(BulletRobotics ${BulletRobotics_SRCS})

SET_TARGET_PROPERTIES(BulletRobotics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletRobotics PROPERTIES SOVERSION ${BULLET_VERSION})

IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletRobotics BulletInverseDynamicsUtils BulletWorldImporter BulletFileLoader BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamics LinearMath Bullet3Common)
ENDIF (BUILD_SHARED_LIBS)

  

INSTALL (
    DIRECTORY ${CMAKE_SOURCE_DIR}/examples/
    DESTINATION include/bullet
    FILES_MATCHING PATTERN "*.h*")

	INSTALL(TARGETS
		BulletRobotics
		LIBRARY DESTINATION lib${LIB_SUFFIX}
		ARCHIVE DESTINATION lib${LIB_SUFFIX}
	)
		
  IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
    SET_TARGET_PROPERTIES(BulletRobotics  PROPERTIES FRAMEWORK true)
    SET_TARGET_PROPERTIES(BulletRobotics PROPERTIES PUBLIC_HEADER "PhysicsClientC_API.h" )
  ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)	

IF(NOT MSVC)
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/bullet_robotics.pc.cmake
                   ${CMAKE_CURRENT_BINARY_DIR}/bullet_robotics.pc @ONLY)
  	INSTALL(
      FILES
      ${CMAKE_CURRENT_BINARY_DIR}/bullet_robotics.pc
      DESTINATION
      ${PKGCONFIG_INSTALL_PREFIX}
    )
ENDIF(NOT MSVC)

