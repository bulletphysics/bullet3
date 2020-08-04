
INCLUDE_DIRECTORIES(
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
)

SET(BulletRoboticsGUI_INCLUDES
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

SET(BulletRoboticsGUI_SRCS ${BulletRoboticsGUI_INCLUDES}

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
	../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
	../../examples/SharedMemory/GraphicsServerExample.cpp
	../../examples/SharedMemory/GraphicsClientExample.cpp
	../../examples/SharedMemory/RemoteGUIHelper.cpp
	../../examples/SharedMemory/RemoteGUIHelperTCP.cpp
	../../examples/SharedMemory/GraphicsServerExample.h
	../../examples/SharedMemory/GraphicsClientExample.h
	../../examples/SharedMemory/RemoteGUIHelper.h
	../../examples/SharedMemory/GraphicsSharedMemoryCommands.h
	../../examples/SharedMemory/GraphicsSharedMemoryPublic.h
	../../examples/SharedMemory/PhysicsServerExample.cpp
	../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
)

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
		LINK_LIBRARIES(  pthread ${DL} )
	ENDIF(NOT APPLE)
ENDIF(WIN32)

IF(BUILD_ENET)
  set(BulletRoboticsGUI_ENET_INCLUDES 
    ../../examples/SharedMemory/PhysicsClientUDP.h		
    ../../examples/SharedMemory/PhysicsClientUDP_C_API.h
    ../../examples/ThirdPartyLibs/enet/include/enet/win32.h
		../../examples/ThirdPartyLibs/enet/include/enet/unix.h
		../../examples/ThirdPartyLibs/enet/include/enet/callbacks.h
		../../examples/ThirdPartyLibs/enet/include/enet/list.h
		../../examples/ThirdPartyLibs/enet/include/enet/protocol.h
  )

  set(BulletRoboticsGUI_INCLUDES
    ${BulletRoboticsGUI_INCLUDES}
    ${BulletRoboticsGUI_ENET_INCLUDES}
	)



ENDIF(BUILD_ENET)


ADD_DEFINITIONS(-DPHYSICS_SERVER_DIRECT)


ADD_LIBRARY(BulletRoboticsGUI ${BulletRoboticsGUI_SRCS})

SET_TARGET_PROPERTIES(BulletRoboticsGUI PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletRoboticsGUI PROPERTIES SOVERSION ${BULLET_VERSION})

IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletRoboticsGUI  BulletExampleBrowserLib BulletRobotics  BulletInverseDynamicsUtils BulletWorldImporter BulletFileLoader BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamics LinearMath Bullet3Common)
ENDIF (BUILD_SHARED_LIBS)

  

	INSTALL(TARGETS
		BulletRoboticsGUI
		LIBRARY DESTINATION lib${LIB_SUFFIX}
		ARCHIVE DESTINATION lib${LIB_SUFFIX}
	)
		
  IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
    SET_TARGET_PROPERTIES(BulletRobotics  PROPERTIES FRAMEWORK true)
    SET_TARGET_PROPERTIES(BulletRobotics PROPERTIES PUBLIC_HEADER "PhysicsClientC_API.h" )
  ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)	

IF(NOT MSVC)
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/bullet_robotics_gui.pc.cmake
                   ${CMAKE_CURRENT_BINARY_DIR}/bullet_robotics_gui.pc @ONLY)
  	INSTALL(
      FILES
      ${CMAKE_CURRENT_BINARY_DIR}/bullet_robotics_gui.pc
      DESTINATION
      ${PKGCONFIG_INSTALL_PREFIX}
    )
ENDIF(NOT MSVC)

