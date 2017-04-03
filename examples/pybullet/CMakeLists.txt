
INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
                ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
		${PYTHON_INCLUDE_DIRS}
  )
IF(BUILD_PYBULLET_NUMPY)
	INCLUDE_DIRECTORIES(
		${PYTHON_NUMPY_INCLUDE_DIR}
	)
ENDIF()

FILE(GLOB GwenGUISupport_SRCS "../../examples/ExampleBrowser/GwenGUISupport/*" )

SET(c_api_SRCS
		../../examples/SharedMemory/IKTrajectoryHelper.cpp
		../../examples/SharedMemory/IKTrajectoryHelper.h
		../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
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
		../../examples/SharedMemory/PhysicsServerExample.cpp
		../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp
		../../examples/SharedMemory/PhysicsServerSharedMemory.cpp
		../../examples/SharedMemory/PhysicsServerSharedMemory.h
		../../examples/SharedMemory/PhysicsDirect.cpp
		../../examples/SharedMemory/PhysicsDirect.h
		../../examples/SharedMemory/PhysicsDirectC_API.cpp
		../../examples/SharedMemory/PhysicsDirectC_API.h
		../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp
		../../examples/SharedMemory/PhysicsServerCommandProcessor.h
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

		../../Extras/InverseDynamics/btMultiBodyTreeCreator.cpp
		../../Extras/InverseDynamics/CloneTreeCreator.cpp
		../../Extras/InverseDynamics/CoilCreator.cpp
		../../Extras/InverseDynamics/DillCreator.cpp
		../../Extras/InverseDynamics/IDRandomUtil.cpp
		../../Extras/InverseDynamics/invdyn_bullet_comparison.cpp
		../../Extras/InverseDynamics/MultiBodyNameMap.cpp
		../../Extras/InverseDynamics/MultiBodyTreeCreator.cpp
		../../Extras/InverseDynamics/MultiBodyTreeDebugGraph.cpp
		../../Extras/InverseDynamics/RandomTreeCreator.cpp
		../../Extras/InverseDynamics/SimpleTreeCreator.cpp
		../../Extras/InverseDynamics/User2InternalIndex.cpp

		../../Extras/Serialize/BulletFileLoader/bFile.cpp
		../../Extras/Serialize/BulletFileLoader/bDNA.cpp
		../../Extras/Serialize/BulletFileLoader/bChunk.cpp
		../../Extras/Serialize/BulletFileLoader/btBulletFile.cpp
		../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
		../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp

		../../examples/ExampleBrowser/OpenGLExampleBrowser.cpp
		../../examples/ExampleBrowser/OpenGLGuiHelper.cpp
		../../examples/ExampleBrowser/GL_ShapeDrawer.cpp
		../../examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp

		../../examples/ThirdPartyLibs/Glew/glew.c

		../../examples/OpenGLWindow/TwFonts.cpp
		../../examples/OpenGLWindow/GLInstancingRenderer.cpp
		../../examples/OpenGLWindow/GLRenderToTexture.cpp
		../../examples/OpenGLWindow/SimpleOpenGL2Renderer.cpp
		../../examples/OpenGLWindow/GLPrimitiveRenderer.cpp
		../../examples/OpenGLWindow/opengl_fontstashcallbacks.cpp
		../../examples/OpenGLWindow/LoadShader.cpp
		../../examples/OpenGLWindow/EGLOpenGLWindow.cpp
		../../examples/OpenGLWindow/fontstash.cpp
		../../examples/OpenGLWindow/GLInstancingRenderer.cpp
		../../examples/OpenGLWindow/GLPrimitiveRenderer.cpp
		../../examples/OpenGLWindow/GLRenderToTexture.cpp
		../../examples/OpenGLWindow/LoadShader.cpp
		../../examples/OpenGLWindow/opengl_fontstashcallbacks.cpp
		../../examples/OpenGLWindow/OpenSans.cpp
		../../examples/OpenGLWindow/SimpleCamera.cpp
		../../examples/OpenGLWindow/SimpleOpenGL2App.cpp
		../../examples/OpenGLWindow/SimpleOpenGL2Renderer.cpp
		../../examples/OpenGLWindow/SimpleOpenGL3App.cpp
		../../examples/OpenGLWindow/TwFonts.cpp
		../../examples/OpenGLWindow/Win32OpenGLWindow.cpp
		../../examples/OpenGLWindow/Win32Window.cpp
		../../examples/OpenGLWindow/X11OpenGLWindow.cpp

		${GwenGUISupport_SRCS}
)

SET(pybullet_SRCS
	pybullet.c
)

IF(BUILD_PYBULLET_CLSOCKET)
 ADD_DEFINITIONS(-DBT_ENABLE_CLSOCKET)
ENDIF(BUILD_PYBULLET_CLSOCKET)

IF(WIN32)
	LINK_LIBRARIES(
		${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)
	IF(BUILD_PYBULLET_ENET)
		ADD_DEFINITIONS(-DWIN32 -DBT_ENABLE_ENET)
	ENDIF(BUILD_PYBULLET_ENET)
	IF(BUILD_PYBULLET_CLSOCKET)
		ADD_DEFINITIONS(-DWIN32)
	ENDIF(BUILD_PYBULLET_CLSOCKET)

ELSE(WIN32)
	IF(BUILD_PYBULLET_ENET)
		ADD_DEFINITIONS(-DHAS_SOCKLEN_T -DBT_ENABLE_ENET)
	ENDIF(BUILD_PYBULLET_ENET)

	IF(BUILD_PYBULLET_CLSOCKET)	
		IF(APPLE)
			ADD_DEFINITIONS(-D_DARWIN)
		ELSE()
			ADD_DEFINITIONS(-D_LINUX)
		ENDIF()
	ENDIF(BUILD_PYBULLET_CLSOCKET)
ENDIF(WIN32)


IF(BUILD_PYBULLET_ENET)
	set(pybullet_SRCS ${pybullet_SRCS} 
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
ENDIF(BUILD_PYBULLET_ENET)

IF(BUILD_PYBULLET_CLSOCKET)
	set(pybullet_SRCS ${pybullet_SRCS}
		 ../../examples/SharedMemory/PhysicsClientTCP.cpp
                 ../../examples/SharedMemory/PhysicsClientTCP.h
                 ../../examples/SharedMemory/PhysicsClientTCP_C_API.cpp
                 ../../examples/SharedMemory/PhysicsClientTCP_C_API.h
		 ../../examples/ThirdPartyLibs/clsocket/src/SimpleSocket.cpp
		 ../../examples/ThirdPartyLibs/clsocket/src/ActiveSocket.cpp
		 ../../examples/ThirdPartyLibs/clsocket/src/PassiveSocket.cpp
	)
ENDIF()

ADD_LIBRARY(pybullet SHARED ${pybullet_SRCS} ${c_api_SRCS})
ADD_LIBRARY(PhysicsClientC_API ${c_api_SRCS})

SET_TARGET_PROPERTIES(pybullet PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES(pybullet PROPERTIES POSTFIX "")

SET_TARGET_PROPERTIES(pybullet PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(pybullet PROPERTIES SOVERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(pybullet PROPERTIES DEBUG_POSTFIX "_d")

SET_TARGET_PROPERTIES(PhysicsClientC_API PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(PhysicsClientC_API PROPERTIES SOVERSION ${BULLET_VERSION})

IF(WIN32)
	IF(BUILD_PYBULLET_ENET OR BUILD_PYBULLET_CLSOCKET)
		TARGET_LINK_LIBRARIES(pybullet ws2_32 )
	ENDIF(BUILD_PYBULLET_ENET OR BUILD_PYBULLET_CLSOCKET)

	SET_TARGET_PROPERTIES(pybullet PROPERTIES SUFFIX ".pyd" )
ENDIF(WIN32)




TARGET_LINK_LIBRARIES(PhysicsClientC_API BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamics LinearMath Bullet3Common gwen)

TARGET_LINK_LIBRARIES(pybullet BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen BussIK Bullet3Common ${PYTHON_LIBRARIES})


IF (INSTALL_LIBS)
	INSTALL(FILES
		../../examples/SharedMemory/PhysicsClientC_API.h
		../../examples/SharedMemory/SharedMemoryPublic.h
		DESTINATION include/bullet
		)
	INSTALL(TARGETS
		PhysicsClientC_API
		LIBRARY DESTINATION lib${LIB_SUFFIX}
		)
ENDIF (INSTALL_LIBS)

