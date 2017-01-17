

SET(SharedMemory_SRCS
	IKTrajectoryHelper.cpp
	IKTrajectoryHelper.h
	PhysicsClient.cpp
	PhysicsClientSharedMemory.cpp
	PhysicsClientExample.cpp
	PhysicsServerExample.cpp
	PhysicsServerSharedMemory.cpp
	PhysicsServerSharedMemory.h
	PhysicsServer.cpp
	PhysicsServer.h
	PhysicsClientC_API.cpp
	SharedMemoryCommands.h
	SharedMemoryPublic.h
	PhysicsServer.cpp
	PosixSharedMemory.cpp
	Win32SharedMemory.cpp
	InProcessMemory.cpp
	PhysicsDirect.cpp
	PhysicsDirect.h
	PhysicsDirectC_API.cpp
	PhysicsDirectC_API.h
	PhysicsLoopBack.cpp
	PhysicsLoopBack.h
	PhysicsLoopBackC_API.cpp
	PhysicsLoopBackC_API.h
	PhysicsClientSharedMemory_C_API.cpp
	PhysicsClientSharedMemory_C_API.h
	PhysicsClientSharedMemory2_C_API.cpp
	PhysicsClientSharedMemory2_C_API.h
	PhysicsClientSharedMemory2.cpp
	PhysicsClientSharedMemory2.h
	SharedMemoryCommandProcessor.cpp
	SharedMemoryCommandProcessor.h
	PhysicsServerCommandProcessor.cpp
	PhysicsServerCommandProcessor.h
	TinyRendererVisualShapeConverter.cpp
	TinyRendererVisualShapeConverter.h
	../TinyRenderer/geometry.cpp
	../TinyRenderer/model.cpp
	../TinyRenderer/tgaimage.cpp
	../TinyRenderer/our_gl.cpp
	../TinyRenderer/TinyRenderer.cpp
	../OpenGLWindow/SimpleCamera.cpp
	../OpenGLWindow/SimpleCamera.h
	../Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h
	../Importers/ImportURDFDemo/MultiBodyCreationInterface.h
	../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
	../Importers/ImportURDFDemo/MyMultiBodyCreator.h
	../Importers/ImportURDFDemo/BulletUrdfImporter.cpp
	../Importers/ImportURDFDemo/BulletUrdfImporter.h
	../Importers/ImportURDFDemo/UrdfParser.cpp
 	../Importers/ImportURDFDemo/urdfStringSplit.cpp
	../Importers/ImportURDFDemo/UrdfParser.cpp
	../Importers/ImportURDFDemo/UrdfParser.h
	../Importers/ImportURDFDemo/URDF2Bullet.cpp
	../Importers/ImportURDFDemo/URDF2Bullet.h
	../Importers/ImportMJCFDemo/BulletMJCFImporter.cpp
	../Importers/ImportMJCFDemo/BulletMJCFImporter.h
	../Utils/b3ResourcePath.cpp
	../Utils/b3Clock.cpp	
	../Importers/ImportURDFDemo/URDFImporterInterface.h
	../Importers/ImportURDFDemo/URDFJointTypes.h
	../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
	../Importers/ImportObjDemo/LoadMeshFromObj.cpp
	../Importers/ImportSTLDemo/ImportSTLSetup.h
	../Importers/ImportSTLDemo/LoadMeshFromSTL.h
	../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
	../Importers/ImportColladaDemo/ColladaGraphicsInstance.h
	../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp	
	../ThirdPartyLibs/tinyxml/tinystr.cpp
	../ThirdPartyLibs/tinyxml/tinyxml.cpp
	../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
	../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp
	../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
	../ThirdPartyLibs/stb_image/stb_image.cpp  
	../MultiThreading/b3ThreadSupportInterface.cpp
	../MultiThreading/b3ThreadSupportInterface.h
)

INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
)

LINK_LIBRARIES(
 Bullet3Common BulletWorldImporter BulletFileLoader BulletInverseDynamicsUtils BulletInverseDynamics BulletDynamics BulletCollision LinearMath BussIK
)

IF (WIN32)
        ADD_EXECUTABLE(App_SharedMemoryPhysics
                ${SharedMemory_SRCS}
                main.cpp
                ../MultiThreading/b3Win32ThreadSupport.cpp
	              ../MultiThreading/b3Win32ThreadSupport.h
                ${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
        )
ELSE(WIN32)
			 IF(APPLE)
			        LINK_LIBRARIES(  pthread dl )
			        ADD_EXECUTABLE(App_SharedMemoryPhysics
			                ${SharedMemory_SRCS}
			                ../MultiThreading/b3PosixThreadSupport.cpp
	               			../MultiThreading/b3PosixThreadSupport.h
			                main.cpp
			        )
							
        ELSE(APPLE)
         			LINK_LIBRARIES(  pthread dl )
			        ADD_EXECUTABLE(App_SharedMemoryPhysics
			                ${SharedMemory_SRCS}
			                ../MultiThreading/b3PosixThreadSupport.cpp
	               			../MultiThreading/b3PosixThreadSupport.h
			                main.cpp
			        )
        ENDIF(APPLE)
ENDIF(WIN32)




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(App_SharedMemoryPhysics PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(App_SharedMemoryPhysics PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(App_SharedMemoryPhysics PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)



INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
)

ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)

LINK_LIBRARIES(
 Bullet3Common BulletWorldImporter BulletInverseDynamicsUtils BulletInverseDynamics BulletDynamics BulletCollision LinearMath BussIK OpenGLWindow
)



IF (WIN32)
 				ADD_DEFINITIONS(-DGLEW_STATIC)
				LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
	
        ADD_EXECUTABLE(App_SharedMemoryPhysics_GUI
                ${SharedMemory_SRCS}
								../StandaloneMain/main_opengl_single_example.cpp
								../ExampleBrowser/OpenGLGuiHelper.cpp
								../ExampleBrowser/GL_ShapeDrawer.cpp
								../ExampleBrowser/CollisionShape2TriangleMesh.cpp
								../MultiThreading/b3Win32ThreadSupport.cpp
								../MultiThreading/b3Win32ThreadSupport.h
			          ${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
        )
ELSE(WIN32)
			 IF(APPLE)
			        LINK_LIBRARIES(  pthread dl )
			        FIND_LIBRARY(COCOA NAMES Cocoa)
              MESSAGE(${COCOA})
              LINK_LIBRARIES(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
                
			        ADD_EXECUTABLE(App_SharedMemoryPhysics_GUI
			                ${SharedMemory_SRCS}
	               			../StandaloneMain/main_opengl_single_example.cpp
											../ExampleBrowser/OpenGLGuiHelper.cpp
											../ExampleBrowser/GL_ShapeDrawer.cpp
											../ExampleBrowser/CollisionShape2TriangleMesh.cpp					
			                ../MultiThreading/b3PosixThreadSupport.cpp
	               			../MultiThreading/b3PosixThreadSupport.h
			        )
							
        ELSE(APPLE)
         			LINK_LIBRARIES(  pthread dl )
         			ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
              ADD_DEFINITIONS("-DGLEW_STATIC")
              ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
                
			        ADD_EXECUTABLE(App_SharedMemoryPhysics_GUI
			                ${SharedMemory_SRCS}
	              			../StandaloneMain/main_opengl_single_example.cpp
											../ExampleBrowser/OpenGLGuiHelper.cpp
											../ExampleBrowser/GL_ShapeDrawer.cpp
											../ExampleBrowser/CollisionShape2TriangleMesh.cpp
			                ../MultiThreading/b3PosixThreadSupport.cpp
	               			../MultiThreading/b3PosixThreadSupport.h
			        )
        ENDIF(APPLE)
ENDIF(WIN32)




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(App_SharedMemoryPhysics_GUI PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(App_SharedMemoryPhysics_GUI PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(App_SharedMemoryPhysics_GUI PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)



				