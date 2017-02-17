
INCLUDE_DIRECTORIES(
	.
	../../src
	../gtest-1.7.0/include
	../../examples
	../../examples/ThirdPartyLibs
)


#ADD_DEFINITIONS(-DGTEST_HAS_PTHREAD=1)
ADD_DEFINITIONS(-DPHYSICS_LOOP_BACK -DPHYSICS_SERVER_DIRECT -DENABLE_GTEST -D_VARIADIC_MAX=10)


LINK_LIBRARIES(
 BulletInverseDynamicsUtils BulletInverseDynamics BulletFileLoader BulletWorldImporter Bullet3Common BulletDynamics BulletCollision LinearMath gtest BussIK
)
IF (USE_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD)
	LINK_LIBRARIES(BulletSoftBody)
ENDIF()

IF (NOT WIN32)
	LINK_LIBRARIES(		pthread	)
ENDIF()

	ADD_EXECUTABLE(Test_PhysicsClientServer
		gtestwrap.cpp
		../../examples/SharedMemory/PhysicsClient.cpp
												../../examples/SharedMemory/IKTrajectoryHelper.cpp
												../../examples/SharedMemory/IKTrajectoryHelper.h
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
                        ../../examples/SharedMemory/PhysicsClientSharedMemory.cpp
                        ../../examples/SharedMemory/PhysicsClientSharedMemory.h
						../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp
                        ../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h
                        
						../../examples/SharedMemory/PhysicsClientC_API.cpp
                        ../../examples/SharedMemory/PhysicsClientC_API.h
			../../examples/SharedMemory/PhysicsLoopBack.cpp
                        ../../examples/SharedMemory/PhysicsLoopBack.h
                        ../../examples/SharedMemory/PhysicsLoopBackC_API.cpp
                        ../../examples/SharedMemory/PhysicsLoopBackC_API.h
                        ../../examples/SharedMemory/Win32SharedMemory.cpp
                        ../../examples/SharedMemory/Win32SharedMemory.h
                        ../../examples/SharedMemory/PosixSharedMemory.cpp
                        ../../examples/SharedMemory/PosixSharedMemory.h
                        ../../examples/Utils/b3ResourcePath.cpp
                        ../../examples/Utils/b3ResourcePath.h
                        ../../examples/Utils/RobotLoggingUtil.cpp
                        ../../examples/Utils/RobotLoggingUtil.h
									      ../../examples/SharedMemory/TinyRendererVisualShapeConverter.cpp
												../../examples/SharedMemory/TinyRendererVisualShapeConverter.h
												../../examples/OpenGLWindow/SimpleCamera.cpp
												../../examples/OpenGLWindow/SimpleCamera.h
												../../examples/TinyRenderer/geometry.cpp
												../../examples/TinyRenderer/model.cpp
												../../examples/TinyRenderer/tgaimage.cpp
												../../examples/TinyRenderer/our_gl.cpp
												../../examples/TinyRenderer/TinyRenderer.cpp
                        ../../examples/ThirdPartyLibs/tinyxml/tinystr.cpp
                        ../../examples/ThirdPartyLibs/tinyxml/tinyxml.cpp
                        ../../examples/ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
                        ../../examples/ThirdPartyLibs/tinyxml/tinyxmlparser.cpp
                        ../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
                        ../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h
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
			../../examples/ThirdPartyLibs/stb_image/stb_image.cpp
                        
	)

ADD_TEST(Test_PhysicsClientServer_PASS Test_PhysicsClientServer)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(Test_PhysicsClientServer PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(Test_PhysicsClientServer  PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(Test_PhysicsClientServer  PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
