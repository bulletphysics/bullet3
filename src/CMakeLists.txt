
IF(BUILD_BULLET3)
	SUBDIRS(  Bullet3OpenCL Bullet3Serialize/Bullet2FileLoader Bullet3Dynamics Bullet3Collision Bullet3Geometry Bullet3Common )
	SUBDIRS(  BulletInverseDynamics  )
ENDIF(BUILD_BULLET3)

SUBDIRS(  BulletSoftBody BulletCollision BulletDynamics LinearMath )


IF(INSTALL_LIBS)
	#INSTALL of other files requires CMake 2.6
	IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			# Don't actually need to install any common files, the frameworks include everything
		ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(FILES btBulletCollisionCommon.h btBulletDynamicsCommon.h  DESTINATION ${INCLUDE_INSTALL_DIR})
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
ENDIF(INSTALL_LIBS)



INCLUDE_DIRECTORIES(
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
                ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
		${PYTHON_INCLUDE_DIRS}
)

SET(c_api_SRCS
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
)

ADD_DEFINITIONS(-DPHYSICS_SERVER_DIRECT)

ADD_LIBRARY(PhysicsClientC_API ${c_api_SRCS})

SET_TARGET_PROPERTIES(PhysicsClientC_API PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(PhysicsClientC_API PROPERTIES SOVERSION ${BULLET_VERSION})

TARGET_LINK_LIBRARIES(PhysicsClientC_API BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamics LinearMath Bullet3Common)

IF (INSTALL_LIBS)
	INSTALL(FILES
		../examples/SharedMemory/PhysicsClientC_API.h
		../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h
		../examples/SharedMemory/PhysicsClientSharedMemory2_C_API.h
		../examples/SharedMemory/PhysicsDirectC_API.h
		../examples/SharedMemory/PhysicsClientUDP_C_API.h
		../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.h
		../examples/SharedMemory/SharedMemoryPublic.h
		DESTINATION include/bullet
		)
	INSTALL(TARGETS
		PhysicsClientC_API
		LIBRARY DESTINATION lib${LIB_SUFFIX}
		ARCHIVE DESTINATION lib${LIB_SUFFIX}
		)
ENDIF (INSTALL_LIBS)

