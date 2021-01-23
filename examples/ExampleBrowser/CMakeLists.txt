
INCLUDE_DIRECTORIES(
	.
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
)

FILE(GLOB GwenGUISupport_SRCS  "GwenGUISupport/*" )
FILE(GLOB GwenGUISupport_HDRS  "GwenGUISupport/*" )


IF (WIN32)
        INCLUDE_DIRECTORIES(
                ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad
        )
				ADD_DEFINITIONS(-DGLEW_STATIC)
ELSE(WIN32)
        IF(APPLE)
                find_library(COCOA NAMES Cocoa)
  			ELSE(APPLE)
                ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
                ADD_DEFINITIONS("-DGLEW_STATIC")
                ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
                INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad )
     		ENDIF(APPLE)
ENDIF(WIN32)


ADD_LIBRARY(BulletExampleBrowserLib
	OpenGLExampleBrowser.cpp
	OpenGLGuiHelper.cpp
	GL_ShapeDrawer.cpp
	CollisionShape2TriangleMesh.cpp
	CollisionShape2TriangleMesh.h
	../Utils/b3Clock.cpp
	../Utils/b3Clock.h
	../Utils/ChromeTraceUtil.cpp
	../Utils/ChromeTraceUtil.h
	../Utils/b3ResourcePath.cpp
	../Utils/b3ResourcePath.h
	../Utils/b3ERPCFMHelper.hpp
	../Utils/b3ReferenceFrameHelper.hpp
	${GwenGUISupport_SRCS}
	${GwenGUISupport_HDRS}
	
)

SET_TARGET_PROPERTIES(BulletExampleBrowserLib PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletExampleBrowserLib PROPERTIES SOVERSION ${BULLET_VERSION})
IF (BUILD_SHARED_LIBS)
		IF (WIN32)
                TARGET_LINK_LIBRARIES(
                        BulletExampleBrowserLib Bullet3Common BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils
                        BulletInverseDynamics LinearMath OpenGLWindow gwen BussIK
                        ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
                )
		ELSE(WIN32)
        IF(APPLE)
                 TARGET_LINK_LIBRARIES(
                                BulletExampleBrowserLib Bullet3Common BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils
                                BulletInverseDynamics LinearMath OpenGLWindow gwen BussIK
                                ${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
                 )
				ELSE(APPLE)
                 FIND_PACKAGE(Threads)
                 TARGET_LINK_LIBRARIES(
                                BulletExampleBrowserLib Bullet3Common BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils
                                BulletInverseDynamics LinearMath OpenGLWindow gwen BussIK
                                ${CMAKE_THREAD_LIBS_INIT} ${DL}
                        )
        ENDIF(APPLE)
	ENDIF(WIN32)
ENDIF(BUILD_SHARED_LIBS)

INSTALL(TARGETS BulletExampleBrowserLib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib${LIB_SUFFIX}
        ARCHIVE DESTINATION lib${LIB_SUFFIX})

####################
#
# Bullet Example Browser main app
#
####################

INCLUDE_DIRECTORIES(
        .
        ${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
)


LINK_LIBRARIES(
        BulletExampleBrowserLib Bullet3Common BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen BussIK
)


add_definitions(-DINCLUDE_CLOTH_DEMOS)

IF (WIN32)
	INCLUDE_DIRECTORIES(
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad
	)
	LINK_LIBRARIES(
		${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
ELSE(WIN32)
	IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
		INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad )
  ELSE(APPLE)
		ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
		ADD_DEFINITIONS("-DGLEW_STATIC")
		ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
		INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad )
		FIND_PACKAGE(Threads)
		LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} ${DL} )
	ENDIF(APPLE)
ENDIF(WIN32)


SET(ExtendedTutorialsSources
	../ExtendedTutorials/Chain.cpp 
	../ExtendedTutorials/Chain.h
	../ExtendedTutorials/Bridge.cpp
	../ExtendedTutorials/Bridge.h
	../ExtendedTutorials/RigidBodyFromObj.cpp
	../ExtendedTutorials/RigidBodyFromObj.h
    ../ExtendedTutorials/SimpleBox.cpp
	../ExtendedTutorials/SimpleBox.h
	../ExtendedTutorials/MultipleBoxes.cpp
	../ExtendedTutorials/MultipleBoxes.h
	../ExtendedTutorials/CompoundBoxes.cpp
	../ExtendedTutorials/CompoundBoxes.h
	../ExtendedTutorials/SimpleCloth.cpp
	../ExtendedTutorials/SimpleCloth.h
	../ExtendedTutorials/SimpleJoint.cpp
	../ExtendedTutorials/SimpleJoint.h
	../ExtendedTutorials/NewtonsCradle.cpp
	../ExtendedTutorials/NewtonsCradle.h
	../ExtendedTutorials/InclinedPlane.cpp
	../ExtendedTutorials/InclinedPlane.h
	../ExtendedTutorials/MultiPendulum.cpp
	../ExtendedTutorials/MultiPendulum.h
)

SET(BulletExampleBrowser_SRCS
        ../BulletRobotics/FixJointBoxes.cpp

        ../BulletRobotics/BoxStack.cpp
        ../BulletRobotics/JointLimit.cpp
        # ../BulletRobotics/GraspBox.cpp

	../TinyRenderer/geometry.cpp
	../TinyRenderer/model.cpp
	../TinyRenderer/tgaimage.cpp
	../TinyRenderer/our_gl.cpp
	../TinyRenderer/TinyRenderer.cpp
	../SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp
	../SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.h
	../SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp
	../SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h
	../SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp
	../SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp
	../SharedMemory/IKTrajectoryHelper.cpp
	../SharedMemory/IKTrajectoryHelper.h
	../SharedMemory/PhysicsServer.cpp
	../SharedMemory/PhysicsClientSharedMemory.cpp
	../SharedMemory/PhysicsClientSharedMemory_C_API.cpp
	../SharedMemory/PhysicsClient.cpp
	../SharedMemory/PhysicsClientC_API.cpp
	../SharedMemory/GraphicsServerExample.cpp
	../SharedMemory/GraphicsClientExample.cpp
	../SharedMemory/RemoteGUIHelper.cpp
	../SharedMemory/GraphicsServerExample.h
	../SharedMemory/GraphicsClientExample.h
	../SharedMemory/RemoteGUIHelper.h
	../SharedMemory/GraphicsSharedMemoryCommands.h
	../SharedMemory/GraphicsSharedMemoryPublic.h
	../SharedMemory/PhysicsServerExample.cpp
	../SharedMemory/PhysicsServerExampleBullet2.cpp
	../SharedMemory/PhysicsClientExample.cpp
	../SharedMemory/PosixSharedMemory.cpp
	../SharedMemory/Win32SharedMemory.cpp
	../SharedMemory/InProcessMemory.cpp
	../SharedMemory/PhysicsServerSharedMemory.cpp
	../SharedMemory/PhysicsDirect.cpp
	../SharedMemory/PhysicsDirect.h
	../SharedMemory/PhysicsDirectC_API.cpp
	../SharedMemory/PhysicsDirectC_API.h
	../SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp
	../SharedMemory/PhysicsLoopBack.cpp
	../SharedMemory/PhysicsLoopBack.h
	../SharedMemory/PhysicsLoopBackC_API.cpp
	../SharedMemory/PhysicsLoopBackC_API.h
	../SharedMemory/PhysicsServerCommandProcessor.cpp
	../SharedMemory/PhysicsServerCommandProcessor.h
	../SharedMemory/SharedMemoryCommands.h
	../SharedMemory/SharedMemoryPublic.h
	../SharedMemory/b3PluginManager.cpp
	../SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp
	../SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h
	../SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp
	../SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h
	../RobotSimulator/b3RobotSimulatorClientAPI.cpp
	../RobotSimulator/b3RobotSimulatorClientAPI.h
	../BasicDemo/BasicExample.cpp
	../BasicDemo/BasicExample.h
	../InverseDynamics/InverseDynamicsExample.cpp
	../InverseDynamics/InverseDynamicsExample.h
	../InverseKinematics/InverseKinematicsExample.cpp
	../InverseKinematics/InverseKinematicsExample.h
	../ForkLift/ForkLiftDemo.cpp
	../ForkLift/ForkLiftDemo.h
	../MultiThreadedDemo/MultiThreadedDemo.cpp
	../MultiThreadedDemo/MultiThreadedDemo.h
	../MultiThreadedDemo/CommonRigidBodyMTBase.cpp
	../MultiThreadedDemo/CommonRigidBodyMTBase.h
	../Heightfield/HeightfieldExample.cpp
	../Heightfield/HeightfieldExample.h
	../Tutorial/Tutorial.cpp
	../Tutorial/Tutorial.h
	../Tutorial/Dof6ConstraintTutorial.cpp
	../Tutorial/Dof6ConstraintTutorial.h
	../Evolution/NN3DWalkers.cpp
	../Evolution/NN3DWalkers.h
	../Evolution/NN3DWalkersTimeWarpBase.h
	../ExtendedTutorials/NewtonsRopeCradle.cpp
	../ExtendedTutorials/NewtonsRopeCradle.h
	../Collision/CollisionSdkC_Api.cpp
	../Collision/CollisionSdkC_Api.h
	../Collision/CollisionTutorialBullet2.cpp
	../Collision/CollisionTutorialBullet2.h
	../Collision/Internal/Bullet2CollisionSdk.cpp
	../Collision/Internal/Bullet2CollisionSdk.h
	../Collision/Internal/CollisionSdkInterface.h
	../Collision/Internal/RealTimeBullet3CollisionSdk.cpp
	../Collision/Internal/RealTimeBullet3CollisionSdk.h
	../GyroscopicDemo/GyroscopicSetup.cpp
	../GyroscopicDemo/GyroscopicSetup.h
	../Planar2D/Planar2D.cpp
	../Planar2D/Planar2D.h
	../RollingFrictionDemo/RollingFrictionDemo.cpp
	../RollingFrictionDemo/RollingFrictionDemo.h
	../FractureDemo/FractureDemo.cpp
	../FractureDemo/btFractureBody.cpp
	../FractureDemo/btFractureDynamicsWorld.cpp
	../FractureDemo/FractureDemo.h
	../FractureDemo/btFractureBody.h
	../FractureDemo/btFractureDynamicsWorld.h
	../DynamicControlDemo/MotorDemo.cpp
	../DynamicControlDemo/MotorDemo.h
	../MultiThreading/MultiThreadingExample.cpp
	../MultiThreading/b3PosixThreadSupport.cpp
	../MultiThreading/b3Win32ThreadSupport.cpp
	../MultiThreading/b3ThreadSupportInterface.cpp
	../RenderingExamples/TinyRendererSetup.cpp
	../RenderingExamples/TimeSeriesCanvas.cpp
	../RenderingExamples/TimeSeriesCanvas.h
	../RenderingExamples/TimeSeriesFontData.cpp
	../RenderingExamples/TimeSeriesFontData.h
	../RenderingExamples/DynamicTexturedCubeDemo.cpp
	../RenderingExamples/DynamicTexturedCubeDemo.h
	../RenderingExamples/TinyVRGui.cpp
	../RenderingExamples/TinyVRGui.h
	
	../RoboticsLearning/GripperGraspExample.cpp
	../RoboticsLearning/GripperGraspExample.h
	../RoboticsLearning/R2D2GraspExample.cpp
	../RoboticsLearning/R2D2GraspExample.h
	../RoboticsLearning/KukaGraspExample.cpp
	../RoboticsLearning/KukaGraspExample.h
	../RenderingExamples/CoordinateSystemDemo.cpp
	../RenderingExamples/CoordinateSystemDemo.h
	../RenderingExamples/RaytracerSetup.cpp
	../RenderingExamples/RaytracerSetup.h
	../RenderingExamples/RenderInstancingDemo.cpp
	../RenderingExamples/RenderInstancingDemo.h
	../RenderingExamples/TimeSeriesExample.cpp
	../Benchmarks/BenchmarkDemo.cpp
	../Benchmarks/BenchmarkDemo.h
	../Benchmarks/landscapeData.h
	../Benchmarks/TaruData
	../Raycast/RaytestDemo.cpp
		../Importers/ImportBsp/BspConverter.h
 	../Importers/ImportBullet/SerializeSetup.cpp
	../Importers/ImportBullet/SerializeSetup.h
	../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
../Importers/ImportMeshUtility/b3ImportMeshUtility.h
	../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
	../../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp
	../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
	../../Extras/Serialize/BulletFileLoader/bChunk.cpp		
	../../Extras/Serialize/BulletFileLoader/bFile.cpp
	../../Extras/Serialize/BulletFileLoader/bDNA.cpp		
	../../Extras/Serialize/BulletFileLoader/btBulletFile.cpp

	../Importers/ImportBsp/BspLoader.h
  ../Importers/ImportBsp/ImportBspExample.h
  ../Importers/ImportColladaDemo/btMatrix4x4.h
  ../Importers/ImportColladaDemo/ColladaGraphicsInstance.h
  ../Importers/ImportColladaDemo/ImportColladaSetup.h
  ../Importers/ImportColladaDemo/LoadMeshFromCollada.h
  ../Importers/ImportObjDemo/ImportObjExample.h
  ../Importers/ImportObjDemo/LoadMeshFromObj.h
  ../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h
  ../Importers/ImportSTLDemo/ImportSTLSetup.h
  ../Importers/ImportSTLDemo/LoadMeshFromSTL.h
  ../Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h
  ../Importers/ImportURDFDemo/ImportURDFSetup.h
  ../Importers/ImportURDFDemo/URDF2Bullet.h
  ../Importers/ImportURDFDemo/urdf_samples.h
  ../Importers/ImportURDFDemo/urdf_samples.h
	../Importers/ImportMJCFDemo/BulletMJCFImporter.cpp
	../Importers/ImportMJCFDemo/BulletMJCFImporter.h
	../Importers/ImportMJCFDemo/ImportMJCFSetup.cpp
	../Importers/ImportMJCFDemo/ImportMJCFSetup.h
  ../Importers/ImportBsp/BspConverter.cpp
  ../Importers/ImportBsp/BspLoader.cpp
  ../Importers/ImportBsp/ImportBspExample.cpp
  ../Importers/ImportColladaDemo/ImportColladaSetup.cpp
  ../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
  ../Importers/ImportObjDemo/ImportObjExample.cpp
  ../Importers/ImportObjDemo/LoadMeshFromObj.cpp
  ../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
  ../Importers/ImportSTLDemo/ImportSTLSetup.cpp
 ../Importers/ImportSDFDemo/ImportSDFSetup.cpp
  ../Importers/ImportURDFDemo/ImportURDFSetup.cpp
  ../Importers/ImportURDFDemo/URDF2Bullet.cpp
  ../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
  ../Importers/ImportURDFDemo/MyMultiBodyCreator.h
  ../Importers/ImportURDFDemo/UrdfParser.cpp
  ../Utils/RobotLoggingUtil.cpp
  ../Utils/RobotLoggingUtil.h
  ../Importers/ImportURDFDemo/urdfStringSplit.cpp
 ../Importers/ImportURDFDemo/urdfStringSplit.h
  ../Importers/ImportURDFDemo/BulletUrdfImporter.cpp
  ../Importers/ImportURDFDemo/BulletUrdfImporter.h
  ../VoronoiFracture/VoronoiFractureDemo.cpp
  ../VoronoiFracture/VoronoiFractureDemo.h
  ../VoronoiFracture/btConvexConvexMprAlgorithm.cpp
  ../VoronoiFracture/btConvexConvexMprAlgorithm.h
  ../Vehicles/Hinge2Vehicle.cpp
  ../Vehicles/Hinge2Vehicle.h
  ../MultiBody/Pendulum.cpp
  ../MultiBody/MultiBodySoftContact.cpp
  ../MultiBody/TestJointTorqueSetup.cpp
  ../MultiBody/TestJointTorqueSetup.h
  ../MultiBody/InvertedPendulumPDControl.cpp
  ../MultiBody/InvertedPendulumPDControl.h
  ../MultiBody/MultiBodyConstraintFeedback.cpp
  ../MultiBody/KinematicMultiBodyExample.cpp
  ../SoftDemo/SoftDemo.cpp
  ../SoftDemo/SoftDemo.h
  ../DeformableDemo/DeformableContact.cpp
  ../DeformableDemo/DeformableContact.h
  ../DeformableDemo/GraspDeformable.cpp
  ../DeformableDemo/GraspDeformable.h
  ../DeformableDemo/Pinch.cpp
  ../DeformableDemo/Pinch.h
  ../DeformableDemo/DeformableSelfCollision.cpp
  ../DeformableDemo/DeformableSelfCollision.h
  ../DeformableDemo/PinchFriction.cpp
  ../DeformableDemo/PinchFriction.h
  ../DeformableDemo/ClothFriction.cpp
  ../DeformableDemo/ClothFriction.h
  ../DeformableDemo/DeformableMultibody.cpp
  ../DeformableDemo/DeformableMultibody.h
  ../DeformableDemo/DeformableRigid.cpp
  ../DeformableDemo/DeformableRigid.h
  ../DeformableDemo/SplitImpulse.cpp
  ../DeformableDemo/SplitImpulse.h
  ../DeformableDemo/VolumetricDeformable.cpp
  ../DeformableDemo/VolumetricDeformable.h
  ../DeformableDemo/Collide.cpp
  ../DeformableDemo/Collide.h
  ../DeformableDemo/LargeDeformation.cpp
  ../DeformableDemo/LargeDeformation.h
  ../DeformableDemo/DeformableClothAnchor.cpp
  ../DeformableDemo/DeformableClothAnchor.h
  ../DeformableDemo/MultibodyClothAnchor.cpp
  ../DeformableDemo/MultibodyClothAnchor.h
  ../MultiBody/MultiDofDemo.cpp
  ../MultiBody/MultiDofDemo.h
  ../RigidBody/RigidBodySoftContact.cpp
  ../RigidBody/KinematicRigidBodyExample.cpp
  ../Constraints/TestHingeTorque.cpp
  ../Constraints/TestHingeTorque.h
  ../Constraints/ConstraintDemo.cpp
  ../Constraints/ConstraintDemo.h
  ../Constraints/Dof6Spring2Setup.cpp
  ../Constraints/Dof6Spring2Setup.h
  ../Constraints/ConstraintPhysicsSetup.cpp
  ../Constraints/ConstraintPhysicsSetup.h


	../ThirdPartyLibs/stb_image/stb_image.cpp
	../ThirdPartyLibs/stb_image/stb_image.h
	../ThirdPartyLibs/stb_image/stb_image_write.cpp
	../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
	
	../ThirdPartyLibs/tinyxml2/tinyxml2.cpp
	InProcessExampleBrowser.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
)



ADD_EXECUTABLE(App_ExampleBrowser
		main.cpp
		ExampleEntries.cpp
		ExampleEntries.h
		${ExtendedTutorialsSources}
		${BulletExampleBrowser_SRCS} 
)

FILE( MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/data" )

ADD_CUSTOM_COMMAND(
                TARGET App_ExampleBrowser
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} ARGS -E copy_directory ${BULLET_PHYSICS_SOURCE_DIR}/data ${PROJECT_BINARY_DIR}/data
        )

IF (BULLET2_USE_TBB_MULTITHREADING AND WIN32)
    # add a post build command to copy some dlls to the executable directory
    set(TBB_VC_VER "vc12")
    set(TBB_VC_ARCH "ia32")
    # assume 32-bit build in VC12 for now
    # checks can be added here at a later time
    ADD_CUSTOM_COMMAND(TARGET App_ExampleBrowser POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${BULLET2_TBB_INCLUDE_DIR}/../bin/${TBB_VC_ARCH}/${TBB_VC_VER}/tbb.dll"
        $<TARGET_FILE_DIR:App_ExampleBrowser>)
    ADD_CUSTOM_COMMAND(TARGET App_ExampleBrowser POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${BULLET2_TBB_INCLUDE_DIR}/../bin/${TBB_VC_ARCH}/${TBB_VC_VER}/tbbmalloc.dll"
        $<TARGET_FILE_DIR:App_ExampleBrowser>)
ENDIF (BULLET2_USE_TBB_MULTITHREADING AND WIN32)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

