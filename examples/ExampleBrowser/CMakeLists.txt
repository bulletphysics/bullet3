

INCLUDE_DIRECTORIES(
	.
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
)

FILE(GLOB ExampleBrowser_SRCS "*" "GwenGUISupport/*" )
FILE(GLOB ExampleBrowser_HDRS "*" "GwenGUISupport/*" )



SET(App_ExampleBrowser_SRCS
	main.cpp
	ExampleEntries.cpp
	ExampleEntries.h
	../BasicDemo/BasicExample.cpp
	../BasicDemo/BasicExample.h
	../ForkLift/ForkLiftDemo.cpp
	../ForkLift/ForkLiftDemo.h
	../MultiThreadedDemo/MultiThreadedDemo.cpp
	../MultiThreadedDemo/MultiThreadedDemo.h
    ../MultiThreadedDemo/ParallelFor.h
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
		../RenderingExamples/CoordinateSystemDemo.cpp
	../RenderingExamples/CoordinateSystemDemo.h
	../RenderingExamples/RaytracerSetup.cpp
	../RenderingExamples/RaytracerSetup.h
	../RenderingExamples/RenderInstancingDemo.cpp
	../RenderingExamples/RenderInstancingDemo.h
	../Benchmarks/BenchmarkDemo.cpp
	../Benchmarks/BenchmarkDemo.h
	../Benchmarks/landscapeData.h
	../Benchmarks/TaruData
	../Raycast/RaytestDemo.cpp
		../Importers/ImportBsp/BspConverter.h
 	../Importers/ImportBullet/SerializeSetup.cpp
	../Importers/ImportBullet/SerializeSetup.h 

	../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
	../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
../../Extras/Serialize/BulletFileLoader/bChunk.cpp		../../Extras/Serialize/BulletFileLoader/bFile.cpp
../../Extras/Serialize/BulletFileLoader/bDNA.cpp		../../Extras/Serialize/BulletFileLoader/btBulletFile.cpp

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
  ../Importers/ImportBsp/BspConverter.cpp
  ../Importers/ImportBsp/BspLoader.cpp
  ../Importers/ImportBsp/ImportBspExample.cpp
  ../Importers/ImportColladaDemo/ImportColladaSetup.cpp
  ../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
  ../Importers/ImportObjDemo/ImportObjExample.cpp
  ../Importers/ImportObjDemo/LoadMeshFromObj.cpp
  ../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
  ../Importers/ImportSTLDemo/ImportSTLSetup.cpp
  ../Importers/ImportURDFDemo/ImportURDFSetup.cpp
  ../Importers/ImportURDFDemo/URDF2Bullet.cpp
  ../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
  ../Importers/ImportURDFDemo/MyMultiBodyCreator.h
  ../Importers/ImportURDFDemo/MyURDFImporter.cpp
  ../Importers/ImportURDFDemo/MyURDFImporter.h
  ../VoronoiFracture/VoronoiFractureDemo.cpp
  ../VoronoiFracture/VoronoiFractureDemo.h
  ../VoronoiFracture/btConvexConvexMprAlgorithm.cpp
  ../VoronoiFracture/btConvexConvexMprAlgorithm.h
  ../Vehicles/Hinge2Vehicle.cpp
  ../Vehicles/Hinge2Vehicle.h
  ../MultiBody/TestJointTorqueSetup.cpp
  ../MultiBody/TestJointTorqueSetup.h
  ../MultiBody/MultiDofDemo.cpp
  ../MultiBody/MultiDofDemo.h
  ../Constraints/ConstraintDemo.cpp
  ../Constraints/ConstraintDemo.h
  ../Constraints/Dof6Spring2Setup.cpp
  ../Constraints/Dof6Spring2Setup.h
  ../Constraints/ConstraintPhysicsSetup.cpp
  ../Constraints/ConstraintPhysicsSetup.h
  
	
    ../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
    ../ThirdPartyLibs/tinyxml/tinystr.cpp
    ../ThirdPartyLibs/tinyxml/tinyxml.cpp
    ../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
    ../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp

	
	../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/pose.cpp
		../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/model.cpp
    ../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/link.cpp
    ../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/joint.cpp
    ../ThirdPartyLibs/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h
    ../ThirdPartyLibs/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h
    ../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h
    ../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h
    ../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h
    ../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h
    ../ThirdPartyLibs/tinyxml/tinystr.cpp
    ../ThirdPartyLibs/tinyxml/tinyxml.cpp
    ../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
    ../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp
    ../ThirdPartyLibs/urdf/boost_replacement/lexical_cast.h
    ../ThirdPartyLibs/urdf/boost_replacement/shared_ptr.h
    ../ThirdPartyLibs/urdf/boost_replacement/printf_console.cpp
    ../ThirdPartyLibs/urdf/boost_replacement/printf_console.h
    ../ThirdPartyLibs/urdf/boost_replacement/string_split.cpp
    ../ThirdPartyLibs/urdf/boost_replacement/string_split.h
	../Utils/b3Clock.cpp
	../Utils/b3Clock.h
	${ExampleBrowser_SRCS}
	${ExampleBrowser_HDRS}
	${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
)

IF (BULLET2_MULTITHREADED_OPEN_MP_DEMO)
    ADD_DEFINITIONS("-DBT_USE_OPENMP=1")
    IF (MSVC)
        ADD_DEFINITIONS("/openmp")
    ELSE (MSVC)
        # GCC, Clang
        ADD_DEFINITIONS("-fopenmp")
    ENDIF (MSVC)
ENDIF (BULLET2_MULTITHREADED_OPEN_MP_DEMO)

IF (BULLET2_MULTITHREADED_PPL_DEMO)
    ADD_DEFINITIONS("-DBT_USE_PPL=1")
ENDIF (BULLET2_MULTITHREADED_PPL_DEMO)

IF (BULLET2_MULTITHREADED_TBB_DEMO)
    SET (BULLET2_TBB_INCLUDE_DIR "not found" CACHE PATH "Directory for Intel TBB includes.")
    SET (BULLET2_TBB_LIB_DIR "not found" CACHE PATH "Directory for Intel TBB libraries.")
    find_library(TBB_LIBRARY tbb PATHS ${BULLET2_TBB_LIB_DIR})
    find_library(TBBMALLOC_LIBRARY tbbmalloc PATHS ${BULLET2_TBB_LIB_DIR})
    ADD_DEFINITIONS("-DBT_USE_TBB=1")
    INCLUDE_DIRECTORIES( ${BULLET2_TBB_INCLUDE_DIR} )
    LINK_LIBRARIES( ${TBB_LIBRARY} ${TBBMALLOC_LIBRARY} )
ENDIF (BULLET2_MULTITHREADED_TBB_DEMO)


LINK_LIBRARIES(
        Bullet3Common BulletSoftBody BulletDynamics BulletCollision LinearMath OpenGLWindow gwen ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

IF (WIN32)
	SET(App_ExampleBrowser_SRCS ${App_ExampleBrowser_SRCS} )
	INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
ELSE(WIN32)
	IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA})
  ELSE(APPLE)
		ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
		ADD_DEFINITIONS("-DGLEW_STATIC")
	        ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
		INCLUDE_DIRECTORIES(
                                ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
	        )
		LINK_LIBRARIES( pthread dl)
	ENDIF(APPLE)
ENDIF(WIN32)
 
	


ADD_EXECUTABLE(App_ExampleBrowser
		${App_ExampleBrowser_SRCS}
)
	

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

IF (BULLET2_MULTITHREADED_TBB_DEMO AND WIN32)
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
ENDIF (BULLET2_MULTITHREADED_TBB_DEMO AND WIN32)
