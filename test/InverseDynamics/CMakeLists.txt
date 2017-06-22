


INCLUDE_DIRECTORIES(
	.
	../../src
	../gtest-1.7.0/include
	../../Extras/InverseDynamics
)


#ADD_DEFINITIONS(-DGTEST_HAS_PTHREAD=1)
ADD_DEFINITIONS(-D_VARIADIC_MAX=10)

LINK_LIBRARIES(
 BulletInverseDynamicsUtils BulletInverseDynamics Bullet3Common LinearMath gtest 
)
IF (NOT WIN32)
	LINK_LIBRARIES(		pthread	)
ENDIF()


	ADD_EXECUTABLE(Test_BulletInverseDynamicsJacobian
		test_invdyn_jacobian.cpp
	)

ADD_TEST(Test_BulletInverseDynamicsJacobian_PASS Test_BulletInverseDynamicsJacobian)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(Test_BulletInverseDynamicsJacobian PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(Test_BulletInverseDynamicsJacobian PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(Test_BulletInverseDynamicsJacobian PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

INCLUDE_DIRECTORIES(
        .
        ../../src
        ../gtest-1.7.0/include
        ../../Extras/InverseDynamics
        ../../examples/ThirdPartyLibs
)


#ADD_DEFINITIONS(-DGTEST_HAS_PTHREAD=1)
ADD_DEFINITIONS(-D_VARIADIC_MAX=10)

LINK_LIBRARIES(
BulletDynamics BulletCollision  BulletInverseDynamicsUtils BulletInverseDynamics Bullet3Common LinearMath gtest 
)
IF (NOT WIN32)
        LINK_LIBRARIES(         pthread )
ENDIF()


        ADD_EXECUTABLE(Test_BulletInverseForwardDynamics
                test_invdyn_bullet.cpp
		../../examples/Utils/b3ResourcePath.cpp
                ../../examples/Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h
        	../../examples/Importers/ImportURDFDemo/MultiBodyCreationInterface.h
        ../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
        ../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.h
        ../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
        ../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.h
        ../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
        ../../examples/Importers/ImportURDFDemo/UrdfParser.cpp
        ../../examples/Importers/ImportURDFDemo/UrdfParser.h
        ../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
        ../../examples/Importers/ImportURDFDemo/URDF2Bullet.h
        ../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
        ../../examples/ThirdPartyLibs/stb_image/stb_image.cpp
        ../../examples/Utils/b3Clock.cpp
        ../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
	../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
	../../Extras/Serialize/BulletFileLoader/bChunk.cpp
	../../Extras/Serialize/BulletFileLoader/bFile.cpp
	../../Extras/Serialize/BulletFileLoader/bDNA.cpp
	../../Extras/Serialize/BulletFileLoader/btBulletFile.cpp
        ../../examples/Importers/ImportURDFDemo/URDFImporterInterface.h
        ../../examples/Importers/ImportURDFDemo/URDFJointTypes.h
        ../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
        ../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
        ../../examples/Importers/ImportSTLDemo/ImportSTLSetup.h
        ../../examples/Importers/ImportSTLDemo/LoadMeshFromSTL.h
        	../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
        	../../examples/Importers/ImportColladaDemo/ColladaGraphicsInstance.h
        	../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
		../../examples/ThirdPartyLibs/tinyxml/tinystr.cpp
		../../examples/ThirdPartyLibs/tinyxml/tinyxml.cpp
		../../examples/ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
		../../examples/ThirdPartyLibs/tinyxml/tinyxmlparser.cpp

        )

ADD_TEST(Test_BulletInverseForwardDynamics_PASS Test_BulletInverseForwardDynamics)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(Test_BulletInverseForwardDynamics PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(Test_BulletInverseForwardDynamics PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(Test_BulletInverseForwardDynamics PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

INCLUDE_DIRECTORIES(
        .
        ../../src
        ../gtest-1.7.0/include
        ../../Extras/InverseDynamics
)


#ADD_DEFINITIONS(-DGTEST_HAS_PTHREAD=1)
ADD_DEFINITIONS(-D_VARIADIC_MAX=10)

LINK_LIBRARIES(
 BulletInverseDynamicsUtils BulletInverseDynamics Bullet3Common LinearMath gtest 
)
IF (NOT WIN32)
        LINK_LIBRARIES(         pthread )
ENDIF()


        ADD_EXECUTABLE(Test_BulletInverseDynamics
                test_invdyn_kinematics.cpp
        )

ADD_TEST(Test_BulletInverseDynamics_PASS Test_BulletInverseDynamics)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(Test_BulletInverseDynamics PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(Test_BulletInverseDynamics PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(Test_BulletInverseDynamics PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

