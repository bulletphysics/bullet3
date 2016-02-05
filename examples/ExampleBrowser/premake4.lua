	
		project "App_ExampleBrowser"

		hasCL = findOpenCL("clew")
	
		if (hasCL) then

				-- project ("App_Bullet3_OpenCL_Demos_" .. vendor)

				initOpenCL("clew")

		end

		language "C++"
				
		kind "ConsoleApp"

  	includedirs {
                ".",
                "../../src",
                "../ThirdPartyLibs",
                }

	if _OPTIONS["lua"] then
		includedirs{"../ThirdPartyLibs/lua-5.2.3/src"}
		links {"lua-5.2.3"}
		defines {"ENABLE_LUA"}
		files {"../LuaDemo/LuaPhysicsSetup.cpp"}
	end

			
		links{"gwen", "OpenGL_Window","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","Bullet3Common"}
		initOpenGL()
		initGlew()

		if (hasCL) then
			links {
				"Bullet3OpenCL_clew",
				"Bullet3Dynamics",
				"Bullet3Collision",
				"Bullet3Geometry",
				"Bullet3Common",
			}
		end
		
		defines {"INCLUDE_CLOTH_DEMOS"}
			


		files {
		"*.cpp",
		"*.h",
		"GwenGUISupport/*.cpp",
		"GwenGUISupport/*.h",
		"../SharedMemory/PhysicsClientC_API.cpp",
		"../SharedMemory/PhysicsClientC_API.h",
		"../SharedMemory/PhysicsServerExample.cpp",
		"../SharedMemory/PhysicsClientExample.cpp",
		"../SharedMemory/PhysicsServer.cpp",
		"../SharedMemory/PhysicsServerSharedMemory.cpp",
		"../SharedMemory/PhysicsClientSharedMemory.cpp",
		"../SharedMemory/PhysicsClient.cpp",
		"../SharedMemory/PosixSharedMemory.cpp",
		"../SharedMemory/Win32SharedMemory.cpp",
		"../SharedMemory/PhysicsDirect.cpp",
		"../SharedMemory/PhysicsDirect.h",
		"../SharedMemory/PhysicsDirectC_API.cpp",
		"../SharedMemory/PhysicsDirectC_API.h",
		"../SharedMemory/PhysicsLoopBack.cpp",
		"../SharedMemory/PhysicsLoopBack.h",
		"../SharedMemory/PhysicsLoopBackC_API.cpp",
		"../SharedMemory/PhysicsLoopBackC_API.h",
		"../SharedMemory/PhysicsServerCommandProcessor.cpp",
		"../SharedMemory/PhysicsServerCommandProcessor.h",
		"../MultiThreading/MultiThreadingExample.cpp",
		"../MultiThreading/b3PosixThreadSupport.cpp",
		"../MultiThreading/b3Win32ThreadSupport.cpp",
		"../MultiThreading/b3ThreadSupportInterface.cpp",
		"../InverseDynamics/InverseDynamicsExample.cpp",
		"../InverseDynamics/InverseDynamicsExample.h",
		"../BasicDemo/BasicExample.*",
		"../Tutorial/*",
		"../Collision/*",
		"../Collision/Internal/*",
		"../Benchmarks/*",
		"../CommonInterfaces/*",
		"../ForkLift/ForkLiftDemo.*",
		"../Importers/**",
		"../../Extras/Serialize/BulletWorldImporter/*",
		"../../Extras/Serialize/BulletFileLoader/*",	
		"../Planar2D/Planar2D.*",
		"../RenderingExamples/*",
		"../VoronoiFracture/*",
		"../SoftDemo/*",
		"../RollingFrictionDemo/*",
		"../FractureDemo/*",
		"../DynamicControlDemo/*",
		"../Constraints/*",
		"../Vehicles/*",
		"../Raycast/*",
		"../MultiBody/MultiDofDemo.cpp",
		"../MultiBody/TestJointTorqueSetup.cpp",
		"../MultiBody/Pendulum.cpp",
		"../MultiBody/MultiBodyConstraintFeedback.cpp",
		"../MultiBody/InvertedPendulumPDControl.cpp",
		"../ThirdPartyLibs/stb_image/*",
		"../ThirdPartyLibs/Wavefront/tiny_obj_loader.*",
		"../ThirdPartyLibs/tinyxml/*",
		"../Utils/b3Clock.*",
		"../Utils/b3ResourcePath.*",
		"../GyroscopicDemo/GyroscopicSetup.cpp",
		"../GyroscopicDemo/GyroscopicSetup.h",
		"../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/pose.cpp",
		"../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/model.cpp",
    "../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/link.cpp",
    "../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/joint.cpp",
    "../ThirdPartyLibs/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h",
    "../ThirdPartyLibs/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h",
    "../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h",
    "../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h",
    "../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h",
    "../ThirdPartyLibs/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h",
    "../ThirdPartyLibs/tinyxml/tinystr.cpp",
    "../ThirdPartyLibs/tinyxml/tinyxml.cpp",
    "../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
    "../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
    "../ThirdPartyLibs/urdf/boost_replacement/lexical_cast.h",
    "../ThirdPartyLibs/urdf/boost_replacement/shared_ptr.h",
    "../ThirdPartyLibs/urdf/boost_replacement/printf_console.cpp",
    "../ThirdPartyLibs/urdf/boost_replacement/printf_console.h",
    "../ThirdPartyLibs/urdf/boost_replacement/string_split.cpp",
    "../ThirdPartyLibs/urdf/boost_replacement/string_split.h",

		}
		
		if (hasCL and findOpenGL3()) then
			files {
				"../OpenCL/broadphase/*",
				"../OpenCL/CommonOpenCL/*",
				"../OpenCL/rigidbody/GpuConvexScene.cpp",
				"../OpenCL/rigidbody/GpuRigidBodyDemo.cpp",
			}
		end

if os.is("Linux") then 
	initX11()
end

if os.is("MacOSX") then
	links{"Cocoa.framework"}
end

			
