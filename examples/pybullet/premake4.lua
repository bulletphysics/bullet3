		

project ("pybullet")
		language "C++"
		kind "SharedLib"
		
		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}
	hasCL = findOpenCL("clew")

	links{"BulletExampleBrowserLib","gwen", "BulletFileLoader","BulletWorldImporter","OpenGL_Window","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","BussIK", "Bullet3Common"}
	initOpenGL()
	initGlew()

  	includedirs {
                ".",
                "../../src",
                "../ThirdPartyLibs",
                }

	if os.is("MacOSX") then
--		targetextension {"so"}
		links{"Cocoa.framework","Python"}
	end

		if (hasCL) then
			links {
				"Bullet3OpenCL_clew",
				"Bullet3Dynamics",
				"Bullet3Collision",
				"Bullet3Geometry",
				"Bullet3Common",
			}
		end

if not _OPTIONS["no-enet"] then

		includedirs {"../../examples/ThirdPartyLibs/enet/include"}
	
		if os.is("Windows") then 
--			targetextension {"dylib"}
			defines { "WIN32" }
			links {"Ws2_32","Winmm"}
		end
		if os.is("Linux") then
		end
		if os.is("MacOSX") then
		end		
		
		links {"enet"}		

		files {
			"../../examples/SharedMemory/PhysicsClientUDP.cpp",
			"../../examples/SharedMemory/PhysicsClientUDP.h",
			"../../examples/SharedMemory/PhysicsClientUDP_C_API.cpp",
			"../../examples/SharedMemory/PhysicsClientUDP_C_API.h",
		}	
		defines {"BT_ENABLE_ENET"}
	end


		files {
			"pybullet.c",
			"../../examples/SharedMemory/IKTrajectoryHelper.cpp",
			"../../examples/SharedMemory/IKTrajectoryHelper.h",
			"../../examples/ExampleBrowser/InProcessExampleBrowser.cpp",
			"../../examples/SharedMemory/TinyRendererVisualShapeConverter.cpp",
			"../../examples/SharedMemory/TinyRendererVisualShapeConverter.h",
			"../../examples/OpenGLWindow/SimpleCamera.cpp",
			"../../examples/OpenGLWindow/SimpleCamera.h",
			"../../examples/TinyRenderer/geometry.cpp",
			"../../examples/TinyRenderer/model.cpp",
			"../../examples/TinyRenderer/tgaimage.cpp",
			"../../examples/TinyRenderer/our_gl.cpp",
			"../../examples/TinyRenderer/TinyRenderer.cpp",
			"../../examples/SharedMemory/InProcessMemory.cpp",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
			"../../examples/SharedMemory/PhysicsServer.cpp",
			"../../examples/SharedMemory/PhysicsServer.h",
			"../../examples/SharedMemory/PhysicsServerExample.cpp",
			"../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.h",
			"../../examples/SharedMemory/PhysicsDirect.cpp",
			"../../examples/SharedMemory/PhysicsDirect.h",
			"../../examples/SharedMemory/PhysicsDirectC_API.cpp",
			"../../examples/SharedMemory/PhysicsDirectC_API.h",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
			"../../examples/ThirdPartyLibs/tinyxml/tinystr.cpp",
			"../../examples/ThirdPartyLibs/tinyxml/tinyxml.cpp",
			"../../examples/ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
			"../../examples/ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../../examples/ThirdPartyLibs/stb_image/stb_image.cpp",
			"../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../../examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../../examples/Importers/ImportURDFDemo/UrdfParser.cpp",
			"../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
			"../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
			"../../examples/MultiThreading/b3PosixThreadSupport.cpp",
			"../../examples/MultiThreading/b3Win32ThreadSupport.cpp",
			"../../examples/MultiThreading/b3ThreadSupportInterface.cpp",
			}
	
	includedirs {
		_OPTIONS["python_include_dir"],
	}
	libdirs {
		_OPTIONS["python_lib_dir"]
	}
	
	if os.is("Linux") then
       		initX11()
	end

	
