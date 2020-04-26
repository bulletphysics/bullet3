		

project ("BulletRobotics")
		language "C++"
		kind "StaticLib"
		
		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}
	hasCL = findOpenCL("clew")

	links{"BulletExampleBrowserLib","gwen", "BulletFileLoader","BulletWorldImporter","OpenGL_Window","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","BussIK", "Bullet3Common"}
	initOpenGL()
	initGlew()

  	includedirs {
                "../../src",
                "../../examples",
                "../../examples/SharedMemory",
                "../ThirdPartyLibs",
                "../ThirdPartyLibs/enet/include",
                "../ThirdPartyLibs/clsocket/src",
                }

	if os.is("MacOSX") then
--		targetextension {"so"}
		links{"Cocoa.framework","Python"}
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

	if not _OPTIONS["no-clsocket"] then

                includedirs {"../../examples/ThirdPartyLibs/clsocket/src"}

		 if os.is("Windows") then
                	defines { "WIN32" }
                	links {"Ws2_32","Winmm"}
       		 end
        	if os.is("Linux") then
                	defines {"_LINUX"}
        	end
        	if os.is("MacOSX") then
                	defines {"_DARWIN"}
        	end

                links {"clsocket"}

                files {
			"../../examples/SharedMemory/RemoteGUIHelperTCP.cpp",
                        "../../examples/SharedMemory/PhysicsClientTCP.cpp",
			"../../examples/SharedMemory/GraphicsServerExample.cpp",
                        "../../examples/SharedMemory/PhysicsClientTCP.h",
                        "../../examples/SharedMemory/PhysicsClientTCP_C_API.cpp",
                        "../../examples/SharedMemory/PhysicsClientTCP_C_API.h",
                }
                defines {"BT_ENABLE_CLSOCKET"}
        end


		files {
		"../../examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp",
		"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
		"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h",
		"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
		"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h",
		"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
		"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
		"../../examples/SharedMemory/IKTrajectoryHelper.cpp",
		"../../examples/SharedMemory/IKTrajectoryHelper.h",
		"../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
		"../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
		"../../examples/SharedMemory/RemoteGUIHelper.cpp",
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
		"../../examples/SharedMemory/PhysicsServerSharedMemory.cpp",
		"../../examples/SharedMemory/PhysicsServerSharedMemory.h",
		"../../examples/SharedMemory/PhysicsDirect.cpp",
		"../../examples/SharedMemory/PhysicsDirect.h",
		"../../examples/SharedMemory/PhysicsDirectC_API.cpp",
		"../../examples/SharedMemory/PhysicsDirectC_API.h",
		"../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
		"../../examples/SharedMemory/PhysicsServerCommandProcessor.h",
		"../../examples/SharedMemory/b3PluginManager.cpp",
		"../../examples/SharedMemory/b3PluginManager.h",
				
		"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
		"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
		"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
		"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
		"../../examples/SharedMemory/PhysicsClientC_API.cpp",
	
		"../../examples/SharedMemory/PhysicsClientC_API.h",
		"../../examples/SharedMemory/SharedMemoryPublic.h",

		"../../examples/SharedMemory/Win32SharedMemory.cpp",
		"../../examples/SharedMemory/Win32SharedMemory.h",
		"../../examples/SharedMemory/PosixSharedMemory.cpp",
		"../../examples/SharedMemory/PosixSharedMemory.h",

		"../../examples/Utils/b3ResourcePath.cpp",
		"../../examples/Utils/b3ResourcePath.h",
		"../../examples/Utils/RobotLoggingUtil.cpp",
		"../../examples/Utils/RobotLoggingUtil.h",
		"../../examples/Utils/b3Clock.cpp",
		"../../examples/Utils/b3ResourcePath.cpp",
		"../../examples/Utils/b3ERPCFMHelper.hpp",
		"../../examples/Utils/b3ReferenceFrameHelper.hpp",
		"../../examples/Utils/ChromeTraceUtil.cpp",

		"../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp",

		"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
		"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h",

		"../../examples/ThirdPartyLibs/stb_image/stb_image.cpp",

		"../../examples/ThirdPartyLibs/BussIK/Jacobian.cpp",
		"../../examples/ThirdPartyLibs/BussIK/LinearR2.cpp",
		"../../examples/ThirdPartyLibs/BussIK/LinearR3.cpp",
		"../../examples/ThirdPartyLibs/BussIK/LinearR4.cpp",
		"../../examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp",
		"../../examples/ThirdPartyLibs/BussIK/Misc.cpp",
		"../../examples/ThirdPartyLibs/BussIK/Node.cpp",
		"../../examples/ThirdPartyLibs/BussIK/Tree.cpp",
		"../../examples/ThirdPartyLibs/BussIK/VectorRn.cpp",

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
			
if (_OPTIONS["enable_static_vr_plugin"]) then
		files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end


	
