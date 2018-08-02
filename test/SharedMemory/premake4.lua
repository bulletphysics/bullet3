project ("Test_SharedMemoryPhysicsClient")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples"}
		links {
			"BulletFileLoader",
			"Bullet3Common", 
			"LinearMath"
		}
		defines {"PHYSICS_SHARED_MEMORY"}
			
		files {
			"test.c",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
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
			"../../examples/Utils/b3Clock.cpp",
			"../../examples/Utils/b3Clock.h",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
		}

project ("Test_PhysicsClientUDP")

                language "C++"
                kind "ConsoleApp"

                includedirs {
                "../../src", 
                "../../examples",
                "../../examples/ThirdPartyLibs/enet/include"
                }
                links {
												"enet",
                        "BulletFileLoader",
                        "Bullet3Common",
                        "LinearMath"
                }
		if os.is("Windows") then
                	defines { "WIN32" }
        	        links {"Ws2_32","Winmm"}
	        end
		if os.is("Linux") then
			links {"pthread"}
		end

                defines {"PHYSICS_UDP"}

                files {
									"test.c",
									"../../examples/SharedMemory/PhysicsClient.cpp",
									"../../examples/SharedMemory/PhysicsClient.h",
									"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
									"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
									"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
									"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
									"../../examples/SharedMemory/PhysicsClientUDP.cpp",
									"../../examples/SharedMemory/PhysicsClientUDP.h",
									"../../examples/SharedMemory/PhysicsClientUDP_C_API.cpp",
									"../../examples/SharedMemory/PhysicsClientUDP_C_API.h",
									"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",	
									"../../examples/SharedMemory/PhysicsClientC_API.cpp",
									"../../examples/SharedMemory/PhysicsClientC_API.h",
									"../../examples/SharedMemory/Win32SharedMemory.cpp",
									"../../examples/SharedMemory/Win32SharedMemory.h",
									"../../examples/SharedMemory/PosixSharedMemory.cpp",
									"../../examples/SharedMemory/PosixSharedMemory.h",
									"../../examples/Utils/b3ResourcePath.cpp",
									"../../examples/Utils/b3ResourcePath.h",
									"../../examples/SharedMemory/PhysicsDirect.cpp",
									"../../examples/Utils/b3Clock.cpp",
									"../../examples/MultiThreading/b3PosixThreadSupport.cpp",
									"../../examples/MultiThreading/b3Win32ThreadSupport.cpp",
									"../../examples/MultiThreading/b3ThreadSupportInterface.cpp",
            }


project ("Test_PhysicsClientTCP")

                language "C++"
                kind "ConsoleApp"

                includedirs {
                "../../src", 
                "../../examples",
                "../../examples/ThirdPartyLibs/clsocket/src"
                }
                links {
												"clsocket",
                        "BulletFileLoader",
                        "Bullet3Common",
                        "LinearMath"
                }
		if os.is("Windows") then
                	defines { "WIN32" }
        	        links {"Ws2_32","Winmm"}
	        end

		if os.is("Windows") then
                	defines { "WIN32","_WINSOCK_DEPRECATED_NO_WARNINGS" }
                	end
                if os.is("Linux") then
                 defines {"_LINUX"}
                end
                if os.is("MacOSX") then
                 defines {"_DARWIN"}
                end

                defines {"PHYSICS_TCP"}

                files {
									"test.c",
									"../../examples/SharedMemory/PhysicsClient.cpp",
									"../../examples/SharedMemory/PhysicsClient.h",
									"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
									"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
									"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
									"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
									"../../examples/SharedMemory/PhysicsClientTCP.cpp",
									"../../examples/SharedMemory/PhysicsClientTCP.h",
									"../../examples/SharedMemory/PhysicsClientTCP_C_API.cpp",
									"../../examples/SharedMemory/PhysicsClientTCP_C_API.h",
									"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",	
									"../../examples/SharedMemory/PhysicsClientC_API.cpp",
									"../../examples/SharedMemory/PhysicsClientC_API.h",
									"../../examples/SharedMemory/Win32SharedMemory.cpp",
									"../../examples/SharedMemory/Win32SharedMemory.h",
									"../../examples/SharedMemory/PosixSharedMemory.cpp",
									"../../examples/SharedMemory/PosixSharedMemory.h",
									"../../examples/Utils/b3ResourcePath.cpp",
									"../../examples/Utils/b3ResourcePath.h",
									"../../examples/SharedMemory/PhysicsDirect.cpp",
									"../../examples/Utils/b3Clock.cpp",
            }

		
project ("Test_PhysicsServerLoopBack")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_LOOP_BACK", "SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD"}
		links {
			"BulletInverseDynamicsUtils",
			"BulletInverseDynamics",
			"BulletFileLoader",
			"BulletWorldImporter",
			"Bullet3Common",
			"BulletDynamics", 
			"BulletCollision", 
			"BussIK",
			"LinearMath"
		}
        if os.is("Linux") then
            links{"dl"}
        end
			
		files {
			"test.c",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h",
			"../../examples/SharedMemory/IKTrajectoryHelper.cpp",
			"../../examples/SharedMemory/IKTrajectoryHelper.h",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
			"../../examples/SharedMemory/PhysicsServer.cpp",
			"../../examples/SharedMemory/PhysicsServer.h",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.h",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.h",
			"../../examples/SharedMemory/b3PluginManager.cpp",
			"../../examples/SharedMemory/PhysicsDirect.cpp",
			"../../examples/SharedMemory/PhysicsLoopBack.cpp",
			"../../examples/SharedMemory/PhysicsLoopBack.h",
			"../../examples/SharedMemory/PhysicsLoopBackC_API.cpp",
			"../../examples/SharedMemory/PhysicsLoopBackC_API.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
			"../../examples/OpenGLWindow/SimpleCamera.cpp",
			"../../examples/OpenGLWindow/SimpleCamera.h",
			"../../examples/TinyRenderer/geometry.cpp",
			"../../examples/TinyRenderer/model.cpp",
			"../../examples/TinyRenderer/tgaimage.cpp",
			"../../examples/TinyRenderer/our_gl.cpp",
			"../../examples/TinyRenderer/TinyRenderer.cpp",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
			"../../examples/Utils/RobotLoggingUtil.cpp",
			"../../examples/Utils/RobotLoggingUtil.h",
			"../../examples/Utils/b3Clock.cpp",
			"../../examples/Utils/b3Clock.h",
			"../../examples/Utils/ChromeTraceUtil.cpp",
			"../../examples/Utils/ChromeTraceUtil.h",
			"../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
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
			"../../examples/ThirdPartyLibs/stb_image/stb_image.cpp",
	}
	
if (_OPTIONS["enable_static_plugins"]) then
		files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end
		
		project ("Test_PhysicsServerDirect")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_SERVER_DIRECT","SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD"}
		links {
			"BulletInverseDynamicsUtils",
			"BulletInverseDynamics",
			"BulletFileLoader",
			"BulletWorldImporter",
			"Bullet3Common",
			"BulletDynamics", 
			"BulletCollision",
			"BussIK",
			"LinearMath"
		}
        if os.is("Linux") then
            links{"dl"}
        end
			
		files {
			"test.c",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h",
			"../../examples/SharedMemory/IKTrajectoryHelper.cpp",
			"../../examples/SharedMemory/IKTrajectoryHelper.h",
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
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
			"../../examples/TinyRenderer/geometry.cpp",
			"../../examples/TinyRenderer/model.cpp",
			"../../examples/TinyRenderer/tgaimage.cpp",
			"../../examples/TinyRenderer/our_gl.cpp",
			"../../examples/TinyRenderer/TinyRenderer.cpp",
			"../../examples/OpenGLWindow/SimpleCamera.cpp",
			"../../examples/OpenGLWindow/SimpleCamera.h",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
			"../../examples/Utils/RobotLoggingUtil.cpp",
			"../../examples/Utils/RobotLoggingUtil.h",
			"../../examples/Utils/b3Clock.cpp",
			"../../examples/Utils/ChromeTraceUtil.cpp",
			"../../examples/Utils/ChromeTraceUtil.h",			
			"../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
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
                        "../../examples/ThirdPartyLibs/stb_image/stb_image.cpp",     	
		}
if (_OPTIONS["enable_static_plugins"]) then
		files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

project ("Test_PhysicsServerInProcessExampleBrowser")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER", "SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD"}
--		links {
--			"BulletExampleBrowserLib",
--			"BulletFileLoader",
--			"BulletWorldImporter",
--			"Bullet3Common",
--			"BulletDynamics", 
--			"BulletCollision", 
--			"LinearMath"
--		}
	hasCL = findOpenCL("clew")

	links{"BulletExampleBrowserLib","gwen", "OpenGL_Window","BulletFileLoader","BulletWorldImporter","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","BussIK","Bullet3Common"}
	initOpenGL()
	initGlew()

  	includedirs {
                ".",
                "../../src",
                "../ThirdPartyLibs",
                }


	if os.is("MacOSX") then
		links{"Cocoa.framework"}
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

		files {
			"test.c",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h",
			"../../examples/SharedMemory/IKTrajectoryHelper.cpp",
			"../../examples/SharedMemory/IKTrajectoryHelper.h",
			"../../examples/ExampleBrowser/InProcessExampleBrowser.cpp",
			"../../examples/SharedMemory/InProcessMemory.cpp",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
			"../../examples/SharedMemory/PhysicsServer.cpp",
			"../../examples/SharedMemory/PhysicsServer.h",
			"../../examples/SharedMemory/PhysicsServerExample.cpp",
			"../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp",
			"../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.h",
			"../../examples/SharedMemory/PhysicsDirect.cpp",
			"../../examples/SharedMemory/PhysicsDirect.h",
			"../../examples/SharedMemory/PhysicsDirectC_API.cpp",
			"../../examples/SharedMemory/PhysicsDirectC_API.h",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.h",
			"../../examples/SharedMemory/b3PluginManager.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
			"../../examples/TinyRenderer/geometry.cpp",
			"../../examples/TinyRenderer/model.cpp",
			"../../examples/TinyRenderer/tgaimage.cpp",
			"../../examples/TinyRenderer/our_gl.cpp",
			"../../examples/TinyRenderer/TinyRenderer.cpp",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
			"../../examples/Utils/RobotLoggingUtil.cpp",
			"../../examples/Utils/RobotLoggingUtil.h",
			"../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../../examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../../examples/Importers/ImportURDFDemo/UrdfParser.cpp",
			"../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
			"../../examples/MultiThreading/b3PosixThreadSupport.cpp",
			"../../examples/MultiThreading/b3Win32ThreadSupport.cpp",
			"../../examples/MultiThreading/b3ThreadSupportInterface.cpp",
			"../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
			"../../examples/ThirdPartyLibs/stb_image/stb_image.cpp",
	}
if (_OPTIONS["enable_static_vr_plugin"]) then
		files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

	if os.is("Linux") then
       		initX11()
	end

	
