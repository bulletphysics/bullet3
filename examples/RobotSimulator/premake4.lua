
myfiles = 
{
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
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/SharedMemory/TinyRendererVisualShapeConverter.cpp",
			"../../examples/SharedMemory/TinyRendererVisualShapeConverter.h",
			"../../examples/TinyRenderer/geometry.cpp",
			"../../examples/TinyRenderer/model.cpp",
			"../../examples/TinyRenderer/tgaimage.cpp",
			"../../examples/TinyRenderer/our_gl.cpp",
			"../../examples/TinyRenderer/TinyRenderer.cpp",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
			"../../examples/Utils/RobotLoggingUtil.cpp",
			"../../examples/Utils/RobotLoggingUtil.h",
			"../../examples/ThirdPartyLibs/tinyxml/tinystr.cpp",
			"../../examples/ThirdPartyLibs/tinyxml/tinyxml.cpp",
			"../../examples/ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
			"../../examples/ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
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

project ("App_RobotSimulator")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}

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
                        "../../examples/SharedMemory/PhysicsClientTCP.cpp",
                        "../../examples/SharedMemory/PhysicsClientTCP.h",
                        "../../examples/SharedMemory/PhysicsClientTCP_C_API.cpp",
                        "../../examples/SharedMemory/PhysicsClientTCP_C_API.h",
                }
                defines {"BT_ENABLE_CLSOCKET"}
        end

		if _OPTIONS["audio"] then
			files {
				"../TinyAudio/b3ADSR.cpp",
				"../TinyAudio/b3AudioListener.cpp",
				"../TinyAudio/b3ReadWavFile.cpp",
				"../TinyAudio/b3SoundEngine.cpp",
				"../TinyAudio/b3SoundSource.cpp",
				"../TinyAudio/b3WriteWavFile.cpp",
				"../TinyAudio/RtAudio.cpp",
			}
			defines {"B3_ENABLE_TINY_AUDIO"}

			if _OPTIONS["serial"] then
				defines{"B3_ENABLE_SERIAL"}
				includedirs {"../../examples/ThirdPartyLibs/serial/include"}
				links {"serial"}
			end
			
			if os.is("Windows") then
				links {"winmm","Wsock32","dsound"}
				defines {"WIN32","__WINDOWS_MM__","__WINDOWS_DS__"}
			end
			
			if os.is("Linux") then initX11() 
			                defines  {"__OS_LINUX__","__LINUX_ALSA__"}
				links {"asound","pthread"}
			end


			if os.is("MacOSX") then
				links{"Cocoa.framework"}
				links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
				defines {"__OS_MACOSX__","__MACOSX_CORE__"}
			end
		end
		files {
			"RobotSimulatorMain.cpp",
			"b3RobotSimulatorClientAPI.cpp",
			"b3RobotSimulatorClientAPI.h",
			"MinitaurSetup.cpp",
			"MinitaurSetup.h",
			myfiles
		}
	if os.is("Linux") then
       		initX11()
	end


if _OPTIONS["serial"] then

project ("App_VRGloveHandSimulator")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}

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

		if _OPTIONS["audio"] then
			files {
				"../TinyAudio/b3ADSR.cpp",
				"../TinyAudio/b3AudioListener.cpp",
				"../TinyAudio/b3ReadWavFile.cpp",
				"../TinyAudio/b3SoundEngine.cpp",
				"../TinyAudio/b3SoundSource.cpp",
				"../TinyAudio/b3WriteWavFile.cpp",
				"../TinyAudio/RtAudio.cpp",
			}
			defines {"B3_ENABLE_TINY_AUDIO"}

			
			defines{"B3_ENABLE_SERIAL"}
			includedirs {"../../examples/ThirdPartyLibs/serial/include"}
			links {"serial"}
		
			if os.is("Windows") then
				links {"winmm","Wsock32","dsound"}
				defines {"WIN32","__WINDOWS_MM__","__WINDOWS_DS__"}
			end
			
			if os.is("Linux") then initX11() 
			                defines  {"__OS_LINUX__","__LINUX_ALSA__"}
				links {"asound","pthread"}
			end


			if os.is("MacOSX") then
				links{"Cocoa.framework"}
				links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
				defines {"__OS_MACOSX__","__MACOSX_CORE__"}
			end
		end
		files {
			"VRGloveSimulatorMain.cpp",
			"b3RobotSimulatorClientAPI.cpp",
			"b3RobotSimulatorClientAPI.h",
			myfiles
		}
	if os.is("Linux") then
       		initX11()
	end
end