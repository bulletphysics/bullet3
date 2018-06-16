

project ("App_RobotSimulator")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"B3_USE_ROBOTSIM_GUI", "PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}


	links{"BulletRobotics", "BulletExampleBrowserLib", "gwen", "OpenGL_Window","BulletFileLoader","BulletWorldImporter","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","Bullet3Common"}
	initOpenGL()
	initGlew()

  	includedirs {
                ".",
                "../../src",
		"../../examples/SharedMemory",
                "../ThirdPartyLibs",
                }


	if os.is("MacOSX") then
		links{"Cocoa.framework"}
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
			"../../examples/ExampleBrowser/InProcessExampleBrowser.cpp",
			"../../examples/SharedMemory/PhysicsServerExample.cpp",
			"../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp",
			"../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
		}

if (_OPTIONS["enable_static_vr_plugin"]) then
	files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

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

	links{"BulletRobotics", "BulletExampleBrowserLib","gwen", "OpenGL_Window","BulletFileLoader","BulletWorldImporter","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","BussIK","Bullet3Common"}
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
			
		}

if (_OPTIONS["enable_static_vr_plugin"]) then
	files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

	if os.is("Linux") then
       		initX11()
	end
end


project ("App_HelloBulletRobotics")

	language "C++"
	kind "ConsoleApp"

	links{"BulletRobotics","BulletFileLoader","BulletWorldImporter","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","Bullet3Common"}
	
  includedirs {
                ".",
                "../../src",
                "../../examples/SharedMemory",
                "../ThirdPartyLibs",
                }

if not _OPTIONS["no-enet"] then

		includedirs {"../../examples/ThirdPartyLibs/enet/include"}
	
		if os.is("Windows") then 
			defines { "WIN32" }
			links {"Ws2_32","Winmm"}
		end
		if os.is("Linux") then
		end
		if os.is("MacOSX") then
		end		
		links {"enet"}		
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
    defines {"BT_ENABLE_CLSOCKET"}
	end

	if os.is("MacOSX") then
		links{"Cocoa.framework"}
	end

	
	if os.is("Linux") then initX11()
                     links {"pthread"}
        end

	
		files {
			 "HelloBulletRobotics.cpp"
		}


