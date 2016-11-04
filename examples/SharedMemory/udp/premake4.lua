
project ("App_PhysicsServerUDP")

	language "C++"
			
	kind "ConsoleApp"
	
	includedirs {"../../ThirdPartyLibs/enet/include","../../../src",".."}
		
	if os.is("Windows") then 
			defines { "WIN32" }

		links {"Ws2_32","Winmm"}
	end
	if os.is("Linux") then
	end
	if os.is("MacOSX") then
	end		
		
	
		links {
			"enet",
			"BulletFileLoader",
			"Bullet3Common", 
			"LinearMath"
		}
	
	files {
		"main.cpp",
		"../PhysicsClient.cpp",
		"../PhysicsClient.h",
		"../PhysicsDirect.cpp",
		"../PhysicsDirect.h",
		"../PhysicsCommandProcessorInterface.h",
		"../SharedMemoryCommandProcessor.cpp",
		"../SharedMemoryCommandProcessor.h",
		"../PhysicsClientC_API.cpp",
		"../PhysicsClientC_API.h",
		"../Win32SharedMemory.cpp",
		"../Win32SharedMemory.h",
		"../PosixSharedMemory.cpp",
		"../PosixSharedMemory.h",
		"../../Utils/b3ResourcePath.cpp",
		"../../Utils/b3ResourcePath.h",
	}

