		project ("Test_SharedMemoryPhysicsClient")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples/SharedMemory"}
		links {
			"BulletFileLoader",
			"Bullet3Common", 
			"LinearMath"
		}
			
		files {
			"test.c",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
		}
		