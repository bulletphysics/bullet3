project ("Test_SharedMemoryPhysicsClient")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples"}
		links {
			"BulletFileLoader",
			"Bullet3Common", 
			"LinearMath"
		}
			
		files {
			"test.c",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
		}
		
project ("Test_PhysicsServerLoopBack")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_LOOP_BACK"}
		links {
			"BulletFileLoader",
			"BulletWorldImporter",
			"Bullet3Common",
			"BulletDynamics", 
			"BulletCollision", 
			"LinearMath"
		}
			
		files {
			"test.c",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
			"../../examples/SharedMemory/PhysicsServer.cpp",
			"../../examples/SharedMemory/PhysicsServer.h",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.h",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.h",
			"../../examples/SharedMemory/PhysicsLoopBack.cpp",
			"../../examples/SharedMemory/PhysicsLoopBack.h",
			"../../examples/SharedMemory/PhysicsLoopBackC_API.cpp",
			"../../examples/SharedMemory/PhysicsLoopBackC_API.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
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
			"../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../../examples/Importers/ImportURDFDemo/UrdfParser.cpp",
			"../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
		}
		
		project ("Test_PhysicsServerDirect")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_SERVER_DIRECT"}
		links {
			"BulletFileLoader",
			"BulletWorldImporter",
			"Bullet3Common",
			"BulletDynamics", 
			"BulletCollision", 
			"LinearMath"
		}
			
		files {
			"test.c",
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
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
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
			"../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../../examples/Importers/ImportURDFDemo/UrdfParser.cpp",
			"../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
		}
		

	
