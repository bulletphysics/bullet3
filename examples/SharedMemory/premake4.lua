
project "App_SharedMemoryPhysics"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {".","../../src", "../ThirdPartyLibs",}

links {
	"Bullet3Common",	"BulletDynamics","BulletCollision", "LinearMath"
}

language "C++"

files {
	"PhysicsClient.cpp",
	"PhysicsClientSharedMemory.cpp",
	"PhysicsClientExample.cpp",
	"PhysicsServerExample.cpp",
	"PhysicsServerSharedMemory.cpp",
	"PhysicsServerSharedMemory.h",
	"PhysicsServer.cpp",
	"PhysicsServer.h",
	"main.cpp",
	"PhysicsClientC_API.cpp",
	"PhysicsServer.cpp",
	"PosixSharedMemory.cpp",
	"Win32SharedMemory.cpp",
	"PhysicsDirect.cpp",
	"PhysicsDirect.h",
	"PhysicsDirectC_API.cpp",
	"PhysicsDirectC_API.h",
	"PhysicsLoopBack.cpp",
	"PhysicsLoopBack.h",
	"PhysicsLoopBackC_API.cpp",
	"PhysicsLoopBackC_API.h",
	"PhysicsServerCommandProcessor.cpp",
	"PhysicsServerCommandProcessor.h",
	"../Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h",
	"../Importers/ImportURDFDemo/MultiBodyCreationInterface.h",
	"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
	"../Importers/ImportURDFDemo/MyMultiBodyCreator.h",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.h",
	"../Importers/ImportURDFDemo/UrdfParser.cpp",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.h",
 	"../Importers/ImportURDFDemo/urdfStringSplit.cpp",
	"../Importers/ImportURDFDemo/UrdfParser.cpp",
	"../Importers/ImportURDFDemo/UrdfParser.h",
	"../Importers/ImportURDFDemo/URDF2Bullet.cpp",
	"../Importers/ImportURDFDemo/URDF2Bullet.h",
	"../Utils/b3ResourcePath.cpp",
	"../Utils/b3Clock.cpp",	
	"../../Extras/Serialize/BulletWorldImporter/*",
	"../../Extras/Serialize/BulletFileLoader/*",	
	"../Importers/ImportURDFDemo/URDFImporterInterface.h",
	"../Importers/ImportURDFDemo/URDFJointTypes.h",
	"../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
	"../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
	"../Importers/ImportSTLDemo/ImportSTLSetup.h",
	"../Importers/ImportSTLDemo/LoadMeshFromSTL.h",
	"../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
	"../Importers/ImportColladaDemo/ColladaGraphicsInstance.h",
	"../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",	
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

