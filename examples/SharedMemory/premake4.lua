
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
	"SharedMemoryCommands.h",
	"SharedMemoryPublic.h",
	"PhysicsServer.cpp",
	"PosixSharedMemory.cpp",
	"Win32SharedMemory.cpp",
	"InProcessMemory.cpp",
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
	"TinyRendererVisualShapeConverter.cpp",
	"TinyRendererVisualShapeConverter.h",
	"../TinyRenderer/geometry.cpp",
	"../TinyRenderer/model.cpp",
	"../TinyRenderer/tgaimage.cpp",
	"../TinyRenderer/our_gl.cpp",
	"../TinyRenderer/TinyRenderer.cpp",
	"../OpenGLWindow/SimpleCamera.cpp",
	"../OpenGLWindow/SimpleCamera.h",
	"../Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h",
	"../Importers/ImportURDFDemo/MultiBodyCreationInterface.h",
	"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
	"../Importers/ImportURDFDemo/MyMultiBodyCreator.h",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.h",
	"../Importers/ImportURDFDemo/UrdfParser.cpp",
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
    "../ThirdPartyLibs/tinyxml/tinystr.cpp",
    "../ThirdPartyLibs/tinyxml/tinyxml.cpp",
    "../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
    "../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
"../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
                        "../ThirdPartyLibs/stb_image/stb_image.cpp",     
}

