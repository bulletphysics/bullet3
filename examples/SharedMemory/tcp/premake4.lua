
project ("App_PhysicsServerSharedMemoryBridgeTCP")

	language "C++"
			
	kind "ConsoleApp"
	
	includedirs {"../../ThirdPartyLibs/clsocket/src","../../../src",".."}
		

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
		
	
		links {
			"clsocket",
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
		"../../Utils/b3Clock.cpp",
		"../../Utils/b3Clock.h",		
	}


project "App_PhysicsServerTCP"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

defines { "NO_SHARED_MEMORY" }
						
includedirs {"..","../../../src", "../../ThirdPartyLibs","../../ThirdPartyLibs/clsocket/src"}

links {
	"clsocket","Bullet3Common","BulletInverseDynamicsUtils", "BulletInverseDynamics",	"BulletSoftBody", "BulletDynamics","BulletCollision", "LinearMath", "BussIK"
}



	if os.is("Windows") then
                defines { "WIN32" }
                links {"Ws2_32","Winmm"}
        end
        if os.is("Linux") then
                defines {"_LINUX"}
                links{"dl"}
        end
        if os.is("MacOSX") then
                defines {"_DARWIN"}
        end

language "C++"

myfiles = 
{
	"../IKTrajectoryHelper.cpp",
	"../IKTrajectoryHelper.h",
	"../SharedMemoryCommands.h",
	"../SharedMemoryPublic.h",
	"../PhysicsServerCommandProcessor.cpp",
	"../PhysicsServerCommandProcessor.h",
	"../b3PluginManager.cpp",
	"../PhysicsDirect.cpp",
	"../PhysicsClientC_API.cpp",
	"../PhysicsClient.cpp",
	"../plugins/collisionFilterPlugin/collisionFilterPlugin.cpp",
	"../plugins/pdControlPlugin/pdControlPlugin.cpp",
	"../plugins/pdControlPlugin/pdControlPlugin.h",
	"../b3RobotSimulatorClientAPI_NoDirect.cpp",
	"../b3RobotSimulatorClientAPI_NoDirect.h",
  "../plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
	"../plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
	"../../TinyRenderer/geometry.cpp",
	"../../TinyRenderer/model.cpp",
	"../../TinyRenderer/tgaimage.cpp",
	"../../TinyRenderer/our_gl.cpp",
	"../../TinyRenderer/TinyRenderer.cpp",
	"../../OpenGLWindow/SimpleCamera.cpp",
	"../../OpenGLWindow/SimpleCamera.h",
	"../../Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h",
	"../../Importers/ImportURDFDemo/MultiBodyCreationInterface.h",
	"../../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
	"../../Importers/ImportURDFDemo/MyMultiBodyCreator.h",
	"../../Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
	"../../Importers/ImportMJCFDemo/BulletMJCFImporter.h",
	"../../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
	"../../Importers/ImportURDFDemo/BulletUrdfImporter.h",
	"../../Importers/ImportURDFDemo/UrdfParser.cpp",
 	"../../Importers/ImportURDFDemo/urdfStringSplit.cpp",
	"../../Importers/ImportURDFDemo/UrdfParser.cpp",
	"../../Importers/ImportURDFDemo/UrdfParser.h",
	"../../Importers/ImportURDFDemo/URDF2Bullet.cpp",
	"../../Importers/ImportURDFDemo/URDF2Bullet.h",
	"../../Utils/b3ResourcePath.cpp",
	"../../Utils/b3Clock.cpp",
	"../../Utils/ChromeTraceUtil.cpp",
	"../../Utils/ChromeTraceUtil.h",
	"../../Utils/RobotLoggingUtil.cpp",
	"../../Utils/RobotLoggingUtil.h",
	"../../../Extras/Serialize/BulletWorldImporter/*",
	"../../../Extras/Serialize/BulletFileLoader/*",	
	"../../Importers/ImportURDFDemo/URDFImporterInterface.h",
	"../../Importers/ImportURDFDemo/URDFJointTypes.h",
	"../../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
	"../../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
	"../../Importers/ImportSTLDemo/ImportSTLSetup.h",
	"../../Importers/ImportSTLDemo/LoadMeshFromSTL.h",
	"../../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
	"../../Importers/ImportColladaDemo/ColladaGraphicsInstance.h",
	"../../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",	
	"../../ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
	"../../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
	"../../ThirdPartyLibs/stb_image/stb_image.cpp",     
}

files {
	myfiles,
	"main.cpp",
}

