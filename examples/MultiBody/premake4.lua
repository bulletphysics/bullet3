
project "App_CustomMultiBodyCreation"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {".", "../../src",  "../ThirdPartyLibs"}

links {
	"Bullet3Common", "BulletDynamics","BulletCollision", "LinearMath"
}

language "C++"

files {
	"MultiBodyCustomURDFDemo.cpp",
	"main.cpp",
	"../Importers/ImportURDFDemo/URDF2Bullet.cpp",
	"../Importers/ImportURDFDemo/MyURDFImporter.cpp",
	"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
	"../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
	"../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
	"../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
	"../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
	"../ThirdPartyLibs/tinyxml/tinystr.cpp",
	"../ThirdPartyLibs/tinyxml/tinyxml.cpp",
	"../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
	"../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
	"../ThirdPartyLibs/urdf/boost_replacement/printf_console.cpp",
	"../ThirdPartyLibs/urdf/boost_replacement/string_split.cpp",
	"../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/pose.cpp",
	"../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/model.cpp",
	"../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/link.cpp",
	"../ThirdPartyLibs/urdf/urdfdom/urdf_parser/src/joint.cpp",
	
}

