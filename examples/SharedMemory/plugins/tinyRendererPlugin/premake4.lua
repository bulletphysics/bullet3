		

project ("pybullet_tinyRendererPlugin")
		language "C++"
		kind "SharedLib"
		
		includedirs {".","../../../../src", "../../../../examples",
		"../../../ThirdPartyLibs"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}
	hasCL = findOpenCL("clew")

	links{"BulletCollision", "Bullet3Common", "LinearMath"}

	if os.is("MacOSX") then
--		targetextension {"so"}
		links{"Cocoa.framework","Python"}
	end


		files {
			"tinyRendererPlugin.cpp",
			"tinyRendererPlugin.h",
			"TinyRendererVisualShapeConverter.cpp",
			"TinyRendererVisualShapeConverter.h",
			"../../../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../../../Importers/ImportColladaDemo/LoadMeshFromCollada.h",
			"../../../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
			"../../../Importers/ImportMeshUtility/b3ImportMeshUtility.h",
			"../../../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../../../Importers/ImportObjDemo/LoadMeshFromObj.h",
			"../../../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../../../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h",
			"../../../TinyRenderer/geometry.cpp",
			"../../../TinyRenderer/model.cpp",
			"../../../TinyRenderer/our_gl.cpp",
			"../../../TinyRenderer/tgaimage.cpp",
			"../../../TinyRenderer/TinyRenderer.cpp",
			"../../../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../../../ThirdPartyLibs/stb_image/stb_image.cpp",
			"../../../ThirdPartyLibs/stb_image/stb_image.h",
			"../../../ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
			"../../../ThirdPartyLibs/tinyxml2/tinyxml2.h",
			"../../../OpenGLWindow/SimpleCamera.cpp",
			"../../../OpenGLWindow/SimpleCamera.h",
			"../../../Utils/b3Clock.cpp",
			"../../../Utils/b3Clock.h",
			"../../../Utils/b3ResourcePath.cpp",
			"../../../Utils/b3ResourcePath.h",
			}
	
	
	
