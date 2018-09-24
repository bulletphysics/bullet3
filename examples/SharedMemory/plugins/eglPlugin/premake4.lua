		

project ("pybullet_eglRendererPlugin")
		language "C++"
		kind "SharedLib"
		initEGL()
		
		includedirs {".","../../../../src", "../../../../examples",
		"../../../ThirdPartyLibs", "../../../ThirdPartyLibs/glad"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER", "STB_AGAIN"}
	hasCL = findOpenCL("clew")

	links{"BulletCollision", "Bullet3Common", "LinearMath"}

	initOpenGL()

	if os.is("Windows") then
		files {"../../../OpenGLWindow/Win32OpenGLWindow.cpp",
		"../../../OpenGLWindow/Win32Window.cpp",}
		
	end
	if os.is("MacOSX") then
--		targetextension {"so"}
		links{"Cocoa.framework"}
	end

  if os.is("Linux") then
	  files {"../../../OpenGLWindow/EGLOpenGLWindow.cpp"}

  end

		files {
			"eglRendererPlugin.cpp",
			"eglRendererPlugin.h",
			"eglRendererVisualShapeConverter.cpp",
			"eglRendererVisualShapeConverter.h",
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
			"../../../ThirdPartyLibs/glad/gl.c",
			"../../../ThirdPartyLibs/glad/egl.c",
			"../../../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../../../ThirdPartyLibs/stb_image/stb_image.cpp",
			"../../../ThirdPartyLibs/stb_image/stb_image.h",
			"../../../ThirdPartyLibs/stb_image/stb_image_write.cpp",
			"../../../ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
			"../../../ThirdPartyLibs/tinyxml2/tinyxml2.h",
			"../../../OpenGLWindow/SimpleCamera.cpp",
			"../../../OpenGLWindow/SimpleCamera.h",
			"../../../OpenGLWindow/GLInstancingRenderer.cpp",
			"../../../OpenGLWindow/GLInstancingRenderer.h",
			"../../../OpenGLWindow/LoadShader.cpp",
			"../../../OpenGLWindow/LoadShader.h",
			"../../../OpenGLWindow/GLRenderToTexture.cpp",
			"../../../OpenGLWindow/GLRenderToTexture.h",
			"../../../Utils/b3Clock.cpp",
			"../../../Utils/b3Clock.h",
			"../../../Utils/b3ResourcePath.cpp",
			"../../../Utils/b3ResourcePath.h",
			}
	
	
	
