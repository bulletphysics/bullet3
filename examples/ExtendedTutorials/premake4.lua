
project "App_RigidBodyFromObjExample"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}
includedirs {"../../src"}

links {
	"BulletInverseDynamicsUtils", "BulletInverseDynamics","Bullet3Common","BulletDynamics","BulletCollision", "LinearMath"
}

language "C++"

files {
	"RigidBodyFromObj.cpp",
	"../CommonInterfaces/*",
	"**.h",
	"../StandaloneMain/main_console_single_example.cpp",
		"../Utils/b3ResourcePath.cpp",
			"../Utils/b3ResourcePath.h",
			"../RenderingExamples/TimeSeriesCanvas.cpp",
			"../RenderingExamples/TimeSeriesFontData.cpp",
			"../MultiBody/InvertedPendulumPDControl.cpp",
			"../ThirdPartyLibs/tinyxml/tinystr.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxml.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../Importers/ImportURDFDemo/UrdfParser.cpp",
			"../Importers/ImportURDFDemo/urdfStringSplit.cpp",
			"../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
  		"../ThirdPartyLibs/stb_image/stb_image.cpp",
}


project "App_RigidBodyFromObjExampleGui"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src"}

links {
        "BulletInverseDynamicsUtils", "BulletInverseDynamics","BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common"
}

	initOpenGL()
  initGlew()


language "C++"

files {
        "RigidBodyFromObj.cpp",
        "*.h",
        "../StandaloneMain/main_opengl_single_example.cpp",
	"../CommonInterfaces/*",
	"../ExampleBrowser/OpenGLGuiHelper.cpp",
	"../ExampleBrowser/GL_ShapeDrawer.cpp",
	"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
			"../Utils/b3ResourcePath.cpp",
			"../Utils/b3ResourcePath.h",
			"../RenderingExamples/TimeSeriesCanvas.cpp",
			"../RenderingExamples/TimeSeriesFontData.cpp",
			"../MultiBody/InvertedPendulumPDControl.cpp",
			"../ThirdPartyLibs/tinyxml/tinystr.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxml.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../Importers/ImportURDFDemo/UrdfParser.cpp",
			"../Importers/ImportURDFDemo/urdfStringSplit.cpp",
			"../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
      "../ThirdPartyLibs/stb_image/stb_image.cpp",
      "../Utils/b3Clock.cpp",
			"../Utils/b3Clock.h",
}

if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end
                          


project "App_RigidBodyFromObjExampleGuiWithSoftwareRenderer"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src"}

links {
        "BulletInverseDynamicsUtils", "BulletInverseDynamics","BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common"
}

	initOpenGL()
        initGlew()


language "C++"

files {
        "RigidBodyFromObj.cpp",
        "*.h",
        "../StandaloneMain/main_sw_tinyrenderer_single_example.cpp",
	"../ExampleBrowser/OpenGLGuiHelper.cpp",
	"../ExampleBrowser/GL_ShapeDrawer.cpp",
	"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
	"../CommonInterfaces/*",
	"../TinyRenderer/geometry.cpp",
	"../TinyRenderer/model.cpp",
	"../TinyRenderer/tgaimage.cpp",
	"../TinyRenderer/our_gl.cpp",
	"../TinyRenderer/TinyRenderer.cpp",
	"../Utils/b3ResourcePath.cpp",
			"../Utils/b3ResourcePath.cpp",
			"../Utils/b3ResourcePath.h",
			"../RenderingExamples/TimeSeriesCanvas.cpp",
			"../RenderingExamples/TimeSeriesFontData.cpp",
			"../MultiBody/InvertedPendulumPDControl.cpp",
			"../ThirdPartyLibs/tinyxml/tinystr.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxml.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../Importers/ImportURDFDemo/UrdfParser.cpp",
			"../Importers/ImportURDFDemo/urdfStringSplit.cpp",
			"../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
			"../ThirdPartyLibs/stb_image/stb_image.cpp",     
}

if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end
                          


project "App_RigidBodyFromObjExampleTinyRenderer"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src"}

links {
        "BulletInverseDynamicsUtils", "BulletInverseDynamics","BulletDynamics","BulletCollision", "LinearMath", "Bullet3Common"
}


language "C++"

files {
        "RigidBodyFromObj.cpp",
        "*.h",
        "../StandaloneMain/main_tinyrenderer_single_example.cpp",
	"../OpenGLWindow/SimpleCamera.cpp",
	"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
	"../CommonInterfaces/*",
	"../TinyRenderer/geometry.cpp",
	"../TinyRenderer/model.cpp",
	"../TinyRenderer/tgaimage.cpp",
	"../TinyRenderer/our_gl.cpp",
	"../TinyRenderer/TinyRenderer.cpp",
	"../Utils/b3ResourcePath.cpp",
			"../Utils/b3ResourcePath.cpp",
			"../Utils/b3ResourcePath.h",
			"../RenderingExamples/TimeSeriesCanvas.cpp",
			"../RenderingExamples/TimeSeriesFontData.cpp",
			"../MultiBody/InvertedPendulumPDControl.cpp",
			"../ThirdPartyLibs/tinyxml/tinystr.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxml.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp",
			"../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../Importers/ImportURDFDemo/UrdfParser.cpp",
			"../Importers/ImportURDFDemo/urdfStringSplit.cpp",
			"../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
                        "../ThirdPartyLibs/stb_image/stb_image.cpp",     
}

