
project "App_BasicExample"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {"../../src"}

links {
	"BulletDynamics","BulletCollision", "LinearMath"
}

language "C++"

files {
	"**.cpp",
	"**.h",
	"../CommonInterfaces/*",
}


project "App_BasicExampleGui"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src"}

links {
        "BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common"
}

	initOpenGL()
  initGlew()


language "C++"

files {
        "BasicExample.cpp",
        "*.h",
        "../StandaloneMain/main_opengl_single_example.cpp",
	"../ExampleBrowser/OpenGLGuiHelper.cpp",
	"../ExampleBrowser/GL_ShapeDrawer.cpp",
	"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
	"../CommonInterfaces/*",
	"../Utils/b3Clock.cpp",
	"../Utils/b3Clock.h",
}

if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end
                          



project "App_BasicExampleGuiWithSoftwareRenderer"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src"}

links {
        "BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common"
}

	initOpenGL()
        initGlew()


language "C++"

files {
        "BasicExample.cpp",
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
	"../Utils/b3Clock.cpp",
	"../Utils/b3Clock.h",
}

if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end
                          

project "App_BasicExampleTinyRenderer"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src"}

links {
        "BulletDynamics","BulletCollision", "LinearMath", "Bullet3Common"
}


language "C++"

files {
        "BasicExample.cpp",
        "*.h",
        "../StandaloneMain/main_tinyrenderer_single_example.cpp",
	"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
	"../CommonInterfaces/*",
	"../OpenGLWindow/SimpleCamera.cpp",
	"../TinyRenderer/geometry.cpp",
	"../TinyRenderer/model.cpp",
	"../TinyRenderer/tgaimage.cpp",
	"../TinyRenderer/our_gl.cpp",
	"../TinyRenderer/TinyRenderer.cpp",
	"../Utils/b3ResourcePath.cpp",
	"../Utils/b3Clock.cpp",
	"../Utils/b3Clock.h",
}




		if _OPTIONS["enable_openvr"] then

project "App_BasicExampleVR"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE","BT_ENABLE_VR"}



includedirs {"../../src", 
					"../ThirdPartyLibs/openvr/headers",
					"../ThirdPartyLibs/openvr/samples/shared"}

links {
        "BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common", "openvr_api"
}

	initOpenGL()
  initGlew()


language "C++"

files {
        "BasicExample.cpp",
        "*.h",
        "../StandaloneMain/hellovr_opengl_main.cpp",
	"../CommonInterfaces/*",
				"../ExampleBrowser/OpenGLGuiHelper.cpp",
				"../ExampleBrowser/GL_ShapeDrawer.cpp",
				"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
				"../ThirdPartyLibs/openvr/samples/shared/lodepng.cpp",
				"../ThirdPartyLibs/openvr/samples/shared/lodepng.h",
				"../ThirdPartyLibs/openvr/samples/shared/Matrices.cpp",
				"../ThirdPartyLibs/openvr/samples/shared/Matrices.h",
				"../ThirdPartyLibs/openvr/samples/shared/pathtools.cpp",
				"../ThirdPartyLibs/openvr/samples/shared/pathtools.h",
				"../ThirdPartyLibs/openvr/samples/shared/Vectors.h",
				"../Utils/b3Clock.cpp",
				"../Utils/b3Clock.h",
				
}

if os.is("Windows") then 
	libdirs {"../ThirdPartyLibs/openvr/lib/win32"}
end

if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end

end
