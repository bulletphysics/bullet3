
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
	"../TinyRenderer/geometry.cpp",
	"../TinyRenderer/model.cpp",
	"../TinyRenderer/tgaimage.cpp",
	"../TinyRenderer/our_gl.cpp",
	"../TinyRenderer/TinyRenderer.cpp",
	"../Utils/b3ResourcePath.cpp"
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
	"../OpenGLWindow/SimpleCamera.cpp",
	"../TinyRenderer/geometry.cpp",
	"../TinyRenderer/model.cpp",
	"../TinyRenderer/tgaimage.cpp",
	"../TinyRenderer/our_gl.cpp",
	"../TinyRenderer/TinyRenderer.cpp",
	"../Utils/b3ResourcePath.cpp"
}
