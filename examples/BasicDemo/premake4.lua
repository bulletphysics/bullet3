
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
defines {"USE_GUI"}

includedirs {"../../src"}

links {
        "BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common"
}

	initOpenGL()
        initGlew()


language "C++"

files {
        "**.cpp",
        "**.h",
	"../ExampleBrowser/OpenGLGuiHelper.cpp",
	"../ExampleBrowser/GL_ShapeDrawer.cpp"
}

if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end
                          
