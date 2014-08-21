project "App_BasicDemoCustomOpenGL2"
			
		kind "ConsoleApp"
		targetdir "../../bin"
	
	  includedirs {
	  	"../../src",
	  	"../../btgui"
	  }
		
		defines { "DONT_USE_GLUT", "USE_OPENGL2"}
		
		initOpenGL()
		initGlew()
	
		links { 
				"OpenGL_Window","BulletDynamics", "BulletCollision", "LinearMath"
			}
					
		files     { 
		 	"*.cpp" ,
		 	"*.h",
		 	"../../Demos/BasicDemo/BasicDemoPhysicsSetup.cpp",
			"../../Demos/BasicDemo/BasicDemoPhysicsSetup.h",
			"../../Demos/OpenGL/DemoApplication.cpp",
			"../../Demos/OpenGL/DemoApplication.h",
			"../../Demos/OpenGL/GL_ShapeDrawer.cpp",
			"../../Demos/OpenGL/GL_ShapeDrawer.h",
			"../../Demos/OpenGL/GL_ShapeDrawer.cpp",
			"../../Demos/OpenGL/GLDebugDrawer.cpp",
			"../../Demos/OpenGL/GLDebugDrawer.h",
			"../../Demos/OpenGL/GLDebugFont.cpp",
			"../../Demos/OpenGL/GLDebugFont.h"			
		 }
if os.is("Linux") then initX11() end

if os.is("MacOSX") then
                        links{"Cocoa.framework"}
end
