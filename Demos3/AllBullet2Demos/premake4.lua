	
		project "App_AllBullet2Demos"

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

  	includedirs {
                ".",
                "../../src",
                "../../btgui"
                }

		initOpenGL()
		initGlew()
			
		links{"gwen", "OpenGL_Window","OpenGL_TrueTypeFont","BulletSoftBody","BulletDynamics","BulletCollision","LinearMath"}
		
		files {
		"**.cpp",
		"**.h",
		"../../src/Bullet3Common/**.cpp",
 		"../../src/Bullet3Common/**.h",
		"../../btgui/Timing/b3Clock.cpp",
		"../../btgui/Timing/b3Clock.h",
		"../GpuDemos/gwenUserInterface.cpp",
		"../GpuDemos/gwenUserInterface.h"
		}

if os.is("MacOSX") then
                        links{"Cocoa.framework"}
end
