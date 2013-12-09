	
		project "App2_FeatherstoneMultiBodyDemo"

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"

  	includedirs {
                ".",
                "../../../src",
                "../../../btgui"
                }

		initOpenGL()
		initGlew()
			
		links{"gwen", "BulletDynamics", "BulletCollision","LinearMath",
			"OpenGL_Window", "OpenGL_TrueTypeFont"
		}
		
		files {
		"**.cpp",
		"**.h",
		"../../../src/Bullet3Common/**.cpp",
 		"../../../src/Bullet3Common/**.h",
		"../../../btgui/Timing/b3Clock.cpp",
		"../../../btgui/Timing/b3Clock.h"

		}

	if os.is("Linux") then 
		links ("X11")
	end

	if os.is("MacOSX") then
		links{"Cocoa.framework"}
	end
