project "App_BasicDemoConsole"
			
		kind "ConsoleApp"
		targetdir "../../bin"
	
	  includedirs {"../../src"}
		configuration { "Windows" }
--	 		links { "opengl32","glu32","gdi32","winmm", "user32" }
	 		files   { "../../build3/bullet.rc" }
--		configuration {"MacOSX"}
			--print "hello"
	-- 		linkoptions { "-framework Carbon -framework OpenGL -framework AGL -framework Glut" } 	
--		configuration {"not Windows", "not MacOSX"}
--			links {"GL","GLU","glut"}
		configuration{}
	
		links { 
				"BulletDynamics", "BulletCollision", "LinearMath"
			}
		
		files     { 
		 	"*.cpp" ,
		 	"*.h",
		 	"../../Demos/BasicDemo/BasicDemoPhysicsSetup.cpp",
			"../../Demos/BasicDemo/BasicDemoPhysicsSetup.h"
		 }
