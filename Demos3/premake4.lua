
  
    
function createDemos( demos, incdirs, linknames)
	for _, name in ipairs(demos) do
				
			project ( "App_" .. name )
			
			kind "ConsoleApp"
			targetdir ".."
	
	  	includedirs {incdirs}
	  
		configuration { "Windows" }
			defines { "GLEW_STATIC"}
	 		links { "opengl32","glu32","gdi32","winmm", "user32" }
			includedirs{	"../ObsoleteDemos/Glut"	}
	 		libdirs {"../ObsoleteDemos/Glut"}
	 		files   { "../build3/bullet.rc" }
	 		
	 		configuration {"Windows", "x32"}
				links {"glew32s","glut32"}
			configuration {"Windows", "x64"}
				links {"glew64s", "glut64"}
	
		configuration {"MacOSX"}
			--print "hello"
	 		linkoptions { "-framework Carbon -framework OpenGL -framework AGL -framework Glut" } 
		
		configuration {"not Windows", "not MacOSX"}
			links {"GL","GLU","glut"}
		configuration{}
	
		links { 
				linknames
			}
		
		files     { 
		 	"./" .. name .. "/*.cpp" ,
		 	"./" .. name .. "/*.h"
		 }
	end
end


function createGpuDemos( demos, incdirs, linknames, vendor)
	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then
	
		for _, name in ipairs(demos) do
					
				project ( "App_" .. name .. "_" .. vendor)
				
				initOpenCL(vendor)
				
				kind "ConsoleApp"
				targetdir "../bin"
		
				links {"Bullet3OpenCL_" .. vendor }
				
		  	includedirs {incdirs}
		  
				configuration { "Windows" }
				defines { "GLEW_STATIC"}
		 		links { "opengl32" ,"glu32","gdi32","winmm", "user32"}
				includedirs{	"../ObsoleteDemos/Glut"	}
		 		libdirs {"../ObsoleteDemos/Glut"}
		 		files   { "../build3/bullet.rc" }
		 		
		 		configuration {"Windows", "x32"}
					links {"glew32s","glut32"}
				configuration {"Windows", "x64"}
					links {"glew64s", "glut64"}
		
			configuration {"MacOSX"}
				--print "hello"
		 		linkoptions { "-framework Carbon -framework OpenGL -framework AGL -framework Glut" } 
			
			configuration {"not Windows", "not MacOSX"}
				links {"GL","GLU","glut"}
			configuration{}
		
			links { 
					linknames
				}
			
			files     { 
			 	"./" .. name .. "/*.cpp" ,
			 	"./" .. name .. "/*.h"
			 }
		end
	end
end

-- "CharacterDemo", fixme: it includes BspDemo files

 local localdemos = {
    "BasicDemo",
  }
  
   local localgpudemos = {
 		"BasicGpuDemo",
  }

-- the following demos require custom include or link settings

-- createDemos({"HelloWorld"},{"../src"},{"BulletDynamics","BulletCollision","LinearMath"})

 createDemos(localdemos,{"../src","../ObsoleteDemos/OpenGL"},{"OpenGLSupport","BulletDynamics", "BulletCollision", "LinearMath"})
 createGpuDemos(localgpudemos,{"../src","../ObsoleteDemos/OpenGL"},{"OpenGLSupport",
 					"Bullet3Dynamics","Bullet3Collision","Bullet3Geometry","Bullet3Common",
 					"BulletDynamics", "BulletCollision", "LinearMath"},
 					"NVIDIA")

createGpuDemos(localgpudemos,{"../src","../ObsoleteDemos/OpenGL"},{"OpenGLSupport",
 					"Bullet3Dynamics","Bullet3Collision","Bullet3Geometry","Bullet3Common",
 					"BulletDynamics", "BulletCollision", "LinearMath"},
 					"clew")
 					
createGpuDemos(localgpudemos,{"../src","../ObsoleteDemos/OpenGL"},{"OpenGLSupport",
 					"Bullet3Dynamics","Bullet3Collision","Bullet3Geometry","Bullet3Common",
 					"BulletDynamics", "BulletCollision", "LinearMath"},
 					"AMD")

createGpuDemos(localgpudemos,{"../src","../ObsoleteDemos/OpenGL"},{"OpenGLSupport",
 					"Bullet3Dynamics","Bullet3Collision","Bullet3Geometry","Bullet3Common",
 					"BulletDynamics", "BulletCollision", "LinearMath"},
 					"Intel") 					

createGpuDemos(localgpudemos,{"../src","../ObsoleteDemos/OpenGL"},{"OpenGLSupport",
 					"Bullet3Dynamics","Bullet3Collision","Bullet3Geometry","Bullet3Common",
 					"BulletDynamics", "BulletCollision", "LinearMath"},
 					"Apple") 					
 					 				
include "../ObsoleteDemos/OpenGL"
 
 
