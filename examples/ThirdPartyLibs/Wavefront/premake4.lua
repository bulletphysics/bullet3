
	project "App_WavefrontObjLoader"
		
	kind "ConsoleApp"
	
--	defines {  }
	
	targetdir "../../bin"
	
	includedirs 
	{
		".","../../src"
	}

			
	links { "Bullet3Common" }
	
	
	files {
		"**.cpp",
		"**.h"
	}

