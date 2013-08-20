	project "Bullet3Common"

	language "C++"
				
	kind "StaticLib"
		
	targetdir "../../bin"
	
	includedirs {".."}

	files {
		"**.cpp",
		"**.h"
	}