	project "Bullet3Geometry"

	language "C++"
				
	kind "StaticLib"
		
	includedirs {".."}
	targetdir "../../bin"

	files {
		"**.cpp",
		"**.h"
	}