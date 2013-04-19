	project "Bullet3Collision"

	language "C++"
				
	kind "StaticLib"
		
	includedirs {".."}
	targetdir "../../bin"

	files {
		"**.cpp",
		"**.h"
	}