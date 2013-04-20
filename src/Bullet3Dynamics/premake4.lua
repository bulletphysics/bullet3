	project "Bullet3Dynamics"

	language "C++"
				
	kind "StaticLib"

	includedirs {
		".."
	}		
	targetdir "../../bin"

	files {
		"**.cpp",
		"**.h"
	}