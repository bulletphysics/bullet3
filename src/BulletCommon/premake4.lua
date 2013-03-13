	project "BulletCommon"

	language "C++"
				
	kind "StaticLib"
		
	targetdir "../../bin"

	files {
		"**.cpp",
		"**.h"
	}