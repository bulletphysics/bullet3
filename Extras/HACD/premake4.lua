	project "HACD"
		
	kind "StaticLib"
	targetdir "../../lib"
	includedirs {"."}
	files {
		"**.cpp",
		"**.h"
	}