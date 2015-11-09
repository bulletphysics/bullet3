	project "VHACD"
		
	kind "StaticLib"
	
	includedirs {".", "inc", "../../src"}
	files {
		"**.cpp",
		"**.h"
	}