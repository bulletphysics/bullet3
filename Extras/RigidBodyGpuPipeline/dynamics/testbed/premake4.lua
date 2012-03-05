	project "testbed"
		
	kind "StaticLib"
	targetdir "../../build/lib"	
	includedirs {
		".",
		"../../bullet2"
	}
	configuration {"Windows"}
	includedirs {
		"../../rendering/GlutGlewWindows"
	}
	configuration{}

	files {
		"**.cpp",
		"**.h"
	}
