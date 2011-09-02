	project "OpenGLSupport"
		
	kind "StaticLib"
	targetdir "../../lib"
	includedirs {
		".",
		"../../src"
	}
	configuration {"Windows"}
	includedirs {
		"../../Glut"
	}
	configuration{}

	files {
		"**.cpp",
		"**.h"
	}
