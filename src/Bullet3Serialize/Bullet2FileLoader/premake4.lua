	project "Bullet2FileLoader"
		
	kind "StaticLib"
	targetdir "../../../lib"
	includedirs {
		"../../../src"
	}
	 
	files {
		"**.cpp",
		"**.h"
	}