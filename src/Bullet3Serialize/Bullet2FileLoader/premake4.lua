	project "Bullet2FileLoader"

	configuration { "*" }
		kind "StaticLib"
	configuration { "*DLL" }
		kind "SharedLib"
	configuration {}

	includedirs {
		"../../../src"
	}

	files {
		"**.cpp",
		"**.h"
	}
