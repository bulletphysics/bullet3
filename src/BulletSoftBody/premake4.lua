	project "BulletSoftBody"

	configuration { "*" }
		kind "StaticLib"
	configuration { "*DLL" }
		kind "SharedLib"
	configuration {}

	includedirs {
		"..",
	}

	files {
		"**.cpp",
		"**.h"
	}

