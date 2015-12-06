	project "Bullet3Dynamics"

	configuration { "*" }
		kind "StaticLib"
	configuration { "*DLL" }
		kind "SharedLib"
	configuration {}

	includedirs {
		".."
	}

	files {
		"**.cpp",
		"**.h"
	}
