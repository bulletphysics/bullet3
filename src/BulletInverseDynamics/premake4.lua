	project "BulletInverseDynamics"

	configuration { "*" }
		kind "StaticLib"
	configuration { "*DLL" }
		kind "SharedLib"
	configuration {}

	includedirs {
		"..",
	}

	files {
		"IDMath.cpp",
		"MultiBodyTree.cpp",
		"details/MultiBodyTreeInitCache.cpp",
		"details/MultiBodyTreeImpl.cpp",
	}

