	project "BulletInverseDynamics"

		kind "StaticLib"

		includedirs {
			"..",
		}

		files {
			"IDMath.cpp",
			"MultiBodyTree.cpp",
			"details/MultiBodyTreeInitCache.cpp",
			"details/MultiBodyTreeImpl.cpp",
		}


	project "BulletInverseDynamicsDLL"

		kind "SharedLib"

		includedirs {
			"..",
		}

		files {
			"IDMath.cpp",
			"MultiBodyTree.cpp",
			"details/MultiBodyTreeInitCache.cpp",
			"details/MultiBodyTreeImpl.cpp",
		}
