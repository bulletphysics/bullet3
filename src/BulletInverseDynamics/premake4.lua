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

        targetname "BulletInverseDynamics"

		includedirs {
			"..",
		}

		files {
			"IDMath.cpp",
			"MultiBodyTree.cpp",
			"details/MultiBodyTreeInitCache.cpp",
			"details/MultiBodyTreeImpl.cpp",
		}
