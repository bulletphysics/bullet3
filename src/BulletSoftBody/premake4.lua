	project "BulletSoftBody"

		kind "StaticLib"

		includedirs {
			"..",
		}

		files {
			"**.cpp",
			"**.h"
		}


	project "BulletSoftBodyDLL"

		kind "SharedLib"

		includedirs {
			"..",
		}

		files {
			"**.cpp",
			"**.h"
		}
