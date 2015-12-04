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

        targetname "BulletSoftBody"

		includedirs {
			"..",
		}

		files {
			"**.cpp",
			"**.h"
		}
