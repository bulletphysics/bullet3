	project "LinearMath"

		kind "StaticLib"

		includedirs {
			"..",
		}

		files {
			"*.cpp",
			"*.h"
		}


	project "LinearMathDLL"

		kind "SharedLib"

        targetname "LinearMath"

		includedirs {
			"..",
		}

		files {
			"*.cpp",
			"*.h"
		}
