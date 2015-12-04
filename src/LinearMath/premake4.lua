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

		includedirs {
			"..",
		}

		files {
			"*.cpp",
			"*.h"
		}
