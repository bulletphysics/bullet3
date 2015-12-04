	project "BulletDynamics"

		kind "StaticLib"

		includedirs {
			"..",
		}

		files {
			"Dynamics/*.cpp",
			"Dynamics/*.h",
			"ConstraintSolver/*.cpp",
			"ConstraintSolver/*.h",
			"Featherstone/*.cpp",
			"Featherstone/*.h",
			"MLCPSolvers/*.cpp",
			"MLCPSolvers/*.h",
			"Vehicle/*.cpp",
			"Vehicle/*.h",
			"Character/*.cpp",
			"Character/*.h"
		}


	project "BulletDynamicsDLL"

		kind "SharedLib"

        targetname "BulletDynamics"

		includedirs {
			"..",
		}

		files {
			"Dynamics/*.cpp",
			"Dynamics/*.h",
			"ConstraintSolver/*.cpp",
			"ConstraintSolver/*.h",
			"Featherstone/*.cpp",
			"Featherstone/*.h",
			"MLCPSolvers/*.cpp",
			"MLCPSolvers/*.h",
			"Vehicle/*.cpp",
			"Vehicle/*.h",
			"Character/*.cpp",
			"Character/*.h"
		}
