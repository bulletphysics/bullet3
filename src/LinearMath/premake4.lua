	project "LinearMath"

	kind "StaticLib"
	includedirs {
		"..",
	}
	files {
		"*.cpp",
		"*.h"
	}flags { "Symbols" }
