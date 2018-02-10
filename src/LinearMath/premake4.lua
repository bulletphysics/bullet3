	project "LinearMath"

	kind "StaticLib"
	if os.is("Linux") then
	    buildoptions{"-fPIC"}
	end
	includedirs {
		"..",
	}
	files {
		"*.cpp",
		"*.h"
	}
