	project "vhacd"

	kind "StaticLib"
    if os.is("Linux") then
        buildoptions{"-fPIC"}
    end
	includedirs {
		"../inc","../public",
	}
	files {
		"*.cpp",
		"*.h"
	}
