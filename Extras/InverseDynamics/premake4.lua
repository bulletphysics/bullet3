	project "BulletInverseDynamicsUtils"

	kind "StaticLib"

	includedirs {
		"../../src"
	}

    if os.is("Linux") then
        buildoptions{"-fPIC"}
    end
	files {
		"*.cpp",
		"*.hpp"
	}
