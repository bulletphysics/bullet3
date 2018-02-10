	project "Bullet3Collision"

	language "C++"
				
	kind "StaticLib"
		
	includedirs {".."}

    if os.is("Linux") then
        buildoptions{"-fPIC"}
    end

	files {
		"**.cpp",
		"**.h"
	}